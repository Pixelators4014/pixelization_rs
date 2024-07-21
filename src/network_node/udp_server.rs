use std::io;
use std::sync::Arc;

use std::net::{IpAddr, Ipv4Addr, SocketAddr};

use tokio::net::UdpSocket;
use tokio::sync::mpsc;
use tokio::sync::RwLock;

use thiserror::Error;

use nalgebra::{Quaternion, Rotation3, UnitQuaternion};

use log::{debug, error, info, trace, warn};

use nav_msgs::msg::Path as PathMsg;
use rclrs::RclrsError;

// TODO: should be configurable
const HOST: IpAddr = IpAddr::V4(Ipv4Addr::new(10, 0, 0, 1));

pub const PROTOCOL_VERSION: u16 = 1;

/// This is how the server represents a pose, the representation follows this ordering
#[derive(Copy, Clone, Debug)]
struct Pose {
    x: f32,
    y: f32,
    z: f32,
    roll: f32,
    pitch: f32,
    yaw: f32,
}

#[derive(Copy, Clone, Error, Debug)]
pub enum PoseParseError {
    #[error("Invalid number of bytes")]
    InvalidNumberOfBytes,
    #[error("Failed to parse float")]
    ParseFloatError(#[from] std::array::TryFromSliceError),
}

impl Pose {
    fn from_bytes(bytes: &[u8]) -> Result<Self, PoseParseError> {
        if bytes.len() != 24 {
            return Err(PoseParseError::InvalidNumberOfBytes);
        }
        Ok(Self {
            x: f32::from_le_bytes(bytes[0..4].try_into()?),
            y: f32::from_le_bytes(bytes[4..8].try_into()?),
            z: f32::from_le_bytes(bytes[8..12].try_into()?),
            roll: f32::from_le_bytes(bytes[12..16].try_into()?),
            pitch: f32::from_le_bytes(bytes[16..20].try_into()?),
            yaw: f32::from_le_bytes(bytes[20..24].try_into()?),
        })
    }

    fn to_bytes(&self) -> [u8; 24] {
        let mut bytes = [0u8; 24];
        bytes[0..4].copy_from_slice(&self.x.to_le_bytes());
        bytes[4..8].copy_from_slice(&self.y.to_le_bytes());
        bytes[8..12].copy_from_slice(&self.z.to_le_bytes());
        bytes[12..16].copy_from_slice(&self.roll.to_le_bytes());
        bytes[16..20].copy_from_slice(&self.pitch.to_le_bytes());
        bytes[20..24].copy_from_slice(&self.yaw.to_le_bytes());
        return bytes;
    }
}

#[derive(Error, Debug)]
#[repr(u16)]
pub enum ServerError {
    #[error("I/O error: {0}")]
    IoError(#[from] io::Error) = 100,
    #[error("Failed due to rclrs error (probably a ros2 failure): {0}")]
    RclrsError(#[from] RclrsError) = 101,
    #[error("Unknown request first byte: {0}")]
    UnknownFirstByte(u8) = 200,
    #[error("Request too short, expected at least 1 byte.")]
    TooShort = 201,
    #[error("Unsupported Client Version")]
    UnsupportedClientVersion = 202,
    #[error("Failed to parse pose: {0}")]
    PoseError(#[from] PoseParseError) = 203,
    #[error("Request too short, expected at least 25 bytes if SetVslamPose is requested (first byte is 1).")]
    TooShortForSetVslamPose = 204,
    #[error("Unexpected server shutdown/panic")]
    ServerShutdown = 300,
    #[error("No vslam data in path")]
    NoVslamData = 400,
    #[error("No path recieved")]
    NoVslamPath = 401,
    #[error("Failed to set slam pose due to service error")]
    SetSlamPoseServiceError = 402
}
#[derive(Copy, Clone, Debug)]
enum Request {
    GetVslamPose,
    SetVslamPose(Pose),
    GetDetections,
    Ping
}

impl Request {
    fn from_bytes(bytes: &[u8]) -> Result<Self, ServerError> {
        if bytes.len() < 3 {
            return Err(ServerError::TooShort);
        }

        let client_protocol_version = u16::from_le_bytes([bytes[0], bytes[1]]);
        if client_protocol_version > 10 {
            return Err(ServerError::UnsupportedClientVersion);
        }

        return if bytes[2] == 0 {
            Ok(Self::GetVslamPose)
        } else if bytes[2] == 1 {
            if bytes.len() < 25 {
                return Err(ServerError::TooShortForSetVslamPose);
            }
            Ok(Self::SetVslamPose(Pose::from_bytes(&bytes[1..26])?))
        } else if bytes[2] == 2 {
            Ok(Self::GetDetections)
        } else if bytes[2] == 255 {
            Ok(Self::Ping)
        } else {
            Err(ServerError::UnknownFirstByte(bytes[0]))
        };
    }
}

enum Response {
    Pose(Pose),
    Success,
    Error(ServerError),
    Version
}

impl Response {
    fn to_bytes(&self) -> Vec<u8> {
        return match self {
            Self::Success => {
                let bytes = [0u8];
                bytes.to_vec()
            }
            Self::Error(msg) => {
                let d = unsafe { <*const ServerError>::from(msg).cast::<u16>().read() };
                let d_bytes = d.to_le_bytes();
                let msg = format!("{:?}", msg);
                let mut bytes = [0u8; 1024];
                bytes[0] = 1;
                bytes[1] = d_bytes[0];
                bytes[2] = d_bytes[1];
                bytes[3..msg.len() + 3].copy_from_slice(msg.as_bytes());
                bytes.to_vec()
            }
            Self::Pose(pose) => {
                let mut bytes = [0u8; 32];
                bytes[0] = 255;
                bytes[1..25].copy_from_slice(&pose.to_bytes());
                bytes.to_vec()
            }
            Self::Version => {
                let mut bytes = PROTOCOL_VERSION.to_le_bytes();
                bytes.to_vec()
            }
        };
    }
}

impl From<Pose> for Response {
    fn from(pose: Pose) -> Self {
        Self::Pose(pose)
    }
}

struct Packet {
    buf: Vec<u8>,
    addr: SocketAddr,
}

pub struct Server {
    data: Arc<RwLock<Option<PathMsg>>>,
    client: Arc<rclrs::Client<isaac_ros_visual_slam_interfaces::srv::SetSlamPose>>,
    socket: Arc<UdpSocket>,
}

impl Server {
    pub async fn new(
        data: Arc<RwLock<Option<PathMsg>>>,
        client: Arc<rclrs::Client<isaac_ros_visual_slam_interfaces::srv::SetSlamPose>>,
        port: u16,
        ip_addr: IpAddr
    ) -> io::Result<Self> {
        let socket = UdpSocket::bind(SocketAddr::new(ip_addr, port)).await?;
        Ok(Self {
            data,
            client,
            socket: Arc::new(socket),
        })
    }

    async fn process_request(
        data: Arc<RwLock<Option<PathMsg>>>,
        client: Arc<rclrs::Client<isaac_ros_visual_slam_interfaces::srv::SetSlamPose>>,
        request: Request,
    ) -> Response {
        trace!("Request: {request:?}");
        return match request {
            Request::GetVslamPose => {
                if let Some(msg) = data.read().await.as_ref() {
                    if let Some(last) = msg.poses.last() {
                        info!("Last pose: {last:?}");
                        let rotation =
                            Rotation3::from(UnitQuaternion::new_normalize(Quaternion::new(
                                last.pose.orientation.w,
                                last.pose.orientation.x,
                                last.pose.orientation.y,
                                last.pose.orientation.z,
                            )))
                            .euler_angles();
                        let response = Pose {
                            x: last.pose.position.x as f32,
                            y: last.pose.position.y as f32,
                            z: last.pose.position.z as f32,
                            roll: rotation.0 as f32,
                            pitch: rotation.1 as f32,
                            yaw: rotation.2 as f32,
                        };
                        trace!("Response: {response:?}");
                        Response::Pose(response.into())
                    } else {
                        warn!("No VSLAM data received, vslam might still being initializing");
                        Response::Error(ServerError::NoVslamData)
                    }
                } else {
                    warn!("No VSLAM data received");
                    Response::Error(ServerError::NoVslamPath)
                }
            }
            Request::SetVslamPose(pose) => {
                let unit_quaternion =
                    UnitQuaternion::from_euler_angles(pose.roll, pose.pitch, pose.yaw);
                let quaternion = unit_quaternion.quaternion();
                let service_request = isaac_ros_visual_slam_interfaces::srv::SetSlamPose_Request {
                    pose: geometry_msgs::msg::Pose {
                        position: geometry_msgs::msg::Point {
                            x: pose.x as f64,
                            y: pose.y as f64,
                            z: pose.z as f64,
                        },
                        orientation: geometry_msgs::msg::Quaternion {
                            w: quaternion.coords[3] as f64,
                            x: quaternion.coords[0] as f64,
                            y: quaternion.coords[1] as f64,
                            z: quaternion.coords[2] as f64,
                        },
                    },
                };
                let response_res = client.call_async(&service_request).await;
                match response_res {
                    Ok(response) => {
                        if response.success {
                            Response::Success
                        } else {
                            warn!("Failed to set VSLAM pose due to service error: {service_request:?}");
                            Response::Error(ServerError::SetSlamPoseServiceError)
                        }
                    }
                    Err(e) => {
                        warn!("Failed to set VSLAM pose due to rcl error: {e}");
                        Response::Error(ServerError::RclrsError(e))
                    }
                }
            }
            Request::GetDetections => {
                // TODO: Fix (actually return detections)
                Response::Success
            }
            Request::Ping => {
                Response::Version
            }
        };
    }

    pub async fn handle_bytes(
        data: Arc<RwLock<Option<PathMsg>>>,
        client: Arc<rclrs::Client<isaac_ros_visual_slam_interfaces::srv::SetSlamPose>>,
        bytes: &Vec<u8>,
    ) -> Vec<u8> {
        let request = Request::from_bytes(bytes);
        match request {
            Ok(request) => Self::process_request(data, client, request)
                .await
                .to_bytes(),
            Err(e) => {
                warn!("Invalid Request: {e:?}");
                Response::Error(e).to_bytes()
            }
        }
    }

    /// This runs the server by having the main packet routing on this loop, and handling the packets to send the response back on a second set of tasks.
    pub async fn run(&self) -> Result<(), ServerError> {
        info!("Listening on {}", self.socket.local_addr()?);
        let (tx, mut rx) = mpsc::channel(128);
        let task_socket = Arc::clone(&self.socket);
        let task_data = Arc::clone(&self.data);
        let task_client = Arc::clone(&self.client);
        let t = tokio::spawn(async move {
            loop {
                let mut buffer = [0u8; 512];
                let result = Arc::clone(&task_socket).recv_from(&mut buffer).await;
                if let Ok((_, src)) = result {
                    trace!("Request from {src:?}");
                    let packet = Packet {
                        buf: buffer.to_vec(),
                        addr: src,
                    };

                    let shared_tx = tx.clone();
                    let inner_data = Arc::clone(&task_data);
                    let inner_client = Arc::clone(&task_client);
                    tokio::spawn(async move {
                        let new_bytes = Self::handle_bytes(
                            Arc::clone(&inner_data),
                            Arc::clone(&inner_client),
                            &packet.buf,
                        )
                        .await;
                        let packet = Packet {
                            addr: packet.addr,
                            buf: new_bytes,
                        };
                        let res = shared_tx.send(packet).await;
                        if let Err(e) = res {
                            error!("Failed to send response: {e:?}");
                        }
                    });
                }
            }
        });

        while let Some(packet) = rx.recv().await {
            if t.is_finished() {
                error!("Server handler task finished unexpectedly.");
                return Err(ServerError::ServerShutdown);
            }
            // transfer response packet to the client.
            debug!("Response to {addr:?}", addr = packet.addr);
            let _ = Arc::clone(&self.socket)
                .send_to(&packet.buf, &packet.addr)
                .await;
        }
        Ok(())
    }
}
