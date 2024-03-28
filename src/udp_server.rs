use nav_msgs::msg::Path as PathMsg;
use std::io;
use std::sync::Arc;
use tokio::net::UdpSocket;
use tokio::sync::mpsc;
use tokio::sync::RwLock;

use nalgebra::{Quaternion, Rotation3, UnitQuaternion};

use log::{debug, info, warn};

struct Pose {
    x: f32,
    y: f32,
    z: f32,
    roll: f32,
    pitch: f32,
    yaw: f32,
}

impl Pose {
    fn from_bytes(bytes: &[u8]) -> Result<Self, std::array::TryFromSliceError> {
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

enum Request {
    GetVslamPose,
    SetVslamPose(Pose),
    GetDetections,
}

impl Request {
    fn from_bytes(bytes: &[u8]) -> Result<Self, String> {
        return if bytes[0] == 0 {
            Ok(Self::GetVslamPose)
        } else if bytes[0] == 1 {
            Ok(Self::SetVslamPose(Pose::from_bytes(&bytes[1..]).map_err(|e| e.to_string())?))
        } else if bytes[0] == 2 {
            Ok(Self::GetDetections)
        } else {
            Err(format!("Unknown request first byte: {}", bytes[0]))
        };
    }
}

enum Response {
    Pose(Pose),
    Success,
    Error(String),
}

impl Response {
    fn to_bytes(&self) -> Vec<u8> {
        return match self {
            Self::Success => {
                let bytes = [0u8];
                bytes.to_vec()
            }
            Self::Error(msg) => {
                let mut bytes = [0u8; 1024];
                bytes[0] = 1;
                bytes[1..msg.len() + 1].copy_from_slice(msg.as_bytes());
                bytes.to_vec()
            }
            Self::Pose(pose) => {
                let mut bytes = [0u8; 32];
                bytes[0] = 255;
                bytes[1..25].copy_from_slice(&pose.to_bytes());
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
    addr: std::net::SocketAddr,
}

pub struct Server {
    data: Arc<RwLock<Option<PathMsg>>>,
    client: Arc<rclrs::Client<isaac_ros_visual_slam_interfaces::srv::SetOdometryPose>>,
    socket: Arc<UdpSocket>,
}

impl Server {
    pub async fn new(
        data: Arc<RwLock<Option<PathMsg>>>,
        client: Arc<rclrs::Client<isaac_ros_visual_slam_interfaces::srv::SetOdometryPose>>,
    ) -> Self {
        Self {
            data,
            client,
            socket: Arc::new(UdpSocket::bind("127.0.0.1:8080").await.unwrap()),
        }
    }

    async fn process_request(
        data: Arc<RwLock<Option<PathMsg>>>,
        client: Arc<rclrs::Client<isaac_ros_visual_slam_interfaces::srv::SetOdometryPose>>,
        request: Request,
    ) -> Response {
        return match request {
            Request::GetVslamPose => {
                if let Some(msg) = data.read().await.as_ref() {
                    if let Some(last) = msg.poses.last() {
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
                        Response::Pose(response.into())
                    } else {
                        Response::Error(
                            "Server Error: No VSLAM data, please wait or check logs".to_string(),
                        )
                    }
                } else {
                    Response::Error("Server Error: No VSLAM data".to_string())
                }
            }
            Request::SetVslamPose(pose) => {
                let unit_quaternion =
                    nalgebra::UnitQuaternion::from_euler_angles(pose.roll, pose.pitch, pose.yaw);
                let quaternion = unit_quaternion.quaternion();
                let service_request =
                    isaac_ros_visual_slam_interfaces::srv::SetOdometryPose_Request {
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
                            // TODO: make sure this works
                            Response::Success
                        } else {
                            warn!("Failed to set VSLAM pose due to service error: {service_request:?}");
                            Response::Error("Server Error: Failed to set VSLAM pose (service error)".to_string())
                        }
                    },
                    Err(e) => {
                        warn!("Failed to set VSLAM pose due to rcl error: {service_request:?}");
                        Response::Error("Server Error: Failed to set VSLAM pose (rcl error)".to_string())
                    }
                }
            }
            Request::GetDetections => {
                // TODO: Fix
                Response::Success
            }
        };
    }

    pub async fn handle_bytes(
        data: Arc<RwLock<Option<PathMsg>>>,
        client: Arc<rclrs::Client<isaac_ros_visual_slam_interfaces::srv::SetOdometryPose>>,
        bytes: &Vec<u8>,
    ) -> Vec<u8> {
        let request = Request::from_bytes(bytes);
        match request {
            Ok(request) => Self::process_request(data, client, request).await.to_bytes(),
            Err(e) => {
                warn!("Invalid Request: {e}");
                Response::Error(format!("Invalid Request: {e}".to_string())).to_bytes()
            },
        }
    }

    pub async fn run(&self) -> io::Result<()> {
        info!("Listening on {}", self.socket.local_addr()?);
        let (tx, mut rx) = mpsc::channel(128);
        let task_socket = Arc::clone(&self.socket);
        let task_data = Arc::clone(&self.data);
        let task_client = Arc::clone(&self.client);
        tokio::spawn(async move {
            loop {
                let mut buffer = [0u8; 512];
                let result = Arc::clone(&task_socket).recv_from(&mut buffer).await;
                if let Ok((_, src)) = result {
                    info!("Request from {src:?}");
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
                        shared_tx.send(packet).await.unwrap(); // Send them to queue back to clients
                    });
                }
            }
        });

        while let Some(packet) = rx.recv().await {
            // transfer response packet to the client.
            debug!("Response to {addr:?}", addr = packet.addr);
            let _ = Arc::clone(&self.socket)
                .send_to(&packet.buf, &packet.addr)
                .await;
        }
        Ok(())
    }
}
