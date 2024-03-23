use nav_msgs::msg::Path as PathMsg;
use std::io;
use tokio::net::UdpSocket;
use std::sync::{Arc, Mutex};

#[derive(Copy, Clone, Debug)]
pub struct Pose {
    x: f32,
    y: f32,
    z: f32,
    angle_w: f32,
    angle_x: f32,
    angle_y: f32,
    angle_z: f32,
}

impl Pose {
    fn from_bytes(bytes: &[u8]) -> Self {
        Self {
            x: f32::from_le_bytes(bytes[0..4].try_into().unwrap()),
            y: f32::from_le_bytes(bytes[4..8].try_into().unwrap()),
            z: f32::from_le_bytes(bytes[8..12].try_into().unwrap()),
            angle_w: f32::from_le_bytes(bytes[12..16].try_into().unwrap()),
            angle_x: f32::from_le_bytes(bytes[16..20].try_into().unwrap()),
            angle_y: f32::from_le_bytes(bytes[20..24].try_into().unwrap()),
            angle_z: f32::from_le_bytes(bytes[24..28].try_into().unwrap()),
        }
    }

    fn to_bytes(&self) -> [u8; 28] {
        let mut bytes = [0u8; 28];
        bytes[0..4].copy_from_slice(&self.x.to_le_bytes());
        bytes[4..8].copy_from_slice(&self.y.to_le_bytes());
        bytes[8..12].copy_from_slice(&self.z.to_le_bytes());
        bytes[12..16].copy_from_slice(&self.angle_w.to_le_bytes());
        bytes[16..20].copy_from_slice(&self.angle_x.to_le_bytes());
        bytes[20..24].copy_from_slice(&self.angle_y.to_le_bytes());
        bytes[24..28].copy_from_slice(&self.angle_z.to_le_bytes());
        bytes
    }
}

enum Request {
    GetVslamPose,
    SetVslamPose(Pose),
    GetDetections
}

impl Request {
    fn from_bytes(bytes: &[u8]) -> Option<Self> {
        return if bytes[0] == 0 {
            Some(Self::GetVslamPose)
        } else if bytes[0] == 1 {
            Some(Self::SetVslamPose(Pose::from_bytes(&bytes[1..])))
        } else if bytes[0] == 2 {
            Some(Self::GetDetections)
        } else {
            None
        };
    }
}

enum Response {
    Pose(Pose),
    Success,
    Error(String)
}

impl Response {
    fn to_bytes(&self) -> Vec<u8> {
        return match self {
            Self::Success => {
                let bytes = [0u8];
                bytes.to_vec()
            }
            Self::Error(msg) => {
                let mut bytes = [0u8; 512];
                bytes[0] = 1;
                bytes[1..].copy_from_slice(msg.as_bytes());
                bytes.to_vec()
            }
            Self::Pose(pose) => {
                let mut bytes = [0u8; 28];
                bytes[0] = 255;
                bytes[1..29].copy_from_slice(&pose.to_bytes());
                bytes.to_vec()
            }
        }
    }
}

impl From<Pose> for Response {
    fn from(pose: Pose) -> Self {
        Self::Pose(pose)
    }
}

pub struct Server {
    data: Arc<Mutex<Option<PathMsg>>>,
    socket: UdpSocket,
    milli_start: u32, // TODO: Fix
}

fn now_millis_u31() -> u32 {
    let time = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap()
        .as_millis();
    let modded = time % 2_u128.pow(31);
    modded as u32
}

impl Server {
    pub fn new(data: Arc<Mutex<Option<PathMsg>>>) -> Self {
        Self {
            data,
            socket: UdpSocket::bind("127.0.0.1:8080").await.unwrap(),
            milli_start: now_millis_u31(),
        }
    }

    async fn process_request(&mut self, request: Request) -> Response {
        return match request {
            Request::GetVslamPose => {
                if let Some(msg) = self.data.lock() {
                    if let Some(last) = msg.poses.last() {
                        let now = now_millis_u31();
                        let header = (now - self.milli_start) as u32; // TODO: rework into return system
                        let response = Pose {
                            x: last.pose.position.x as f32,
                            y: last.pose.position.y as f32,
                            z: last.pose.position.z as f32,
                            angle_w: last.pose.orientation.w as f32,
                            angle_x: last.pose.orientation.x as f32,
                            angle_y: last.pose.orientation.y as f32,
                            angle_z: last.pose.orientation.z as f32,
                        };
                        response.into()
                    } else {
                        Response::Error("Server Error:No VSLAM data, please wait or check logs".to_string())
                    }
                } else {
                    Response::Error("Server Error: Poisoned Node Mutex".to_string())
                }
            }
            Request::SetVslamPose(pose) => {
                // TODO: Fix
                Response::Success
            }
            Request::GetDetections => {
                // TODO: Fix
                Response::Success
            }
        }
    }

    pub async fn run(&mut self) -> io::Result<()> {
        println!("Listening on {}", self.socket.local_addr()?);
        loop {
            let mut buf = [0u8; 512];
            if let Ok(req) = self.socket.recv_from(buf.as_mut()).await {
                let (size, return_addr) = req;
                println!("Request from {}", return_addr);
                let bytes = &buf[0..size];
                let parsed_request = Request::from_bytes(bytes);
                let response = if let Some(request) = parsed_request {
                    self.process_request(request).await
                } else {
                    Response::Error("Invalid Request (first byte not valid)".to_string())
                };
                let bytes = response.to_bytes();
                self.socket.send_to(&bytes, return_addr).await?;
            }
        }
    }
}
