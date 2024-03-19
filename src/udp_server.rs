use std::io;
use nav_msgs::msg::Path as PathMsg;
use std::net::UdpSocket;
use std::sync::{Arc, Mutex};

struct Response {
    header: u32,
    x: f32,
    y: f32,
    z: f32,
    angle_x: f32,
    angle_y: f32,
    angle_z: f32,
    angle_w: f32,
}

impl Response {
    fn to_bytes(&self) -> [u8; 32] {
        let mut bytes = [0u8; 32];
        bytes[0..4].copy_from_slice(&self.header.to_le_bytes());
        bytes[4..8].copy_from_slice(&self.x.to_le_bytes());
        bytes[8..12].copy_from_slice(&self.y.to_le_bytes());
        bytes[12..16].copy_from_slice(&self.z.to_le_bytes());
        bytes[16..20].copy_from_slice(&self.angle_x.to_le_bytes());
        bytes[20..24].copy_from_slice(&self.angle_y.to_le_bytes());
        bytes[24..28].copy_from_slice(&self.angle_z.to_le_bytes());
        bytes[28..32].copy_from_slice(&self.angle_w.to_le_bytes());
        bytes
    }
}

pub(crate) struct Server {
    data: Arc<Mutex<Option<PathMsg>>>,
    socket: UdpSocket,
    milli_start: u64, // TODO: Fix
}

impl Server {
    pub(crate) fn new(data: Arc<Mutex<Option<PathMsg>>>) -> Self {
        Self {
            data,
            socket: UdpSocket::bind("127.0.0.1:8080").unwrap(),
            milli_start: 0,
        }
    }

    pub(crate) fn run(&mut self) -> io::Result<()> {
        loop {
            let mut buf = [0u8; 32];
            let (_size, return_addr) = self.socket.recv_from(buf.as_mut())?;
            if let Some(msg) = self.data.lock().unwrap().as_ref() {
                if let Some(last) = msg.poses.last() {
                    let response = Response {
                        header: 0,
                        x: last.pose.position.x,
                        y: last.pose.position.y,
                        z: last.pose.position.z,
                        angle_x: last.pose.orientation.x,
                        angle_y: last.pose.orientation.y,
                        angle_z: last.pose.orientation.z,
                        angle_w: last.pose.orientation.w,
                    };
                    self.socket
                        .send_to(&response.to_bytes(), return_addr)?;
                } else {
                    self.socket
                        .send_to(&[1u8; 32], return_addr)?;
                }
            } else {
                self.socket
                    .send_to(&[2u8; 32], return_addr)?;
            }
        }
    }
}
