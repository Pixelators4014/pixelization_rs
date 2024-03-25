use std::ops::{Add, Neg};

use nalgebra::Vector3;

pub type Point = Vector3<f32>;

#[derive(Copy, Clone, Debug)]
pub struct EulerAngles {
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32
}

impl EulerAngles {
    pub fn new(roll: f32, pitch: f32, yaw: f32) -> Self {
        Self { roll, pitch, yaw }
    }

    pub fn crop(&self) -> Self {
        Self::new(self.roll % (2.0 * std::f32::consts::PI), self.pitch % (2.0 * std::f32::consts::PI), self.yaw % (2.0 * std::f32::consts::PI))
    }
}

impl From<Quaternion> for EulerAngles {
    fn from(q: Quaternion) -> Self {
        let sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z);
        let cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
        let roll = sinr_cosp.atan2(cosr_cosp);

        let sinp = (1.0 + 2.0 * (q.w * q.y - q.x * q.z)).sqrt();
        let cosp = (1.0 - 2.0 * (q.w * q.y - q.x * q.z)).sqrt();
        let pitch = 2.0 * sinp.atan2(cosp) - std::f32::consts::PI / 2.0;

        let siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        let cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        let yaw = siny_cosp.atan2(cosy_cosp);

        EulerAngles { roll, pitch, yaw }
    }
}

impl Add for EulerAngles {
    type Output = EulerAngles;

    fn add(self, rhs: Self) -> Self::Output {
        EulerAngles { roll: self.roll + rhs.roll, pitch: self.pitch + rhs.pitch, yaw: self.yaw + rhs.yaw }.crop()
    }
}

impl Neg for EulerAngles {
    type Output = EulerAngles;

    fn neg(self) -> Self::Output {
        EulerAngles { roll: self.roll + std::f32::consts::PI, pitch: self.pitch + std::f32::consts::PI, yaw: self.yaw + std::f32::consts::PI }.crop()
    }
}

#[derive(Copy, Clone, Debug)]
pub struct Quaternion {
    pub w: f32,
    pub x: f32,
    pub y: f32,
    pub z: f32
}

impl From<EulerAngles> for Quaternion {
    fn from(e: EulerAngles) -> Self {
        let (w, x, y, z) = to_quaternion(e);
        Quaternion { w, x, y, z }
    }
}

fn to_quaternion(euler_angles: EulerAngles) -> (f32, f32, f32, f32) {
    let cr = (euler_angles.roll * 0.5).cos();
    let sr = (euler_angles.roll * 0.5).sin();
    let cp = (euler_angles.pitch * 0.5).cos();
    let sp = (euler_angles.pitch * 0.5).sin();
    let cy = (euler_angles.yaw * 0.5).cos();
    let sy = (euler_angles.yaw * 0.5).sin();

    let w = cr * cp * cy + sr * sp * sy;
    let x = sr * cp * cy - cr * sp * sy;
    let y = cr * sp * cy + sr * cp * sy;
    let z = cr * cp * sy - sr * sp * cy;

    (w, x, y, z)
}

#[derive(Copy, Clone, Debug)]
pub struct Pose {
    pub position: Point,
    pub orientation: Quaternion
}

impl Pose {
    pub fn from_bytes(bytes: &[u8]) -> Self {
        Self {
            position: Point::new(
                f32::from_le_bytes(bytes[0..4].try_into().unwrap()),
                f32::from_le_bytes(bytes[4..8].try_into().unwrap()),
                f32::from_le_bytes(bytes[8..12].try_into().unwrap())),
            orientation: Quaternion {
                w: f32::from_le_bytes(bytes[12..16].try_into().unwrap()),
                x: f32::from_le_bytes(bytes[16..20].try_into().unwrap()),
                y: f32::from_le_bytes(bytes[20..24].try_into().unwrap()),
                z: f32::from_le_bytes(bytes[24..28].try_into().unwrap()),
            }
        }
    }

    pub fn to_bytes(&self) -> [u8; 28] {
        let mut bytes = [0u8; 28];
        bytes[0..4].copy_from_slice(&self.position.x.to_le_bytes());
        bytes[4..8].copy_from_slice(&self.position.y.to_le_bytes());
        bytes[8..12].copy_from_slice(&self.position.z.to_le_bytes());
        bytes[12..16].copy_from_slice(&self.orientation.w.to_le_bytes());
        bytes[16..20].copy_from_slice(&self.orientation.x.to_le_bytes());
        bytes[20..24].copy_from_slice(&self.orientation.y.to_le_bytes());
        bytes[24..28].copy_from_slice(&self.orientation.z.to_le_bytes());
        bytes
    }
}

impl From<geometry_msgs::msg::Pose> for Pose {
    fn from(pose: geometry_msgs::msg::Pose) -> Self {
        Self {
            position: Point::new(
                pose.position.x as f32,
                pose.position.y as f32,
                pose.position.z as f32
            ),
            orientation: Quaternion {
                w: pose.orientation.w as f32,
                x: pose.orientation.x as f32,
                y: pose.orientation.y as f32,
                z: pose.orientation.z as f32
            }
        }
    }
}

impl From<&geometry_msgs::msg::Pose> for Pose {
    fn from(pose: &geometry_msgs::msg::Pose) -> Self {
        Self {
            position: Point::new(
                pose.position.x as f32,
                pose.position.y as f32,
                pose.position.z as f32
            ),
            orientation: Quaternion {
                w: pose.orientation.w as f32,
                x: pose.orientation.x as f32,
                y: pose.orientation.y as f32,
                z: pose.orientation.z as f32
            }
        }
    }
}

