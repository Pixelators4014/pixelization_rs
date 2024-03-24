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
