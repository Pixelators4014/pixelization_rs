#[derive(Copy, Clone, Debug)]
struct EulerAngles {
    roll: f32,
    pitch: f32,
    yaw: f32
}

fn to_euler(w: f32, x: f32, y: f32, z: f32) -> EulerAngles {
    let sinr_cosp = 2.0 * (w * x + y * z);
    let cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    let roll = sinr_cosp.atan2(cosr_cosp);

    let sinp = (1.0 + 2.0 * (w * y - x * z)).sqrt();
    let cosp = (1.0 - 2.0 * (w * y - x * z)).sqrt();
    let pitch = 2.0 * sinp.atan2(cosp) - std::f32::consts::PI / 2.0;

    let siny_cosp = 2.0 * (w * z + x * y);
    let cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    let yaw = siny_cosp.atan2(cosy_cosp);

    EulerAngles { roll, pitch, yaw }
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