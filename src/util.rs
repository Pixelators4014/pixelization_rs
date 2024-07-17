use nalgebra::{Isometry3, Quaternion};

// TODO: Replace with From<>
pub fn pose_to_isometry(pose: &geometry_msgs::msg::Pose) -> Isometry3<f32> {
    Isometry3::from_parts(
        nalgebra::Translation3::new(
            pose.position.x as f32,
            pose.position.y as f32,
            pose.position.z as f32,
        ),
        nalgebra::base::Unit::new_normalize(Quaternion::new(
            pose.orientation.w as f32,
            pose.orientation.x as f32,
            pose.orientation.y as f32,
            pose.orientation.z as f32,
        )),
    )
}
