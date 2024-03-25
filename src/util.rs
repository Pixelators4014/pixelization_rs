use nalgebra::{Isometry3, Quaternion};

pub fn pose_to_isometry(pose: &geometry_msgs::msg::Pose) -> Isometry3<f32> {
    Isometry3::from_parts(
        nalgebra::Translation3::new(pose.position.x, pose.position.y, pose.position.z),
        nalgebra::base::Unit::new_normalize(Quaternion::new(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z))
    )
}