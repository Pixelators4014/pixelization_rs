use lazy_static::lazy_static;

use isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray;

use crate::pose::{Point, Pose, EulerAngles, Quaternion};

macro_rules! add_april_tag {
    ($x:literal, $y:literal, $z:literal, $angle:literal) => {
        Pose {
            position: Point {
                x: $x * 39.37,
                y: $y * 39.37,
                z: $z * 39.37,
            },
            orientation: EulerAngles {
                roll: 0.0,
                pitch: 0.0,
                yaw: ($angle as f32).to_radians(),
            }.into()
        }
    }
}

// TODO: use vec
lazy_static! {
    static ref APRIL_TAG_LOCATIONS: Vec<Option<Pose>> = {
        let mut v = Vec::new();
        v.push(None)
        v.push(Some(dd_april_tag!(593.68, 9.68, 53.38, 120)));
        v.push(Some(dd_april_tag!(637.21, 34.79, 53.38, 120)));
        v.push(Some(dd_april_tag!(652.73, 196.17, 57.13, 180)));
        v.push(Some(dd_april_tag!(652.73, 218.42, 57.13, 180)));
        v.push(Some(dd_april_tag!(578.77, 323.00, 53.38, 270)));
        v.push(Some(dd_april_tag!(72.5, 323.00, 53.38, 270)));
        v.push(Some(dd_april_tag!(-1.50, 218.42, 57.13, 0)));
        v.push(Some(dd_april_tag!(-1.50, 196.17, 57.13, 0)));
        v.push(Some(dd_april_tag!(14.02, 34.79, 53.38, 60)));
        v.push(Some(add_april_tag!(57.54, 9.68, 53.38, 60)));
        v.push(Some(add_april_tag!(468.69, 146.19, 52.00, 300)));
        v.push(Some(add_april_tag!(468.69, 177.10, 52.00, 60)));
        v.push(Some(add_april_tag!(441.74, 161.62, 52.00, 180)));
        v.push(Some(add_april_tag!(441.74, 192.53, 52.00, 180)));
        v.push(Some(add_april_tag!(414.79, 177.10, 52.00, 180)));
        v.push(Some(add_april_tag!(414.79, 146.19, 52.00, 180)));
        v
    };
}

// Extract April Tag Locations
// (id, x in, y in, z in, yaw in degrees)
// 1 593.68 9.68 53.38 120°
// 2 637.21 34.79 53.38 120°
// 3 652.73 196.17 57.13 180°
// 4 652.73 218.42 57.13 180°
// 5 578.77 323.00 53.38 270°
// 6 72.5 323.00 53.38 270°
// 7 -1.50 218.42 57.13 0°
// 8 -1.50 196.17 57.13 0°
// 9 14.02 34.79 53.38 60°
// 10 57.54 9.68 53.38 60°
// 11 468.69 146.19 52.00 300°
// 12 468.69 177.10 52.00 60°
// 13 441.74 161.62 52.00 180°
// 14 209.48 161.62 52.00 0°
// 15 182.73 177.10 52.00 120°
// 16 182.73 146.19 52.00 240°

pub fn localize(detections: &AprilTagDetectionArray) -> Option<Pose> {
    let mut absolute_position = Vec::new();
    for detection in detections.detections.iter() {
        if let Some(position_option) = APRIL_TAG_LOCATIONS.get(&detection.id as usize) {
            if let Some(position) = position_option {
                let april_tag_pose = position;
                let relative_robot_pose = Pose::from(detection.pose.pose.pose);
                let translation = Point {
                    x: relative_robot_pose.position,
                    y: relative_robot_pose.position,
                    z: relative_robot_pose.position
                };
            }
        }
    }
    None
}
