use std::collections::HashMap;

use lazy_static::lazy_static;

use isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray;

use crate::pose::{Point, Pose, EulerAngles};

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

lazy_static! {
    static ref APRIL_TAG_LOCATIONS: HashMap<u32, Pose> = {
        let mut m = HashMap::new();
        m.insert(1, add_april_tag!(593.68, 9.68, 53.38, 120));
        m.insert(2, add_april_tag!(637.21, 34.79, 53.38, 120));
        m.insert(3, add_april_tag!(652.73, 196.17, 57.13, 180));
        m.insert(4, add_april_tag!(652.73, 218.42, 57.13, 180));
        m.insert(5, add_april_tag!(578.77, 323.00, 53.38, 270));
        m.insert(6, add_april_tag!(72.5, 323.00, 53.38, 270));
        m.insert(7, add_april_tag!(-1.50, 218.42, 57.13, 0));
        m.insert(8, add_april_tag!(-1.50, 196.17, 57.13, 0));
        m.insert(9, add_april_tag!(14.02, 34.79, 53.38, 60));
        m.insert(10, add_april_tag!(57.54, 9.68, 53.38, 60));
        m.insert(11, add_april_tag!(468.69, 146.19, 52.00, 300));
        m.insert(12, add_april_tag!(468.69, 177.10, 52.00, 60));
        m.insert(13, add_april_tag!(441.74, 161.62, 52.00, 180));
        m.insert(14, add_april_tag!(441.74, 192.53, 52.00, 180));
        m.insert(15, add_april_tag!(414.79, 177.10, 52.00, 180));
        m.insert(16, add_april_tag!(414.79, 146.19, 52.00, 180));
        m
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

pub fn localize(detections: AprilTagDetectionArray) -> Option<Pose> {
    None
}
