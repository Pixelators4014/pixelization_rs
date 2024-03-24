use lazy_static::lazy_static;
use std::collections::HashMap;
use crate::pose::{Point, Pose, EulerAngles};

macro_rules! add_april_tag {
    ($tag_id:literal, $x:literal, $y:literal, $z:literal, $angle:literal) => {
        m.insert($tag_id, Pose {
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
        })
    }
}

lazy_static! {
    static ref APRIL_TAG_LOCATIONS: HashMap<u32, Pose> = {
        let mut m = HashMap::new();
        add_april_tag!(1, 593.68, 9.68, 53.38, 120);
        add_april_tag!(2, 637.21, 34.79, 53.38, 120);
        add_april_tag!(3, 652.73, 196.17, 57.13, 180);
        add_april_tag!(4, 652.73, 218.42, 57.13, 180);
        add_april_tag!(5, 578.77, 323.00, 53.38, 270);
        add_april_tag!(6, 72.5, 323.00, 53.38, 270);
        add_april_tag!(7, -1.50, 218.42, 57.13, 0);
        add_april_tag!(8, -1.50, 196.17, 57.13, 0);
        add_april_tag!(9, 14.02, 34.79, 53.38, 60);
        add_april_tag!(10, 57.54, 9.68, 53.38, 60);
        add_april_tag!(11, 468.69, 146.19, 52.00, 300);
        add_april_tag!(12, 468.69, 177.10, 52.00, 60);
        add_april_tag!(13, 441.74, 161.62, 52.00, 180);
        add_april_tag!(14, 441.74, 192.53, 52.00, 180);
        add_april_tag!(15, 414.79, 177.10, 52.00, 180);
        add_april_tag!(16, 414.79, 146.19, 52.00, 180);
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
