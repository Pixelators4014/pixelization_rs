use lazy_static::lazy_static;
use std::collections::HashMap;
use crate::pose::{Point, Pose, EulerAngles};

lazy_static! {
    static ref APRIL_TAG_LOCATIONS: HashMap<u32, Pose> = {
        let mut m = HashMap::new();
        m.insert(1, Pose {
            position: Point {
                x: 593.68 * 39.37,
                y: 9.68 * 39.37,
                z: 53.38 * 39.37,
            },
            orientation: EulerAngles {
                roll: 0.0,
                pitch: 0.0,
                yaw: 120.0_f32.to_radians(),
            }.into()
        });
        m
    };
}

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