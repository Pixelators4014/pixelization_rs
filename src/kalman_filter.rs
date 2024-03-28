use nalgebra::{Rotation3, UnitQuaternion, Quaternion, Translation3, Isometry3};

/// Kalman filter implementation for a single axis
pub struct AxisKalmanFilter {
    value: f32,
    covariance1: f32,
    covariance2: f32,
    covariance: f32,
    kalman_gain: f32,
    previous_value: f32,
    previous_covariance: f32,
}

impl AxisKalmanFilter {
    pub fn new(
        value: f32,
        kalman_gain: f32,
        covariance1: f32,
        covariance2: f32,
        resulting_covariance: f32,
    ) -> Self {
        Self {
            value,
            covariance1,
            covariance2,
            covariance: resulting_covariance,
            kalman_gain,
            previous_value: value,
            previous_covariance: resulting_covariance,
        }
    }

    pub fn update(&mut self, value1: f32, value2: f32) {
        self.value += value1;
        self.covariance += self.covariance1;
        self.kalman_gain = self.covariance / (self.covariance + self.covariance2);
        self.value += self.kalman_gain * (value2 - self.value);
        self.covariance = (1.0 - self.kalman_gain) * self.covariance;

        self.previous_value = self.value;
        self.previous_covariance = self.covariance;
    }

    pub fn value(&self) -> f32 {
        self.value
    }
}

/// Kalman filter implementation for a Translation
pub struct TranslationKalmanFilter {
    x: AxisKalmanFilter,
    y: AxisKalmanFilter,
    z: AxisKalmanFilter,
}

impl TranslationKalmanFilter {
    pub fn new(translation: Translation3<f32>, kalman_gain: f32, covariance1: f32, covariance2: f32, resulting_covariance: f32) -> Self {
        Self {
            x: AxisKalmanFilter::new(translation.vector.data.x, kalman_gain, covariance1, covariance2, resulting_covariance),
            y: AxisKalmanFilter::new(translation.vector.data.y, kalman_gain, covariance1, covariance2, resulting_covariance),
            z: AxisKalmanFilter::new(translation.vector.data.z, kalman_gain, covariance1, covariance2, resulting_covariance),
        }
    }

    pub fn update(&mut self, translation1: Translation3<f32>, translation2: Translation3<f32>) {
        self.x.update(translation1.vector.data.x, translation2.vector.data.x);
        self.y.update(translation1.vector.data.y, translation2.vector.data.y);
        self.z.update(translation1.vector.data.z, translation2.vector.data.z);
    }

    pub fn value(&self) -> Translation3<f32> {
        Translation3::new(self.x.value(), self.y.value(), self.z.value())
    }
}

struct QuaternionKalmanFilter {
    w: AxisKalmanFilter,
    x: AxisKalmanFilter,
    y: AxisKalmanFilter,
    z: AxisKalmanFilter,
}

impl QuaternionKalmanFilter {
    pub fn new(unit_quaternion: UnitQuaternion<f32>, kalman_gain: f32, covariance1: f32, covariance2: f32, resulting_covariance: f32) -> Self {
        Self {
            w: AxisKalmanFilter::new(unit_quaternion.quaternion()[3], kalman_gain, covariance1, covariance2, resulting_covariance),
            x: AxisKalmanFilter::new(unit_quaternion.quaternion()[0], kalman_gain, covariance1, covariance2, resulting_covariance),
            y: AxisKalmanFilter::new(unit_quaternion.quaternion()[1], kalman_gain, covariance1, covariance2, resulting_covariance),
            z: AxisKalmanFilter::new(unit_quaternion.quaternion()[2], kalman_gain, covariance1, covariance2, resulting_covariance),
        }
    }

    pub fn update(&mut self, unit_quaternion1: UnitQuaternion<f32>, unit_quaternion2: UnitQuaternion<f32>) {
        self.w.update(unit_quaternion1.quaternion()[3], unit_quaternion2.quaternion()[3]);
        self.x.update(unit_quaternion1.quaternion()[0], unit_quaternion2.quaternion()[0]);
        self.y.update(unit_quaternion1.quaternion()[1], unit_quaternion2.quaternion()[1]);
        self.z.update(unit_quaternion1.quaternion()[2], unit_quaternion2.quaternion()[2]);
    }

    pub fn value(&self) -> UnitQuaternion<f32> {
        UnitQuaternion::new_normalize(Quaternion::new(
            self.w.value(),
            self.x.value(),
            self.y.value(),
            self.z.value(),
        ))
    }
}

struct PoseKalmanFilter {
    translation: TranslationKalmanFilter,
    rotation: QuaternionKalmanFilter
}

impl PoseKalmanFilter {
    pub fn new(isometry: &Isometry3<f32>, kalman_gain: f32, covariance1: f32, covariance2: f32, resulting_covariance: f32) -> Self {
        Self {
            translation: TranslationKalmanFilter::new(isometry.translation, kalman_gain, covariance1, covariance2, resulting_covariance),
            rotation: QuaternionKalmanFilter::new(isometry.rotation, kalman_gain, covariance1, covariance2, resulting_covariance),
        }
    }

    pub fn update(&mut self, isometry1: &Isometry3<f32>, isometry2: &Isometry3<f32>) {
        self.translation.update(isometry1.translation, isometry2.translation);
        self.rotation.update(isometry1.rotation, isometry2.rotation);
    }

    pub fn value(&self) -> Isometry3<f32> {
        Isometry3::from_parts(self.translation.value(), self.rotation.value())
    }
}

