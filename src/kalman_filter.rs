use nalgebra::{Rotation3, UnitQuaternion, Quaternion, Translation3};

/// Kalman filter implementation for a single axis
pub struct AxisKalmanFilter {
    value: f32,
    covariance1: f32,
    covariance2: f32,
    covariance: f32,
    kalman_gain: f32,
    previous_value: f32,
    previous_covariance: f32,
    u: f32,
    z: f32,
}

impl AxisKalmanFilter {
    pub fn new(value: f32, kalman_gain: f32, covariance1: f32, covariance2: f32, resulting_covariance: f32) -> Self {
        Self {
            value,
            covariance1,
            covariance2,
            covariance: resulting_covariance,
            kalman_gain,
            previous_value: value,
            previous_covariance: resulting_covariance,
            u: 0.0,
            z: 0.0,
        }
    }

    pub fn update(&mut self, value1: f32, value2: f32) {
        self.value += value1;
        self.covariance += self.covariance1;
        self.kalman_gain = self.covariance / (self.covariance + self.covariance2);
        self.value += self.kalman_gain * (self.z - self.value);
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
    pub fn new(w: f32, x: f32, y: f32, z: f32, kalman_gain: f32, covariance1: f32, covariance2: f32, resulting_covariance: f32) -> Self {
        Self {
            w: AxisKalmanFilter::new(w, kalman_gain, covariance1, covariance2, resulting_covariance),
            x: AxisKalmanFilter::new(x, kalman_gain, covariance1, covariance2, resulting_covariance),
            y: AxisKalmanFilter::new(y, kalman_gain, covariance1, covariance2, resulting_covariance),
            z: AxisKalmanFilter::new(z, kalman_gain, covariance1, covariance2, resulting_covariance),
        }
    }

    pub fn update(&mut self, w1: f32, x1: f32, y1: f32, z1: f32, w2: f32, x2: f32, y2: f32, z2: f32) {
        self.w.update(w1, w2);
        self.x.update(x1, x2);
        self.y.update(y1, y2);
        self.z.update(z1, z2);
    }

    pub fn w(&self) -> f32 {
        self.w.value()
    }

    pub fn x(&self) -> f32 {
        self.x.value()
    }

    pub fn y(&self) -> f32 {
        self.y.value()
    }

    pub fn z(&self) -> f32 {
        self.z.value()
    }
}

struct PoseKalmanFilter {
    translation: TranslationKalmanFilter,
    rotation: QuaternionKalmanFilter
}
