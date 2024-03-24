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
}
