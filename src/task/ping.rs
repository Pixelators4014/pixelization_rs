use crate::node::TaskContext;
use crate::task::Task;
use async_trait::async_trait;
use isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray;
use log::{debug, warn};
use nav_msgs::msg::Path as PathMsg;
use std::sync::Arc;
use tokio::sync::RwLock;

pub struct Ping {
    path: Arc<RwLock<Option<PathMsg>>>,
    april_tags: Arc<RwLock<Option<AprilTagDetectionArray>>>,
}

impl Ping {
    pub async fn new(context: TaskContext) -> Self {
        Self {
            path: context.path,
            april_tags: context.april_tags,
        }
    }
}

#[async_trait]
impl Task for Ping {
    async fn run(&self) {
        loop {
            let data = self.path.read().await;
            if let Some(path_option) = data.as_ref() {
                if let Some(path) = path_option.poses.last() {
                    debug!("VSLAM is running: {path:?}");
                } else {
                    warn!("VSLAM has not initialized yet")
                }
            } else {
                warn!("VSLAM not connected yet");
            }
            drop(data);
            let data = self.april_tags.read().await;
            if let Some(april_tags) = data.as_ref() {
                debug!(
                    "April Tags is running: {} april tags",
                    april_tags.detections.len()
                );
            } else {
                warn!("April Tags not connected yet");
            }
            tokio::time::sleep(std::time::Duration::from_millis(1000)).await;
        }
    }
}
