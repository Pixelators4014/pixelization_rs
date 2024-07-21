use crate::node::{Parameters, TaskContext};
use crate::task::Task;
use async_trait::async_trait;
use log::{debug, warn};
use nav_msgs::msg::Path as PathMsg;
use std::sync::Arc;
use tokio::sync::RwLock;

pub struct Ping {
    path: Arc<RwLock<Option<PathMsg>>>,
    _parameters: Parameters
}

impl Ping {
    pub fn new(context: TaskContext) -> Self {
        Self {
            path: context.path,
            _parameters: context.parameters
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
                    warn!("VSLAM has not initialized yet");
                }
            } else {
                warn!("VSLAM not connected yet");
            }
            drop(data);
            tokio::time::sleep(std::time::Duration::from_millis(1000)).await;
        }
    }
}
