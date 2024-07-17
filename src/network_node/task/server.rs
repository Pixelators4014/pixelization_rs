use std::sync::Arc;

use async_trait::async_trait;
use isaac_ros_visual_slam_interfaces::srv::SetSlamPose;
use nav_msgs::msg::Path as PathMsg;
use rclrs::Client;
use tokio::sync::RwLock;

use crate::node::TaskContext;
use crate::task::Task;

pub struct Server {
    path: Arc<RwLock<Option<PathMsg>>>,
    set_pose: Arc<Client<SetSlamPose>>,
}

impl Server {
    pub async fn new(context: TaskContext) -> Self {
        Self {
            path: context.path,
            set_pose: context.set_pose,
        }
    }
}

#[async_trait]
impl Task for Server {
    async fn run(&self) {
        let server =
            crate::udp_server::Server::new(Arc::clone(&self.path), Arc::clone(&self.set_pose)).await;
        server.run().await.unwrap();
        // TODO: Fix
        // self.shutdown_trigger.send(()).unwrap();
    }
}
