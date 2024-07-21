use std::net::IpAddr;
use std::sync::Arc;

use async_trait::async_trait;
use isaac_ros_visual_slam_interfaces::srv::SetSlamPose;
use nav_msgs::msg::Path as PathMsg;
use rclrs::Client;
use tokio::sync::RwLock;

use crate::node::TaskContext;
use crate::task::Task;
use crate::udp_server::Server as UdpServer;

pub struct Server {
    udp_server: UdpServer
}

impl Server {
    pub async fn new(context: TaskContext) -> Result<Self, std::io::Error> {
        let udp_server =
            UdpServer::new(Arc::clone(&context.path), Arc::clone(&context.set_pose), context.parameters.server_port, context.parameters.server_ip).await?;
        Ok(Self {
            udp_server
        })
    }
}

#[async_trait]
impl Task for Server {
    async fn run(&self) {
        self.udp_server.run().await.unwrap();
        // TODO: Fix
        // self.shutdown_trigger.send(()).unwrap();
    }
}
