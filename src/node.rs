use std::sync::Arc;
use tokio::sync::RwLock;
use nav_msgs::msg::Path as PathMsg;
use isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray;

use log::info;

pub struct NetworkNode {
    pub node: Arc<rclrs::Node>,
    #[allow(dead_code)]
    pub path_subscription: Arc<rclrs::Subscription<PathMsg>>,
    #[allow(dead_code)]
    pub april_tags_subscription: Arc<rclrs::Subscription<AprilTagDetectionArray>>,
    pub client: Arc<rclrs::Client<isaac_ros_visual_slam_interfaces::srv::SetOdometryPose>>,
    pub path: Arc<RwLock<Option<PathMsg>>>,
    pub april_tags: Arc<RwLock<Option<AprilTagDetectionArray>>>,
}

impl NetworkNode {
    pub fn new(context: &rclrs::Context) -> Result<Self, rclrs::RclrsError> {
        let node = rclrs::Node::new(context, "network_node")?;
        let path = Arc::new(RwLock::new(None));
        let path_cb = Arc::clone(&path);
        let path_subscription =
            // Create a new shared pointer instance that will be owned by the closure
            node.create_subscription(
                "/visual_slam/tracking/slam_path",
                rclrs::QOS_PROFILE_DEFAULT,
                move |msg: PathMsg| {
                    // This subscription now owns the data_cb variable
                    *path_cb.blocking_write() = Some(msg);
                },
            )?;
        let april_tags = Arc::new(RwLock::new(None));
        let april_tags_cb = Arc::clone(&april_tags);
        let april_tags_subscription = node.create_subscription(
            "/tag_detections",
            rclrs::QOS_PROFILE_DEFAULT,
            move |msg: AprilTagDetectionArray| {
                // This subscription now owns the data_cb variable
                *april_tags_cb.blocking_write() = Some(msg);
            },
        )?;

        let client = node.create_client::<isaac_ros_visual_slam_interfaces::srv::SetOdometryPose>("visual_slam/set_odometry_pose")?;
        Ok(Self {
            node,
            path_subscription,
            april_tags_subscription,
            client,
            path,
            april_tags
        })
    }

    pub async fn init(&self) -> Result<(), rclrs::RclrsError> {
        while !self.client.service_is_ready()? {
            tokio::time::sleep(std::time::Duration::from_millis(10)).await;
            info!("Waiting for service to initialize ...");
        }
        Ok(())
    }

    pub async fn run_server(self: Arc<Self>) {
        let server = crate::udp_server::Server::new(Arc::clone(self.path), Arc::clone(self.client)).await;
        server.run().await.unwrap();
    }
}
