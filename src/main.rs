use std::sync::{Arc, Mutex};
use nav_msgs::msg::Path as PathMsg;
use isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray;

mod april_tags;
mod kalman_filter;
pub mod pose;
mod udp_server;

struct NetworkNode {
    node: Arc<rclrs::Node>,
    #[allow(dead_code)]
    path_subscription: Arc<rclrs::Subscription<PathMsg>>,
    #[allow(dead_code)]
    april_tags_subscription: Arc<rclrs::Subscription<AprilTagDetectionArray>>,
    client: Arc<rclrs::Client<isaac_ros_visual_slam_interfaces::srv::SetOdometryPose>>,
    path: Arc<Mutex<Option<PathMsg>>>,
    april_tags: Arc<Mutex<Option<AprilTagDetectionArray>>>,
}

impl NetworkNode {
    fn new(context: &rclrs::Context) -> Result<Self, rclrs::RclrsError> {
        let node = rclrs::Node::new(context, "network_node")?;
        let path = Arc::new(Mutex::new(None));
        let path_cb = Arc::clone(&path);
        let path_subscription =
            // Create a new shared pointer instance that will be owned by the closure
            node.create_subscription(
                "/visual_slam/tracking/slam_path",
                rclrs::QOS_PROFILE_DEFAULT,
                move |msg: PathMsg| {
                    // This subscription now owns the data_cb variable
                    *path_cb.lock().unwrap() = Some(msg);
                },
            )?;
        let april_tags = Arc::new(Mutex::new(None));
        let april_tags_cb = Arc::clone(&april_tags);
        let april_tags_subscription = node.create_subscription(
            "tag_detections",
            rclrs::QOS_PROFILE_DEFAULT,
            move |msg: AprilTagDetectionArray| {
                // This subscription now owns the data_cb variable
                *april_tags_cb.lock().unwrap() = Some(msg);
            },
        )?;

        let client = node.create_client::<isaac_ros_visual_slam_interfaces::srv::SetOdometryPose>("visual_slam/set_odometry_pose")?;
        while !client.service_is_ready()? {
            std::thread::sleep(std::time::Duration::from_millis(10));
            println!("Waiting for service to initialize ...");
        }
        Ok(Self {
            node,
            path_subscription,
            april_tags_subscription,
            client,
            path,
            april_tags
        })
    }
}

#[tokio::main]
async fn main() -> Result<(), rclrs::RclrsError> {
    let context = rclrs::Context::new(std::env::args())?;
    let network_node = Arc::new(NetworkNode::new(&context)?);
    let server_path = Arc::clone(&network_node.path);
    let server_client = Arc::clone(&network_node.client);
    let ping_data = Arc::clone(&network_node.path);
    let localizer_path = Arc::clone(&network_node.path);
    let localizer_client = Arc::clone(&network_node.client);
    let localizer_april_tags = Arc::clone(&network_node.april_tags);

    tokio::task::spawn(async move {
        let server = udp_server::Server::new(server_path, server_client).await;
        server.run().await.unwrap();
    });
    tokio::task::spawn(async move {
        loop {
            let data = ping_data.lock().unwrap();
            if data.is_some() {
                println!("Node is Alive and Running");
            } else {
                println!("Node is Alive with No data");
            }
            std::thread::sleep(std::time::Duration::from_millis(1000));
        }
    });
    tokio::task::spawn(async move {
        loop {
            let april_tags_unlocked = localizer_april_tags.lock().unwrap();
            if april_tags_unlocked.as_ref().is_some() {
                let april_tags_pose = april_tags::localize(april_tags_unlocked.as_ref());
                if let Some(april_tags_pose) = april_tags_pose {
                    // TODO: impl kalman filter
                }
            }
            std::thread::sleep(std::time::Duration::from_millis(50));
        }
    });
    rclrs::spin(Arc::clone(&network_node.node))
}
