use std::sync::{Arc, Mutex};
use nav_msgs::msg::Path as PathMsg;

mod april_tags;
mod kalman_filter;
pub mod pose;
mod udp_server;

struct NetworkNode {
    node: Arc<rclrs::Node>,
    #[allow(dead_code)]
    subscription: Arc<rclrs::Subscription<PathMsg>>,
    client: Arc<rclrs::Client<isaac_ros_visual_slam_interfaces::srv::SetOdometryPose>>,
    path_data: Arc<Mutex<Option<PathMsg>>>,
}

impl NetworkNode {
    fn new(context: &rclrs::Context) -> Result<Self, rclrs::RclrsError> {
        let node = rclrs::Node::new(context, "network_node")?;
        let path_data = Arc::new(Mutex::new(None));
        let data_cb = Arc::clone(&path_data);
        let subscription =
            // Create a new shared pointer instance that will be owned by the closure
            node.create_subscription(
                "/visual_slam/tracking/slam_path",
                rclrs::QOS_PROFILE_DEFAULT,
                move |msg: PathMsg| {
                    // This subscription now owns the data_cb variable
                    *data_cb.lock().unwrap() = Some(msg);
                },
            )?;
        let client = node.create_client::<isaac_ros_visual_slam_interfaces::srv::SetOdometryPose>("visual_slam/set_odometry_pose")?;
        while !client.service_is_ready()? {
            std::thread::sleep(std::time::Duration::from_millis(10));
            println!("Waiting for service to initialize ...");
        }
        Ok(Self {
            node,
            subscription,
            client,
            data,
        })
    }
}

#[tokio::main]
async fn main() -> Result<(), rclrs::RclrsError> {
    let context = rclrs::Context::new(std::env::args())?;
    let network_node = Arc::new(NetworkNode::new(&context)?);
    let server_data = Arc::clone(&network_node.path_data);
    let server_client = Arc::clone(&network_node.client);
    let ping_data = Arc::clone(&network_node.path_data);

    tokio::task::spawn(async move {
        let server = udp_server::Server::new(server_data, server_client).await;
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
    rclrs::spin(Arc::clone(&network_node.node))
}
