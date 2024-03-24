use std::sync::{Arc, Mutex};
use nav_msgs::msg::Path as PathMsg;
use isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray;

mod april_tags;
mod kalman_filter;
pub(crate) mod node;
pub mod pose;
mod udp_server;

async fn run_server(server_path: Arc<Mutex<Option<PathMsg>>>, server_client: Arc<rclrs::Client<isaac_ros_visual_slam_interfaces::srv::SetOdometryPose>>) {
    let server = udp_server::Server::new(server_path, server_client).await;
    server.run().await.unwrap();
}

async fn run_ping(ping_data: Arc<Mutex<Option<PathMsg>>>) {
    loop {
        let data = ping_data.lock().unwrap();
        if data.is_some() {
            println!("Node is Alive and Running");
        } else {
            println!("Node is Alive with No data");
        }
        tokio::thread::sleep(std::time::Duration::from_millis(1000)).await;
    }
}

#[tokio::main]
async fn main() -> Result<(), rclrs::RclrsError> {
    let context = rclrs::Context::new(std::env::args())?;
    let network_node = Arc::new(node::NetworkNode::new(&context)?);
    let server_path = Arc::clone(&network_node.path);
    let server_client = Arc::clone(&network_node.client);
    let ping_data = Arc::clone(&network_node.path);
    let localizer_path = Arc::clone(&network_node.path);
    let localizer_client = Arc::clone(&network_node.client);
    let localizer_april_tags = Arc::clone(&network_node.april_tags);

    tokio::task::spawn(async move {
        run_server(server_path, server_client).await;
    });
    tokio::task::spawn(async move {
        run_ping(ping_data).await;
    });
    tokio::task::spawn(async move {
        loop {
            let april_tags_unlocked_option = localizer_april_tags.lock().unwrap();
            if let Some(april_tags_unlocked) = april_tags_unlocked_option.as_ref() {
                let april_tags_pose = april_tags::localize(april_tags_unlocked);
                if let Some(april_tags_pose) = april_tags_pose {
                    // TODO: impl kalman filter
                }
            }
            std::thread::sleep(std::time::Duration::from_millis(50));
        }
    });
    rclrs::spin(Arc::clone(&network_node.node))
}
