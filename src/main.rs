use std::sync::Arc;
use tokio::sync::RwLock;
use nav_msgs::msg::Path as PathMsg;
use isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray;

mod april_tags;
mod kalman_filter;
pub(crate) mod node;
mod udp_server;

async fn run_server(server_path: Arc<RwLock<Option<PathMsg>>>, server_client: Arc<rclrs::Client<isaac_ros_visual_slam_interfaces::srv::SetOdometryPose>>) {
    let server = udp_server::Server::new(server_path, server_client).await;
    server.run().await.unwrap();
}

async fn run_ping(ping_data: Arc<RwLock<Option<PathMsg>>>) {
    loop {
        let data = ping_data.read().await;
        if data.is_some() {
            println!("Node is Alive and Running");
        } else {
            println!("Node is Alive with No data");
        }
        drop(data);
        tokio::time::sleep(std::time::Duration::from_millis(1000)).await;
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
            let april_tags_unlocked_option = localizer_april_tags.read().await;
            if let Some(april_tags_unlocked) = april_tags_unlocked_option.as_ref() {
                let april_tags_pose = april_tags::localize(april_tags_unlocked);
                if let Some(april_tags_pose) = april_tags_pose {
                    // TODO: impl kalman filter
                    let final_pose = april_tags_pose;
                    let client = Arc::clone(&localizer_client);
                    let service_request = isaac_ros_visual_slam_interfaces::srv::SetOdometryPose_Request {
                        pose: geometry_msgs::msg::Pose {
                            position: geometry_msgs::msg::Point {
                                x: final_pose.position.x as f64,
                                y: final_pose.position.y as f64,
                                z: final_pose.position.z as f64,
                            },
                            orientation: geometry_msgs::msg::Quaternion {
                                w: final_pose.orientation.w as f64,
                                x: final_pose.orientation.x as f64,
                                y: final_pose.orientation.y as f64,
                                z: final_pose.orientation.z as f64,
                            }
                        }
                    };
                    let response = client.call_async(&service_request).await.unwrap();
                }
            }
            std::thread::sleep(std::time::Duration::from_millis(50));
        }
    });
    std::thread::spawn(move || {
        if let Err(e) = rclrs::spin(Arc::clone(&network_node.node)) {
            println!("{:?}", e);
        }
    });
    Ok(())
}
