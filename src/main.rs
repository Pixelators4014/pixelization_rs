use std::sync::Arc;
use tokio::sync::RwLock;
use nav_msgs::msg::Path as PathMsg;
use isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray;

use log::{info, warn, error};

mod april_tags;
mod kalman_filter;
pub(crate) mod node;
mod udp_server;
pub mod util;

async fn run_server(server_path: Arc<RwLock<Option<PathMsg>>>, server_client: Arc<rclrs::Client<isaac_ros_visual_slam_interfaces::srv::SetOdometryPose>>) {
    let server = udp_server::Server::new(server_path, server_client).await;
    server.run().await.unwrap();
}

async fn run_ping(path_data: Arc<RwLock<Option<PathMsg>>>, april_tags_data: Arc<RwLock<Option<AprilTagDetectionArray>>>) {
    loop {
        let data = path_data.read().await;
        if let Some(path_option) = data.as_ref() {
            if let Some(path) = path_option.poses.last() {
                info!("VSLAM is running: {path:?}");
            } else {
                warn!("VSLAM has not initialized yet")
            }
        } else {
            warn!("VSLAM not connected yet");
        }
        drop(data);
        let data = april_tags_data.read().await;
        if let Some(april_tags) = data.as_ref() {
            info!("April Tags is running: {} april tags", april_tags.detections.len());
        } else {
            warn!("April Tags not connected yet");
        }
        tokio::time::sleep(std::time::Duration::from_millis(1000)).await;
    }
}

#[tokio::main]
async fn main() -> Result<(), rclrs::RclrsError> {
    let context = rclrs::Context::new(std::env::args())?;
    let network_node = Arc::new(node::NetworkNode::new(&context)?);
    ros2_logger::init_with_level(Arc::clone(&network_node.node), log::Level::Trace).unwrap();
    info!("Starting Pixelization Node");
    network_node.init().await; // TODO: join instead
    let server_path = Arc::clone(&network_node.path);
    let server_client = Arc::clone(&network_node.client);
    let ping_path = Arc::clone(&network_node.path);
    let ping_april_tags = Arc::clone(&network_node.april_tags);
    let localizer_path = Arc::clone(&network_node.path);
    let localizer_client = Arc::clone(&network_node.client);
    let localizer_april_tags = Arc::clone(&network_node.april_tags);

    let t = tokio::task::spawn(async move {
        run_server(server_path, server_client).await;
    });
    tokio::task::spawn(async move {
        run_ping(ping_path, ping_april_tags).await;
    });
    tokio::task::spawn(async move {
        loop {
            let april_tags_unlocked_option = localizer_april_tags.read().await;
            if let Some(april_tags_unlocked) = april_tags_unlocked_option.as_ref() {
                let april_tags_pose = april_tags::localize(april_tags_unlocked);
                if let Some(april_tags_pose) = april_tags_pose {
                    info!("Using April Tags Pose: {april_tags_pose:?}");
                    // TODO: impl kalman filter
                    let final_pose = april_tags_pose;
                    let client = Arc::clone(&localizer_client);
                    let service_request = isaac_ros_visual_slam_interfaces::srv::SetOdometryPose_Request {
                        pose: geometry_msgs::msg::Pose {
                            position: geometry_msgs::msg::Point {
                                x: final_pose.translation.x as f64,
                                y: final_pose.translation.y as f64,
                                z: final_pose.translation.z as f64,
                            },
                            orientation: geometry_msgs::msg::Quaternion {
                                w: final_pose.rotation.quaternion().coords[3] as f64,
                                x: final_pose.rotation.quaternion().coords[0] as f64,
                                y: final_pose.rotation.quaternion().coords[1] as f64,
                                z: final_pose.rotation.quaternion().coords[2] as f64,
                            }
                        }
                    };
                    let response = client.call_async(&service_request).await.unwrap();
                }
            }
            tokio::time::sleep(std::time::Duration::from_millis(50)).await;
        }
    });
    std::thread::spawn(move || {
        if let Err(e) = rclrs::spin(Arc::clone(&network_node.node)) {
            error!("{:?}", e);
        }
    });
    info!("Pixelization Node Up; Main Loop Idling");
    t.await;
    Ok(())
}
