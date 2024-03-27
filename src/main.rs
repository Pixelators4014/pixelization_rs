use std::sync::Arc;
use tokio::sync::RwLock;
use nav_msgs::msg::Path as PathMsg;
use isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray;

use log::{info, warn, error};

mod april_tags;
mod kalman_filter;
pub(crate) mod node;
pub(crate) mod udp_server;
pub mod util;


#[tokio::main]
async fn main() -> Result<(), rclrs::RclrsError> {
    let context = rclrs::Context::new(std::env::args())?;
    let network_node = Arc::new(node::NetworkNode::new(&context)?);
    ros2_logger::init_with_level(Arc::clone(&network_node.node), log::Level::Trace).unwrap();
    info!("Starting Pixelization Node");
    network_node.init().await?; // TODO: join instead
    let server_network_node = Arc::clone(&network_node);
    let ping_network_node = Arc::clone(&network_node);
    let localizer_network_node = Arc::clone(&network_node);

    let t = tokio::task::spawn(async move {
        server_network_node.run_server().await;
    });
    tokio::task::spawn(async move {
        ping_network_node.run_ping().await;
    });
    tokio::task::spawn(async move {
        localizer_network_node.run_localizer().await;
    });
    std::thread::spawn(move || {
        if let Err(e) = rclrs::spin(Arc::clone(&network_node.node)) {
            error!("{:?}", e);
        }
    });
    info!("Pixelization Node Up; Main Loop Idling");
    let _ = t.await;
    Ok(())
}
