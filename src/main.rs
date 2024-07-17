/// # Pixelization
/// ## What is this?
/// This aims to be a "communications" node, it feeds in "odometry" data to vslam
/// and takes the stuff from vslam and object detection and sets up a UDP server to send it to RobotRIO.
/// ## Parameters
/// `april_tags` - (true by default) Enables or disables april tags.
/// `object_detection` - (true by default) Enables or disables object detection.
/// `vslam` - (true by default) Enables or disables VSLAM.
/// ## Tasks
/// - April tag localizer (TODO: should be its own node)
/// - Ping, which ensures everything is working
/// - Server, which broadcasts the data out to the world
use std::sync::Arc;

use log::{error, info};

mod april_tags;
pub mod error;
pub(crate) mod node;
mod task;
pub(crate) mod udp_server;
pub mod util;

pub use tokio::sync::oneshot;

pub type Result<T> = std::result::Result<T, error::Error>;

#[tokio::main]
async fn main() -> Result<()> {
    let context = rclrs::Context::new(std::env::args())?;
    let (tx, rx) = oneshot::channel();
    let network_node = Arc::new(node::NetworkNode::new(&context, tx).await?);
    ros2_logger::init_with_level(Arc::clone(&network_node.node), log::Level::Debug).unwrap();
    info!("Starting Pixelization Node");

    network_node.init().await?;
    let _ = network_node.run_tasks();

    std::thread::spawn(move || {
        if let Err(e) = rclrs::spin(Arc::clone(&network_node.node)) {
            error!("RCLRS Error on Spin: {:?}", e);
        }
    });

    info!("Pixelization Node Up; Main Loop Idling");
    if let Ok(_) = rx.await {
        info!("Pixelization Node Shutting Down on server request.");
        return Ok(());
    } else {
        error!("rx channel closed unexpectedly, waiting on server.");
    }
    Ok(())
}
