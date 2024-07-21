//! # Network Node
//! ## What is this?
//! This aims to be a "communications" node, it feeds in "odometry" data to vslam
//! and takes the stuff from vslam and object detection and sets up a UDP server to send it to RobotRIO.
//! ## Parameters
//! `object_detection` - (true by default) Enables or disables object detection.
//! `server` - (true by default) Enables or disables the udp server (useful for debugging only).
//! ## Tasks
//! - Ping, which ensures everything is working
//! - Server, which broadcasts the data out to the world

use std::sync::Arc;

use log::{error, info};

pub mod error;
pub mod node;
mod task;
pub mod udp_server;

use tokio::sync::oneshot;

pub use error::Error;

pub type Result<T> = std::result::Result<T, error::Error>;

#[tokio::main]
async fn main() -> Result<()> {
    let context = rclrs::Context::new(std::env::args())?;
    let (tx, rx) = oneshot::channel();
    let network_node = Arc::new(node::NetworkNode::new(&context, tx).await?);
    ros2_logger::init_with_level(Arc::clone(&network_node.node), log::Level::Debug).unwrap();
    info!("Starting Pixelization Node");

    network_node.init().await?;
    tokio::task::spawn({
        let network_node = network_node.clone();
        async move {
            network_node.run_tasks().await;
        }
    });

    std::thread::spawn(move || {
        if let Err(e) = rclrs::spin(Arc::clone(&network_node.node)) {
            error!("RCLRS Error on Spin: {:?}", e);
        }
    });

    info!("Pixelization Node Up; Main Loop Idling");
    match rx.await {
        Ok(()) => info!("Pixelization Node Shutting Down on server request."),
        Err(e) => error!("rx channel closed unexpectedly {e:?}.")
    }
    Ok(())
}
