use std::sync::Arc;

use log::{error, info};

mod april_tags;
mod kalman_filter;
pub(crate) mod node;
pub(crate) mod udp_server;
pub mod util;

pub use tokio::sync::oneshot;

#[tokio::main]
async fn main() -> Result<(), rclrs::RclrsError> {
    let context = rclrs::Context::new(std::env::args())?;
    let network_node = Arc::new(node::NetworkNode::new(&context)?);
    ros2_logger::init_with_level(Arc::clone(&network_node.node), log::Level::Debug).unwrap();
    info!("Starting Pixelization Node");
    let server_network_node = Arc::clone(&network_node);
    let ping_network_node = Arc::clone(&network_node);
    let localizer_network_node = Arc::clone(&network_node);

    let (tx, rx) = oneshot::channel();

    let t = tokio::task::spawn(async move {
        server_network_node.run_server(tx).await;
    });

    tokio::task::spawn(async move {
        ping_network_node.run_ping().await;
    });

    network_node.init().await?;

    std::thread::spawn(move || {
        if let Err(e) = rclrs::spin(Arc::clone(&network_node.node)) {
            error!("{:?}", e);
        }
    });
    info!("Pixelization Node Up; Main Loop Idling");
    if let Ok(_) = rx.await {
        info!("Pixelization Node Shutting Down on server request.");
        return Ok(());
    }
    let _ = t.await;
    Ok(())
}
