use std::sync::Arc;

use log::{error, info};

mod april_tags;
pub(crate) mod node;
pub(crate) mod udp_server;
pub mod util;
pub mod error;
mod task;

pub use tokio::sync::oneshot;

pub type Result<T> = std::result::Result<T, error::Error>;

#[tokio::main]
async fn main() -> Result<()> {
    let context = rclrs::Context::new(std::env::args())?;
    let (tx, rx) = oneshot::channel();
    let network_node = Arc::new(node::NetworkNode::new(&context, tx).await?);
    ros2_logger::init_with_level(Arc::clone(&network_node.node), log::Level::Debug).unwrap();
    info!("Starting Pixelization Node");
    let april_tags_localizer_node = Arc::clone(&network_node);


    let _ = network_node.run_tasks();

    std::thread::spawn(move || {
        // Create the runtime
        let rt = tokio::runtime::Runtime::new().unwrap();

        // TODO: april_tags_localizer_node.run_april_tag_localizer() is not SEND
        rt.block_on(async {
            april_tags_localizer_node.run_april_tag_localizer().await;
        });
    });

    network_node.init().await?;

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
