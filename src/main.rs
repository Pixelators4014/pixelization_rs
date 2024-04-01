use std::sync::Arc;
use std::process::{Command, Stdio};


use log::{error, info};

mod april_tags;
mod kalman_filter;
pub(crate) mod node;
pub(crate) mod udp_server;
pub mod util;
pub mod error;

pub use tokio::sync::oneshot;

pub type Result<T> = std::result::Result<T, error::Error>;

#[tokio::main]
async fn main() -> Result<()> {
    let context = rclrs::Context::new(std::env::args())?;
    let network_node = Arc::new(node::NetworkNode::new(&context)?);
    ros2_logger::init_with_level(Arc::clone(&network_node.node), log::Level::Debug).unwrap();
    info!("Starting Pixelization Node");
    let server_network_node = Arc::clone(&network_node);
    let ping_network_node = Arc::clone(&network_node);
    let april_tags_localizer_node = Arc::clone(&network_node);

    let (tx, rx) = oneshot::channel();

    let t = tokio::task::spawn(async move {
        server_network_node.run_server(tx).await;
    });

    tokio::task::spawn(async move {
        ping_network_node.run_ping().await;
    });

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

    std::thread::spawn(move || {
        Command::new("/opt/ros/humble/bin/ros2")
            .arg("launch")
            .arg("isaac_ros_visual_slam")
            .arg("isaac_ros_visual_slam_realsense.launch.py")
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .spawn().unwrap();
    });

    info!("Pixelization Node Up; Main Loop Idling");
    if let Ok(_) = rx.await {
        info!("Pixelization Node Shutting Down on server request.");
        return Ok(());
    } else {
        error!("rx channel closed unexpectedly, waiting on server.");
    }
    let _ = t.await;
    Ok(())
}
