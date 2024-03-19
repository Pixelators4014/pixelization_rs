use std::sync::{Arc, Mutex};
use nav_msgs::msg::Path as PathMsg;

struct NetworkNode {
    node: Arc<rclrs::Node>,
    subscription: Arc<rclrs::Subscription<PathMsg>>,
    data: Option<PathMsg>,
}

impl NetworkNode {
    fn new(context: &rclrs::Context) -> Result<Self, rclrs::RclrsError> {
        let node = rclrs::Node::new(context, "network_node")?;
        let data = Arc::new(Mutex::new(None));
        let data_cb = Arc::clone(&data);
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
        Ok(Self {
            node,
            subscription,
            data,
        })
    }
}

fn main() -> Result<(), rclrs::RclrsError> {
    let context = rclrs::Context::new(std::env::args())?;
    let network_node = Arc::new(NetworkNode::new(&context)?);
    rclrs::spin(Arc::clone(&network_node.node))
}
