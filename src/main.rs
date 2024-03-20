use std::sync::{Arc, Mutex};
use nav_msgs::msg::Path as PathMsg;

mod udp_server;

struct NetworkNode {
    node: Arc<rclrs::Node>,
    #[allow(dead_code)]
    subscription: Arc<rclrs::Subscription<PathMsg>>,
    data: Arc<Mutex<Option<PathMsg>>>,
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
    let server_data = Arc::clone(&network_node.data);
    let ping_data = Arc::clone(&network_node.data);

    std::thread::spawn(move || {
        let mut server = udp_server::Server::new(server_data);
        server.run().unwrap();
    });
    std::thread::spawn(move || {
        loop {
            let data = ping_data.lock().unwrap();
            if data.is_some() {
                println!("Node is Alive and Running");
            } else {
                println!("Node is Alive with No data");
            }
            std::thread::sleep(std::time::Duration::from_millis(1000));
        }
    });
    rclrs::spin(Arc::clone(&network_node.node))
}
