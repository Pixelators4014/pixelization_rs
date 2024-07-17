use log::info;
use std::sync::Arc;
use isaac_ros_visual_slam_interfaces::srv::SetSlamPose;
use tokio::sync::{oneshot, watch, Mutex, RwLock};

use crate::task::Task;
use crate::task;
use nav_msgs::msg::Path as PathMsg;
use rclrs::Client;
use tokio::join;

#[derive(Clone)]
pub struct TaskContext {
    pub _path_subscription: Arc<rclrs::Subscription<PathMsg>>,
    pub path: Arc<RwLock<Option<PathMsg>>>,
    pub set_pose: Arc<Client<SetSlamPose>>,
    pub parameters: Parameters,
}

pub struct NetworkNode {
    pub node: Arc<rclrs::Node>,
    pub tasks: Vec<Arc<dyn Task>>,
    pub task_context: TaskContext,
}

#[derive(Copy, Clone)]
pub struct Parameters {
    pub object_detection: bool,
    pub server: bool,
}

impl NetworkNode {
    pub async fn new(
        context: &rclrs::Context,
        _shutdown_trigger: oneshot::Sender<()>,
    ) -> Result<Self, rclrs::RclrsError> {
        let node = rclrs::Node::new(context, "network_node")?;
        let param_object_detection = node
            .declare_parameter("object_detection")
            .default(true)
            .mandatory()
            .unwrap();
        let param_server = node
            .declare_parameter("server")
            .default(true)
            .mandatory()
            .unwrap();

        let parameters = Parameters {
            object_detection: param_object_detection.get(),
            server: param_server.get(),
        };

        let path = Arc::new(RwLock::new(None));
        let path_subscription =
            // Create a new shared pointer instance that will be owned by the closure
            node.create_subscription(
                "/visual_slam/tracking/slam_path",
                rclrs::QOS_PROFILE_DEFAULT,
                {
                    let path_cb = Arc::clone(&path);
                    move |msg: PathMsg| {
                        // This subscription now owns the data_cb variable
                        *path_cb.blocking_write() = Some(msg);
                    }
                }
            )?;
        let set_pose = node.create_client::<SetSlamPose>(
            "visual_slam/set_slam_pose",
        )?;
        let task_context = TaskContext {
            _path_subscription: path_subscription,
            path,
            set_pose,
            parameters
        };

        let mut tasks: Vec<Arc<dyn Task>> = vec![];
        tasks.push(Arc::new(task::Ping::new(task_context.clone()).await));
        if parameters.server {
            tasks.push(Arc::new(task::Server::new(task_context.clone()).await));
        }

        Ok(Self {
            tasks,
            node,
            task_context,
        })
    }

    pub async fn init(&self) -> Result<(), rclrs::RclrsError> {
        while !self.task_context.set_pose.service_is_ready()? {
            tokio::time::sleep(std::time::Duration::from_millis(10)).await;
            info!("Waiting for service to initialize ...");
        }
        Ok(())
    }

    pub async fn run_tasks(&self) {
        let task_futures = self.tasks.clone();
        let mut tasks = vec![];
        for task in task_futures {
            tasks.push(tokio::task::spawn(async move {
                task.run().await;
                ()
            }));
        }
        for task in tasks {
            task.await.expect("join failed");
        }
    }
}
