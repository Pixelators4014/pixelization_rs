use std::sync::{Arc, Mutex};
use log::{debug, info, error, trace};

use tokio::sync::{oneshot, RwLock, watch};

use isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray;
use nav_msgs::msg::Path as PathMsg;
use tokio::join;
use crate::{april_tags, task};
use crate::task::Task;

#[derive(Clone)]
pub struct TaskContext {
    pub _path_subscription: Arc<rclrs::Subscription<PathMsg>>,
    pub _april_tags_subscription: Arc<rclrs::Subscription<AprilTagDetectionArray>>,
    pub client: Arc<rclrs::Client<isaac_ros_visual_slam_interfaces::srv::SetSlamPose>>,
    pub path: Arc<RwLock<Option<PathMsg>>>,
    pub april_tags: Arc<RwLock<Option<AprilTagDetectionArray>>>,
    pub april_tags_receiver: Arc<Mutex<watch::Receiver<chrono::DateTime<chrono::Utc>>>>,
}

pub struct NetworkNode {
    pub node: Arc<rclrs::Node>,
    pub tasks: Vec<Arc<dyn Task>>,
    pub task_context: TaskContext,
    pub parameters: Parameters
}

pub struct Parameters {
    pub april_tags: bool,
    pub object_detection: bool,
    pub vslam: bool,
}

impl NetworkNode {
    pub async fn new(context: &rclrs::Context, _shutdown_trigger: oneshot::Sender<()>) -> Result<Self, rclrs::RclrsError> {
        let node = rclrs::Node::new(context, "network_node")?;
        let param_april_tags = node.declare_parameter("april_tags").default(true).mandatory().unwrap();
        let param_object_detection = node.declare_parameter("object_detection").default(true).mandatory().unwrap();
        let param_vslam = node.declare_parameter("vslam").default(true).mandatory().unwrap();

        let parameters = Parameters {
            april_tags: param_april_tags.get(),
            object_detection: param_object_detection.get(),
            vslam: param_vslam.get()
        };

        let client = node.create_client::<isaac_ros_visual_slam_interfaces::srv::SetSlamPose>(
            "visual_slam/set_odometry_pose",
        )?;

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


        // Setup send channel
        let now = chrono::Utc::now();
        let (tx, rx) = watch::channel(now);

        let april_tags = Arc::new(RwLock::new(None));
        let april_tags_subscription = node.create_subscription(
            "/tag_detections",
            rclrs::QOS_PROFILE_DEFAULT,
            {
                let april_tags_cb = Arc::clone(&april_tags);
                move |msg: AprilTagDetectionArray| {
                    *april_tags_cb.blocking_write() = Some(msg.clone());
                    tx.send(chrono::Utc::now()).unwrap();
                }
            }
        )?;

        let task_context = TaskContext {
            _path_subscription: path_subscription,
            _april_tags_subscription: april_tags_subscription,
            client,
            path,
            april_tags,
            april_tags_receiver: Arc::new(Mutex::new(rx)),
        };

        let ping_task = task::Ping::new(task_context.clone());
        let server_task = task::Server::new(task_context.clone());
        let tasks = join!(ping_task, server_task);
        let tasks: Vec<Arc<dyn Task>> = vec![Arc::new(tasks.0), Arc::new(tasks.1)];

        Ok(Self {
            tasks,
            parameters,
            node,
            task_context
        })
    }

    pub async fn init(&self) -> Result<(), rclrs::RclrsError> {
        while !self.task_context.client.service_is_ready()? {
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
