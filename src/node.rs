use std::sync::{Arc, Mutex};

use log::{debug, info, warn, error, trace};

use tokio::sync::{oneshot, RwLock, watch};

use isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray;
use nav_msgs::msg::Path as PathMsg;

use crate::april_tags;

pub struct NetworkNode {
    pub node: Arc<rclrs::Node>,
    #[allow(dead_code)]
    pub path_subscription: Arc<rclrs::Subscription<PathMsg>>,
    #[allow(dead_code)]
    pub april_tags_subscription: Arc<rclrs::Subscription<AprilTagDetectionArray>>,
    pub client: Arc<rclrs::Client<isaac_ros_visual_slam_interfaces::srv::SetOdometryPose>>,
    pub path: Arc<RwLock<Option<PathMsg>>>,
    pub april_tags: Arc<RwLock<Option<AprilTagDetectionArray>>>,
    pub april_tags_receiver: Mutex<watch::Receiver<AprilTagDetectionArray>>,
}

impl NetworkNode {
    pub fn new(context: &rclrs::Context) -> Result<Self, rclrs::RclrsError> {
        let node = rclrs::Node::new(context, "network_node")?;

        let client = node.create_client::<isaac_ros_visual_slam_interfaces::srv::SetOdometryPose>(
            "visual_slam/set_odometry_pose",
        )?;

        let path = Arc::new(RwLock::new(None));
        let path_cb = Arc::clone(&path);
        let path_subscription =
            // Create a new shared pointer instance that will be owned by the closure
            node.create_subscription(
                "/visual_slam/tracking/slam_path",
                rclrs::QOS_PROFILE_DEFAULT,
                move |msg: PathMsg| {
                    // This subscription now owns the data_cb variable
                    *path_cb.blocking_write() = Some(msg);
                },
            )?;

        let blank_detection_array = AprilTagDetectionArray::default();
        let (tx, rx) = watch::channel(blank_detection_array);

        let april_tags = Arc::new(RwLock::new(None));
        let april_tags_cb = Arc::clone(&april_tags);
        let april_tags_subscription = node.create_subscription(
            "/tag_detections",
            rclrs::QOS_PROFILE_DEFAULT,
            move |msg: AprilTagDetectionArray| {
                *april_tags_cb.blocking_write() = Some(msg);
            },
        )?;
        Ok(Self {
            node,
            path_subscription,
            april_tags_subscription,
            client,
            path,
            april_tags,
            april_tags_receiver: Mutex::new(rx),
        })
    }

    pub async fn init(&self) -> Result<(), rclrs::RclrsError> {
        while !self.client.service_is_ready()? {
            tokio::time::sleep(std::time::Duration::from_millis(10)).await;
            info!("Waiting for service to initialize ...");
        }
        Ok(())
    }

    pub async fn run_server(&self, shutdown_trigger: oneshot::Sender<()>) {
        let server =
            crate::udp_server::Server::new(Arc::clone(&self.path), Arc::clone(&self.client)).await;
        server.run().await.unwrap();
        shutdown_trigger.send(()).unwrap(); // TODO: Redo this
    }

    pub async fn run_ping(&self) {
        loop {
            let data = self.path.read().await;
            if let Some(path_option) = data.as_ref() {
                if let Some(path) = path_option.poses.last() {
                    debug!("VSLAM is running: {path:?}");
                } else {
                    warn!("VSLAM has not initialized yet")
                }
            } else {
                warn!("VSLAM not connected yet");
            }
            drop(data);
            let data = self.april_tags.read().await;
            if let Some(april_tags) = data.as_ref() {
                debug!(
                    "April Tags is running: {} april tags",
                    april_tags.detections.len()
                );
            } else {
                warn!("April Tags not connected yet");
            }
            tokio::time::sleep(std::time::Duration::from_millis(1000)).await;
        }
    }

    pub async fn run_april_tag_localizer(&self) {
        let mut rx = self.april_tags_receiver.lock().unwrap();
        loop {
            if rx.has_changed().unwrap() {
                let april_tags = rx.borrow_and_update();
                trace!("April Tags: {april_tags:?}");
                let april_tags_pose = april_tags::localize(&april_tags);
                drop(april_tags);
                if let Some(april_tags_pose) = april_tags_pose {
                    info!("Using April Tags Pose: {april_tags_pose:?}");
                    // TODO: impl kalman filter
                    let final_pose = april_tags_pose;
                    let client = Arc::clone(&self.client);
                    let service_request =
                        isaac_ros_visual_slam_interfaces::srv::SetOdometryPose_Request {
                            pose: geometry_msgs::msg::Pose {
                                position: geometry_msgs::msg::Point {
                                    x: final_pose.translation.x as f64,
                                    y: final_pose.translation.y as f64,
                                    z: final_pose.translation.z as f64,
                                },
                                orientation: geometry_msgs::msg::Quaternion {
                                    w: final_pose.rotation.quaternion().coords[3] as f64,
                                    x: final_pose.rotation.quaternion().coords[0] as f64,
                                    y: final_pose.rotation.quaternion().coords[1] as f64,
                                    z: final_pose.rotation.quaternion().coords[2] as f64,
                                },
                            },
                        };
                    let response = client.call_async(&service_request).await.unwrap();
                    if response.success {
                        debug!("Set Odometry Pose: {service_request:?}");
                    } else {
                        error!("Failed to set Odometry Pose: {service_request:?}");
                    }
                }
            }
            if rx.changed().await.is_err() {
                return;
            }
        }
    }
}
