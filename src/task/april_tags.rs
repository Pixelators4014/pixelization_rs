use std::sync::Arc;
use async_trait::async_trait;
use isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray;
use isaac_ros_visual_slam_interfaces::srv::SetSlamPose;
use log::{debug, error, info, trace, warn};
use tokio::sync::{Mutex, RwLock, watch};
use crate::node::TaskContext;
use crate::task::Task;
use nav_msgs::msg::Path as PathMsg;
use rclrs::Client;

pub struct AprilTags {
    path: Arc<RwLock<Option<PathMsg>>>,
    april_tags: Arc<RwLock<Option<AprilTagDetectionArray>>>,
    april_tags_receiver: Arc<Mutex<watch::Receiver<chrono::DateTime<chrono::Utc>>>>,
    client: Arc<Client<SetSlamPose>>,
}

impl AprilTags {
    pub async fn new(context: TaskContext) -> Self {
        Self {
            path: context.path,
            april_tags: context.april_tags,
            april_tags_receiver: context.april_tags_receiver,
            client: context.client
        }
    }
}

#[async_trait]
impl Task for AprilTags {
    async fn run(&self) {
        let mut rx = self.april_tags_receiver.lock().await;
        loop {
            if rx.has_changed().unwrap() {
                let _date = rx.borrow_and_update();
                drop(_date);
                let current_april_tags = self.april_tags.read().await;
                trace!("April Tags: {:?}", current_april_tags.as_ref());
                let april_tags_pose = crate::april_tags::localize(current_april_tags.as_ref().unwrap());
                drop(current_april_tags);
                if let Some(april_tags_pose) = april_tags_pose {
                    info!("Using April Tags Pose: {april_tags_pose:?}");
                    let final_pose = april_tags_pose;
                    let client = Arc::clone(&self.client);
                    let service_request =
                        isaac_ros_visual_slam_interfaces::srv::SetSlamPose_Request {
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