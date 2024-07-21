pub mod positions;
pub mod util;

use std::{env, thread};
use std::sync::Arc;
use isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray;
use isaac_ros_visual_slam_interfaces::srv::SetSlamPose;
use log::{debug, error, info, trace};
use rclrs::{Client, Context, create_node, Node, RclrsError, Subscription};
use tokio::sync::{Mutex, RwLock, watch};

pub struct LocalizerNode {
    pub node: Arc<Node>,
    pub april_tags: Arc<RwLock<Option<AprilTagDetectionArray>>>,
    pub april_tags_receiver: Arc<Mutex<watch::Receiver<chrono::DateTime<chrono::Utc>>>>,
    pub client: Arc<Client<SetSlamPose>>,
    pub _april_tags_subscription: Arc<Subscription<AprilTagDetectionArray>>
}

impl LocalizerNode {
    pub fn new(context: &Context) -> Result<Self, rclrs::RclrsError> {
        let node = create_node(context, "localizer")?;
        let now = chrono::Utc::now();
        let (tx, rx) = watch::channel(now);

        let april_tags = Arc::new(RwLock::new(None));
        let april_tags_subscription =
            node.create_subscription("/tag_detections", rclrs::QOS_PROFILE_DEFAULT, {
                let april_tags_cb = Arc::clone(&april_tags);
                move |msg: AprilTagDetectionArray| {
                    *april_tags_cb.blocking_write() = Some(msg.clone());
                    tx.send(chrono::Utc::now()).unwrap();
                }
        })?;
        let client = node.create_client::<SetSlamPose>(
            "visual_slam/set_slam_pose",
        )?;
        Ok(Self {
            node,
            april_tags_receiver: Arc::new(Mutex::new(rx)),
            april_tags,
            _april_tags_subscription: april_tags_subscription,
            client
        })
    }

    pub async fn run(&self) {
        let mut rx = self.april_tags_receiver.lock().await;
        loop {
            if rx.has_changed().unwrap() {
                let _date = rx.borrow_and_update();
                drop(_date);
                let current_april_tags = self.april_tags.read().await;
                let april_tags_pose =
                    current_april_tags.as_ref()
                        .map(|o| positions::localize(o))
                        .flatten();
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

#[tokio::main]
async fn main() -> Result<(), RclrsError> {
    let context = Context::new(env::args())?;
    let node = Arc::new(LocalizerNode::new(&context)?);
    thread::spawn({
        let node = Arc::clone(&node);
        move || {
            rclrs::spin(Arc::clone(&node.node)).expect("RCLRS failed on spin");
        }
    });
    node.run().await;
    Ok(())
}