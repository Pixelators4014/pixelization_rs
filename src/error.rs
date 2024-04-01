use thiserror::Error;

#[derive(Error, Debug)]
pub enum Error {
    #[error("RCLRS Error: {0}")]
    RCLRS(#[from] rclrs::RclrsError),
    #[error["Server Error: {0}"]]
    Server(#[from] crate::udp_server::ServerError),
}
