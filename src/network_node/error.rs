use thiserror::Error;

#[derive(Error, Debug)]
pub enum Error {
    #[error("RCLRS Error: {0}")]
    RCLRS(#[from] rclrs::RclrsError),
    #[error("Paramter Declaration Error: {0}")]
    ParameterDeclarationError(#[from] rclrs::DeclarationError),
    #[error("Server Error: {0}")]
    Server(#[from] crate::udp_server::ServerError),
    #[error("I/O Error: {0}")]
    IO(#[from] std::io::Error)
}
