mod april_tags;
mod ping;
mod server;

pub use april_tags::AprilTags;
pub use ping::Ping;
pub use server::Server;

use async_trait::async_trait;

#[async_trait]
pub trait Task: Send + Sync {
    async fn run(&self);
}
