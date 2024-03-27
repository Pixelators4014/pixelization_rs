//! A logger that prints all messages using the ros2 log format.
//!
//! It uses the ros2 log subscriber to publish logs.
//!
//! ```rust
//! ros2_logger::Ros2Logger::new().env().init().unwrap();
//!
//! log::warn!("This is an example message.");
//! ```
//!
//! Some shortcuts are available for common use cases.
//!
//! Just initialize logging without any configuration:
//!
//! ```rust
//! ros2_logger::init().unwrap();
//! ```
//!
//! Set the log level from the `RUST_LOG` environment variable:
//!
//! ```rust
//! ros2_logger::init_with_env().unwrap();
//! ```
//!
//! Hardcode a default log level:
//!
//! ```rust
//! ros2_logger::init_with_level(log::Level::Warn).unwrap();
//! ```

use log::{Level, LevelFilter, Log, Metadata, Record, SetLoggerError};
use std::str::FromStr;
use std::time::{SystemTime, UNIX_EPOCH};

/// Implements [`Log`] and a set of simple builder methods for configuration.
///
/// Use the various "builder" methods on this struct to configure the logger,
/// then call [`init`] to configure the [`log`] crate.
pub struct Ros2Logger {
    /// The default logging level
    default_level: LevelFilter,

    /// The specific logging level for each module
    ///
    /// This is used to override the default value for some specific modules.
    ///
    /// This must be sorted from most-specific to least-specific, so that [`enabled`](#method.enabled) can scan the
    /// vector for the first match to give us the desired log level for a module.
    module_levels: Vec<(String, LevelFilter)>,
    ros_out_publisher: std::sync::Arc<rclrs::Publisher<rcl_interfaces::msg::Log>>,
}

impl Ros2Logger {
    /// Initializes the global logger with a Ros2Logger instance with
    /// default log level set to `Level::Trace`.
    ///
    /// ```no_run
    /// use ros2_logger::Ros2Logger;
    /// let context = rclrs::Context::new(env::args())?;
    /// let node = rclrs::create_node(&context, "minimal_node")?;
    /// Ros2Logger::new(Arc::clone(&node)).env().init().unwrap();
    /// log::warn!("This is an example message.");
    /// ```
    ///
    /// [`init`]: #method.init
    #[must_use = "You must call init() to begin logging"]
    pub fn new(node: std::sync::Arc<rclrs::Node>) -> Ros2Logger {
        let publisher = node.create_publisher::<rcl_interfaces::msg::Log>("/ros_out", rclrs::QOS_PROFILE_DEFAULT).unwrap();
        Ros2Logger {
            default_level: LevelFilter::Trace,
            module_levels: Vec::new(),
            ros_out_publisher: publisher,
        }
    }

    /// Enables the user to choose log level by setting `RUST_LOG=<level>`
    /// environment variable. This will use the default level set by
    /// [`with_level`] if `RUST_LOG` is not set or can't be parsed as a
    /// standard log level.
    ///
    /// This must be called after [`with_level`]. If called before
    /// [`with_level`], it will have no effect.
    ///
    /// [`with_level`]: #method.with_level
    #[must_use = "You must call init() to begin logging"]
    pub fn env(mut self) -> Ros2Logger {
        self.default_level = std::env::var("RUST_LOG")
            .ok()
            .as_deref()
            .map(log::LevelFilter::from_str)
            .and_then(Result::ok)
            .unwrap_or(self.default_level);

        self
    }

    /// Set the 'default' log level.
    ///
    /// You can override the default level for specific modules and their sub-modules using [`with_module_level`]
    ///
    /// This must be called before [`env`]. If called after [`env`], it will override the value loaded from the environment.
    ///
    /// [`env`]: #method.env
    /// [`with_module_level`]: #method.with_module_level
    #[must_use = "You must call init() to begin logging"]
    pub fn with_level(mut self, level: LevelFilter) -> Ros2Logger {
        self.default_level = level;
        self
    }

    /// Override the log level for some specific modules.
    ///
    /// This sets the log level of a specific module and all its sub-modules.
    /// When both the level for a parent module as well as a child module are set,
    /// the more specific value is taken. If the log level for the same module is
    /// specified twice, the resulting log level is implementation defined.
    ///
    /// # Examples
    ///
    /// Silence an overly verbose crate:
    ///
    /// ```no_run
    /// use ros2_logger::Ros2Logger;
    /// use log::LevelFilter;
    ///
    /// Ros2Logger::new().with_module_level("chatty_dependency", LevelFilter::Warn).init().unwrap();
    /// ```
    ///
    /// Disable logging for all dependencies:
    ///
    /// ```no_run
    /// use ros2_logger::Ros2Logger;
    /// use log::LevelFilter;
    ///
    /// Ros2Logger::new()
    ///     .with_level(LevelFilter::Off)
    ///     .with_module_level("my_crate", LevelFilter::Info)
    ///     .init()
    ///     .unwrap();
    /// ```
    //
    // This method *must* sort `module_levels` for the [`enabled`](#method.enabled) method to work correctly.
    #[must_use = "You must call init() to begin logging"]
    pub fn with_module_level(mut self, target: &str, level: LevelFilter) -> Ros2Logger {
        self.module_levels.push((target.to_string(), level));
        self.module_levels
            .sort_by_key(|(name, _level)| name.len().wrapping_neg());
        self
    }

    /// Configure the logger
    pub fn max_level(&self) -> LevelFilter {
        let max_level = self.module_levels.iter().map(|(_name, level)| level).copied().max();
        max_level
            .map(|lvl| lvl.max(self.default_level))
            .unwrap_or(self.default_level)
    }

    /// 'Init' the actual logger and instantiate it,
    /// this method MUST be called in order for the logger to be effective.
    pub fn init(self) -> Result<(), SetLoggerError> {
        log::set_max_level(self.max_level());
        log::set_boxed_logger(Box::new(self))
    }
}

impl Log for Ros2Logger {
    fn enabled(&self, _metadata: &Metadata) -> bool {
        true
    }

    fn log(&self, record: &Record) {
        if self.enabled(record.metadata()) {
            let level_string = {
                format!("{}", record.level().to_string())
            };

            let target = if !record.target().is_empty() {
                record.target()
            } else {
                record.module_path().unwrap_or_default()
            };

            let thread = {
                ""
            };

            let timestamp = {
                ""
            };

            let time = SystemTime::now().duration_since(UNIX_EPOCH).unwrap().as_secs();

            let message = format!("[{target}{thread}] [{level_string}] [{time}]: {}", record.args());

            eprintln!("{}", message);
            let mut log = rcl_interfaces::msg::Log::default();
            // TODO: TimeStamp
            let timestamp = builtin_interfaces::msg::Time {
                sec: time as i32,
                nanosec: 0
            };
            log.stamp = timestamp;
            log.level = match record.level() {
                Level::Error => 40,
                Level::Warn => 30,
                Level::Info => 20,
                Level::Debug => 10,
                Level::Trace => 10,
            };
            log.msg = record.args().to_string();
            log.name = record.target().to_string();
            log.file = record.file().unwrap_or_default().to_string();
            log.line = record.line().unwrap_or_default() as u32;
            self.ros_out_publisher.publish(&log); // TODO: eprintln if this doesn't work
        }
    }

    fn flush(&self) {}
}

/// Initialise the logger with its default configuration.
///
/// Log messages will not be filtered.
/// The `RUST_LOG` environment variable is not used.
pub fn init(node: std::sync::Arc<rclrs::Node>) -> Result<(), SetLoggerError> {
    Ros2Logger::new(node).init()
}

/// Initialise the logger with the `RUST_LOG` environment variable.
///
/// Log messages will be filtered based on the `RUST_LOG` environment variable.
pub fn init_with_env(node: std::sync::Arc<rclrs::Node>) -> Result<(), SetLoggerError> {
    Ros2Logger::new(node).env().init()
}

/// Initialise the logger with a specific log level.
///
/// Log messages below the given [`Level`] will be filtered.
/// The `RUST_LOG` environment variable is not used.
pub fn init_with_level(node: std::sync::Arc<rclrs::Node>, level: Level) -> Result<(), SetLoggerError> {
    Ros2Logger::new(node).with_level(level.to_level_filter()).init()
}
