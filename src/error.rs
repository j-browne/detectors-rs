use std::path::PathBuf;
use thiserror::Error;

#[derive(Debug, Error)]
pub enum Error {
    #[error("Unknown template: {0}")]
    UnknownTemplate(String),
    #[error("Failed to write to stdout")]
    Stdout { source: std::io::Error },
    #[error("Failed to open file: '{filename}'")]
    FileOpen {
        filename: PathBuf,
        source: std::io::Error,
    },
    #[error("Failed to deserialize JSON: '{filename}'")]
    JsonDeserialize {
        filename: PathBuf,
        source: serde_json::Error,
    },
    #[error("Failed to serialize output")]
    JsonSerialize { source: serde_json::Error },
}
