#[derive(Debug)]
pub enum Error {
    UnknownTemplate,
    Io(std::io::Error),
    Json(serde_json::Error),
}

impl From<std::io::Error> for Error {
    fn from(e: std::io::Error) -> Self {
        Error::Io(e)
    }
}

impl From<serde_json::Error> for Error {
    fn from(e: serde_json::Error) -> Self {
        Error::Json(e)
    }
}

impl std::fmt::Display for Error {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f, "detectors-rs error")
    }
}

impl std::error::Error for Error {
    fn description(&self) -> &str {
        "detectors-rs error"
    }

    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Error::Io(e) => Some(e),
            Error::Json(e) => Some(e),
            _ => None,
        }
    }
}
