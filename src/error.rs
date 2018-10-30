#[derive(Debug)]
pub enum Error {
    Io(std::io::Error),
    Ron(ron::de::Error),
}

impl From<std::io::Error> for Error {
    fn from(e: std::io::Error) -> Self {
        Error::Io(e)
    }
}

impl From<ron::de::Error> for Error {
    fn from(e: ron::de::Error) -> Self {
        Error::Ron(e)
    }
}
