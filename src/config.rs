use crate::{
    detector::{Detector, Raw as DetectorRaw},
    error::Error,
    surface::MaybeTemplate,
};
use std::{collections::HashMap, io::Read};

#[derive(Serialize, Deserialize, Debug, Default)]
#[serde(deny_unknown_fields)]
pub struct Config {
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    detectors: Vec<DetectorRaw>,
    #[serde(default, skip_serializing_if = "HashMap::is_empty")]
    templates: HashMap<String, MaybeTemplate>,
}

impl Config {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn add_from_reader<R: Read>(&mut self, reader: R) -> Result<(), Error> {
        let other: Self = serde_json::from_reader(reader)?;
        self.append(other);
        Ok(())
    }

    pub fn append(&mut self, mut other: Self) {
        self.detectors.append(&mut other.detectors);
        self.templates.extend(other.templates.drain());
    }

    pub fn simplify(self) -> Result<Vec<(Vec<u32>, Detector)>, Error> {
        let Config {
            detectors,
            templates,
            ..
        } = self;
        Ok(detectors
            .into_iter()
            .enumerate()
            .map(|(id, d)| d.simplify(&templates, vec![id as u32]))
            .collect::<Result<Vec<_>, _>>()?
            .into_iter()
            .flatten()
            .collect::<Vec<_>>())
    }
}
