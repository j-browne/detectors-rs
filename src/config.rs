use crate::{error::Error, surface::{Surface, MaybeTemplate, NotTemplate}};
use std::{collections::HashMap, io::Read};

#[derive(Serialize, Deserialize, Debug, Default)]
pub struct Config {
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    detectors: Vec<MaybeTemplate>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    shadows: Vec<MaybeTemplate>,
    #[serde(default, skip_serializing_if = "HashMap::is_empty")]
    templates: HashMap<String, NotTemplate>,
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
        self.shadows.append(&mut other.shadows);
        self.templates.extend(other.templates.drain());
    }

    pub fn apply_templates(&mut self) -> Result<(), Error> {
        for s in self.detectors.iter_mut().chain(self.shadows.iter_mut()) {
            s.apply_templates(&self.templates)?;
        }

        Ok(())
    }

    pub fn simplify(self) -> Vec<(Vec<u32>, Surface)> {
        let mut v = Vec::new();
        let mut id = 0;

        for s in self.detectors.iter() {
            v.append(&mut s.simplify(vec![id]));
            id += 1;
        }

        v
    }
}
