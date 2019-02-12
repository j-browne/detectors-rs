use crate::{
    error::Error,
    surface::{MaybeTemplate, NotTemplate, Surface},
};
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

type D = (Vec<u32>, Surface);
type S = (Vec<u32>, Surface);
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

    pub fn apply_templates(&self) -> Result<(Vec<NotTemplate>, Vec<NotTemplate>), Error> {
        let mut detectors = Vec::new();
        let mut shadows = Vec::new();

        for s in self.detectors.iter() {
            detectors.push(s.apply_templates(&self.templates)?);
        }

        for s in self.shadows.iter() {
            shadows.push(s.apply_templates(&self.templates)?);
        }

        Ok((detectors, shadows))
    }

    pub fn simplify(self) -> Result<(Vec<D>, Vec<S>), Error> {
        let mut simplified_detectors = Vec::new();
        let (detectors, shadows) = self.apply_templates()?;
        for (id, s) in detectors.iter().enumerate() {
            simplified_detectors.append(&mut s.simplify(vec![id as u32]));
        }

        let mut simplified_shadows = Vec::new();
        for (id, s) in shadows.iter().enumerate() {
            simplified_shadows.append(&mut s.simplify(vec![id as u32]));
        }

        Ok((simplified_detectors, simplified_shadows))
    }
}
