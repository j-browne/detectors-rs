use crate::{CoordinateSystem, Detector, Transformation};
use serde::{Deserialize, Serialize};
use std::{
    collections::HashMap,
    io::Read,
};

#[derive(Serialize, Deserialize, Debug, Default)]
pub struct Config {
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    detectors: Vec<Surface>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    shadows: Vec<Surface>,
    #[serde(default, skip_serializing_if = "HashMap::is_empty")]
    templates: HashMap<String, Surface>,
}

impl Config {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn add_from_reader<R: Read>(&mut self, reader: R) -> Result<(), Box<std::error::Error>> {
        let other: Self = ron::de::from_reader(reader)?;
        self.append(other);
        Ok(())
    }

    pub fn append(&mut self, mut other: Self) {
        self.detectors.append(&mut other.detectors);
        self.shadows.append(&mut other.shadows);
        self.templates.extend(other.templates.drain());
    }

    pub fn apply_templates(&mut self) {
        for _det in &mut self.detectors {
        }
    }

    pub fn to_detectors(self) -> Vec<Vec<Detector>> {
        Vec::new()
    }
}

#[derive(Serialize, Deserialize, Debug)]
pub struct BaseSurface {
    coords: CoordinateSystem,
    u_limits: (f64, f64),
    v_limits: (f64, f64),
    #[serde(default, rename = "transformations", skip_serializing_if = "Vec::is_empty")]
    trans: Vec<Transformation>,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct GroupSurface {
    surfaces: Vec<Surface>,
    #[serde(default, rename = "transformations", skip_serializing_if = "Vec::is_empty")]
    trans: Vec<Transformation>,
}

#[derive(Serialize, Deserialize, Debug)]
pub enum NonTemplateSurface {
    Base(BaseSurface),
    Group(GroupSurface),
}

#[derive(Serialize, Deserialize, Debug)]
pub enum Surface {
    Base(BaseSurface),
    Group(GroupSurface),
    Template {
        name: String,
        surface: Box<Surface>, 
    },
}
