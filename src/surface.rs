use crate::{CoordinateSystem, Transformation};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

#[derive(Serialize, Deserialize, Debug)]
pub struct Config {
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    detectors: Vec<Surface>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    shadows: Vec<Surface>,
    #[serde(default, skip_serializing_if = "HashMap::is_empty")]
    templates: HashMap<String, Surface>,
}

#[derive(Serialize, Deserialize, Debug)]
pub enum Surface {
    Base {
        coords: CoordinateSystem,
        u_limits: (f64, f64),
        v_limits: (f64, f64),
        #[serde(default, rename = "transformations", skip_serializing_if = "Vec::is_empty")]
        trans: Vec<Transformation>,
    },
    Group {
        surfaces: Vec<Surface>,
        #[serde(default, rename = "transformations", skip_serializing_if = "Vec::is_empty")]
        trans: Vec<Transformation>,
    },
    Template {
        name: String,
        #[serde(default, rename = "transformations", skip_serializing_if = "Vec::is_empty")]
        trans: Vec<Transformation>,
    },
}
