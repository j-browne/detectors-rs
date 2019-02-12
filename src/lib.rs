#![feature(custom_attribute)]
#[macro_use]
extern crate serde_derive;

pub use crate::error::Error;
use nalgebra::{Point2, Point3, Rotation3, Translation3, Vector3};
use serde::{Deserialize, Serialize};

pub mod config;
pub mod error;
pub mod surface;

#[derive(Serialize, Deserialize, Debug, Clone, Copy)]
pub enum CoordinateSystem {
    CartesianZ,
    PolarZ,
}

impl CoordinateSystem {
    pub fn local_to_world(&self, p: Point2<f64>) -> Point3<f64> {
        match self {
            CoordinateSystem::CartesianZ => Point3::new(p[0], p[1], 0.0),
            CoordinateSystem::PolarZ => Point3::new(
                p[0] * f64::cos(p[1].to_radians()),
                p[0] * f64::sin(p[1].to_radians()),
                0.0,
            ),
        }
    }

    pub fn world_to_local(&self, p: Point3<f64>) -> Point3<f64> {
        match self {
            CoordinateSystem::CartesianZ => p,
            CoordinateSystem::PolarZ => {
                let rho = f64::sqrt(p[0].powi(2) + p[1].powi(2));
                let phi = f64::atan2(p[1], p[0]);
                Point3::new(rho, phi.to_degrees(), p[2])
            }
        }
    }
}

#[derive(Serialize, Deserialize, Clone)]
pub enum Transformation {
    #[serde(
        serialize_with = "serialize_rotation",
        deserialize_with = "deserialize_rotation"
    )]
    Rotation(Rotation3<f64>),
    #[serde(
        serialize_with = "serialize_translation",
        deserialize_with = "deserialize_translation"
    )]
    Translation(Translation3<f64>),
}

impl std::fmt::Debug for Transformation {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        match self {
            Transformation::Rotation(r) => {
                let r = r.scaled_axis().data;
                write!(
                    f,
                    "Rotation({}, {}, {})",
                    r[0].to_degrees(),
                    r[1].to_degrees(),
                    r[2].to_degrees()
                )?;
            }
            Transformation::Translation(t) => {
                let t = t.vector.data;
                write!(f, "Translation({}, {}, {})", t[0], t[1], t[2])?;
            }
        }
        Ok(())
    }
}

fn serialize_rotation<S: serde::Serializer>(r: &Rotation3<f64>, s: S) -> Result<S::Ok, S::Error> {
    let r = r.scaled_axis().data;
    (r[0].to_degrees(), r[1].to_degrees(), r[2].to_degrees()).serialize(s)
}

fn deserialize_rotation<'de, D: serde::Deserializer<'de>>(
    d: D,
) -> Result<Rotation3<f64>, D::Error> {
    let tup = <(f64, f64, f64)>::deserialize(d)?;
    Ok(Rotation3::new(Vector3::new(
        tup.0.to_radians(),
        tup.1.to_radians(),
        tup.2.to_radians(),
    )))
}

fn serialize_translation<S: serde::Serializer>(
    t: &Translation3<f64>,
    s: S,
) -> Result<S::Ok, S::Error> {
    let t = t.vector.data;
    (t[0], t[1], t[2]).serialize(s)
}

fn deserialize_translation<'de, D: serde::Deserializer<'de>>(
    d: D,
) -> Result<Translation3<f64>, D::Error> {
    let tup = <(f64, f64, f64)>::deserialize(d)?;
    Ok(Translation3::new(tup.0, tup.1, tup.2))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_minimize_1d() {
        let mut output = Vec::new();

        let f = |x: f64| (x - 2.5).powi(2) + 4.0;

        for ulim in [(-1.0, 1.0), (1.0, 3.0), (3.0, 10.0)].into_iter() {
            output.push((ulim, minimize_1d(&f, *ulim, 1e-4)));
        }

        for o in output {
            println!("{:?}: {:?}", o.0, o.1);
        }
    }

    #[test]
    fn test_minimize_2d() {
        let mut output = Vec::new();

        let f = |x: f64, y: f64| (x - 2.5).powi(2) + (y + 1.25).powi(2) + 4.0;

        for ulim in [(-1.0, 1.0), (1.0, 3.0), (3.0, 10.0)].into_iter() {
            for vlim in [(-10.0, -2.0), (-2.0, -1.0), (-1.0, 10.0)].into_iter() {
                output.push(((ulim, vlim), minimize_2d(&f, *ulim, *vlim, 1e-6)));
            }
        }

        for o in output {
            println!("{:?}: {:?}", o.0, o.1);
        }
    }
}
