use crate::{statistics::randomize, unc::ValUnc};
use nalgebra::{Point2, Point3, Rotation3, Translation3, Vector3};
use rand::Rng;
use std::ops::Mul;

#[derive(Serialize, Deserialize, Debug, Clone, Copy)]
pub enum CoordinateSystem {
    CartesianX,
    CartesianY,
    CartesianZ,
    PolarX,
    PolarY,
    PolarZ,
    SphericalX(f64),
    SphericalY(f64),
    SphericalZ(f64),
}

impl CoordinateSystem {
    pub fn local_to_world(self, p: Point2<f64>) -> Point3<f64> {
        match self {
            CoordinateSystem::CartesianX => Point3::new(0.0, p[0], p[1]),
            CoordinateSystem::CartesianY => Point3::new(p[1], 0.0, p[0]),
            CoordinateSystem::CartesianZ => Point3::new(p[0], p[1], 0.0),
            CoordinateSystem::PolarX => Point3::new(
                0.0,
                p[0] * f64::cos(p[1].to_radians()),
                p[0] * f64::sin(p[1].to_radians()),
            ),
            CoordinateSystem::PolarY => Point3::new(
                p[0] * f64::sin(p[1].to_radians()),
                0.0,
                p[0] * f64::cos(p[1].to_radians()),
            ),
            CoordinateSystem::PolarZ => Point3::new(
                p[0] * f64::cos(p[1].to_radians()),
                p[0] * f64::sin(p[1].to_radians()),
                0.0,
            ),
            CoordinateSystem::SphericalX(r) => Point3::new(
                r * f64::cos(p[0].to_radians()),
                r * f64::sin(p[0].to_radians()) * f64::cos(p[1].to_radians()),
                r * f64::sin(p[0].to_radians()) * f64::sin(p[1].to_radians()),
            ),
            CoordinateSystem::SphericalY(r) => Point3::new(
                r * f64::sin(p[0].to_radians()) * f64::sin(p[1].to_radians()),
                r * f64::cos(p[0].to_radians()),
                r * f64::sin(p[0].to_radians()) * f64::cos(p[1].to_radians()),
            ),
            CoordinateSystem::SphericalZ(r) => Point3::new(
                r * f64::sin(p[0].to_radians()) * f64::cos(p[1].to_radians()),
                r * f64::sin(p[0].to_radians()) * f64::sin(p[1].to_radians()),
                r * f64::cos(p[0].to_radians()),
            ),
        }
    }

    pub fn world_to_local(self, p: Point3<f64>) -> Point3<f64> {
        match self {
            CoordinateSystem::CartesianX => Point3::new(p[1], p[2], p[0]),
            CoordinateSystem::CartesianY => Point3::new(p[2], p[0], p[1]),
            CoordinateSystem::CartesianZ => Point3::new(p[0], p[1], p[2]),
            CoordinateSystem::PolarX => {
                let rho = f64::sqrt(p[1].powi(2) + p[2].powi(2));
                let phi = f64::atan2(p[2], p[1]);
                Point3::new(rho, phi.to_degrees(), p[0])
            }
            CoordinateSystem::PolarY => {
                let rho = f64::sqrt(p[2].powi(2) + p[0].powi(2));
                let phi = f64::atan2(p[0], p[2]);
                Point3::new(rho, phi.to_degrees(), p[1])
            }
            CoordinateSystem::PolarZ => {
                let rho = f64::sqrt(p[0].powi(2) + p[1].powi(2));
                let phi = f64::atan2(p[1], p[0]);
                Point3::new(rho, phi.to_degrees(), p[2])
            }
            CoordinateSystem::SphericalX(r) => {
                let theta = f64::acos(p[0] / f64::sqrt(p[1].powi(2) + p[2].powi(2) + p[0].powi(2))).to_degrees();
                let phi = f64::atan2(p[2], p[1]).to_degrees();
                let r_actual = f64::sqrt(p[1].powi(2) + p[2].powi(2) + p[0].powi(2));
                Point3::new(theta, phi, r_actual - r)
            }
            CoordinateSystem::SphericalY(r) => {
                let theta = f64::acos(p[1] / f64::sqrt(p[2].powi(2) + p[0].powi(2) + p[1].powi(2))).to_degrees();
                let phi = f64::atan2(p[0], p[2]).to_degrees();
                let r_actual = f64::sqrt(p[2].powi(2) + p[0].powi(2) + p[1].powi(2));
                Point3::new(theta, phi, r_actual - r)
            }
            CoordinateSystem::SphericalZ(r) => {
                let theta = f64::acos(p[2] / f64::sqrt(p[0].powi(2) + p[1].powi(2) + p[2].powi(2))).to_degrees();
                let phi = f64::atan2(p[1], p[0]).to_degrees();
                let r_actual = f64::sqrt(p[0].powi(2) + p[1].powi(2) + p[2].powi(2));
                Point3::new(theta, phi, r_actual - r)
            }
        }
    }
}

use self::Transformation::*;

#[derive(Serialize, Deserialize, Clone, Debug)]
pub enum Transformation {
    Rotation((ValUnc, ValUnc, ValUnc)),
    Translation((ValUnc, ValUnc, ValUnc)),
}

impl Transformation {
    pub fn randomize<R: Rng>(&mut self, rng: &mut R) {
        *self = match self {
            Rotation(r) => Rotation((
                    randomize(r.0, rng),
                    randomize(r.1, rng),
                    randomize(r.2, rng),
            )),
            Translation(t) => Translation((
                    randomize(t.0, rng),
                    randomize(t.1, rng),
                    randomize(t.2, rng),
            )),
        };
    }

    pub fn inverse(&self) -> InverseTransformation {
        InverseTransformation(self)
    }
}

impl Mul<Point3<f64>> for &Transformation {
    type Output = Point3<f64>;

    fn mul(self, rhs: Point3<f64>) -> Self::Output {
        match self {
            Rotation(r) => {
                Rotation3::<f64>::new(Vector3::<f64>::new(
                    r.0.val.to_radians(),
                    r.1.val.to_radians(),
                    r.2.val.to_radians(),
                )) * rhs
            }
            Translation(t) => Translation3::<f64>::new(t.0.val, t.1.val, t.2.val) * rhs,
        }
    }
}

pub struct InverseTransformation<'a>(&'a Transformation);

impl<'a> Mul<Point3<f64>> for InverseTransformation<'a> {
    type Output = Point3<f64>;

    fn mul(self, rhs: Point3<f64>) -> Self::Output {
        match self {
            InverseTransformation(Rotation(r)) => {
                Rotation3::<f64>::new(Vector3::<f64>::new(
                    r.0.val.to_radians(),
                    r.1.val.to_radians(),
                    r.2.val.to_radians(),
                ))
                .inverse()
                    * rhs
            }
            InverseTransformation(Translation(t)) => {
                Translation3::<f64>::new(t.0.val, t.1.val, t.2.val).inverse() * rhs
            }
        }
    }
}
