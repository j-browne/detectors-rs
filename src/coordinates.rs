use nalgebra::{Point2, Point3, Rotation3, Translation3, Vector3};
use rand::Rng;
use val_unc::ValUnc;
use std::ops::Mul;

#[derive(Serialize, Deserialize, Debug, Clone, Copy)]
pub enum CoordinateSystem {
    CartesianZ,
    PolarZ,
}

impl CoordinateSystem {
    pub fn local_to_world(self, p: Point2<f64>) -> Point3<f64> {
        match self {
            CoordinateSystem::CartesianZ => Point3::new(p[0], p[1], 0.0),
            CoordinateSystem::PolarZ => Point3::new(
                p[0] * f64::cos(p[1].to_radians()),
                p[0] * f64::sin(p[1].to_radians()),
                0.0,
            ),
        }
    }

    pub fn world_to_local(self, p: Point3<f64>) -> Point3<f64> {
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

use Transformation::*;

#[derive(Serialize, Deserialize, Clone, Debug)]
pub enum Transformation {
    Rotation((ValUnc, ValUnc, ValUnc)),
    Translation((ValUnc, ValUnc, ValUnc)),
}

impl Transformation {
    pub fn randomize<R: Rng>(&mut self, rng: &mut R) {
        *self = match self {
            Rotation(r) => Rotation((
                r.0.rand(rng),
                r.1.rand(rng),
                r.2.rand(rng)
            )),
            Translation(t) => Translation((
                t.0.rand(rng),
                t.1.rand(rng),
                t.2.rand(rng)
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
                Rotation3::<f64>::new(Vector3::<f64>::new(r.0.val, r.1.val, r.2.val)) * rhs
            }
            Translation(t) => {
                Translation3::<f64>::new(t.0.val, t.1.val, t.2.val) * rhs
            }
        }
    }
}

pub struct InverseTransformation<'a>(&'a Transformation);

impl<'a> Mul<Point3<f64>> for InverseTransformation<'a> {
    type Output = Point3<f64>;

    fn mul(self, rhs: Point3<f64>) -> Self::Output {
        match self {
            InverseTransformation(Rotation(r)) => {
                Rotation3::<f64>::new(Vector3::<f64>::new(r.0.val, r.1.val, r.2.val)).inverse() * rhs
            }
            InverseTransformation(Translation(t)) => {
                Translation3::<f64>::new(t.0.val, t.1.val, t.2.val).inverse() * rhs
            }
        }
    }
}
