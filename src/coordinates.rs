use crate::{statistics::randomize, unc::ValUnc};
use nalgebra::{Point2, Point3, Rotation3, Translation3, Vector3};
use rand::Rng;
use std::ops::Mul;

#[non_exhaustive]
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
            CoordinateSystem::CartesianX => p.yzx(),
            CoordinateSystem::CartesianY => p.zxy(),
            CoordinateSystem::CartesianZ => p.xyz(),
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
                let theta = f64::acos(p[0] / f64::sqrt(p[1].powi(2) + p[2].powi(2) + p[0].powi(2)))
                    .to_degrees();
                let phi = f64::atan2(p[2], p[1]).to_degrees();
                let r_actual = f64::sqrt(p[1].powi(2) + p[2].powi(2) + p[0].powi(2));
                Point3::new(theta, phi, r_actual - r)
            }
            CoordinateSystem::SphericalY(r) => {
                let theta = f64::acos(p[1] / f64::sqrt(p[2].powi(2) + p[0].powi(2) + p[1].powi(2)))
                    .to_degrees();
                let phi = f64::atan2(p[0], p[2]).to_degrees();
                let r_actual = f64::sqrt(p[2].powi(2) + p[0].powi(2) + p[1].powi(2));
                Point3::new(theta, phi, r_actual - r)
            }
            CoordinateSystem::SphericalZ(r) => {
                let theta = f64::acos(p[2] / f64::sqrt(p[0].powi(2) + p[1].powi(2) + p[2].powi(2)))
                    .to_degrees();
                let phi = f64::atan2(p[1], p[0]).to_degrees();
                let r_actual = f64::sqrt(p[0].powi(2) + p[1].powi(2) + p[2].powi(2));
                Point3::new(theta, phi, r_actual - r)
            }
        }
    }

    /// src and dest are in the Cartesian version of the local coordinates (i.e. all the
    /// transformations are reversed, but the transformation into 2D local coordinates is not
    /// applied yet).
    ///
    /// The intersection is found in these Cartesian coordinates and then that point is converted
    /// into the 2D local coordinates. This is necessary, for example, for circular coordinates,
    /// in which wrapping causes interpolation to fail (e.g. 180 degrees is not the midpoint
    /// between 350 degrees and 10 degrees, 0 degrees is).
    ///
    /// NOTE: Spherical coordinates are not implemented yet
    pub(crate) fn intersects(
        &self,
        src: Point3<f64>,
        dest: Point3<f64>,
        u_limits: (f64, f64),
        v_limits: (f64, f64),
    ) -> bool {
        match self {
            CoordinateSystem::CartesianX => {
                CoordinateSystem::CartesianZ.intersects(src.yzx(), dest.yzx(), u_limits, v_limits)
            }
            CoordinateSystem::CartesianY => {
                CoordinateSystem::CartesianZ.intersects(src.zxy(), dest.zxy(), u_limits, v_limits)
            }
            CoordinateSystem::CartesianZ => {
                let int_w = Point3::new(
                    -(src.x - dest.x) / (src.z - dest.z) * src.z + src.x,
                    -(src.y - dest.y) / (src.z - dest.z) * src.z + src.y,
                    0.0,
                );
                let int_l = self.world_to_local(int_w);

                let is_in_u = int_l.x >= u_limits.0 && int_l.x <= u_limits.1;
                let is_in_v = int_l.y >= v_limits.0 && int_l.y <= v_limits.1;
                // whether src.z and dest.z are opposite signs
                let is_between =
                    (src.z.signum() * dest.z.signum() - -1.0).abs() < std::f64::EPSILON;

                is_in_u && is_in_v && is_between
            }
            CoordinateSystem::PolarX => {
                CoordinateSystem::PolarZ.intersects(src.yzx(), dest.yzx(), u_limits, v_limits)
            }
            CoordinateSystem::PolarY => {
                CoordinateSystem::PolarZ.intersects(src.zxy(), dest.zxy(), u_limits, v_limits)
            }
            CoordinateSystem::PolarZ => {
                let int_w = Point3::new(
                    -(src.x - dest.x) / (src.z - dest.z) * src.z + src.x,
                    -(src.y - dest.y) / (src.z - dest.z) * src.z + src.y,
                    0.0,
                );
                let int_l = self.world_to_local(int_w);

                let is_in_u = int_l.x >= u_limits.0 && int_l.x <= u_limits.1;
                let is_in_v = int_l.y >= v_limits.0 && int_l.y <= v_limits.1;
                // whether src.z and dest.z are opposite signs
                let is_between =
                    (src.z.signum() * dest.z.signum() - -1.0).abs() < std::f64::EPSILON;

                is_in_u && is_in_v && is_between
            }
            CoordinateSystem::SphericalX(_r) => unimplemented!(),
            CoordinateSystem::SphericalY(_r) => unimplemented!(),
            CoordinateSystem::SphericalZ(_r) => unimplemented!(),
        }
    }
}

#[non_exhaustive]
#[derive(Serialize, Deserialize, Clone, Debug)]
pub enum Transformation {
    Rotation((ValUnc, ValUnc, ValUnc)),
    Translation((ValUnc, ValUnc, ValUnc)),
}

impl Transformation {
    pub fn randomize<R: Rng>(&mut self, rng: &mut R) {
        use Transformation::*;

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
        use Transformation::*;

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
        use Transformation::*;

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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_intersects_cartesian_x() {
        let coords = CoordinateSystem::CartesianX;
        let u_lim = (-3.0, 3.0);
        let v_lim = (-3.0, 3.0);

        let src = Point3::new(-1.0, 2.0, 0.0);
        let dest = Point3::new(1.0, 2.0, 0.0);
        assert!(coords.intersects(src, dest, u_lim, v_lim));

        let src = Point3::new(-1.0, -2.0, 0.0);
        let dest = Point3::new(1.0, -2.0, 0.0);
        assert!(coords.intersects(src, dest, u_lim, v_lim));

        let src = Point3::new(-1.0, 0.0, 2.0);
        let dest = Point3::new(1.0, 0.0, 2.0);
        assert!(coords.intersects(src, dest, u_lim, v_lim));

        let src = Point3::new(-1.0, 0.0, -2.0);
        let dest = Point3::new(1.0, 0.0, -2.0);
        assert!(coords.intersects(src, dest, u_lim, v_lim));

        let src = Point3::new(-1.0, 4.0, 0.0);
        let dest = Point3::new(1.0, 4.0, 0.0);
        assert!(!coords.intersects(src, dest, u_lim, v_lim));

        let src = Point3::new(1.0, -1.0, 0.0);
        let dest = Point3::new(1.0, 1.0, 0.0);
        assert!(!coords.intersects(src, dest, u_lim, v_lim));

        let src = Point3::new(1.0, 0.0, -1.0);
        let dest = Point3::new(1.0, 0.0, 1.0);
        assert!(!coords.intersects(src, dest, u_lim, v_lim));
    }

    #[test]
    fn test_intersects_cartesian_y() {
        let coords = CoordinateSystem::CartesianY;
        let u_lim = (-3.0, 3.0);
        let v_lim = (-3.0, 3.0);

        let src = Point3::new(2.0, -1.0, 0.0);
        let dest = Point3::new(2.0, 1.0, 0.0);
        assert!(coords.intersects(src, dest, u_lim, v_lim));

        let src = Point3::new(-2.0, -1.0, 0.0);
        let dest = Point3::new(-2.0, 1.0, 0.0);
        assert!(coords.intersects(src, dest, u_lim, v_lim));

        let src = Point3::new(0.0, -1.0, 2.0);
        let dest = Point3::new(0.0, 1.0, 2.0);
        assert!(coords.intersects(src, dest, u_lim, v_lim));

        let src = Point3::new(0.0, -1.0, -2.0);
        let dest = Point3::new(0.0, 1.0, -2.0);
        assert!(coords.intersects(src, dest, u_lim, v_lim));

        let src = Point3::new(4.0, -1.0, 0.0);
        let dest = Point3::new(4.0, 1.0, 0.0);
        assert!(!coords.intersects(src, dest, u_lim, v_lim));

        let src = Point3::new(-1.0, 1.0, 0.0);
        let dest = Point3::new(1.0, 1.0, 0.0);
        assert!(!coords.intersects(src, dest, u_lim, v_lim));

        let src = Point3::new(0.0, 1.0, -1.0);
        let dest = Point3::new(0.0, 1.0, 1.0);
        assert!(!coords.intersects(src, dest, u_lim, v_lim));
    }

    #[test]
    fn test_intersects_cartesian_z() {
        let coords = CoordinateSystem::CartesianZ;
        let u_lim = (-3.0, 3.0);
        let v_lim = (-3.0, 3.0);

        let src = Point3::new(2.0, 0.0, -1.0);
        let dest = Point3::new(2.0, 0.0, 1.0);
        assert!(coords.intersects(src, dest, u_lim, v_lim));

        let src = Point3::new(-2.0, 0.0, -1.0);
        let dest = Point3::new(-2.0, 0.0, 1.0);
        assert!(coords.intersects(src, dest, u_lim, v_lim));

        let src = Point3::new(0.0, 2.0, -1.0);
        let dest = Point3::new(0.0, 2.0, 1.0);
        assert!(coords.intersects(src, dest, u_lim, v_lim));

        let src = Point3::new(0.0, -2.0, -1.0);
        let dest = Point3::new(0.0, -2.0, 1.0);
        assert!(coords.intersects(src, dest, u_lim, v_lim));

        let src = Point3::new(4.0, 0.0, -1.0);
        let dest = Point3::new(4.0, 0.0, 1.0);
        assert!(!coords.intersects(src, dest, u_lim, v_lim));

        let src = Point3::new(-1.0, 0.0, 1.0);
        let dest = Point3::new(1.0, 0.0, 1.0);
        assert!(!coords.intersects(src, dest, u_lim, v_lim));

        let src = Point3::new(0.0, -1.0, 1.0);
        let dest = Point3::new(0.0, 1.0, 1.0);
        assert!(!coords.intersects(src, dest, u_lim, v_lim));
    }

    #[test]
    fn test_intersects_polar_x() {
        let coords = CoordinateSystem::PolarX;
        let u_lim = (1.0, 3.0);
        let v_lim = (-180.0, 180.0);

        let src = Point3::new(-1.0, 2.0, 0.0);
        let dest = Point3::new(1.0, 2.0, 0.0);
        assert!(coords.intersects(src, dest, u_lim, v_lim));

        let src = Point3::new(-1.0, -2.0, 0.0);
        let dest = Point3::new(1.0, -2.0, 0.0);
        assert!(coords.intersects(src, dest, u_lim, v_lim));

        let src = Point3::new(-1.0, 0.0, 2.0);
        let dest = Point3::new(1.0, 0.0, 2.0);
        assert!(coords.intersects(src, dest, u_lim, v_lim));

        let src = Point3::new(-1.0, 0.0, -2.0);
        let dest = Point3::new(1.0, 0.0, -2.0);
        assert!(coords.intersects(src, dest, u_lim, v_lim));

        let src = Point3::new(-1.0, 0.0, 0.0);
        let dest = Point3::new(1.0, 0.0, 0.0);
        assert!(!coords.intersects(src, dest, u_lim, v_lim));

        let src = Point3::new(-1.0, 2.0, 2.0);
        let dest = Point3::new(1.0, -2.0, -2.0);
        assert!(!coords.intersects(src, dest, u_lim, v_lim));

        let src = Point3::new(-1.0, 4.0, 0.0);
        let dest = Point3::new(1.0, 4.0, 0.0);
        assert!(!coords.intersects(src, dest, u_lim, v_lim));

        let src = Point3::new(1.0, -1.0, 0.0);
        let dest = Point3::new(1.0, 1.0, 0.0);
        assert!(!coords.intersects(src, dest, u_lim, v_lim));

        let src = Point3::new(1.0, 0.0, -1.0);
        let dest = Point3::new(1.0, 0.0, 1.0);
        assert!(!coords.intersects(src, dest, u_lim, v_lim));
    }

    #[test]
    fn test_intersects_polar_y() {
        let coords = CoordinateSystem::PolarY;
        let u_lim = (1.0, 3.0);
        let v_lim = (-180.0, 180.0);

        let src = Point3::new(2.0, -1.0, 0.0);
        let dest = Point3::new(2.0, 1.0, 0.0);
        assert!(coords.intersects(src, dest, u_lim, v_lim));

        let src = Point3::new(-2.0, -1.0, 0.0);
        let dest = Point3::new(-2.0, 1.0, 0.0);
        assert!(coords.intersects(src, dest, u_lim, v_lim));

        let src = Point3::new(0.0, -1.0, 2.0);
        let dest = Point3::new(0.0, 1.0, 2.0);
        assert!(coords.intersects(src, dest, u_lim, v_lim));

        let src = Point3::new(0.0, -1.0, -2.0);
        let dest = Point3::new(0.0, 1.0, -2.0);
        assert!(coords.intersects(src, dest, u_lim, v_lim));

        let src = Point3::new(0.0, -1.0, 0.0);
        let dest = Point3::new(0.0, 1.0, 0.0);
        assert!(!coords.intersects(src, dest, u_lim, v_lim));

        let src = Point3::new(2.0, -1.0, 2.0);
        let dest = Point3::new(-2.0, 1.0, -2.0);
        assert!(!coords.intersects(src, dest, u_lim, v_lim));

        let src = Point3::new(4.0, -1.0, 0.0);
        let dest = Point3::new(4.0, 1.0, 0.0);
        assert!(!coords.intersects(src, dest, u_lim, v_lim));

        let src = Point3::new(-1.0, 1.0, 0.0);
        let dest = Point3::new(1.0, 1.0, 0.0);
        assert!(!coords.intersects(src, dest, u_lim, v_lim));

        let src = Point3::new(0.0, 1.0, -1.0);
        let dest = Point3::new(0.0, 1.0, 1.0);
        assert!(!coords.intersects(src, dest, u_lim, v_lim));
    }

    #[test]
    fn test_intersects_polar_z() {
        let coords = CoordinateSystem::PolarZ;
        let u_lim = (1.0, 3.0);
        let v_lim = (-180.0, 180.0);

        let src = Point3::new(2.0, 0.0, -1.0);
        let dest = Point3::new(2.0, 0.0, 1.0);
        assert!(coords.intersects(src, dest, u_lim, v_lim));

        let src = Point3::new(-2.0, 0.0, -1.0);
        let dest = Point3::new(-2.0, 0.0, 1.0);
        assert!(coords.intersects(src, dest, u_lim, v_lim));

        let src = Point3::new(0.0, 2.0, -1.0);
        let dest = Point3::new(0.0, 2.0, 1.0);
        assert!(coords.intersects(src, dest, u_lim, v_lim));

        let src = Point3::new(0.0, -2.0, -1.0);
        let dest = Point3::new(0.0, -2.0, 1.0);
        assert!(coords.intersects(src, dest, u_lim, v_lim));

        let src = Point3::new(0.0, 0.0, -1.0);
        let dest = Point3::new(0.0, 0.0, 1.0);
        assert!(!coords.intersects(src, dest, u_lim, v_lim));

        let src = Point3::new(2.0, 2.0, -1.0);
        let dest = Point3::new(-2.0, -2.0, 1.0);
        assert!(!coords.intersects(src, dest, u_lim, v_lim));

        let src = Point3::new(4.0, 0.0, -1.0);
        let dest = Point3::new(4.0, 0.0, 1.0);
        assert!(!coords.intersects(src, dest, u_lim, v_lim));

        let src = Point3::new(-1.0, 0.0, 1.0);
        let dest = Point3::new(1.0, 0.0, 1.0);
        assert!(!coords.intersects(src, dest, u_lim, v_lim));

        let src = Point3::new(0.0, -1.0, 1.0);
        let dest = Point3::new(0.0, 1.0, 1.0);
        assert!(!coords.intersects(src, dest, u_lim, v_lim));
    }
}
