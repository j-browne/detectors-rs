//#![feature(custom_attribute)]
use nalgebra::{Point2, Point3, Rotation3, Translation3, Vector3};
use ndarray::{Array, ArrayView1};
use optimize::{Minimizer, NelderMeadBuilder};
use rgsl::{numerical_differentiation::deriv_central, IntegrationWorkspace};
use serde::{Deserialize, Serialize};
use std::{collections::HashMap, io::Read};
pub use crate::error::Error;

pub mod error;

pub fn dets_from_readers<T, U>(
    reader_dets: T,
    reader_det_types: U,
) -> Result<Vec<Vec<Detector>>, Error>
where
    T: Read,
    U: Read,
{
    let detector_types: HashMap<String, DetectorType> = ron::de::from_reader(reader_det_types)?;
    let detectors: Vec<DetectorBuilder> = ron::de::from_reader(reader_dets)?;

    let mut dets = Vec::with_capacity(detectors.len());
    for d in &detectors {
        let t = detector_types
            .get(&d.detector_type)
            .expect("detector type not found");
        let mut strips = Vec::with_capacity(t.strips.len());
        for s in &t.strips {
            let mut trans = Vec::with_capacity(d.trans.len() + t.trans.len() + s.trans.len());
            trans.extend_from_slice(&s.trans);
            trans.extend_from_slice(&t.trans);
            trans.extend_from_slice(&d.trans);
            strips.push(Detector {
                coords: s.coords,
                trans,
                u_lim: s.u_lim,
                v_lim: s.v_lim,
            });
        }
        dets.push(strips);
    }
    Ok(dets)
}

#[derive(Debug, Clone)]
pub struct Detector {
    coords: CoordinateSystem,
    trans: Vec<Transformation>,
    u_lim: (f64, f64),
    v_lim: (f64, f64),
}

impl Detector {
    pub fn coords(&self) -> CoordinateSystem {
        self.coords
    }

    pub fn trans(&self) -> &Vec<Transformation> {
        &self.trans
    }

    pub fn u_lim(&self) -> (f64, f64) {
        self.u_lim
    }

    pub fn v_lim(&self) -> (f64, f64) {
        self.v_lim
    }

    pub fn forward(&self, p: Point2<f64>) -> Point3<f64> {
        let mut res = self.coords.forward(p);
        for t in &self.trans {
            res = match t {
                Transformation::Rotation(t) => t * res,
                Transformation::Translation(t) => t * res,
            };
        }
        res
    }

    pub fn backward(&self, mut p: Point3<f64>) -> Point3<f64> {
        for t in self.trans.iter().rev() {
            p = match t {
                Transformation::Rotation(t) => t.inverse() * p,
                Transformation::Translation(t) => t.inverse() * p,
            };
        }
        self.coords.backward(p)
    }

    pub fn func_min<F>(&self, f: F) -> f64 where F: Fn(Point3<f64>) -> f64 {
        let f = |u, v| { let p = self.forward(Point2::new(u, v)); f(p) };
        minimize_2d(&f, self.u_lim, self.v_lim, 1e-8).1
    }

    pub fn func_max<F>(&self, f: F) -> f64 where F: Fn(Point3<f64>) -> f64 {
        let f = |u, v| { let p = self.forward(Point2::new(u, v)); -f(p) };
        -minimize_2d(&f, self.u_lim, self.v_lim, 1e-8).1
    }

    pub fn func_avg<F>(&self, f: F) -> f64 where F: Fn(Point3<f64>) -> f64 {
        let f = |u, v| { let p2 = Point2::new(u, v); let p3 = self.forward(p2); f(p3) * self.d_solid_angle(p2) };
        integral_2d(&f, self.u_lim, self.v_lim, (1e-8, 1e-5)) / self.solid_angle()
    }

    pub fn d_solid_angle(&self, p: Point2<f64>) -> f64 {
        let fu: fn(f64, &mut (f64, &Detector, usize)) -> f64 =
            |x, params| Detector::forward(params.1, Point2::new(x, params.0))[params.2];
        let mut dr_du = Vector3::new(0.0, 0.0, 0.0);
        let mut dr_du_err = Vector3::new(0.0, 0.0, 0.0);
        deriv_central(
            fu,
            &mut (p[1], self, 0),
            0.0,
            1e-8,
            &mut dr_du[0],
            &mut dr_du_err[0],
        );
        deriv_central(
            fu,
            &mut (p[1], self, 1),
            0.0,
            1e-8,
            &mut dr_du[1],
            &mut dr_du_err[1],
        );
        deriv_central(
            fu,
            &mut (p[1], self, 2),
            0.0,
            1e-8,
            &mut dr_du[2],
            &mut dr_du_err[2],
        );

        let fv: fn(f64, &mut (f64, &Detector, usize)) -> f64 =
            |x, params| Detector::forward(params.1, Point2::new(params.0, x))[params.2];
        let mut dr_dv = Vector3::new(0.0, 0.0, 0.0);
        let mut dr_dv_err = Vector3::new(0.0, 0.0, 0.0);
        deriv_central(
            fv,
            &mut (p[0], self, 0),
            0.0,
            1e-8,
            &mut dr_dv[0],
            &mut dr_dv_err[0],
        );
        deriv_central(
            fv,
            &mut (p[0], self, 1),
            0.0,
            1e-8,
            &mut dr_dv[1],
            &mut dr_dv_err[1],
        );
        deriv_central(
            fv,
            &mut (p[0], self, 2),
            0.0,
            1e-8,
            &mut dr_dv[2],
            &mut dr_dv_err[2],
        );

        // Surface area
        // dS = dr/du x dr/dv
        let ds = dr_du.cross(&dr_dv);
        let r = self.forward(p).coords;

        // Solid angle
        // dΩ = r.dS / |r|^3
        // Take the abs because whether the surface points in or out doesn't matter
        r.dot(&ds).abs() / r.norm().powi(3)
    }

    pub fn solid_angle(&self) -> f64 {
        let f = |u, v| self.d_solid_angle(Point2::new(u, v));
        integral_2d(&f, self.u_lim, self.v_lim, (1e-8, 1e-5))
    }
}

#[derive(Serialize, Deserialize, Debug)]
struct DetectorBuilder {
    #[serde(rename = "type")]
    detector_type: String,
    #[serde(default, rename = "transformations", skip_serializing_if = "Vec::is_empty")]
    trans: Vec<Transformation>,
}

#[derive(Serialize, Deserialize, Debug)]
struct DetectorType {
    strips: Vec<DetectorStrip>,
    #[serde(default, rename = "transformations", skip_serializing_if = "Vec::is_empty")]
    trans: Vec<Transformation>,
}

#[derive(Serialize, Deserialize, Debug)]
struct DetectorStrip {
    coords: CoordinateSystem,
    u_lim: (f64, f64),
    v_lim: (f64, f64),
    #[serde(default, rename = "transformations", skip_serializing_if = "Vec::is_empty")]
    trans: Vec<Transformation>,
}

#[derive(Serialize, Deserialize, Debug, Clone, Copy)]
pub enum CoordinateSystem {
    CartesianZ,
    PolarZ,
}

impl CoordinateSystem {
    pub fn forward(&self, p: Point2<f64>) -> Point3<f64> {
        match self {
            CoordinateSystem::CartesianZ => Point3::new(p[0], p[1], 0.0),
            CoordinateSystem::PolarZ => Point3::new(
                p[0] * f64::cos(p[1].to_radians()),
                p[0] * f64::sin(p[1].to_radians()),
                0.0,
            ),
        }
    }

    pub fn backward(&self, p: Point3<f64>) -> Point3<f64> {
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
    #[serde(serialize_with = "serialize_rotation", deserialize_with = "deserialize_rotation")]
    Rotation(Rotation3<f64>),
    #[serde(serialize_with = "serialize_translation",deserialize_with = "deserialize_translation")]
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
                );
            }
            Transformation::Translation(t) => {
                let t = t.vector.data;
                write!(f, "Translation({}, {}, {})", t[0], t[1], t[2]);
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

fn integral_2d<'a>(
    f: &'a Fn(f64, f64) -> f64,
    u_lim: (f64, f64),
    v_lim: (f64, f64),
    eps: (f64, f64),
) -> f64 {
    use rgsl::GaussKonrodRule::Gauss15;
    let mut iw1 = IntegrationWorkspace::new(200).unwrap();
    let mut iw2 = IntegrationWorkspace::new(200).unwrap();

    let fu: fn(f64, &mut (&'a Fn(f64, f64) -> f64, f64)) -> f64 = |x, params| params.0(x, params.1);
    let fv: fn(
        f64,
        &mut (&'a Fn(f64, f64) -> f64, _, (f64, f64), (f64, f64), &mut IntegrationWorkspace),
    ) -> f64 = |x, params| {
        let mut result = 0.0;
        let mut error = 0.0;
        params.4.qag(
            params.1,
            &mut (params.0, x),
            (params.2).0,
            (params.2).1,
            (params.3).0,
            (params.3).1,
            200,
            Gauss15,
            &mut result,
            &mut error,
        );
        result
    };

    let mut result = 0.0;
    let mut error = 0.0;
    let mut params = (f, fu, u_lim, eps, &mut iw2);
    iw1.qag(
        fv,
        &mut params,
        v_lim.0,
        v_lim.1,
        eps.0,
        eps.1,
        200,
        Gauss15,
        &mut result,
        &mut error,
    );
    result
}

pub fn minimize_1d<'a>(
    f: &'a Fn(f64) -> f64,
    lim: (f64, f64),
    eps: f64,
) -> (f64, f64) {
    let mut candidates = Vec::new();

    let minimizer = NelderMeadBuilder::default()
        .adaptive(true)
        .ftol(eps)
        .build()
        .unwrap();// FIXME

    let func = |x: ArrayView1<f64>| if x[0] < lim.0 || x[0] > lim.1 { std::f64::INFINITY } else { f(x[0]) };

    for u in [lim.0, (lim.0 + lim.1) / 2.0, lim.1].into_iter() {
        let x = minimizer.minimize(func, Array::from_vec(vec![*u]).view());

        candidates.push(x);
    }

    candidates.sort_by(|a, b| f(a[0]).partial_cmp(&f(b[0])).unwrap().reverse()); //FIXME

    let min = candidates.pop().unwrap(); // FIXME
    (min[0], f(min[0]))
}

pub fn minimize_2d<'a>(
    f: &'a Fn(f64, f64) -> f64,
    u_lim: (f64, f64),
    v_lim: (f64, f64),
    eps: f64,
) -> ((f64, f64), f64) {
    let mut candidates = Vec::new();

    let minimizer = NelderMeadBuilder::default()
        .adaptive(true)
        .ftol(eps)
        .build()
        .unwrap();// FIXME

    let func = |x: ArrayView1<f64>| if x[0] < u_lim.0 || x[0] > u_lim.1 || x[1] < v_lim.0 || x[1] > v_lim.1 { std::f64::INFINITY } else { f(x[0], x[1]) };

    for u in [u_lim.0, (u_lim.0 + u_lim.1) / 2.0, u_lim.1].into_iter() {
        for v in [v_lim.0, (v_lim.0 + v_lim.1) / 2.0, v_lim.1].into_iter() {
            let x = minimizer.minimize(func, Array::from_vec(vec![*u, *v]).view());

            candidates.push(x);
        }
    }

    candidates.sort_by(|a, b| f(a[0], a[1]).partial_cmp(&f(b[0], b[1])).unwrap().reverse()); //FIXME

    let min = candidates.pop().unwrap(); // FIXME
    ((min[0], min[1]), f(min[0], min[1]))
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
