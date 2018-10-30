#![feature(custom_attribute)]
#[macro_use]
extern crate serde_derive;
use nalgebra::{Point2, Point3, Rotation3, Translation3, Vector3};
use rgsl::{numerical_differentiation::deriv_central, IntegrationWorkspace};
use serde::{Deserialize, Serialize};
use serde_json;
use std::{collections::HashMap, fs::File, io::BufReader, path::Path};

pub fn get_detectors_from_files<T, U>(
    name_dets: T,
    name_det_types: U,
) -> Result<Vec<Vec<Detector>>, std::io::Error>
where
    T: AsRef<Path>,
    U: AsRef<Path>,
{
    let file = File::open(name_det_types)?;
    let file = BufReader::new(file);
    let detector_types: HashMap<String, DetectorType> = serde_json::from_reader(file)?;
    let file = File::open(name_dets)?;
    let file = BufReader::new(file);
    let detectors: Vec<DetectorBuilder> = serde_json::from_reader(file)?;

    let mut dets = Vec::with_capacity(detectors.len());
    for d in &detectors {
        let t = detector_types
            .get(&d.detector_type)
            .expect("detector type not found");
        let mut strips = Vec::with_capacity(t.strips.len());
        for s in &t.strips {
            let mut trans = Vec::with_capacity(d.trans.len() + t.trans.len() + s.trans.len());
            trans.extend_from_slice(&d.trans);
            trans.extend_from_slice(&t.trans);
            trans.extend_from_slice(&s.trans);
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

    /*
    pub fn th_min(&self) -> f64 {
        use rgsl::MinimizerType::Gauss15;
        let m = Minimizer::new();
    }
    */

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
    #[serde(rename = "transformations")]
    trans: Vec<Transformation>,
}

#[derive(Serialize, Deserialize, Debug)]
struct DetectorType {
    strips: Vec<DetectorStrip>,
    #[serde(default)]
    #[serde(rename = "transformations")]
    trans: Vec<Transformation>,
}

#[derive(Serialize, Deserialize, Debug)]
struct DetectorStrip {
    coords: CoordinateSystem,
    u_lim: (f64, f64),
    v_lim: (f64, f64),
    #[serde(default)]
    #[serde(rename = "transformations")]
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

#[derive(Serialize, Deserialize, Clone, Debug)]
pub enum Transformation {
    #[serde(serialize_with = "serialize_rotation")]
    #[serde(deserialize_with = "deserialize_rotation")]
    Rotation(Rotation3<f64>),
    #[serde(serialize_with = "serialize_translation")]
    #[serde(deserialize_with = "deserialize_translation")]
    Translation(Translation3<f64>),
}
/*
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
*/
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
    let mut iw1 = IntegrationWorkspace::new(100).unwrap();
    let mut iw2 = IntegrationWorkspace::new(100).unwrap();

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
            100,
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
        100,
        Gauss15,
        &mut result,
        &mut error,
    );
    result
}

// FIXME: This doesn't work
//        Instead, make a list of:
//        * the corners
//        * anywhere along the edges where the 1D deriv is 0
//        * anywhere the 2D deriv is 0
//        pick the min from that list
fn minimize_1d<'a>(
    f: &'a Fn(f64) -> f64,
    lim: (f64, f64),
    eps: (f64, f64),
) -> (f64, f64) {
    /*
    let mut candidates = Vec::new(); //TODO: Priority Queue?

    // Add ends
    candidates.push((lim.0, f(lim.0)));
    candidates.push((lim.1, f(lim.1)));

    // Add points where deriv is 0
    //

    candidates.sort_by(|a, b| a.1.partial_cmp(&b.1).expect("unable to compare two candiates in minimize_1d").reverse());
    candidates.pop().expect("unfilled candidates vec in minimize_1d")
    */
    unimplemented!()
}

fn minimize_2d<'a>(
    f_2d: &'a Fn(f64, f64) -> f64,
    u_lim: (f64, f64),
    v_lim: (f64, f64),
    eps: (f64, f64),
) -> ((f64, f64), f64) {
    /*
    let mut candidates = Vec::new(); //TODO: Priority Queue?

    // Add corners
    candidates.push(((u_lim.0, v_lim.0), f_2d(u_lim.0, v_lim.0)));
    candidates.push(((u_lim.0, v_lim.1), f_2d(u_lim.0, v_lim.1)));
    candidates.push(((u_lim.1, v_lim.0), f_2d(u_lim.1, v_lim.0)));
    candidates.push(((u_lim.1, v_lim.1), f_2d(u_lim.1, v_lim.1)));

    // Add minima of Edges, use minimize_1d? Do you need corners then?
    // Add points where 2D deriv is 0

    candidates.sort_by(|a, b| a.1.partial_cmp(&b.1).expect("unable to compare two candiates in minimize_2d").reverse());
    candidates.pop().expect("unfilled candidates vec in minimize_2d")
    */
    unimplemented!()
}

pub trait Spherical {
    fn sphere_r(&self) -> f64;
    fn sphere_th(&self) -> f64;
    fn sphere_phi(&self) -> f64;
    fn sphere(&self) -> (f64, f64, f64);
}

impl Spherical for Point3<f64> {
    fn sphere_r(&self) -> f64 {
        f64::sqrt(self[0].powi(2) + self[1].powi(2) + self[2].powi(2))
    }
    fn sphere_th(&self) -> f64 {
        let r = self.sphere_r();
        f64::acos(self[2] / r).to_degrees()
    }
    fn sphere_phi(&self) -> f64 {
        f64::atan2(self[1], self[0]).to_degrees()
    }

    fn sphere(&self) -> (f64, f64, f64) {
        let r = f64::sqrt(self[0].powi(2) + self[1].powi(2) + self[2].powi(2));
        let th = f64::acos(self[2] / r).to_degrees();
        let phi = f64::atan2(self[0], self[1]).to_degrees();
        (r, th, phi)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_minimize_1d() {
        let f = |x: f64| (x - 2.5).powi(2) + 4.0;
        println!("{:?}", minimize_1d(&f, (1.0, 3.0), (1e-5, 1e-4)));
        println!("{:?}", minimize_1d(&f, (3.0, 10.0), (1e-5, 1e-4)));
    }

    #[test]
    fn test_minimize_2d() {
    }
}
