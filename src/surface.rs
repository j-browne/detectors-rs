use crate::{error::Error, CoordinateSystem, Transformation};
use nalgebra::{Point2, Point3, Vector3};
use ndarray::{Array, ArrayView1};
use optimize::{Minimizer, NelderMeadBuilder};
use rgsl::{numerical_differentiation::deriv_central, IntegrationWorkspace};
use std::collections::HashMap;

pub type Surface = SurfaceBase;

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct SurfaceBase {
    coords: CoordinateSystem,
    u_limits: (f64, f64),
    v_limits: (f64, f64),
    #[serde(
        default,
        rename = "transformations",
        skip_serializing_if = "Vec::is_empty"
    )]
    trans: Vec<Transformation>,
}

impl SurfaceBase {
    pub fn trans(&self) -> &Vec<Transformation> {
        &self.trans
    }

    pub fn trans_mut(&mut self) -> &mut Vec<Transformation> {
        &mut self.trans
    }

    pub fn add_trans(&mut self, t: &mut Vec<Transformation>) {
        self.trans_mut().append(t);
    }

    pub fn simplify(&self, id: Vec<u32>) -> (Vec<u32>, Surface) {
        (id, self.clone())
    }

    pub fn coords_local_to_world(&self, p: Point2<f64>) -> Point3<f64> {
        let mut res = self.coords.local_to_world(p);
        for t in &self.trans {
            res = match t {
                Transformation::Rotation(t) => t * res,
                Transformation::Translation(t) => t * res,
            };
        }
        res
    }

    pub fn coords_world_to_local(&self, mut p: Point3<f64>) -> Point3<f64> {
        for t in self.trans.iter().rev() {
            p = match t {
                Transformation::Rotation(t) => t.inverse() * p,
                Transformation::Translation(t) => t.inverse() * p,
            };
        }
        self.coords.world_to_local(p)
    }

    // TODO: If the two z values are the same you can get inf.
    pub fn intersects(&self, p_src_world: Point3<f64>, p_dest_world: Point3<f64>) -> bool {
        let p1 = self.coords_world_to_local(p_dest_world);
        let p2 = self.coords_world_to_local(p_src_world);

        let int = Point2::new(
            -(p1.x - p2.x) / (p1.z - p2.z) * p1.z + p1.x,
            -(p1.y - p2.y) / (p1.z - p2.z) * p1.z + p1.y,
        );

        // Make sure the point is within u and v limits
        //  AND the surface is between src and dest
        int.x >= self.u_limits.0
            && int.x <= self.u_limits.1
            && int.y >= self.v_limits.0
            && int.y <= self.v_limits.1
            && p1.z.signum() * p2.z.signum() == -1.0
    }

    pub fn func_min<F>(&self, f: F) -> f64
    where
        F: Fn(Point3<f64>) -> f64,
    {
        let f = |u, v| {
            let p = self.coords_local_to_world(Point2::new(u, v));
            f(p)
        };
        minimize_2d(&f, self.u_limits, self.v_limits, 1e-8).1
    }

    pub fn func_max<F>(&self, f: F) -> f64
    where
        F: Fn(Point3<f64>) -> f64,
    {
        let f = |u, v| {
            let p = self.coords_local_to_world(Point2::new(u, v));
            -f(p)
        };
        -minimize_2d(&f, self.u_limits, self.v_limits, 1e-8).1
    }

    pub fn func_avg<F>(&self, f: F) -> f64
    where
        F: Fn(Point3<f64>) -> f64,
    {
        let f = |u, v| {
            let p2 = Point2::new(u, v);
            let p3 = self.coords_local_to_world(p2);
            f(p3) * self.d_solid_angle(p2)
        };
        integral_2d(&f, self.u_limits, self.v_limits, (1e-8, 1e-5)) / self.solid_angle()
    }

    pub fn d_solid_angle(&self, p: Point2<f64>) -> f64 {
        let fu: fn(f64, &mut (f64, &Self, usize)) -> f64 =
            |x, params| Self::coords_local_to_world(params.1, Point2::new(x, params.0))[params.2];
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

        let fv: fn(f64, &mut (f64, &Self, usize)) -> f64 =
            |x, params| Self::coords_local_to_world(params.1, Point2::new(params.0, x))[params.2];
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
        let r = self.coords_local_to_world(p).coords;

        // Solid angle
        // dΩ = r.dS / |r|^3
        // Take the abs because whether the surface points in or out doesn't matter
        r.dot(&ds).abs() / r.norm().powi(3)
    }

    pub fn solid_angle(&self) -> f64 {
        let f = |u, v| self.d_solid_angle(Point2::new(u, v));
        integral_2d(&f, self.u_limits, self.v_limits, (1e-8, 1e-5))
    }

    pub fn solid_angle_with_shadows(&self, shadows: &[Surface]) -> f64 {
        let f = |u, v| {
            let mut blocked = false;
            let p_local = Point2::new(u, v);
            let p_world = self.coords_local_to_world(p_local);
            for s in shadows {
                // TODO: Check that the shadow is between the source and the detector
                if s.intersects(Point3::origin(), p_world) {
                    blocked = true;
                    break;
                }
            }

            if blocked {
                0.0
            } else {
                self.d_solid_angle(p_local)
            }
        };
        integral_2d(&f, self.u_limits, self.v_limits, (1e-8, 1e-5))
    }
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct SurfaceGroup {
    surfaces: Vec<NotTemplate>,
    #[serde(
        default,
        rename = "transformations",
        skip_serializing_if = "Vec::is_empty"
    )]
    trans: Vec<Transformation>,
}

impl SurfaceGroup {
    pub fn trans(&self) -> &Vec<Transformation> {
        &self.trans
    }

    pub fn trans_mut(&mut self) -> &mut Vec<Transformation> {
        &mut self.trans
    }

    pub fn add_trans(&mut self, t: &mut Vec<Transformation>) {
        self.trans_mut().append(t);
    }

    pub fn simplify(&self, mut id: Vec<u32>) -> Vec<(Vec<u32>, Surface)> {
        id.push(0);
        let mut simplified = Vec::new();
        for s in &self.surfaces {
            let mut new = s.simplify(id.clone());

            for (_id, s) in new.iter_mut() {
                s.add_trans(&mut self.trans().clone());
            }

            simplified.append(&mut new);
            let last = id
                .last_mut()
                .expect("cannot get last element of a vector (that should exist)");
            *last += 1;
        }
        simplified
    }
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct SurfaceTemplate {
    name: String,
    #[serde(
        default,
        rename = "transformations",
        skip_serializing_if = "Vec::is_empty"
    )]
    trans: Vec<Transformation>,
}

impl SurfaceTemplate {
    pub fn trans(&self) -> &Vec<Transformation> {
        &self.trans
    }

    pub fn trans_mut(&mut self) -> &mut Vec<Transformation> {
        &mut self.trans
    }

    pub fn add_trans(&mut self, t: &mut Vec<Transformation>) {
        self.trans_mut().append(t);
    }
}

#[derive(Serialize, Deserialize, Debug, Clone)]
#[serde(untagged)]
pub enum NotTemplate {
    Base {
        #[serde(flatten)]
        surface: SurfaceBase,
    },
    Group {
        #[serde(flatten)]
        surface: SurfaceGroup,
    },
}

impl NotTemplate {
    pub fn trans(&self) -> &Vec<Transformation> {
        match self {
            NotTemplate::Base { surface: s } => s.trans(),
            NotTemplate::Group { surface: s } => s.trans(),
        }
    }

    pub fn trans_mut(&mut self) -> &mut Vec<Transformation> {
        match self {
            NotTemplate::Base { surface: s } => s.trans_mut(),
            NotTemplate::Group { surface: s } => s.trans_mut(),
        }
    }

    pub fn add_trans(&mut self, t: &mut Vec<Transformation>) {
        self.trans_mut().append(t);
    }

    pub fn simplify(&self, id: Vec<u32>) -> Vec<(Vec<u32>, Surface)> {
        match self {
            NotTemplate::Base { surface: s } => vec![s.simplify(id)],
            NotTemplate::Group { surface: s } => s.simplify(id),
        }
    }
}

#[derive(Serialize, Deserialize, Debug, Clone)]
#[serde(untagged)]
pub enum MaybeTemplate {
    NotTemplate {
        #[serde(flatten)]
        surface: NotTemplate,
    },
    Template {
        #[serde(flatten)]
        template: SurfaceTemplate,
    },
}

impl MaybeTemplate {
    pub fn trans(&self) -> &Vec<Transformation> {
        match self {
            MaybeTemplate::Template { template: t } => t.trans(),
            MaybeTemplate::NotTemplate { surface: s } => s.trans(),
        }
    }

    pub fn trans_mut(&mut self) -> &mut Vec<Transformation> {
        match self {
            MaybeTemplate::Template { template: t } => t.trans_mut(),
            MaybeTemplate::NotTemplate { surface: s } => s.trans_mut(),
        }
    }

    pub fn apply_templates(
        &self,
        templates: &HashMap<String, NotTemplate>,
    ) -> Result<NotTemplate, Error> {
        match self {
            MaybeTemplate::Template { template: t } => {
                let mut temp = templates
                    .get(&t.name)
                    .ok_or(Error::UnknownTemplate)?
                    .clone();
                temp.trans_mut().extend(self.trans().iter().cloned());
                Ok(temp)
            }
            MaybeTemplate::NotTemplate { surface: s } => Ok(s.clone()),
        }
    }
}

fn integral_2d<'a>(
    f: &'a Fn(f64, f64) -> f64,
    u_limits: (f64, f64),
    v_limits: (f64, f64),
    eps: (f64, f64),
) -> f64 {
    use rgsl::GaussKonrodRule::Gauss15;
    let mut iw1 = IntegrationWorkspace::new(200).unwrap();
    let mut iw2 = IntegrationWorkspace::new(200).unwrap();

    let fu: fn(f64, &mut (&'a Fn(f64, f64) -> f64, f64)) -> f64 = |x, params| params.0(x, params.1);
    let fv: fn(
        f64,
        &mut (
            &'a Fn(f64, f64) -> f64,
            _,
            (f64, f64),
            (f64, f64),
            &mut IntegrationWorkspace,
        ),
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
    let mut params = (f, fu, u_limits, eps, &mut iw2);
    iw1.qag(
        fv,
        &mut params,
        v_limits.0,
        v_limits.1,
        eps.0,
        eps.1,
        200,
        Gauss15,
        &mut result,
        &mut error,
    );
    result
}

pub fn minimize_1d<'a>(f: &'a Fn(f64) -> f64, limits: (f64, f64), eps: f64) -> (f64, f64) {
    let mut candidates = Vec::new();

    let minimizer = NelderMeadBuilder::default()
        .adaptive(true)
        .ftol(eps)
        .build()
        .unwrap(); // FIXME

    let func = |x: ArrayView1<f64>| {
        if x[0] < limits.0 || x[0] > limits.1 {
            std::f64::INFINITY
        } else {
            f(x[0])
        }
    };

    for u in [limits.0, (limits.0 + limits.1) / 2.0, limits.1].into_iter() {
        let x = minimizer.minimize(func, Array::from_vec(vec![*u]).view());

        candidates.push(x);
    }

    candidates.sort_by(|a, b| f(a[0]).partial_cmp(&f(b[0])).unwrap().reverse()); //FIXME

    let min = candidates.pop().unwrap(); // FIXME
    (min[0], f(min[0]))
}

pub fn minimize_2d<'a>(
    f: &'a Fn(f64, f64) -> f64,
    u_limits: (f64, f64),
    v_limits: (f64, f64),
    eps: f64,
) -> ((f64, f64), f64) {
    let mut candidates = Vec::new();

    let minimizer = NelderMeadBuilder::default()
        .adaptive(true)
        .ftol(eps)
        .build()
        .unwrap(); // FIXME

    let func = |x: ArrayView1<f64>| {
        if x[0] < u_limits.0 || x[0] > u_limits.1 || x[1] < v_limits.0 || x[1] > v_limits.1 {
            std::f64::INFINITY
        } else {
            f(x[0], x[1])
        }
    };

    for u in [u_limits.0, (u_limits.0 + u_limits.1) / 2.0, u_limits.1].into_iter() {
        for v in [v_limits.0, (v_limits.0 + v_limits.1) / 2.0, v_limits.1].into_iter() {
            let x = minimizer.minimize(func, Array::from_vec(vec![*u, *v]).view());

            candidates.push(x);
        }
    }

    candidates.sort_by(|a, b| f(a[0], a[1]).partial_cmp(&f(b[0], b[1])).unwrap().reverse()); //FIXME

    let min = candidates.pop().unwrap(); // FIXME
    ((min[0], min[1]), f(min[0], min[1]))
}
