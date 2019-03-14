use crate::{
    error::Error,
    surface::{MaybeTemplate, NotTemplate, Surface},
};
use nalgebra::{Point2, Point3, Vector3};
use ndarray::{Array, ArrayView1};
use optimize::{Minimizer, NelderMeadBuilder};
use rgsl::{numerical_differentiation::deriv_central, IntegrationWorkspace};
use std::collections::HashMap;

pub type Detector = Simplified;

#[derive(Serialize, Deserialize, Debug, Clone)]
pub(crate) struct Raw {
    #[serde(flatten)]
    surface: MaybeTemplate,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    shadows: Vec<MaybeTemplate>,
}

impl Raw {
    pub fn simplify(
        &self,
        templates: &HashMap<String, NotTemplate>,
        id: Vec<u32>,
    ) -> Result<Vec<(Vec<u32>, Simplified)>, Error> {
        let Raw { surface, shadows } = self;

        let surface = surface.apply_templates(templates)?.simplify(id.clone());

        let shadows = shadows
            .iter()
            .map(|x| {
                x.apply_templates(templates)
                    .map(|y| y.simplify(vec![]).into_iter().map(|(_id, d)| d))
            })
            .collect::<Result<Vec<_>, _>>()?
            .into_iter()
            .flatten()
            .collect::<Vec<_>>();
        let simplified = surface.into_iter().map(|(id, d)| {
            (
                id,
                Simplified {
                    surface: d,
                    shadows: shadows.clone(),
                },
            )
        });
        Ok(simplified.collect())
    }
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct Simplified {
    #[serde(flatten)]
    surface: Surface,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    shadows: Vec<Surface>,
}

impl Simplified {
    pub fn func_min<F>(&self, f: F) -> f64
    where
        F: Fn(Point3<f64>) -> f64,
    {
        let f = |u, v| {
            let p = self.surface.coords_local_to_world(Point2::new(u, v));
            f(p)
        };
        minimize_2d(&f, self.surface.u_limits(), self.surface.v_limits(), 1e-8).1
    }

    pub fn func_max<F>(&self, f: F) -> f64
    where
        F: Fn(Point3<f64>) -> f64,
    {
        let f = |u, v| {
            let p = self.surface.coords_local_to_world(Point2::new(u, v));
            -f(p)
        };
        -minimize_2d(&f, self.surface.u_limits(), self.surface.v_limits(), 1e-8).1
    }

    pub fn func_avg<F>(&self, f: F) -> f64
    where
        F: Fn(Point3<f64>) -> f64,
    {
        let f = |u, v| {
            let p2 = Point2::new(u, v);
            let p3 = self.surface.coords_local_to_world(p2);
            f(p3) * self.d_solid_angle(p2)
        };
        integral_2d(
            &f,
            self.surface.u_limits(),
            self.surface.v_limits(),
            (1e-8, 1e-5),
        ) / self.solid_angle()
    }

    pub fn d_solid_angle(&self, p: Point2<f64>) -> f64 {
        let fu: fn(f64, &mut (f64, &Surface, usize)) -> f64 = |x, params| {
            Surface::coords_local_to_world(params.1, Point2::new(x, params.0))[params.2]
        };
        let mut dr_du = Vector3::new(0.0, 0.0, 0.0);
        let mut dr_du_err = Vector3::new(0.0, 0.0, 0.0);
        deriv_central(
            fu,
            &mut (p[1], &self.surface, 0),
            0.0,
            1e-8,
            &mut dr_du[0],
            &mut dr_du_err[0],
        );
        deriv_central(
            fu,
            &mut (p[1], &self.surface, 1),
            0.0,
            1e-8,
            &mut dr_du[1],
            &mut dr_du_err[1],
        );
        deriv_central(
            fu,
            &mut (p[1], &self.surface, 2),
            0.0,
            1e-8,
            &mut dr_du[2],
            &mut dr_du_err[2],
        );

        let fv: fn(f64, &mut (f64, &Surface, usize)) -> f64 = |x, params| {
            Surface::coords_local_to_world(params.1, Point2::new(params.0, x))[params.2]
        };
        let mut dr_dv = Vector3::new(0.0, 0.0, 0.0);
        let mut dr_dv_err = Vector3::new(0.0, 0.0, 0.0);
        deriv_central(
            fv,
            &mut (p[0], &self.surface, 0),
            0.0,
            1e-8,
            &mut dr_dv[0],
            &mut dr_dv_err[0],
        );
        deriv_central(
            fv,
            &mut (p[0], &self.surface, 1),
            0.0,
            1e-8,
            &mut dr_dv[1],
            &mut dr_dv_err[1],
        );
        deriv_central(
            fv,
            &mut (p[0], &self.surface, 2),
            0.0,
            1e-8,
            &mut dr_dv[2],
            &mut dr_dv_err[2],
        );

        // Surface area
        // dS = dr/du x dr/dv
        let ds = dr_du.cross(&dr_dv);
        let r = self.surface.coords_local_to_world(p).coords;

        // Solid angle
        // dΩ = r.dS / |r|^3
        // Take the abs because whether the surface points in or out doesn't matter
        r.dot(&ds).abs() / r.norm().powi(3)
    }

    pub fn solid_angle(&self) -> f64 {
        let f = |u, v| self.d_solid_angle(Point2::new(u, v));
        integral_2d(
            &f,
            self.surface.u_limits(),
            self.surface.v_limits(),
            (1e-8, 1e-5),
        )
    }

    pub fn solid_angle_with_shadows(&self) -> f64 {
        let f = |u, v| {
            let mut blocked = false;
            let p_local = Point2::new(u, v);
            let p_world = self.surface.coords_local_to_world(p_local);
            for s in &self.shadows {
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
        integral_2d(
            &f,
            self.surface.u_limits(),
            self.surface.v_limits(),
            (1e-8, 1e-5),
        )
    }
}

fn integral_2d(
    f: &Fn(f64, f64) -> f64,
    u_limits: (f64, f64),
    v_limits: (f64, f64),
    eps: (f64, f64),
) -> f64 {
    use rgsl::GaussKonrodRule::Gauss15;
    let mut iw1 = IntegrationWorkspace::new(200).unwrap();
    let mut iw2 = IntegrationWorkspace::new(200).unwrap();

    type F1<'a> = fn(f64, &mut (&'a Fn(f64, f64) -> f64, f64)) -> f64;
    type F2<'a> = fn(
        f64,
        &mut (
            &'a Fn(f64, f64) -> f64,
            for<'r> fn(f64, &'r mut (&'a (dyn std::ops::Fn(f64, f64) -> f64 + 'a), f64)) -> f64,
            (f64, f64),
            (f64, f64),
            &mut IntegrationWorkspace,
        ),
    ) -> f64;
    let fu: F1 = |x, params| params.0(x, params.1);
    let fv: F2 = |x, params| {
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

/*
fn minimize_1d(f: &Fn(f64) -> f64, limits: (f64, f64), eps: f64) -> (f64, f64) {
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

    for u in [limits.0, (limits.0 + limits.1) / 2.0, limits.1].iter() {
        let x = minimizer.minimize(func, Array::from_vec(vec![*u]).view());

        candidates.push(x);
    }

    candidates.sort_by(|a, b| f(a[0]).partial_cmp(&f(b[0])).unwrap().reverse()); //FIXME

    let min = candidates.pop().unwrap(); // FIXME
    (min[0], f(min[0]))
}
*/

fn minimize_2d(
    f: &Fn(f64, f64) -> f64,
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

    for u in [u_limits.0, (u_limits.0 + u_limits.1) / 2.0, u_limits.1].iter() {
        for v in [v_limits.0, (v_limits.0 + v_limits.1) / 2.0, v_limits.1].iter() {
            let x = minimizer.minimize(func, Array::from_vec(vec![*u, *v]).view());

            candidates.push(x);
        }
    }

    candidates.sort_by(|a, b| f(a[0], a[1]).partial_cmp(&f(b[0], b[1])).unwrap().reverse()); //FIXME

    let min = candidates.pop().unwrap(); // FIXME
    ((min[0], min[1]), f(min[0], min[1]))
}
