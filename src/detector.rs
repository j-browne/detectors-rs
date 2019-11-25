use crate::{
    error::Error,
    statistics::{stats, stats_vec},
    surface::{MaybeTemplate, Surface},
};
use nalgebra::{Point2, Point3, Vector3};
use ndarray::{Array, ArrayView1};
use optimize::{Minimizer, NelderMeadBuilder};
use rgsl::{numerical_differentiation::deriv_central, IntegrationWorkspace};
use std::{cell::RefCell, collections::HashMap, iter::repeat_with, sync::Arc};
use val_unc::ValUnc;

pub use self::Simplified as Detector;

#[derive(Serialize, Deserialize, Debug, Clone)]
pub(crate) struct Raw {
    #[serde(flatten)]
    surface: MaybeTemplate,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    shadows: Vec<MaybeTemplate>,
}

impl Raw {
    pub fn simplify(
        self,
        templates: &HashMap<String, MaybeTemplate>,
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
                    solid_angle: Default::default(),
                },
            )
        });
        Ok(simplified.collect())
    }
}

#[derive(Serialize, Deserialize, Debug)]
pub struct Simplified {
    #[serde(flatten)]
    surface: Surface,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    shadows: Vec<Surface>,
    #[serde(skip)]
    solid_angle: RefCell<Arc<Option<f64>>>,
}

impl Simplified {
    pub fn surface(&self) -> &Surface {
        &self.surface
    }

    pub fn shadows(&self) -> &Vec<Surface> {
        &self.shadows
    }

    fn rand(&self) -> Self {
        let mut other = self.clone();
        other.surface.randomize();
        for s in &mut other.shadows {
            s.randomize();
        }
        other
    }

    pub fn func_min<F>(&self, f: &F) -> f64
    where
        F: Fn(Point3<f64>) -> f64,
    {
        let f = |u, v| {
            let p = self.surface.coords_local_to_world(Point2::new(u, v));
            f(p)
        };
        let u_limits = (self.surface.u_limits().0.val, self.surface.u_limits().1.val);
        let v_limits = (self.surface.v_limits().0.val, self.surface.v_limits().1.val);
        minimize_2d(&f, u_limits, v_limits, 1e-8).1 // FIXME: Allow configuration of error limits
    }

    pub fn func_max<F>(&self, f: &F) -> f64
    where
        F: Fn(Point3<f64>) -> f64,
    {
        let f = |u, v| {
            let p = self.surface.coords_local_to_world(Point2::new(u, v));
            -f(p)
        };
        let u_limits = (self.surface.u_limits().0.val, self.surface.u_limits().1.val);
        let v_limits = (self.surface.v_limits().0.val, self.surface.v_limits().1.val);
        -minimize_2d(&f, u_limits, v_limits, 1e-8).1 // FIXME: Allow configuration of error limits
    }

    pub fn func_avg<F>(&self, f: &F) -> f64
    where
        F: Fn(Point3<f64>) -> f64,
    {
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
                f(p_world) * self.d_solid_angle(p_local)
            }
        };
        let u_limits = (self.surface.u_limits().0.val, self.surface.u_limits().1.val);
        let v_limits = (self.surface.v_limits().0.val, self.surface.v_limits().1.val);
        integral_2d(&f, u_limits, v_limits, (1e1, 1e-3)) / self.solid_angle() // FIXME: Allow configuration of error limits
    }

    pub fn dir_avg(&self) -> Vector3<f64> {
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
                nalgebra::zero()
            } else {
                p_world.coords.normalize() * self.d_solid_angle(p_local)
            }
        };
        let u_limits = (self.surface.u_limits().0.val, self.surface.u_limits().1.val);
        let v_limits = (self.surface.v_limits().0.val, self.surface.v_limits().1.val);

        // FIXME: Allow configuration of error limits
        let x =
            integral_2d(&|u, v| f(u, v)[0], u_limits, v_limits, (1e1, 1e-3)) / self.solid_angle();
        let y =
            integral_2d(&|u, v| f(u, v)[1], u_limits, v_limits, (1e1, 1e-3)) / self.solid_angle();
        let z =
            integral_2d(&|u, v| f(u, v)[2], u_limits, v_limits, (1e1, 1e-3)) / self.solid_angle();

        Vector3::new(x, y, z)
    }

    pub fn d_solid_angle(&self, p: Point2<f64>) -> f64 {
        let fu: fn(f64, &mut (f64, &Surface, usize)) -> f64 = |x, params| {
            Surface::coords_local_to_world(params.1, Point2::new(x, params.0))[params.2]
        };
        let mut dr_du = Vector3::new(0.0, 0.0, 0.0);
        let mut dr_du_err = Vector3::new(0.0, 0.0, 0.0);
        for i in 0..3 {
            deriv_central(
                fu,
                &mut (p[1], &self.surface, i),
                0.0,
                1e-8, // FIXME: Allow configuration of error limits
                &mut dr_du[i],
                &mut dr_du_err[i],
            );
        }

        let fv: fn(f64, &mut (f64, &Surface, usize)) -> f64 = |x, params| {
            Surface::coords_local_to_world(params.1, Point2::new(params.0, x))[params.2]
        };
        let mut dr_dv = Vector3::new(0.0, 0.0, 0.0);
        let mut dr_dv_err = Vector3::new(0.0, 0.0, 0.0);
        for i in 0..3 {
            deriv_central(
                fv,
                &mut (p[0], &self.surface, i),
                0.0,
                1e-8, // FIXME: Allow configuration of error limits
                &mut dr_dv[i],
                &mut dr_dv_err[i],
            );
        }

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
        if self.solid_angle.borrow().is_none() {
            let _ = self
                .solid_angle
                .replace(Arc::new(Some(self.solid_angle_calculate())));
        }
        self.solid_angle
            .borrow()
            .expect("solid angle is still None after being set")
    }

    pub fn solid_angle_calculate(&self) -> f64 {
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
        let u_limits = (self.surface.u_limits().0.val, self.surface.u_limits().1.val);
        let v_limits = (self.surface.v_limits().0.val, self.surface.v_limits().1.val);
        integral_2d(&f, u_limits, v_limits, (1e1, 1e-3)) // FIXME: Allow configuration of error limits
    }

    pub fn func_min_monte_carlo<F>(&self, f: &F, steps: usize, v: &mut Vec<f64>)
    where
        F: Fn(Point3<f64>) -> f64,
    {
        v.clear();
        v.reserve(steps);
        v.extend(repeat_with(|| self.rand().func_min(f)).take(steps));
    }

    pub fn func_max_monte_carlo<F>(&self, f: &F, steps: usize, v: &mut Vec<f64>)
    where
        F: Fn(Point3<f64>) -> f64,
    {
        v.clear();
        v.reserve(steps);
        v.extend(repeat_with(|| self.rand().func_max(f)).take(steps));
    }

    pub fn func_avg_monte_carlo<F>(&self, f: &F, steps: usize, v: &mut Vec<f64>)
    where
        F: Fn(Point3<f64>) -> f64,
    {
        v.clear();
        v.reserve(steps);
        v.extend(repeat_with(|| self.rand().func_avg(f)).take(steps));
    }

    pub fn dir_avg_monte_carlo(&self, steps: usize, v: &mut Vec<Vector3<f64>>) {
        v.clear();
        v.reserve(steps);
        v.extend(repeat_with(|| self.rand().dir_avg()).take(steps));
    }

    pub fn solid_angle_monte_carlo(&self, steps: usize, v: &mut Vec<f64>) {
        v.clear();
        v.reserve(steps);
        v.extend(repeat_with(|| self.rand().solid_angle()).take(steps));
    }

    pub fn func_min_unc<F>(&self, f: &F, steps: usize) -> ValUnc
    where
        F: Fn(Point3<f64>) -> f64,
    {
        let vals = repeat_with(|| self.rand().func_min(f))
            .take(steps)
            .collect::<Vec<_>>();
        stats(&vals)
    }

    pub fn func_max_unc<F>(&self, f: &F, steps: usize) -> ValUnc
    where
        F: Fn(Point3<f64>) -> f64,
    {
        let vals = repeat_with(|| self.rand().func_max(f))
            .take(steps)
            .collect::<Vec<_>>();
        stats(&vals)
    }

    pub fn func_avg_unc<F>(&self, f: &F, steps: usize) -> ValUnc
    where
        F: Fn(Point3<f64>) -> f64,
    {
        let vals = repeat_with(|| self.rand().func_avg(f))
            .take(steps)
            .collect::<Vec<_>>();
        stats(&vals)
    }

    pub fn dir_avg_unc(&self, steps: usize) -> (Vector3<f64>, Vector3<f64>) {
        let vals = repeat_with(|| self.rand().dir_avg())
            .take(steps)
            .collect::<Vec<_>>();
        stats_vec(&vals)
    }

    pub fn solid_angle_unc(&self, steps: usize) -> ValUnc {
        let vals = repeat_with(|| self.rand().solid_angle())
            .take(steps)
            .collect::<Vec<_>>();
        stats(&vals)
    }
}

impl Clone for Simplified {
    fn clone(&self) -> Self {
        Self {
            surface: self.surface.clone(),
            shadows: self.shadows.clone(),
            solid_angle: Default::default(),
        }
    }
}

fn integral_2d(
    f: &dyn Fn(f64, f64) -> f64,
    u_limits: (f64, f64),
    v_limits: (f64, f64),
    eps: (f64, f64),
) -> f64 {
    use rgsl::GaussKonrodRule::Gauss15;
    let mut iw1 = IntegrationWorkspace::new(200).unwrap();
    let mut iw2 = IntegrationWorkspace::new(200).unwrap();

    type F1<'a> = fn(f64, &mut (&'a dyn Fn(f64, f64) -> f64, f64)) -> f64;
    type F2<'a> = fn(
        f64,
        &mut (
            &'a dyn Fn(f64, f64) -> f64,
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

fn minimize_2d(
    f: &dyn Fn(f64, f64) -> f64,
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

#[cfg(test)]
mod tests {
    use super::*;

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
