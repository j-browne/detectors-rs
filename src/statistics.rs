use crate::unc::{Unc, ValUnc};
use nalgebra::Vector3;
use rand::{distributions::Distribution, Rng};
use rand_distr::Normal;

pub fn stats(vals: &[f64]) -> ValUnc {
    let mean = vals.iter().sum::<f64>() / (vals.len() as f64);
    let std_dev = (vals.iter().map(|x| (x - mean) * (x - mean)).sum::<f64>()
        / ((vals.len() - 1) as f64))
        .sqrt();
    ValUnc {
        val: mean,
        unc: std_dev.into(),
    }
}

pub fn stats_vec(vals: &[Vector3<f64>]) -> (Vector3<f64>, Vector3<Unc>) {
    let vals_x = vals.iter().map(|x| x[0]).collect::<Vec<_>>();
    let vals_y = vals.iter().map(|x| x[1]).collect::<Vec<_>>();
    let vals_z = vals.iter().map(|x| x[2]).collect::<Vec<_>>();

    let ValUnc {
        val: val_x,
        unc: unc_x,
    } = stats(&vals_x);
    let ValUnc {
        val: val_y,
        unc: unc_y,
    } = stats(&vals_y);
    let ValUnc {
        val: val_z,
        unc: unc_z,
    } = stats(&vals_z);

    (
        Vector3::new(val_x, val_y, val_z),
        Vector3::new(unc_x, unc_y, unc_z),
    )
}


pub fn randomize<R: Rng>(v: ValUnc, rng: &mut R) -> ValUnc {
    if let Ok(norm) = Normal::new(v.val, v.unc.0.abs()) {
        ValUnc {
            val: norm.sample(rng),
            unc: v.unc,
        }
    } else {
        v
    }
}
