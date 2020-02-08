use crate::unc::{Unc, ValUnc};
use nalgebra::Vector3;
use rand::{distributions::Distribution, Rng};
use rand_distr::Normal;

pub fn stats(vals: &[f64]) -> ValUnc {
    let mean = vals.iter().sum::<f64>() / (vals.len() as f64);
    let std_dev = (vals.iter().map(|x| (x - mean) * (x - mean)).sum::<f64>()
        / ((vals.len() - 1) as f64))
        .sqrt();
    ValUnc::new(mean, std_dev.into())
}

pub fn stats_vec(vals: &[Vector3<f64>]) -> (Vector3<f64>, Vector3<Unc>) {
    let vals_x = vals.iter().map(|x| x[0]).collect::<Vec<_>>();
    let vals_y = vals.iter().map(|x| x[1]).collect::<Vec<_>>();
    let vals_z = vals.iter().map(|x| x[2]).collect::<Vec<_>>();

    let x = stats(&vals_x);
    let y = stats(&vals_y);
    let z = stats(&vals_z);

    (
        Vector3::new(x.val, y.val, z.val),
        Vector3::new(x.unc, y.unc, z.unc),
    )
}

pub fn randomize<R: Rng>(v: ValUnc, rng: &mut R) -> ValUnc {
    if let Ok(norm) = Normal::new(v.val, v.unc.0.abs()) {
        ValUnc::new(norm.sample(rng), v.unc)
    } else {
        v
    }
}
