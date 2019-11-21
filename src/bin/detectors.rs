#[macro_use]
extern crate serde_derive;

use detectors_rs::{config::Config, error::Error, statistics::stats};
use nalgebra::Point3;
use pbr::ProgressBar;
use rayon::prelude::*;
use serde_json;
use std::{
    fs::File,
    io::{stderr, stdout, Write},
    path::PathBuf,
    sync::Mutex,
};
use structopt::StructOpt;
use val_unc::ValUnc;

#[derive(Debug, StructOpt)]
#[structopt(name = "detectors", no_version)]
/// A program to calculate detector properties.
struct Opt {
    #[structopt(short, long)]
    /// Suppresses the output of the progress bar.
    quiet: bool,
    #[structopt(short, name = "steps")]
    /// Perform a Monte Carlo simulation with <steps> iterations.
    ///
    /// This option causes the program to do multiple runs for each detector, varying the
    /// transformation parameters based on their uncertainties and calculating the uncertainties of
    /// the resulting properties.
    monte_carlo: Option<usize>,
    #[structopt(parse(from_os_str))]
    /// The input files specifying the detector geometry.
    ///
    /// The input format is JSON.
    files: Vec<PathBuf>,
}

fn theta(p: Point3<f64>) -> f64 {
    f64::acos(p[2] / f64::sqrt(p[0].powi(2) + p[1].powi(2) + p[2].powi(2))).to_degrees()
}

fn theta_diff(avg: f64) -> impl Fn(Point3<f64>) -> f64 {
    move |p: Point3<f64>| theta(p) - avg
}

fn phi(p: Point3<f64>) -> f64 {
    let phi = f64::atan2(p[0], p[1]).to_degrees();
    if phi < 0.0 {
        phi + 360.0
    } else {
        phi
    }
}

fn phi_diff(avg: f64) -> impl Fn(Point3<f64>) -> f64 {
    move |p: Point3<f64>| {
        let phi = f64::atan2(p[0], p[1]).to_degrees();
        let diff = phi - avg;

        if diff < -180.0 {
            diff + 360.0
        } else if diff > 180.0 {
            diff - 360.0
        } else {
            diff
        }
    }
}

#[derive(Debug, Serialize)]
struct OutputData {
    det_id: Vec<u32>,
    theta_min: ValUnc,
    theta_max: ValUnc,
    theta_avg: ValUnc,
    phi_min: ValUnc,
    phi_max: ValUnc,
    phi_avg: ValUnc,
    solid_angle: ValUnc,
}

fn main() -> Result<(), Error> {
    //rayon::ThreadPoolBuilder::new().num_threads(1).build_global().unwrap();

    let opt = Opt::from_args();
    if opt.files.len() < 1 {
        let mut out = stdout();
        Opt::clap()
            .write_long_help(&mut out)
            .expect("failed to write to stdout");
        writeln!(&mut out)?;
        return Ok(());
    }
    let quiet = opt.quiet;

    let mut config = Config::new();
    for file_path in opt.files {
        let file = File::open(file_path)?;
        config.add_from_reader(file)?;
    }

    let detectors = config.simplify()?;

    let pb = Mutex::new(ProgressBar::on(stderr(), detectors.len() as u64));
    if !quiet {
        pb.lock().unwrap().set(0);
    }

    let mut output = if let Some(steps) = opt.monte_carlo {
        detectors
            .into_par_iter()
            .map(|(id, surface)| {
                let (dir_avg, _dir_avg_unc) = surface.dir_avg_unc(steps);
                let theta_avg = theta(dir_avg.into());
                let phi_avg = phi(dir_avg.into());

                let mut buffer = Vec::with_capacity(steps);
                surface.func_min_monte_carlo(&theta, steps, &mut buffer);
                let theta_min = stats(&buffer);
                surface.func_max_monte_carlo(&theta, steps, &mut buffer);
                let theta_max = stats(&buffer);
                surface.func_min_monte_carlo(&phi, steps, &mut buffer);
                let phi_min = stats(&buffer);
                surface.func_max_monte_carlo(&phi, steps, &mut buffer);
                let phi_max = stats(&buffer);
                surface.solid_angle_monte_carlo(steps, &mut buffer);
                let solid_angle = stats(&buffer);

                OutputData {
                    det_id: id.to_vec(),
                    theta_min,
                    theta_max,
                    theta_avg: theta_avg.into(), //FIXME: No unc
                    phi_min,
                    phi_max,
                    phi_avg: phi_avg.into(), //FIXME: No unc
                    solid_angle,
                }
            })
            .inspect(|_| {
                if !quiet {
                    let _ = pb.lock().unwrap().inc();
                }
            })
            .collect::<Vec<_>>()
    } else {
        detectors
            .into_par_iter()
            .map(|(id, surface)| {
                let det_id = id.to_vec();
                let dir_avg = surface.dir_avg();

                let theta_avg = theta(dir_avg.into());
                let theta_min = (surface.func_min(&theta_diff(theta_avg)) + theta_avg).into();
                let theta_max = (surface.func_max(&theta_diff(theta_avg)) + theta_avg).into();
                let theta_avg = theta_avg.into();

                let phi_avg = phi(dir_avg.into());
                let phi_min = (surface.func_min(&phi_diff(phi_avg)) + phi_avg).into();
                let phi_max = (surface.func_max(&phi_diff(phi_avg)) + phi_avg).into();
                let phi_avg = phi_avg.into();

                let solid_angle = surface.solid_angle().into();

                OutputData {
                    det_id,
                    theta_min,
                    theta_max,
                    theta_avg,
                    phi_min,
                    phi_max,
                    phi_avg,
                    solid_angle,
                }
            })
            .inspect(|_| {
                if !quiet {
                    let _ = pb.lock().unwrap().inc();
                }
            })
            .collect::<Vec<_>>()
    };

    if !quiet {
        pb.into_inner().unwrap().finish();
        eprintln!();
    }

    output.sort_by_key(|x| x.det_id.get(2).copied().unwrap_or(0));
    output.sort_by_key(|x| x.det_id.get(1).copied().unwrap_or(0));
    output.sort_by_key(|x| x.det_id.get(0).copied().unwrap_or(0));

    println!("{}", serde_json::to_string(&output)?);

    Ok(())
}
