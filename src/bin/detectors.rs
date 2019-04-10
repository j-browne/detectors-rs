use detectors_rs::{config::Config, error::Error};
use nalgebra::Point3;
use pbr::ProgressBar;
use rayon::prelude::*;
use std::{fs::File, io::stderr, path::PathBuf, sync::Mutex};
use structopt::{clap::AppSettings, StructOpt};
use val_unc::ValUnc;

#[derive(Debug, StructOpt)]
#[structopt(
    name = "detectors",
    about = "A program to calculate detector properties.",
    version = "",
    author = "",
    raw(global_settings = "&[AppSettings::DisableVersion]")
)]
struct Opt {
    #[structopt(short = "m")]
    monte_carlo: Option<usize>,
    #[structopt(name = "FILE", parse(from_os_str))]
    files: Vec<PathBuf>,
}

fn theta(p: Point3<f64>) -> f64 {
    f64::acos(p[2] / f64::sqrt(p[0].powi(2) + p[1].powi(2) + p[2].powi(2))).to_degrees()
}

fn phi(p: Point3<f64>) -> f64 {
    let mut phi = f64::atan2(p[0], p[1]).to_degrees();
    if phi < 0.0 {
        phi += 360.0
    };
    phi
}

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

impl OutputData {
    fn format(&self) -> String {
        format!(
            "{}\t{}\t{}\t{:.3}\t{:.3}\t{:.3}\t{:.3}\t{:.3}\t{:.3}\t{}",
            self.det_id.get(0).unwrap_or(&0) + 1,
            self.det_id.get(1).unwrap_or(&0),
            self.det_id.get(2).unwrap_or(&0),
            self.theta_min.val,
            self.theta_max.val,
            self.theta_avg.val,
            self.phi_min.val,
            self.phi_max.val,
            self.phi_avg.val,
            self.solid_angle.val,
        )
    }

    fn format_unc(&self) -> String {
        format!(
            "{}\t{}\t{}\t{:.3}\t{:.3}\t{:.3}\t{:.3}\t{:.3}\t{:.3}\t{:.3}\t{:.3}\t{:.3}\t{:.3}\t{:.3}\t{:.3}\t{}\t{}",
            self.det_id.get(0).unwrap_or(&0) + 1,
            self.det_id.get(1).unwrap_or(&0),
            self.det_id.get(2).unwrap_or(&0),
            self.theta_min.val,
            self.theta_min.unc,
            self.theta_max.val,
            self.theta_max.unc,
            self.theta_avg.val,
            self.theta_avg.unc,
            self.phi_min.val,
            self.phi_min.unc,
            self.phi_max.val,
            self.phi_max.unc,
            self.phi_avg.val,
            self.phi_avg.unc,
            self.solid_angle.val,
            self.solid_angle.unc,
        )
    }
}

fn main() -> Result<(), Error> {
    //rayon::ThreadPoolBuilder::new().num_threads(1).build_global().unwrap();

    let opt = Opt::from_args();
    let mut config = Config::new();
    for file_path in opt.files {
        let file = File::open(file_path)?;
        config.add_from_reader(file)?;
    }

    let detectors = config.simplify()?;
    let pb = Mutex::new(ProgressBar::on(stderr(), detectors.len() as u64));
    pb.lock().unwrap().set(0);

    let mut output = if let Some(steps) = opt.monte_carlo {
        detectors
            .into_par_iter()
            .map(|(id, surface)| OutputData {
                det_id: id.to_vec(),
                theta_min: surface.func_min_unc(&theta, steps),
                theta_max: surface.func_max_unc(&theta, steps),
                theta_avg: surface.func_avg_unc(&theta, steps),
                phi_min: surface.func_min_unc(&phi, steps),
                phi_max: surface.func_max_unc(&phi, steps),
                phi_avg: surface.func_avg_unc(&phi, steps),
                solid_angle: surface.solid_angle_unc(steps),
            })
            .inspect(|_| {
                let _ = pb.lock().unwrap().inc();
            })
            .collect::<Vec<_>>()
    } else {
        detectors
            .into_par_iter()
            .map(|(id, surface)| OutputData {
                det_id: id.to_vec(),
                theta_min: surface.func_min(&theta).into(),
                theta_max: surface.func_max(&theta).into(),
                theta_avg: surface.func_avg(&theta).into(),
                phi_min: surface.func_min(&phi).into(),
                phi_max: surface.func_max(&phi).into(),
                phi_avg: surface.func_avg(&phi).into(),
                solid_angle: surface.solid_angle().into(),
            })
            .inspect(|_| {
                let _ = pb.lock().unwrap().inc();
            })
            .collect::<Vec<_>>()
    };
    pb.into_inner().unwrap().finish();
    println!();

    output.sort_by_key(|x| x.det_id.get(2).copied().unwrap_or(0));
    output.sort_by_key(|x| x.det_id.get(1).copied().unwrap_or(0));
    output.sort_by_key(|x| x.det_id.get(0).copied().unwrap_or(0));

    if opt.monte_carlo.is_some() {
        for o in output {
            println!("{}", o.format_unc());
        }
    } else {
        for o in output {
            println!("{}", o.format());
        }
    }

    Ok(())
}
