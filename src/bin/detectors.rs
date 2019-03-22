use detectors_rs::{config::Config, error::Error};
use nalgebra::Point3;
use rayon::prelude::*;
use std::{fs::File, path::PathBuf, sync::Mutex};
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
    det_no: u32,
    det_ch: u32,
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
            "{}\t{}\t{:.3}\t{:.3}\t{:.3}\t{:.3}\t{:.3}\t{:.3}\t{}",
            self.det_no + 1,
            self.det_ch,
            self.theta_min.val(),
            self.theta_max.val(),
            self.theta_avg.val(),
            self.phi_min.val(),
            self.phi_max.val(),
            self.phi_avg.val(),
            self.solid_angle.val(),
        )
    }

    fn format_unc(&self) -> String {
        format!(
            "{}\t{}\t{:.3}\t{:.3}\t{:.3}\t{:.3}\t{:.3}\t{:.3}\t{:.3}\t{:.3}\t{:.3}\t{:.3}\t{:.3}\t{:.3}\t{}\t{}",
            self.det_no + 1,
            self.det_ch,
            self.theta_min.val(),
            self.theta_min.unc(),
            self.theta_max.val(),
            self.theta_max.unc(),
            self.theta_avg.val(),
            self.theta_avg.unc(),
            self.phi_min.val(),
            self.phi_min.unc(),
            self.phi_max.val(),
            self.phi_max.unc(),
            self.phi_avg.val(),
            self.phi_avg.unc(),
            self.solid_angle.val(),
            self.solid_angle.unc(),
        )
    }
}

fn main() -> Result<(), Error> {
    let opt = Opt::from_args();
    let mut config = Config::new();
    for file_path in opt.files {
        let file = File::open(file_path)?;
        config.add_from_reader(file)?;
    }

    //println!("{:#?}", config);
    let detectors = config.simplify()?;
    //println!("{:#?}", detectors);

    let output = Mutex::new(Vec::new());
    if let Some(steps) = opt.monte_carlo {
        detectors.par_iter().for_each(|(id, surface)| {
            output.lock().unwrap().push(OutputData{
                det_no: id[0],
                det_ch: id[1],
                theta_min: surface.func_min_unc(&theta, steps),
                theta_max: surface.func_max_unc(&theta, steps),
                theta_avg: surface.func_avg_unc(&theta, steps),
                phi_min: surface.func_min_unc(&phi, steps),
                phi_max: surface.func_max_unc(&phi, steps),
                phi_avg: surface.func_avg_unc(&phi, steps),
                solid_angle: surface.solid_angle_with_shadows_unc(steps),
            });
        });
    } else {
        detectors.par_iter().for_each(|(id, surface)| {
            output.lock().unwrap().push(OutputData{
                det_no: id[0],
                det_ch: id[1],
                theta_min: surface.func_min(&theta).into(),
                theta_max: surface.func_max(&theta).into(),
                theta_avg: surface.func_avg(&theta).into(),
                phi_min: surface.func_min(&phi).into(),
                phi_max: surface.func_max(&phi).into(),
                phi_avg: surface.func_avg(&phi).into(),
                solid_angle: surface.solid_angle_with_shadows().into(),
            });
        });
    }

    let mut output = output.into_inner().unwrap();
    output.sort_by_key(|x| x.det_ch);
    output.sort_by_key(|x| x.det_no);

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
