use detectors_rs::{config::Config, error::Error};
use nalgebra::Point2;
use std::{fs::File, path::PathBuf};
use structopt::StructOpt;

#[derive(Debug, StructOpt)]
#[structopt(
    name = "detectors",
    about = "A program to calculate detector properties."
)]
struct Opt {
    #[structopt(short = "g", default_value = "10")]
    grid: u32,
    #[structopt(name = "FILE", parse(from_os_str))]
    files: Vec<PathBuf>,
}

fn main() -> Result<(), Error> {
    let opt = Opt::from_args();
    let mut config = Config::new();
    for file_path in opt.files {
        let file = File::open(file_path)?;
        config.add_from_reader(file)?;
    }

    let detectors = config.simplify()?;

    for (_, det) in detectors {
        let max_u = opt.grid + 1;
        let max_v = opt.grid + 1;

        let surface = det.surface();
        let u_limits = surface.u_limits_val();
        let v_limits = surface.v_limits_val();

        for u_idx in 0..max_u {
            let u = f64::from(u_idx) * (u_limits.1 - u_limits.0) / f64::from(max_u - 1) + u_limits.0;
            for v_idx in 0..max_v {
                let v = f64::from(v_idx) * (v_limits.1 - v_limits.0) / f64::from(max_v - 1) + v_limits.0;

                let p = surface.coords_local_to_world(Point2::new(u, v));
                println!("{} {} {}", p[0], p[1], p[2]);
            }
            println!();
        }
        println!();
    }

    Ok(())
}
