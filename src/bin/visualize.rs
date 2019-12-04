use detectors_rs::{config::Config, error::Error};
use nalgebra::Point2;
use std::{
    fs::File,
    io::{stdout, Write},
    path::PathBuf,
};
use structopt::StructOpt;

#[derive(Debug, StructOpt)]
#[structopt(name = "visualize", no_version)]
/// A program to output a grid of points for each detector for use with gnuplot.
struct Opt {
    #[structopt(short, name = "sections", default_value = "10")]
    /// The number of grid sections for each direction.
    ///
    /// Note that the total number of grid points is (<sections> + 1)^2.
    grid: u32,
    #[structopt(parse(from_os_str))]
    /// The input files specifying the detector geometry.
    ///
    /// The input format is JSON.
    files: Vec<PathBuf>,
}

fn main() -> Result<(), Error> {
    let opt = Opt::from_args();
    if opt.files.is_empty() {
        let mut out = stdout();
        Opt::clap()
            .write_long_help(&mut out)
            .expect("failed to write to stdout");
        writeln!(&mut out)?;
        return Ok(());
    }

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
            let u =
                f64::from(u_idx) * (u_limits.1 - u_limits.0) / f64::from(max_u - 1) + u_limits.0;
            for v_idx in 0..max_v {
                let v = f64::from(v_idx) * (v_limits.1 - v_limits.0) / f64::from(max_v - 1)
                    + v_limits.0;

                let p = surface.coords_local_to_world(Point2::new(u, v));
                println!("{} {} {}", p[0], p[1], p[2]);
            }
            println!();
        }
        println!();
    }

    Ok(())
}
