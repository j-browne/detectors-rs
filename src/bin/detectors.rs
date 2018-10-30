use std::sync::Mutex;
use detectors_rs::get_detectors_from_files;
use rayon::prelude::*;
use structopt::{clap::AppSettings, StructOpt};
use detectors_rs::Error;

#[derive(Debug, StructOpt)]
#[structopt(name = "detectors", about = "A program to calculate detector properties.", version="", author = "", raw(global_settings = "&[AppSettings::DisableVersion]"))]
struct Opt {
    #[structopt(short = "d", long = "detectors")]
    det_file: String,
    #[structopt(short = "t", long = "detector-types")]
    types_file: String,
}

fn main() -> Result<(), Error> {
    let opt = Opt::from_args();
    let dets = get_detectors_from_files(opt.det_file, opt.types_file)?;
    let output = Mutex::new(Vec::new());
    dets.par_iter().enumerate().for_each(|(i, d)| {
        d.par_iter().enumerate().for_each(|(j, s)| {
            output.lock().unwrap().push((i + 1, j, s.solid_angle()));
        });
    });

    let mut output = output.into_inner().unwrap();
    output.sort_by_key(|x| x.1);
    output.sort_by_key(|x| x.0);

    for (det, chan, solid_angle) in output {
        println!("{} {} {}", det, chan, solid_angle);
    }
    Ok(())
}
