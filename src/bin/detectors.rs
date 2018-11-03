use std::{sync::Mutex, path::Path, fs::File, io::BufReader};
use rayon::prelude::*;
use structopt::{clap::AppSettings, StructOpt};
use detectors_rs::{Error, Detector, dets_from_readers};

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
    let dets = dets_from_names(opt.det_file, opt.types_file)?;
    let output = Mutex::new(Vec::new());
    dets.par_iter().enumerate().for_each(|(i, d)| {
        d.par_iter().enumerate().for_each(|(j, s)| {
            output.lock().unwrap().push((i + 1, j,
                                         s.th_min(), s.th_max(), s.th_avg(),
                                         s.phi_min(), s.phi_max(), s.phi_avg(),
                                         s.solid_angle()));
        });
    });

    let mut output = output.into_inner().unwrap();
    output.sort_by_key(|x| x.1);
    output.sort_by_key(|x| x.0);

    for (det, chan, th_min, th_max, th_avg, phi_min, phi_max, phi_avg, solid_angle) in output {
        println!("{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}", det, chan, th_min, th_max, th_avg, phi_min, phi_max, phi_avg, solid_angle);
    }
    Ok(())
}

fn dets_from_names<T, U>(
    name_dets: T,
    name_det_types: U,
) -> Result<Vec<Vec<Detector>>, Error>
where
    T: AsRef<Path>,
    U: AsRef<Path>,
{
    let file_dets = File::open(name_dets)?;
    let file_dets = BufReader::new(file_dets);
    let file_types = File::open(name_det_types)?;
    let file_types = BufReader::new(file_types);
    dets_from_readers(file_dets, file_types)
}
