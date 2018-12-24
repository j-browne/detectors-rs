use detectors_rs::surface::Config;
use std::fs::File;
use std::io::BufReader;
use ron::{ser::{self, PrettyConfig}, de};

fn main() -> Result<(), Box<std::error::Error>> {
    let file = File::open("config/detector_types_new.ron")?;
    let file = BufReader::new(file);
    let config: Config = de::from_reader(file)?;
    println!("{}", ser::to_string_pretty(&config, PrettyConfig::default())?);

    Ok(())
}
