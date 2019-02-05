use detectors_rs::config::Config;
use std::fs::File;
use std::io::BufReader;

fn main() -> Result<(), Box<std::error::Error>> {
    let file = File::open("config/detectors.json")?;
    let file = BufReader::new(file);
    let config: Config = serde_json::from_reader(file)?;
    println!("{:#?}", config);

    Ok(())
}
