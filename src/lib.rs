#[macro_use]
extern crate serde_derive;

pub mod config;
pub mod coordinates;
pub mod detector;
pub mod error;
pub mod statistics;
pub mod surface;
pub mod unc;

pub use crate::{
    config::Config, detector::Simplified as Detector, error::Error, surface::Base as Surface,
};
