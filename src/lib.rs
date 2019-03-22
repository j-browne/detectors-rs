#![feature(custom_attribute)]
#[macro_use]
extern crate serde_derive;

pub mod config;
pub mod coordinates;
pub(crate) mod detector;
pub mod error;
pub(crate) mod surface;

pub use crate::detector::Detector;
pub use crate::error::Error;
pub use crate::surface::Surface;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_minimize_1d() {
        let mut output = Vec::new();

        let f = |x: f64| (x - 2.5).powi(2) + 4.0;

        for ulim in [(-1.0, 1.0), (1.0, 3.0), (3.0, 10.0)].into_iter() {
            output.push((ulim, minimize_1d(&f, *ulim, 1e-4)));
        }

        for o in output {
            println!("{:?}: {:?}", o.0, o.1);
        }
    }

    #[test]
    fn test_minimize_2d() {
        let mut output = Vec::new();

        let f = |x: f64, y: f64| (x - 2.5).powi(2) + (y + 1.25).powi(2) + 4.0;

        for ulim in [(-1.0, 1.0), (1.0, 3.0), (3.0, 10.0)].into_iter() {
            for vlim in [(-10.0, -2.0), (-2.0, -1.0), (-1.0, 10.0)].into_iter() {
                output.push(((ulim, vlim), minimize_2d(&f, *ulim, *vlim, 1e-6)));
            }
        }

        for o in output {
            println!("{:?}: {:?}", o.0, o.1);
        }
    }
}
