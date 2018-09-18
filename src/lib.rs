use nalgebra::{Rotation3, Point2, Point3, Translation3};

enum Transformation {
    Rotation(Rotation3<f64>),
    Translation(Translation3<f64>),
}

enum Coordinates {
    CartesianZ,
    PolarZ,
}

impl Coordinates {
    fn forward(self) -> Box<Fn(Point2<f64>) -> Point3<f64>> {
        match self {
            Coordinates::CartesianZ => Box::new(|p: Point2<f64>| Point3::new(p.x, p.y, 0.0)),
            Coordinates::PolarZ => Box::new(|p: Point2<f64>| Point3::new(p.x*f64::cos(p.y), p.x*f64::sin(p.y), 0.0)),
        }
    }
}

struct Detector {
    tranformations: Vec<Transformation>,
    init_coords: Coordinates,
    u_lim: (f64, f64),
    v_lim: (f64, f64),
}



#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}
