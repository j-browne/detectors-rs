use nalgebra::{Rotation3, Point2, Point3, Translation3};

pub enum Transformation {
    Rotation(Rotation3<f64>),
    Translation(Translation3<f64>),
}

impl From<Rotation3<f64>> for Transformation {
    fn from(r: Rotation3<f64>) -> Transformation {
        Transformation::Rotation(r)
    }
}

impl From<Translation3<f64>> for Transformation {
    fn from(r: Translation3<f64>) -> Transformation {
        Transformation::Translation(r)
    }
}

enum Coordinates {
    CartesianZ,
    PolarZ,
}

impl Coordinates {
    fn forward(&self, p: Point2<f64>) -> Point3<f64> {
        match self {
            Coordinates::CartesianZ => Point3::new(p.x, p.y, 0.0),
            Coordinates::PolarZ =>  Point3::new(p.x * f64::cos(p.y), p.x * f64::sin(p.y), 0.0),
        }
    }
}

pub struct Detector {
    transformations: Vec<Transformation>,
    init_coords: Coordinates,
    u_lim: (f64, f64),
    v_lim: (f64, f64),
}

impl Detector {
    pub fn forward(&self, p: Point2<f64>) -> Point3<f64> {
        let mut res = self.init_coords.forward(p);
        for t in &self.transformations {
            res = match t {
                Transformation::Rotation(t) => t * res,
                Transformation::Translation(t) => t * res,
            }
        }
        res
    }

    pub fn new_cartesian() -> Self {
        Self {
            transformations: Vec::new(),
            init_coords: Coordinates::CartesianZ,
            u_lim: (0.0, 0.0),
            v_lim: (0.0, 0.0),
        }
    }

    pub fn new_polar() -> Self {
        Self {
            transformations: Vec::new(),
            init_coords: Coordinates::PolarZ,
            u_lim: (0.0, 0.0),
            v_lim: (0.0, 0.0),
        }
    }

    pub fn add_transformation(&mut self, t: Transformation) {
        self.transformations.push(t);
    }
}


#[cfg(test)]
mod tests {
    use nalgebra::{Rotation3, Point2, Translation3, Vector3};
    use super::*;
    const pi: f64 = std::f64::consts::PI;

    #[test]
    fn bb10() {
        let o_x = 0.0;
        let o_y = 0.0;
        let o_z = 0.0;
        let s_x = 0.0;
        let s_y = 0.0;
        let s_z = 1.0;
        let a = pi / 6.0;
        let b = 0.0;
        let mut d = Detector::new_cartesian();
        d.add_transformation(Translation3::new(o_x, o_y, o_z).into());
        d.add_transformation(Rotation3::new(Vector3::new(0., 0., b)).into());
        d.add_transformation(Translation3::new(s_x, s_y, s_z).into());
        d.add_transformation(Rotation3::new(Vector3::new(a, 0., 0.)).into());
        println!("{:?}", d.forward(Point2::new(1.0, 0.0)));
    }
}
