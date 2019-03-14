use crate::{error::Error, CoordinateSystem, Transformation};
use nalgebra::{Point2, Point3};
use std::collections::HashMap;

pub type Surface = Base;

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct Base {
    coords: CoordinateSystem,
    u_limits: (f64, f64),
    v_limits: (f64, f64),
    #[serde(
        default,
        rename = "transformations",
        skip_serializing_if = "Vec::is_empty"
    )]
    trans: Vec<Transformation>,
}

impl Base {
    pub fn u_limits(&self) -> (f64, f64) {
        self.u_limits
    }

    pub fn u_limits_mut(&mut self) -> &mut (f64, f64) {
        &mut self.u_limits
    }

    pub fn v_limits(&self) -> (f64, f64) {
        self.v_limits
    }

    pub fn v_limits_mut(&mut self) -> &mut (f64, f64) {
        &mut self.v_limits
    }

    pub fn trans(&self) -> &Vec<Transformation> {
        &self.trans
    }

    pub fn trans_mut(&mut self) -> &mut Vec<Transformation> {
        &mut self.trans
    }

    pub fn add_trans(&mut self, t: &mut Vec<Transformation>) {
        self.trans_mut().append(t);
    }

    /// Converts a `Base` to a `Surface` with an id.
    ///
    /// This just clones `self` and adds the id because `Surface`
    /// is just an alias of `Base`.
    pub fn simplify(&self, id: Vec<u32>) -> (Vec<u32>, Surface) {
        (id, self.clone())
    }

    pub fn coords_local_to_world(&self, p: Point2<f64>) -> Point3<f64> {
        let mut res = self.coords.local_to_world(p);
        for t in &self.trans {
            res = match t {
                Transformation::Rotation(t) => t * res,
                Transformation::Translation(t) => t * res,
            };
        }
        res
    }

    pub fn coords_world_to_local(&self, mut p: Point3<f64>) -> Point3<f64> {
        for t in self.trans.iter().rev() {
            p = match t {
                Transformation::Rotation(t) => t.inverse() * p,
                Transformation::Translation(t) => t.inverse() * p,
            };
        }
        self.coords.world_to_local(p)
    }

    // TODO: If the two z values are the same you can get inf.
    pub fn intersects(&self, p_src_world: Point3<f64>, p_dest_world: Point3<f64>) -> bool {
        let p1 = self.coords_world_to_local(p_dest_world);
        let p2 = self.coords_world_to_local(p_src_world);

        let int = Point2::new(
            -(p1.x - p2.x) / (p1.z - p2.z) * p1.z + p1.x,
            -(p1.y - p2.y) / (p1.z - p2.z) * p1.z + p1.y,
        );

        // Make sure the point is within u and v limits
        //  AND the surface is between src and dest
        int.x >= self.u_limits.0
            && int.x <= self.u_limits.1
            && int.y >= self.v_limits.0
            && int.y <= self.v_limits.1
            && (p1.z.signum() * p2.z.signum() - -1.0).abs() < std::f64::EPSILON
    }
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub(crate) struct Group {
    surfaces: Vec<NotTemplate>,
    #[serde(
        default,
        rename = "transformations",
        skip_serializing_if = "Vec::is_empty"
    )]
    trans: Vec<Transformation>,
}

impl Group {
    pub fn trans(&self) -> &Vec<Transformation> {
        &self.trans
    }

    pub fn trans_mut(&mut self) -> &mut Vec<Transformation> {
        &mut self.trans
    }

    #[allow(dead_code)]
    pub fn add_trans(&mut self, t: &mut Vec<Transformation>) {
        self.trans_mut().append(t);
    }

    /// Converts a `Group` to a `Surface` with an id.
    ///
    /// This iterates through the `NotTemplate` elements in `surfaces`, and
    /// calls `simplify` on each element. It then adds `self`'s transformations
    /// and shadows to the simplified surface's transformations and shadows.
    pub fn simplify(&self, mut id: Vec<u32>) -> Vec<(Vec<u32>, Surface)> {
        id.push(0);
        let mut simplified = Vec::new();
        for s in &self.surfaces {
            let mut new = s.simplify(id.clone());

            for (_id, s) in new.iter_mut() {
                s.add_trans(&mut self.trans().clone());
            }

            simplified.append(&mut new);
            let last = id
                .last_mut()
                .expect("cannot get last element of a vector (that should exist)");
            *last += 1;
        }
        simplified
    }
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub(crate) struct Template {
    name: String,
    #[serde(
        default,
        rename = "transformations",
        skip_serializing_if = "Vec::is_empty"
    )]
    trans: Vec<Transformation>,
}

impl Template {
    pub fn trans(&self) -> &Vec<Transformation> {
        &self.trans
    }

    #[allow(dead_code)]
    pub fn trans_mut(&mut self) -> &mut Vec<Transformation> {
        &mut self.trans
    }

    #[allow(dead_code)]
    pub fn add_trans(&mut self, t: &mut Vec<Transformation>) {
        self.trans_mut().append(t);
    }
}

#[derive(Serialize, Deserialize, Debug, Clone)]
#[serde(untagged)]
pub(crate) enum NotTemplate {
    Base {
        #[serde(flatten)]
        surface: Base,
    },
    Group {
        #[serde(flatten)]
        surface: Group,
    },
}

impl NotTemplate {
    pub fn trans(&self) -> &Vec<Transformation> {
        match self {
            NotTemplate::Base { surface: s } => s.trans(),
            NotTemplate::Group { surface: s } => s.trans(),
        }
    }

    pub fn trans_mut(&mut self) -> &mut Vec<Transformation> {
        match self {
            NotTemplate::Base { surface: s } => s.trans_mut(),
            NotTemplate::Group { surface: s } => s.trans_mut(),
        }
    }

    #[allow(dead_code)]
    pub fn add_trans(&mut self, t: &mut Vec<Transformation>) {
        self.trans_mut().append(t);
    }

    /// Converts a `NotTemplate` to a `Surface` with an id.
    ///
    /// This calls the appropriate `simplify` function, depending on which
    /// variant `self` is.
    pub fn simplify(&self, id: Vec<u32>) -> Vec<(Vec<u32>, Surface)> {
        match self {
            NotTemplate::Base { surface: s } => vec![s.simplify(id)],
            NotTemplate::Group { surface: s } => s.simplify(id),
        }
    }
}

#[derive(Serialize, Deserialize, Debug, Clone)]
#[serde(untagged)]
pub(crate) enum MaybeTemplate {
    NotTemplate {
        #[serde(flatten)]
        surface: NotTemplate,
    },
    Template {
        #[serde(flatten)]
        template: Template,
    },
}

impl MaybeTemplate {
    pub fn trans(&self) -> &Vec<Transformation> {
        match self {
            MaybeTemplate::Template { template: t } => t.trans(),
            MaybeTemplate::NotTemplate { surface: s } => s.trans(),
        }
    }

    #[allow(dead_code)]
    pub fn trans_mut(&mut self) -> &mut Vec<Transformation> {
        match self {
            MaybeTemplate::Template { template: t } => t.trans_mut(),
            MaybeTemplate::NotTemplate { surface: s } => s.trans_mut(),
        }
    }

    pub fn apply_templates(
        &self,
        templates: &HashMap<String, NotTemplate>,
    ) -> Result<NotTemplate, Error> {
        match self {
            MaybeTemplate::Template { template: t } => {
                let mut temp = templates
                    .get(&t.name)
                    .ok_or(Error::UnknownTemplate)?
                    .clone();
                temp.trans_mut().extend(self.trans().iter().cloned());
                Ok(temp)
            }
            MaybeTemplate::NotTemplate { surface: s } => Ok(s.clone()),
        }
    }
}
