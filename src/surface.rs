use crate::{error::Error, CoordinateSystem, Transformation};
use std::collections::HashMap;

pub type Surface = SurfaceBase;

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct SurfaceBase {
    coords: CoordinateSystem,
    u_limits: (f64, f64),
    v_limits: (f64, f64),
    #[serde(default, rename = "transformations", skip_serializing_if = "Vec::is_empty")]
    trans: Vec<Transformation>,
}

impl SurfaceBase {
    pub fn trans(&self) -> &Vec<Transformation> {
        &self.trans
    }

    pub fn trans_mut(&mut self) -> &mut Vec<Transformation> {
        &mut self.trans
    }

    pub fn add_trans(&mut self, t: &mut Vec<Transformation>) {
        self.trans_mut().append(t);
    }

    pub fn simplify(&self, id: Vec<u32>) -> (Vec<u32>, Surface) {
        (id, self.clone())
    }
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct SurfaceGroup {
    surfaces: Vec<NotTemplate>,
    #[serde(default, rename = "transformations", skip_serializing_if = "Vec::is_empty")]
    trans: Vec<Transformation>,
}

impl SurfaceGroup {
    pub fn trans(&self) -> &Vec<Transformation> {
        &self.trans
    }

    pub fn trans_mut(&mut self) -> &mut Vec<Transformation> {
        &mut self.trans
    }

    pub fn add_trans(&mut self, t: &mut Vec<Transformation>) {
        self.trans_mut().append(t);
    }

    pub fn simplify(&self, mut id: Vec<u32>) -> Vec<(Vec<u32>, Surface)> {
        id.push(0);
        let mut simplified = Vec::new();
        for s in &self.surfaces {
            let mut new = s.simplify(id.clone());

            for (_id, s) in new.iter_mut() {
                s.add_trans(&mut self.trans().clone());
            }

            simplified.append(&mut new);
            let last = id.last_mut()
                .expect("cannot get last element of a vector (that should exist)");
            *last += 1;
        }
        simplified
    }
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct SurfaceTemplate {
    name: String,
    #[serde(default, rename = "transformations", skip_serializing_if = "Vec::is_empty")]
    trans: Vec<Transformation>,
}

impl SurfaceTemplate {
    pub fn trans(&self) -> &Vec<Transformation> {
        &self.trans
    }

    pub fn trans_mut(&mut self) -> &mut Vec<Transformation> {
        &mut self.trans
    }

    pub fn add_trans(&mut self, t: &mut Vec<Transformation>) {
        self.trans_mut().append(t);
    }

    pub fn simplify(&self, id: Vec<u32>) -> Vec<(Vec<u32>, Surface)> {
        unimplemented!()
    }
}


#[derive(Serialize, Deserialize, Debug, Clone)]
#[serde(untagged)]
pub enum NotTemplate {
    Base {
        #[serde(flatten)]
        surface: SurfaceBase,
    },
    Group{
        #[serde(flatten)]
        surface: SurfaceGroup,
    },
}

impl NotTemplate {
    pub fn trans(&self) -> &Vec<Transformation> {
        match self {
            NotTemplate::Base {surface: s} => s.trans(),
            NotTemplate::Group {surface: s} => s.trans(),
        }
    }

    pub fn trans_mut(&mut self) -> &mut Vec<Transformation> {
        match self {
            NotTemplate::Base {surface: s} => s.trans_mut(),
            NotTemplate::Group {surface: s} => s.trans_mut(),
        }
    }

    pub fn add_trans(&mut self, t: &mut Vec<Transformation>) {
        self.trans_mut().append(t);
    }

    pub fn simplify(&self, id: Vec<u32>) -> Vec<(Vec<u32>, Surface)> {
        match self {
            NotTemplate::Base {surface: s} => vec![s.simplify(id)],
            NotTemplate::Group {surface: s} => s.simplify(id),
        }
    }
}

#[derive(Serialize, Deserialize, Debug, Clone)]
#[serde(untagged)]
pub enum MaybeTemplate {
    NotTemplate {
        #[serde(flatten)]
        surface: NotTemplate,
    },
    Template {
        #[serde(flatten)]
        template: SurfaceTemplate,
    },
}

impl MaybeTemplate {
    pub fn trans(&self) -> &Vec<Transformation> {
        match self {
            MaybeTemplate::Template {template: t} => t.trans(),
            MaybeTemplate::NotTemplate {surface: s} => s.trans(),
        }
    }

    pub fn trans_mut(&mut self) -> &mut Vec<Transformation> {
        match self {
            MaybeTemplate::Template {template: t} => t.trans_mut(),
            MaybeTemplate::NotTemplate {surface: s} => s.trans_mut(),
        }
    }

    pub fn apply_templates(&mut self, templates: &HashMap<String, NotTemplate>) -> Result<(), Error>{
        match self {
            MaybeTemplate::Template{ template: t } => {
                let mut temp = MaybeTemplate::NotTemplate{
                    surface: templates
                        .get(&t.name)
                        .ok_or(Error::UnknownTemplate)?
                        .clone()
                };
                temp.trans_mut().append(self.trans_mut());
                *self = temp;
            }
            _ => {}
        }

        Ok(())
    }

    pub fn simplify(&self, id: Vec<u32>) -> Vec<(Vec<u32>, Surface)> {
        match self {
            MaybeTemplate::Template {template: t} => t.simplify(id),
            MaybeTemplate::NotTemplate {surface: s} => s.simplify(id),
        }
    }
}
