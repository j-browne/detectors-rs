use super::Unc;
use serde::{Deserialize, Deserializer, Serialize, Serializer};
use val_unc::{traits::UncZero, ValUnc};

pub(super) fn serialize<S>(
    v: &ValUnc<f64, Unc>,
    serializer: S,
) -> core::result::Result<S::Ok, S::Error>
where
    S: Serializer,
{
    ValUncTuple::from(*v).serialize(serializer)
}

pub(super) fn deserialize<'de, D>(
    deserializer: D,
) -> core::result::Result<ValUnc<f64, Unc>, D::Error>
where
    D: Deserializer<'de>,
{
    ValUncTuple::deserialize(deserializer).map(ValUnc::from)
}

#[derive(Serialize, Deserialize, Debug, Clone)]
#[serde(untagged)]
pub(super) enum ValUncTuple {
    NoUnc(f64),
    Unc(f64, Unc),
}

impl From<ValUncTuple> for ValUnc<f64, Unc> {
    fn from(v: ValUncTuple) -> ValUnc<f64, Unc> {
        match v {
            ValUncTuple::NoUnc(val) => val.into(),
            ValUncTuple::Unc(val, unc) => ValUnc { val, unc },
        }
    }
}

impl From<ValUnc<f64, Unc>> for ValUncTuple {
    fn from(v: ValUnc<f64, Unc>) -> ValUncTuple {
        if UncZero::is_zero(&v.unc) {
            ValUncTuple::NoUnc(v.val)
        } else {
            ValUncTuple::Unc(v.val, v.unc)
        }
    }
}
