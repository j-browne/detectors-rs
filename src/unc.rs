use std::ops::{Deref, DerefMut, Div, Mul};
use val_unc::traits::*;

mod val_unc_serde;

#[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Default, Serialize, Deserialize)]
#[repr(transparent)]
pub struct ValUnc(#[serde(with = "val_unc_serde")] pub val_unc::ValUnc<f64, Unc>);

impl ValUnc {
    pub fn new(val: f64, unc: Unc) -> Self {
        Self(val_unc::ValUnc { val, unc })
    }
}

impl From<f64> for ValUnc {
    fn from(v: f64) -> Self {
        Self(val_unc::ValUnc::from(v))
    }
}

impl Deref for ValUnc {
    type Target = val_unc::ValUnc<f64, Unc>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DerefMut for ValUnc {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

#[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Default, Serialize, Deserialize)]
#[serde(transparent)]
pub struct Unc(pub f64);

impl From<f64> for Unc {
    fn from(val: f64) -> Self {
        Self(val)
    }
}

impl Mul<Unc> for f64 {
    type Output = Unc;
    fn mul(self, other: Unc) -> Unc {
        Unc(other.0 * self)
    }
}

impl Mul<f64> for Unc {
    type Output = Self;
    fn mul(self, other: f64) -> Self {
        Self(other * self.0)
    }
}

impl Div<f64> for Unc {
    type Output = Self;
    fn div(self, other: f64) -> Self {
        Self(self.0 / other)
    }
}

impl UncAdd<f64> for Unc {
    fn unc_add(self, _self_val: f64, other: Unc, _other_val: f64) -> Unc {
        Unc(f64::sqrt(f64::powi(self.0, 2) + f64::powi(other.0, 2)))
    }
}

impl UncDiv<f64> for Unc {
    fn unc_div(self, self_val: f64, other: Unc, other_val: f64) -> Unc {
        self_val / other_val
            * Unc(f64::sqrt(
                f64::powi(self.0 / self_val, 2) + f64::powi(other.0 / other_val, 2),
            ))
    }
}

impl UncMul<f64> for Unc {
    fn unc_mul(self, self_val: f64, other: Unc, other_val: f64) -> Unc {
        self_val
            * other_val
            * Unc(f64::sqrt(
                f64::powi(self.0 / self_val, 2) + f64::powi(other.0 / other_val, 2),
            ))
    }
}

impl UncNeg<f64> for Unc {
    fn unc_neg(self, _self_val: f64) -> Unc {
        self
    }
}

impl UncSub<f64> for Unc {
    fn unc_sub(self, _self_val: f64, other: Unc, _other_val: f64) -> Unc {
        Unc(f64::sqrt(f64::powi(self.0, 2) + f64::powi(other.0, 2)))
    }
}

impl UncZero for Unc {
    fn zero() -> Self {
        Unc(UncZero::zero())
    }

    fn is_zero(&self) -> bool {
        UncZero::is_zero(&self.0)
    }

    fn set_zero(&mut self) {
        UncZero::set_zero(&mut self.0)
    }
}
