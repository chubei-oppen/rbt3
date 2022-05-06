//! Rigid body transform.

#![warn(
    explicit_outlives_requirements,
    keyword_idents,
    macro_use_extern_crate,
    meta_variable_misuse,
    missing_abi,
    missing_docs,
    missing_debug_implementations,
    noop_method_call,
    pointer_structural_match,
    trivial_casts,
    trivial_numeric_casts,
    unsafe_code,
    unsafe_op_in_unsafe_fn,
    unused_import_braces,
    unused_lifetimes,
    unused_qualifications,
    unused_results,
    variant_size_differences
)]

pub use glam::{Mat3, Mat4, Quat, Vec3, Vec4};
use std::ops::Mul;

#[derive(Debug, Clone, PartialEq, Default)]
/// A rigid body transform.
pub struct Rbt {
    /// The translation part.
    pub translation: Vec3,
    /// The rotation part.
    pub rotation: Quat,
}

impl Rbt {
    /// Returns the identity transform.
    pub fn new() -> Self {
        Default::default()
    }

    /// Creates a Rbt from translation with identity rotation.
    pub fn from_t(translation: Vec3) -> Self {
        Rbt {
            translation,
            rotation: Quat::default(),
        }
    }

    /// Creates a Rbt from rotation with zero translation.
    pub fn from_r(rotation: Quat) -> Self {
        Rbt {
            translation: Vec3::default(),
            rotation,
        }
    }

    /// Creates a Rbt from translation and rotation.
    pub fn from_t_r(translation: Vec3, rotation: Quat) -> Self {
        Rbt {
            translation,
            rotation,
        }
    }

    /// Converts from a 4x4 matrix.
    ///
    /// Won't check for rigidness.
    pub fn from_mat4(mat4: &Mat4) -> Self {
        Rbt {
            translation: mat4.w_axis.truncate(),
            rotation: Quat::from_mat4(mat4),
        }
    }

    /// Converts to a 4x4 matrix.
    pub fn to_mat4(&self) -> Mat4 {
        let mut result = Mat4::from_quat(self.rotation);
        result.w_axis = self.translation.extend(1.0);
        result
    }

    /// Returns the inverse of `self`.
    pub fn inverse(&self) -> Self {
        let r = self.rotation.as_ref();
        let inv_r = Quat::from_xyzw(r[0], r[1], r[2], -r[3]);
        Rbt {
            translation: -(inv_r * self.translation),
            rotation: inv_r,
        }
    }

    /// Perform transform `m` to coordinate system `o` with repect to coordinate system `a`.
    pub fn do_m_to_o_wrt_a(m: &Rbt, o: &Rbt, a: &Rbt) -> Rbt {
        a * m * a.inverse() * o
    }
}

impl Mul for &Rbt {
    type Output = Rbt;

    fn mul(self, rhs: &Rbt) -> Rbt {
        Rbt {
            translation: self.rotation * rhs.translation + self.translation,
            rotation: (self.rotation * rhs.rotation).normalize(),
        }
    }
}

impl Mul<Vec4> for &Rbt {
    type Output = Vec4;

    fn mul(self, rhs: Vec4) -> Vec4 {
        (self.rotation * rhs.truncate() + self.translation * rhs.w).extend(rhs.w)
    }
}

impl Mul<&Rbt> for Rbt {
    type Output = Rbt;

    fn mul(self, rhs: &Rbt) -> Rbt {
        &self * rhs
    }
}

impl Mul for Rbt {
    type Output = Rbt;

    fn mul(self, rhs: Rbt) -> Rbt {
        &self * &rhs
    }
}

impl Mul<Vec4> for Rbt {
    type Output = Vec4;

    fn mul(self, rhs: Vec4) -> Vec4 {
        &self * rhs
    }
}
