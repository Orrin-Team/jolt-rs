use std::ops::{Add, AddAssign, Div, Mul, MulAssign, Neg, Sub, SubAssign};

use jolt_sys as sys;

/// A 3D vector. Layout-compatible with `JPC_Vec3`.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Vec3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Vec3 {
    pub const ZERO: Self = Self::new(0.0, 0.0, 0.0);
    pub const ONE: Self = Self::new(1.0, 1.0, 1.0);
    pub const X: Self = Self::new(1.0, 0.0, 0.0);
    pub const Y: Self = Self::new(0.0, 1.0, 0.0);
    pub const Z: Self = Self::new(0.0, 0.0, 1.0);

    pub const fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }

    pub const fn splat(v: f32) -> Self {
        Self::new(v, v, v)
    }

    pub fn dot(self, rhs: Self) -> f32 {
        self.x * rhs.x + self.y * rhs.y + self.z * rhs.z
    }

    pub fn cross(self, rhs: Self) -> Self {
        Self::new(
            self.y * rhs.z - self.z * rhs.y,
            self.z * rhs.x - self.x * rhs.z,
            self.x * rhs.y - self.y * rhs.x,
        )
    }

    pub fn length_squared(self) -> f32 {
        self.dot(self)
    }

    pub fn length(self) -> f32 {
        self.length_squared().sqrt()
    }

    /// Returns the zero vector when the length is zero.
    pub fn normalize_or_zero(self) -> Self {
        let len = self.length();
        if len > 0.0 { self / len } else { Self::ZERO }
    }
}

impl Add for Vec3 {
    type Output = Self;
    fn add(self, rhs: Self) -> Self {
        Self::new(self.x + rhs.x, self.y + rhs.y, self.z + rhs.z)
    }
}

impl Sub for Vec3 {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self {
        Self::new(self.x - rhs.x, self.y - rhs.y, self.z - rhs.z)
    }
}

impl Mul<f32> for Vec3 {
    type Output = Self;
    fn mul(self, rhs: f32) -> Self {
        Self::new(self.x * rhs, self.y * rhs, self.z * rhs)
    }
}

impl Mul<Vec3> for f32 {
    type Output = Vec3;
    fn mul(self, rhs: Vec3) -> Vec3 {
        rhs * self
    }
}

impl Div<f32> for Vec3 {
    type Output = Self;
    fn div(self, rhs: f32) -> Self {
        Self::new(self.x / rhs, self.y / rhs, self.z / rhs)
    }
}

impl Neg for Vec3 {
    type Output = Self;
    fn neg(self) -> Self {
        Self::new(-self.x, -self.y, -self.z)
    }
}

impl AddAssign for Vec3 {
    fn add_assign(&mut self, rhs: Self) {
        *self = *self + rhs;
    }
}

impl SubAssign for Vec3 {
    fn sub_assign(&mut self, rhs: Self) {
        *self = *self - rhs;
    }
}

impl MulAssign<f32> for Vec3 {
    fn mul_assign(&mut self, rhs: f32) {
        *self = *self * rhs;
    }
}

impl From<[f32; 3]> for Vec3 {
    fn from([x, y, z]: [f32; 3]) -> Self {
        Self::new(x, y, z)
    }
}

impl From<Vec3> for [f32; 3] {
    fn from(v: Vec3) -> Self {
        [v.x, v.y, v.z]
    }
}

/// A rotation quaternion. Layout-compatible with `JPC_Quat`.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Quat {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub w: f32,
}

impl Default for Quat {
    fn default() -> Self {
        Self::IDENTITY
    }
}

impl Quat {
    pub const IDENTITY: Self = Self::from_xyzw(0.0, 0.0, 0.0, 1.0);

    pub const fn from_xyzw(x: f32, y: f32, z: f32, w: f32) -> Self {
        Self { x, y, z, w }
    }

    /// `axis` must be normalized.
    pub fn from_axis_angle(axis: Vec3, angle: f32) -> Self {
        let (s, c) = (angle * 0.5).sin_cos();
        Self::from_xyzw(axis.x * s, axis.y * s, axis.z * s, c)
    }

    pub fn from_rotation_x(angle: f32) -> Self {
        Self::from_axis_angle(Vec3::X, angle)
    }

    pub fn from_rotation_y(angle: f32) -> Self {
        Self::from_axis_angle(Vec3::Y, angle)
    }

    pub fn from_rotation_z(angle: f32) -> Self {
        Self::from_axis_angle(Vec3::Z, angle)
    }

    pub fn conjugate(self) -> Self {
        Self::from_xyzw(-self.x, -self.y, -self.z, self.w)
    }

    pub fn length(self) -> f32 {
        (self.x * self.x + self.y * self.y + self.z * self.z + self.w * self.w).sqrt()
    }

    pub fn normalize(self) -> Self {
        let len = self.length();
        Self::from_xyzw(self.x / len, self.y / len, self.z / len, self.w / len)
    }

    pub fn rotate(self, v: Vec3) -> Vec3 {
        let q = Vec3::new(self.x, self.y, self.z);
        let t = 2.0 * q.cross(v);
        v + self.w * t + q.cross(t)
    }
}

impl Mul for Quat {
    type Output = Self;
    fn mul(self, r: Self) -> Self {
        Self::from_xyzw(
            self.w * r.x + self.x * r.w + self.y * r.z - self.z * r.y,
            self.w * r.y - self.x * r.z + self.y * r.w + self.z * r.x,
            self.w * r.z + self.x * r.y - self.y * r.x + self.z * r.w,
            self.w * r.w - self.x * r.x - self.y * r.y - self.z * r.z,
        )
    }
}

impl Mul<Vec3> for Quat {
    type Output = Vec3;
    fn mul(self, v: Vec3) -> Vec3 {
        self.rotate(v)
    }
}

/// A column-major 4x4 matrix. Layout-compatible with `JPC_Mat44`.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Mat44 {
    pub columns: [[f32; 4]; 4],
}

impl Default for Mat44 {
    fn default() -> Self {
        Self::IDENTITY
    }
}

impl Mat44 {
    pub const IDENTITY: Self = Self {
        columns: [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]],
    };

    pub fn from_rotation_translation(rotation: Quat, translation: Vec3) -> Self {
        let x = rotation.rotate(Vec3::X);
        let y = rotation.rotate(Vec3::Y);
        let z = rotation.rotate(Vec3::Z);
        Self {
            columns: [
                [x.x, x.y, x.z, 0.0],
                [y.x, y.y, y.z, 0.0],
                [z.x, z.y, z.z, 0.0],
                [translation.x, translation.y, translation.z, 1.0],
            ],
        }
    }

    pub fn from_translation(translation: Vec3) -> Self {
        Self::from_rotation_translation(Quat::IDENTITY, translation)
    }

    pub fn translation(&self) -> Vec3 {
        let [x, y, z, _] = self.columns[3];
        Vec3::new(x, y, z)
    }

    pub fn transform_point(&self, p: Vec3) -> Vec3 {
        let c = &self.columns;
        Vec3::new(
            c[0][0] * p.x + c[1][0] * p.y + c[2][0] * p.z + c[3][0],
            c[0][1] * p.x + c[1][1] * p.y + c[2][1] * p.z + c[3][1],
            c[0][2] * p.x + c[1][2] * p.y + c[2][2] * p.z + c[3][2],
        )
    }
}

/// An axis aligned bounding box.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct AABox {
    pub min: Vec3,
    pub max: Vec3,
}

impl AABox {
    pub fn center(&self) -> Vec3 {
        (self.min + self.max) * 0.5
    }

    pub fn extent(&self) -> Vec3 {
        (self.max - self.min) * 0.5
    }
}

const _: () = {
    use std::mem::{align_of, size_of};
    assert!(size_of::<Vec3>() == size_of::<sys::JPC_Vec3>() && align_of::<Vec3>() == align_of::<sys::JPC_Vec3>());
    assert!(size_of::<Quat>() == size_of::<sys::JPC_Quat>() && align_of::<Quat>() == align_of::<sys::JPC_Quat>());
    assert!(size_of::<Mat44>() == size_of::<sys::JPC_Mat44>() && align_of::<Mat44>() == align_of::<sys::JPC_Mat44>());
};

impl From<Vec3> for sys::JPC_Vec3 {
    fn from(v: Vec3) -> Self {
        Self { x: v.x, y: v.y, z: v.z }
    }
}

impl From<sys::JPC_Vec3> for Vec3 {
    fn from(v: sys::JPC_Vec3) -> Self {
        Self::new(v.x, v.y, v.z)
    }
}

impl From<Quat> for sys::JPC_Quat {
    fn from(q: Quat) -> Self {
        Self { x: q.x, y: q.y, z: q.z, w: q.w }
    }
}

impl From<sys::JPC_Quat> for Quat {
    fn from(q: sys::JPC_Quat) -> Self {
        Self::from_xyzw(q.x, q.y, q.z, q.w)
    }
}

impl From<Mat44> for sys::JPC_Mat44 {
    fn from(m: Mat44) -> Self {
        let col = |c: [f32; 4]| sys::JPC_Vec4 { x: c[0], y: c[1], z: c[2], w: c[3] };
        Self { columns: m.columns.map(col) }
    }
}

impl From<sys::JPC_Mat44> for Mat44 {
    fn from(m: sys::JPC_Mat44) -> Self {
        Self { columns: m.columns.map(|c| [c.x, c.y, c.z, c.w]) }
    }
}

impl From<sys::JPC_AABox> for AABox {
    fn from(b: sys::JPC_AABox) -> Self {
        Self { min: b.min.into(), max: b.max.into() }
    }
}

#[cfg(feature = "glam")]
mod glam_impls {
    use super::{Mat44, Quat, Vec3};

    impl From<glam::Vec3> for Vec3 {
        fn from(v: glam::Vec3) -> Self {
            Self::new(v.x, v.y, v.z)
        }
    }

    impl From<Vec3> for glam::Vec3 {
        fn from(v: Vec3) -> Self {
            Self::new(v.x, v.y, v.z)
        }
    }

    impl From<glam::Quat> for Quat {
        fn from(q: glam::Quat) -> Self {
            Self::from_xyzw(q.x, q.y, q.z, q.w)
        }
    }

    impl From<Quat> for glam::Quat {
        fn from(q: Quat) -> Self {
            Self::from_xyzw(q.x, q.y, q.z, q.w)
        }
    }

    impl From<glam::Mat4> for Mat44 {
        fn from(m: glam::Mat4) -> Self {
            Self { columns: m.to_cols_array_2d() }
        }
    }

    impl From<Mat44> for glam::Mat4 {
        fn from(m: Mat44) -> Self {
            Self::from_cols_array_2d(&m.columns)
        }
    }
}
