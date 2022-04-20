#![no_std]

mod vector2;
mod vector3;

pub use vector2::Vector2;
pub use vector3::Vector3;

pub type F32x3 = Vector3<f32>;
pub type F32x2 = Vector2<f32>;
