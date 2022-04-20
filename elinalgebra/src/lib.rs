#![no_std]

mod vector3;
mod vector2;

pub use vector3::Vector3;
pub use vector2::Vector2;

pub type F32x3 = Vector3<f32>;
pub type F32x2 = Vector2<f32>;
