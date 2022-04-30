#![no_std]

//! A platform-agnostic linear algebra library focused on embedded enviornments.

mod utils;
mod vector2;
mod vector3;

pub use utils::*;
pub use vector2::Vector2;
pub use vector3::Vector3;

pub type F32x3 = Vector3<f32>;
pub type F32x2 = Vector2<f32>;

pub fn to_vec3<T>(vec2: &Vector2<T>, z: T) -> Vector3<T>
where
    T: Copy,
{
    Vector3::<T>::new(vec2.x, vec2.y, z)
}
