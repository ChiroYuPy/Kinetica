//! Math types used by the shapes and the collision routines.
//!
//! Vectors come from [`glam`]; this crate adds the few 2D types glam does not provide.

mod aabb;
mod rot;
mod transform;

pub use aabb::Aabb;
pub use glam::{Vec2, vec2};
pub use rot::Rot;
pub use transform::Transform;
