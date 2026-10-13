//! # Kinetica collision
//!
//! 2D shapes and collision detection, without any notion of bodies, forces or time: the
//! geometry the [Kinetica](https://docs.rs/kinetica) physics engine is built on.
//!
//! - [`shape`]: circles, capsules, convex polygons and segments, with their mass properties,
//!   bounding boxes and ray casts.
//! - [`collide`]: the contact points between two placed shapes, as a [`Manifold`].
//! - [`separation_lower_bound`] and [`time_of_impact`]: how far apart two shapes are, and when
//!   a moving one reaches the other.
//! - [`DynamicTree`]: a bounding box hierarchy to find which of many boxes overlap.
//!
//! Units are meters and kilograms; the Y axis points up.
//!
//! ```
//! use kinetica_collision::math::{Rot, Transform, vec2};
//! use kinetica_collision::shape::{Circle, Shape};
//! use kinetica_collision::collide;
//!
//! let ball: Shape = Circle::new(0.5).into();
//! let left = Transform::new(vec2(0.0, 0.0), Rot::IDENTITY);
//! let right = Transform::new(vec2(0.9, 0.0), Rot::IDENTITY);
//!
//! let manifold = collide(&ball, &left, &ball, &right);
//! assert_eq!(manifold.normal(), vec2(1.0, 0.0));
//! assert!(manifold.points()[0].separation() < 0.0);
//! ```

#![warn(missing_docs)]

mod constants;
mod dynamic_tree;
mod manifold;
pub mod math;
mod narrow_phase;
pub mod shape;

pub use constants::{LINEAR_SLOP, SPECULATIVE_DISTANCE};
pub use dynamic_tree::{DynamicTree, ProxyId};
pub use manifold::{Manifold, ManifoldPoint};
pub use narrow_phase::{collide, separation_lower_bound, time_of_impact};
