//! Collision detection: which shapes touch, and where.
//!
//! The [`World`](crate::World) runs this automatically every step and exposes the results as
//! [`Contact`](crate::Contact)s. [`collide`] is public for direct use on shapes.

pub(crate) mod broad_phase;
mod manifold;
mod narrow_phase;

pub use manifold::{Manifold, ManifoldPoint};
pub use narrow_phase::collide;
