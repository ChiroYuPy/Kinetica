mod contact;

pub use contact::{ContactConstraint, ContactManifold};

use crate::core::RigidBody;

pub trait Constraint: Send + Sync {
    fn solve_velocity(&mut self, bodies: &mut [RigidBody]);
    fn solve_position(&mut self, bodies: &mut [RigidBody]);
}
