pub mod euler;

use crate::core::RigidBody;

pub trait Integrator: Send + Sync {
    fn integrate(&self, bodies: &mut [RigidBody], dt: f32);
}

pub use euler::SemiImplicitEuler;