pub mod euler;

pub trait Integrator: Send + Sync {
    fn integrate(&self, bodies: &mut [RigidBody], dt: f32);
}

pub use euler::SemiImplicitEuler;
use crate::RigidBody;