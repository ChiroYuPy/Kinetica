pub mod euler;

use crate::core::RigidBody;

pub trait Integrator: Send + Sync {
    fn integrate(&self, bodies: &mut [RigidBody], dt: f32);

    fn reset_forces(&self, bodies: &mut [RigidBody]) {
        for body in bodies.iter_mut() {
            body.force = crate::math::Vec2::ZERO;
        }
    }
}

pub use euler::SemiImplicitEuler;