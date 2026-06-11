mod linear_gravity;

pub trait ForceGenerator: Send + Sync {
    fn apply(&self, body: &mut RigidBody);

    fn apply_to_all(&self, bodies: &mut [RigidBody]) {
        for body in bodies.iter_mut() {
            if body.mass.inverse_mass > 0.0 {
                self.apply(body);
            }
        }
    }
}

pub use linear_gravity::LinearGravity;
use crate::RigidBody;