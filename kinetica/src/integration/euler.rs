use crate::core::RigidBody;

pub struct SemiImplicitEuler;

impl super::Integrator for SemiImplicitEuler {
    fn integrate(&self, bodies: &mut [RigidBody], dt: f32) {
        for body in bodies.iter_mut() {
            if body.inv_mass() == 0.0 {
                continue;
            }

            let inv_mass = body.inv_mass();
            let force = body.force();
            let velocity = body.velocity();

            // v(t+dt) = v(t) + a(t) * dt
            *body.velocity_mut() += force * inv_mass * dt;

            // x(t+dt) = x(t) + v(t+dt) * dt
            *body.position_mut() += velocity * dt;
        }
    }
}
