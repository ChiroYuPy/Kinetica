use crate::RigidBody;

pub struct SemiImplicitEuler;

impl super::Integrator for SemiImplicitEuler {
    fn integrate(&self, bodies: &mut [RigidBody], dt: f32) {
        for body in bodies.iter_mut() {
            if body.mass.inverse_mass == 0.0 {
                continue;
            }

            let inv_mass = body.mass.inverse_mass;
            let force = body.motion.accumulated_force;
            let velocity = body.motion.linear_velocity;

            // v(t+dt) = v(t) + a(t) * dt
            body.motion.linear_velocity += force * inv_mass * dt;

            // x(t+dt) = x(t) + v(t+dt) * dt
            body.transform.position += velocity * dt;
        }
    }
}
