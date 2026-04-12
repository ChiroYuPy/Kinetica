use crate::core::RigidBody;

struct SemiImplicitEuler;

impl super::Integrator for SemiImplicitEuler {
    fn integrate(&self, bodies: &mut [RigidBody], dt: f32) {
        for body in bodies.iter_mut() {
            if body.inv_mass == 0.0 {
                continue;
            }

            // v(t+dt) = v(t) + a(t) * dt
            body.velocity += body.force * body.inv_mass * dt;

            // x(t+dt) = x(t) + v(t+dt) * dt
            body.position += body.velocity * dt;
        }

        self.reset_forces(bodies);
    }
}