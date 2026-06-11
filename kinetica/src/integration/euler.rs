use crate::RigidBody;

pub struct SemiImplicitEuler;

impl super::Integrator for SemiImplicitEuler {
    fn integrate(&self, bodies: &mut [RigidBody], dt: f32) {
        for body in bodies.iter_mut() {
            if body.props.inv_mass == 0.0 {
                continue;
            }

            let inv_mass = body.props.inv_mass;
            let force = body.state.force;
            let velocity = body.state.velocity;

            // v(t+dt) = v(t) + a(t) * dt
            body.state.velocity += force * inv_mass * dt;

            // x(t+dt) = x(t) + v(t+dt) * dt
            body.state.position += velocity * dt;
        }
    }
}
