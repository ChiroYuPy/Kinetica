use crate::constraints::Constraint;
use crate::core::RigidBody;
use crate::math::Vec2;

#[derive(Clone, Debug)]
pub struct ContactManifold {
    pub body_a: usize,
    pub body_b: usize,
    pub normal: Vec2,
    pub penetration: f32,
}

pub struct ContactConstraint {
    pub manifold: ContactManifold,
    pub restitution: f32,
}

impl ContactConstraint {
    pub fn new(manifold: ContactManifold) -> Self {
        Self {
            manifold,
            restitution: 0.5,
        }
    }

    pub fn with_restitution(manifold: ContactManifold, restitution: f32) -> Self {
        Self { manifold, restitution }
    }

    unsafe fn bodies_unsafe<'a>(&self, bodies: &'a mut [RigidBody]) -> (&'a mut RigidBody, &'a mut RigidBody) {
        let a = self.manifold.body_a;
        let b = self.manifold.body_b;
        assert!(a != b && a < bodies.len() && b < bodies.len());
        let ptr = bodies.as_mut_ptr();
        unsafe {
            (&mut *ptr.add(a), &mut *ptr.add(b))
        }
    }
}

impl Constraint for ContactConstraint {
    fn solve_velocity(&mut self, bodies: &mut [RigidBody]) {
        let m = &self.manifold;
        let (body_a, body_b) = unsafe { self.bodies_unsafe(bodies) };

        let inv_mass_a = body_a.props.inv_mass;
        let inv_mass_b = body_b.props.inv_mass;
        let total_inv_mass = inv_mass_a + inv_mass_b;
        if total_inv_mass == 0.0 {
            return;
        }

        let rel_velocity = body_b.state.velocity - body_a.state.velocity;
        let vel_along_normal = rel_velocity.dot(m.normal);

        if vel_along_normal > 0.0 {
            return;
        }

        let j = -(1.0 + self.restitution) * vel_along_normal;
        let j = j / total_inv_mass;

        let impulse = m.normal * j;
        body_a.state.velocity -= impulse * inv_mass_a;
        body_b.state.velocity += impulse * inv_mass_b;
    }

    fn solve_position(&mut self, bodies: &mut [RigidBody]) {
        let m = &self.manifold;
        let (body_a, body_b) = unsafe { self.bodies_unsafe(bodies) };

        let inv_mass_a = body_a.props.inv_mass;
        let inv_mass_b = body_b.props.inv_mass;
        let total_inv_mass = inv_mass_a + inv_mass_b;
        if total_inv_mass == 0.0 {
            return;
        }

        let correction = (m.penetration / total_inv_mass) * 0.8;
        body_a.state.position -= m.normal * (correction * inv_mass_a);
        body_b.state.position += m.normal * (correction * inv_mass_b);
    }
}
