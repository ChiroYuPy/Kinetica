use crate::math::Vec2;
use crate::RigidBody;

pub struct LinearGravity {
    pub acceleration: Vec2,
}

impl LinearGravity {
    pub fn new(acceleration: Vec2) -> Self {
        Self { acceleration }
    }
}

impl super::ForceGenerator for LinearGravity {
    fn apply(&self, body: &mut RigidBody) {
        let force = self.acceleration * body.mass.mass;
        body.motion.accumulated_force += force;
    }
}
