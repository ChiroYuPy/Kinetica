use crate::core::RigidBody;
use crate::math::Vec2;

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
        let force = self.acceleration * body.mass();
        *body.force_mut() += force;
    }
}
