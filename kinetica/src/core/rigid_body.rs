use crate::math::Vec2;
use crate::shape::Shape;

pub struct RigidBody {
    pub position: Vec2,
    pub velocity: Vec2,
    pub force: Vec2,

    pub mass: f32,
    pub inv_mass: f32,

    pub shape: Shape,
}

impl RigidBody {
    pub fn new(position: Vec2, mass: f32, shape: Shape) -> Self {
        let inv_mass = if mass == 0.0 { 0.0 } else { 1.0 / mass };
        
        Self {
            position,
            velocity: Vec2::ZERO,
            force: Vec2::ZERO,
            mass,
            inv_mass,
            shape,
        }
    }

    pub fn apply_force(&mut self, force: Vec2) {
        self.force += force;
    }

    pub fn apply_impulse(&mut self, impulse: Vec2) {
        self.velocity += impulse * self.inv_mass;
    }
}