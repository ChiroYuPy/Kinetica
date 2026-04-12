use crate::core::shape::Shape;
use crate::math::Vec2;

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum BodyType {
    Static,
    Dynamic,
}

pub struct RigidBody {
    pub position: Vec2,
    pub velocity: Vec2,
    pub force: Vec2,

    pub body_type: BodyType,
    pub mass: f32,
    pub inv_mass: f32,

    pub shape: Shape,
}

impl RigidBody {
    pub fn new(position: Vec2, mass: f32, shape: Shape) -> Self {
        let inv_mass = if mass == 0.0 { 0.0 } else { 1.0 / mass };
        let body_type = if mass == 0.0 { BodyType::Static } else { BodyType::Dynamic };

        Self {
            position,
            velocity: Vec2::ZERO,
            force: Vec2::ZERO,
            body_type,
            mass,
            inv_mass,
            shape,
        }
    }

    pub fn static_body(position: Vec2, shape: Shape) -> Self {
        Self {
            position,
            velocity: Vec2::ZERO,
            force: Vec2::ZERO,
            body_type: BodyType::Static,
            mass: 0.0,
            inv_mass: 0.0,
            shape,
        }
    }

    pub fn is_static(&self) -> bool {
        self.body_type == BodyType::Static
    }

    pub fn is_dynamic(&self) -> bool {
        self.body_type == BodyType::Dynamic
    }

    pub fn apply_force(&mut self, force: Vec2) {
        self.force += force;
    }

    pub fn apply_impulse(&mut self, impulse: Vec2) {
        self.velocity += impulse * self.inv_mass;
    }
}
