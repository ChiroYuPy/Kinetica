use crate::core::shape::Shape;
use crate::math::Vec2;

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum BodyType {
    Static,
    Dynamic,
}

pub struct RigidBody {
    position: Vec2,
    velocity: Vec2,
    force: Vec2,
    body_type: BodyType,
    mass: f32,
    inv_mass: f32,
    shape: Shape,
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

    // Getters
    pub fn position(&self) -> Vec2 {
        self.position
    }

    pub fn velocity(&self) -> Vec2 {
        self.velocity
    }

    pub fn force(&self) -> Vec2 {
        self.force
    }

    pub fn body_type(&self) -> BodyType {
        self.body_type
    }

    pub fn mass(&self) -> f32 {
        self.mass
    }

    pub fn inv_mass(&self) -> f32 {
        self.inv_mass
    }

    pub fn shape(&self) -> &Shape {
        &self.shape
    }

    pub fn is_static(&self) -> bool {
        self.body_type == BodyType::Static
    }

    pub fn is_dynamic(&self) -> bool {
        self.body_type == BodyType::Dynamic
    }

    // Setters
    pub fn set_position(&mut self, position: Vec2) {
        self.position = position;
    }

    pub fn set_velocity(&mut self, velocity: Vec2) {
        self.velocity = velocity;
    }

    pub fn set_force(&mut self, force: Vec2) {
        self.force = force;
    }

    // Méthodes utilitaires
    pub fn apply_force(&mut self, force: Vec2) {
        self.force += force;
    }

    pub fn apply_impulse(&mut self, impulse: Vec2) {
        self.velocity += impulse * self.inv_mass;
    }

    // Accès interne pour les systèmes physiques
    pub(crate) fn position_mut(&mut self) -> &mut Vec2 {
        &mut self.position
    }

    pub(crate) fn velocity_mut(&mut self) -> &mut Vec2 {
        &mut self.velocity
    }

    pub(crate) fn force_mut(&mut self) -> &mut Vec2 {
        &mut self.force
    }
}
