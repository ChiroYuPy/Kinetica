use crate::core::RigidBody;
use crate::forces::{ForceGenerator, LinearGravity};
use crate::integration::{Integrator, SemiImplicitEuler};
use crate::math::Vec2;

pub struct World {
    pub bodies: Vec<RigidBody>,
    pub integrator: Box<dyn Integrator>,
    pub force_generators: Vec<Box<dyn ForceGenerator>>,
}

impl World {
    pub fn new() -> Self {
        let bodies = Vec::new();
        let integrator = Box::new(SemiImplicitEuler);
        let force_generators: Vec<Box<dyn ForceGenerator>> = vec![
            Box::new(LinearGravity { acceleration: Vec2::new(0.0, -9.81) })
        ];

        Self { bodies, integrator, force_generators }
    }

    pub fn add_force_generator(&mut self, generator: Box<dyn ForceGenerator>) {
        self.force_generators.push(generator);
    }

    pub fn add_body(&mut self, body: RigidBody) -> usize {
        let index = self.bodies.len();
        self.bodies.push(body);
        index
    }

    fn reset_forces(&mut self) {
        for body in self.bodies.iter_mut() {
            body.force = Vec2::ZERO;
        }
    }

    pub fn step(&mut self, dt: f32) {
        self.reset_forces();

        for generator in &self.force_generators {
            generator.apply_to_all(&mut self.bodies);
        }

        self.integrator.integrate(&mut self.bodies, dt);
    }

    pub fn len(&self) -> usize {
        self.bodies.len()
    }

    pub fn is_empty(&self) -> bool {
        self.bodies.is_empty()
    }

    pub fn clear(&mut self) {
        self.bodies.clear();
    }
}

impl Default for World {
    fn default() -> Self {
        Self::new()
    }
}