use crate::core::RigidBody;

pub struct World {
    pub bodies: Vec<RigidBody>,
}

impl World {
    pub fn new() -> Self {
        Self {
            bodies: Vec::new(),
        }
    }

    pub fn add_body(&mut self, body: RigidBody) -> usize {
        let index = self.bodies.len();
        self.bodies.push(body);
        index
    }

    pub fn step(&mut self, dt: f32) {
        todo!("step")
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