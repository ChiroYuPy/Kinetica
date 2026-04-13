use crate::collisions::CollisionDetector;
use crate::constraints::{ContactConstraint, Constraint};
use crate::core::RigidBody;
use crate::forces::ForceGenerator;
use crate::integration::{Integrator, SemiImplicitEuler};
use crate::math::Vec2;
use crate::solver::ImpulseSolver;

pub struct World {
    pub bodies: Vec<RigidBody>,
    pub integrator: Box<dyn Integrator>,
    pub force_generators: Vec<Box<dyn ForceGenerator>>,
    pub collision_detector: Option<CollisionDetector>,
    pub constraints: Vec<Box<dyn Constraint>>,
    pub solver: ImpulseSolver,
    pub default_restitution: f32,
    contact_constraints: Vec<Box<dyn Constraint>>,
}

impl World {
    pub fn new() -> Self {
        Self {
            bodies: Vec::new(),
            integrator: Box::new(SemiImplicitEuler),
            force_generators: Vec::new(),
            collision_detector: None,
            constraints: Vec::new(),
            solver: ImpulseSolver::default(),
            default_restitution: 0.5,
            contact_constraints: Vec::new(),
        }
    }

    pub fn add_force_generator(&mut self, generator: Box<dyn ForceGenerator>) {
        self.force_generators.push(generator);
    }

    pub fn add_constraint(&mut self, constraint: Box<dyn Constraint>) {
        self.constraints.push(constraint);
    }

    pub fn add_body(&mut self, body: RigidBody) -> usize {
        let index = self.bodies.len();
        self.bodies.push(body);
        index
    }

    fn reset_forces(&mut self) {
        for body in self.bodies.iter_mut() {
            body.state.force = Vec2::ZERO;
        }
    }

    pub fn step(&mut self, dt: f32) {
        self.reset_forces();

        // 1. Generate forces
        for generator in &self.force_generators {
            generator.apply_to_all(&mut self.bodies);
        }

        // 2. Integrate
        self.integrator.integrate(&mut self.bodies, dt);

        // 3. Detect collisions → contact constraints
        self.contact_constraints.clear();
        if let Some(ref mut detector) = self.collision_detector {
            let contacts = detector.detect(&self.bodies);
            for manifold in contacts {
                self.contact_constraints.push(Box::new(ContactConstraint::with_restitution(manifold, self.default_restitution)));
            }
        }

        // 4. Solve velocity (impulsions)
        for constraint in &mut self.constraints {
            constraint.solve_velocity(&mut self.bodies);
        }
        for constraint in &mut self.contact_constraints {
            constraint.solve_velocity(&mut self.bodies);
        }

        // 5. Solve position (correction de pénétration)
        for constraint in &mut self.constraints {
            constraint.solve_position(&mut self.bodies);
        }
        for constraint in &mut self.contact_constraints {
            constraint.solve_position(&mut self.bodies);
        }
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
