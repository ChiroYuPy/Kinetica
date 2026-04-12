use crate::constraints::Constraint;
use crate::core::RigidBody;

pub struct ImpulseSolver {
    pub iterations: usize,
}

impl Default for ImpulseSolver {
    fn default() -> Self {
        Self { iterations: 8 }
    }
}

impl ImpulseSolver {
    pub fn solve_velocity(&self, constraints: &mut [&mut dyn Constraint], bodies: &mut [RigidBody]) {
        for _ in 0..self.iterations {
            for constraint in &mut *constraints {
                constraint.solve_velocity(bodies);
            }
        }
    }

    pub fn solve_position(&self, constraints: &mut [&mut dyn Constraint], bodies: &mut [RigidBody]) {
        for constraint in &mut *constraints {
            constraint.solve_position(bodies);
        }
    }
}
