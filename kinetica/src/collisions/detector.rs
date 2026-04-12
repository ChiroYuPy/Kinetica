use crate::collisions::{BroadPhase, NaiveBroadPhase, NaiveNarrowPhase, narrow_phase::NarrowPhase};
use crate::constraints::ContactManifold;
use crate::core::RigidBody;

pub trait CollisionDetector: Send + Sync {
    fn detect(&self, bodies: &[RigidBody]) -> Vec<ContactManifold>;
}

pub struct NaiveCollisionDetector {
    broad: NaiveBroadPhase,
    narrow: NaiveNarrowPhase,
}

impl NaiveCollisionDetector {
    pub fn new() -> Self {
        Self {
            broad: NaiveBroadPhase,
            narrow: NaiveNarrowPhase,
        }
    }
}

impl Default for NaiveCollisionDetector {
    fn default() -> Self {
        Self::new()
    }
}

impl CollisionDetector for NaiveCollisionDetector {
    fn detect(&self, bodies: &[RigidBody]) -> Vec<ContactManifold> {
        self.narrow.test_pairs(&self.broad.find_pairs(bodies.len()), bodies)
    }
}
