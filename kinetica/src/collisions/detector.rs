use crate::collisions::broad_phase::SweepAndPrune;
use crate::collisions::narrow_phase::test_pairs;
use crate::constraints::ContactManifold;
use crate::RigidBody;

pub struct CollisionDetector {
    broad: SweepAndPrune,
}

impl CollisionDetector {
    pub fn new() -> Self {
        Self {
            broad: SweepAndPrune::new(),
        }
    }

    pub fn detect(&mut self, bodies: &[RigidBody]) -> Vec<ContactManifold> {
        let aabbs: Vec<_> = bodies
            .iter()
            .map(|b| b.shape.compute_aabb(b.transform.position))
            .collect();

        test_pairs(&self.broad.find_pairs(&aabbs), bodies)
    }
}

impl Default for CollisionDetector {
    fn default() -> Self {
        Self::new()
    }
}
