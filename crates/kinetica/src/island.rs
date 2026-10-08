//! Islands: groups of dynamic bodies linked by contacts and joints, which fall asleep and wake
//! up together.
//!
//! A body resting on a sleeping stack must not keep moving alone, and a body hitting the stack
//! must wake all of it, so sleep is decided per island rather than per body. Fixed and kinematic
//! bodies do not link islands: two boxes on the same ground sleep independently.

use crate::arena::{Arena, Index};
use crate::body::{Body, BodyHandle, BodyType};
use crate::constants::TIME_TO_SLEEP;
use crate::contact::ContactSet;
use crate::joint::Joint;
use crate::math::Vec2;

/// Dynamic bodies grouped by island, rebuilt every step.
#[derive(Debug, Default)]
pub(crate) struct Islands {
    islands: Vec<Vec<Index>>,
    /// Dynamic bodies touching a moving kinematic body, which keeps them awake.
    pushed: Vec<Index>,
}

impl Islands {
    pub(crate) fn build(bodies: &Arena<Body>, contacts: &ContactSet, joints: &Arena<Joint>) -> Self {
        let body_type = |handle: BodyHandle| bodies.get(handle.0).map(Body::body_type);
        let mut sets = DisjointSets::new(bodies.slot_count());
        let mut pushed = Vec::new();

        for contact in contacts.touching() {
            let pair = contact.pair();
            match (body_type(pair.body_a), body_type(pair.body_b)) {
                (Some(BodyType::Dynamic), Some(BodyType::Dynamic)) => sets.union(pair.body_a.0, pair.body_b.0),
                (Some(BodyType::Kinematic), Some(BodyType::Dynamic)) if is_moving(bodies, pair.body_a) => {
                    pushed.push(pair.body_b.0);
                }
                (Some(BodyType::Dynamic), Some(BodyType::Kinematic)) if is_moving(bodies, pair.body_b) => {
                    pushed.push(pair.body_a.0);
                }
                _ => {}
            }
        }
        for (_, joint) in joints.iter() {
            if body_type(joint.body_a()) == Some(BodyType::Dynamic)
                && body_type(joint.body_b()) == Some(BodyType::Dynamic)
            {
                sets.union(joint.body_a().0, joint.body_b().0);
            }
        }

        // Group bodies by root, in body order so the result is deterministic.
        let mut island_of_root = vec![usize::MAX; bodies.slot_count()];
        let mut islands: Vec<Vec<Index>> = Vec::new();
        for (index, body) in bodies.iter() {
            if body.body_type() != BodyType::Dynamic {
                continue;
            }
            let root = sets.find(index.slot());
            if island_of_root[root] == usize::MAX {
                island_of_root[root] = islands.len();
                islands.push(Vec::new());
            }
            islands[island_of_root[root]].push(index);
        }
        Self { islands, pushed }
    }

    /// Wakes every island where at least one body is awake, or pushed by a kinematic body.
    pub(crate) fn wake_up(&self, bodies: &mut Arena<Body>) {
        for &index in &self.pushed {
            if let Some(body) = bodies.get_mut(index) {
                body.wake_up();
            }
        }
        for island in &self.islands {
            let sleeping = |index: &Index| bodies.get(*index).is_some_and(Body::is_sleeping);
            if island.iter().any(sleeping) && !island.iter().all(sleeping) {
                for &index in island {
                    if let Some(body) = bodies.get_mut(index) {
                        body.wake_up();
                    }
                }
            }
        }
    }

    /// Puts to sleep every island whose bodies all stayed still long enough.
    ///
    /// Expects [`wake_up`](Self::wake_up) to have run this step, so each island is either all
    /// awake or all asleep.
    pub(crate) fn fall_asleep(&self, bodies: &mut Arena<Body>, dt: f32) {
        for island in &self.islands {
            let mut rest_time = f32::INFINITY;
            for &index in island {
                if let Some(body) = bodies.get_mut(index)
                    && !body.is_sleeping()
                {
                    rest_time = rest_time.min(body.update_sleep_time(dt));
                }
            }
            if rest_time >= TIME_TO_SLEEP && rest_time.is_finite() {
                for &index in island {
                    if let Some(body) = bodies.get_mut(index) {
                        body.fall_asleep();
                    }
                }
            }
        }
    }
}

fn is_moving(bodies: &Arena<Body>, handle: BodyHandle) -> bool {
    bodies.get(handle.0).is_some_and(|body| body.linear_velocity() != Vec2::ZERO || body.angular_velocity() != 0.0)
}

/// Union-find over arena slots.
struct DisjointSets {
    parents: Vec<usize>,
}

impl DisjointSets {
    fn new(len: usize) -> Self {
        Self { parents: (0..len).collect() }
    }

    fn find(&mut self, mut slot: usize) -> usize {
        while self.parents[slot] != slot {
            // Path halving keeps the trees flat.
            self.parents[slot] = self.parents[self.parents[slot]];
            slot = self.parents[slot];
        }
        slot
    }

    fn union(&mut self, a: Index, b: Index) {
        let (root_a, root_b) = (self.find(a.slot()), self.find(b.slot()));
        // Keep the smaller slot as root, so the grouping does not depend on the union order.
        let (root, child) = if root_a < root_b { (root_a, root_b) } else { (root_b, root_a) };
        self.parents[child] = root;
    }
}
