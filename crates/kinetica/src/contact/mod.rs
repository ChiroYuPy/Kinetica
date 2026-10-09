//! Contacts between colliders, sensor overlaps, and the events reporting when they start and
//! stop.
//!
//! The broad phase proposes pairs of colliders that may touch; [`ContactSet`] keeps one entry
//! per pair while their fat boxes overlap, recomputes its geometry every step, and reports
//! changes as events.

mod events;
mod set;

pub use events::{CollisionEvent, SensorEvent, SensorPair};
pub(crate) use set::ContactSet;

use crate::arena::Arena;
use crate::body::{Body, BodyHandle, BodyType};
use crate::collider::ColliderHandle;
use crate::collision::Manifold;
use crate::joint::Joint;

/// Two colliders in contact, and the bodies they belong to.
///
/// `collider_a` is always the smaller handle, so a given pair is always reported the same way.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ContactPair {
    /// First collider.
    pub collider_a: ColliderHandle,
    /// Second collider.
    pub collider_b: ColliderHandle,
    /// Body of the first collider.
    pub body_a: BodyHandle,
    /// Body of the second collider.
    pub body_b: BodyHandle,
}

impl ContactPair {
    /// Returns `true` if one of the two colliders is `collider`.
    #[must_use]
    pub fn involves_collider(&self, collider: ColliderHandle) -> bool {
        self.collider_a == collider || self.collider_b == collider
    }

    /// Returns `true` if one of the two colliders belongs to `body`.
    #[must_use]
    pub fn involves_body(&self, body: BodyHandle) -> bool {
        self.body_a == body || self.body_b == body
    }

    /// Given one body of the pair, returns the other one.
    #[must_use]
    pub fn other_body(&self, body: BodyHandle) -> Option<BodyHandle> {
        match body {
            _ if body == self.body_a => Some(self.body_b),
            _ if body == self.body_b => Some(self.body_a),
            _ => None,
        }
    }
}

/// Two touching colliders.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Contact {
    pair: ContactPair,
    manifold: Manifold,
    /// Cleared while a one-way platform lets the other collider through. Decided when the
    /// manifold becomes non-empty, and kept until it is empty again.
    enabled: bool,
}

impl Contact {
    /// The colliders and bodies in contact.
    #[must_use]
    pub fn pair(&self) -> &ContactPair {
        &self.pair
    }

    /// Contact geometry; the normal points from `collider_a` to `collider_b`.
    #[must_use]
    pub fn manifold(&self) -> &Manifold {
        &self.manifold
    }

    pub(crate) fn manifold_mut(&mut self) -> &mut Manifold {
        &mut self.manifold
    }

    fn is_touching(&self) -> bool {
        self.enabled && !self.manifold.is_empty()
    }
}

/// Contacts only matter when at least one dynamic body can react to them, and joints may opt
/// their bodies out of colliding.
pub(crate) fn should_collide(pair: &ContactPair, bodies: &Arena<Body>, joints: &Arena<Joint>) -> bool {
    if pair.body_a == pair.body_b {
        return false;
    }
    let (Some(body_a), Some(body_b)) = (bodies.get(pair.body_a.0), bodies.get(pair.body_b.0)) else { return false };
    if body_a.body_type() != BodyType::Dynamic && body_b.body_type() != BodyType::Dynamic {
        return false;
    }
    // Check the body with fewer joints.
    let (body, handle, other) = if body_a.joints().len() <= body_b.joints().len() {
        (body_a, pair.body_a, pair.body_b)
    } else {
        (body_b, pair.body_b, pair.body_a)
    };
    !body.joints().iter().any(|joint| joints.get(joint.0).is_some_and(|joint| joint.prevents_collision(handle, other)))
}
