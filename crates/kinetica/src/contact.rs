//! Contacts between colliders, and the events reporting when they start and stop.

use std::collections::BTreeMap;

use crate::arena::Arena;
use crate::body::{Body, BodyHandle, BodyType};
use crate::collider::{Collider, ColliderHandle};
use crate::collision::broad_phase::{BroadPhase, CandidatePair};
use crate::collision::{Manifold, collide};

/// Two colliders in contact, and the bodies they belong to.
///
/// `collider_a` is always the smaller handle, so a given pair is always reported the same way.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
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
pub struct Contact {
    pair: ContactPair,
    manifold: Manifold,
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
        !self.manifold.is_empty()
    }
}

/// Reported by [`World::collision_events`](crate::World::collision_events) when two colliders
/// start or stop touching.
///
/// Removing a collider or a body does not report `Stopped` for its contacts.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CollisionEvent {
    /// The colliders started touching during the step.
    Started(ContactPair),
    /// The colliders stopped touching during the step.
    Stopped(ContactPair),
}

impl CollisionEvent {
    /// The colliders and bodies concerned.
    #[must_use]
    pub fn pair(&self) -> &ContactPair {
        match self {
            Self::Started(pair) | Self::Stopped(pair) => pair,
        }
    }
}

/// Every pair of colliders whose fat bounding boxes overlap, touching or not.
///
/// An ordered map keeps the iteration order, and therefore the simulation, deterministic.
#[derive(Debug, Clone, Default)]
pub(crate) struct ContactSet {
    contacts: BTreeMap<CandidatePair, Contact>,
}

impl ContactSet {
    /// Creates contacts for new candidate pairs. Their geometry is computed by the next
    /// [`update`](Self::update).
    pub(crate) fn add_pairs(&mut self, pairs: &[CandidatePair], colliders: &Arena<Collider>, bodies: &Arena<Body>) {
        for &(collider_a, collider_b) in pairs {
            if self.contacts.contains_key(&(collider_a, collider_b)) {
                continue;
            }
            let (Some(a), Some(b)) = (colliders.get(collider_a.0), colliders.get(collider_b.0)) else { continue };
            let pair = ContactPair { collider_a, collider_b, body_a: a.body(), body_b: b.body() };
            if should_collide(&pair, bodies) {
                self.contacts.insert((collider_a, collider_b), Contact { pair, manifold: Manifold::default() });
            }
        }
    }

    /// Recomputes every contact, drops the ones whose fat boxes no longer overlap, and reports
    /// changes in touching state.
    pub(crate) fn update(
        &mut self,
        colliders: &Arena<Collider>,
        bodies: &Arena<Body>,
        broad_phase: &BroadPhase,
        events: &mut Vec<CollisionEvent>,
    ) {
        self.contacts.retain(|_, contact| {
            let pair = contact.pair;
            let was_touching = contact.is_touching();
            let colliders = (colliders.get(pair.collider_a.0), colliders.get(pair.collider_b.0));
            let bodies = (bodies.get(pair.body_a.0), bodies.get(pair.body_b.0));
            let ((Some(collider_a), Some(collider_b)), (Some(body_a), Some(body_b))) = (colliders, bodies) else {
                return false;
            };

            let keep = broad_phase.fat_aabb(collider_a.proxy()).overlaps(&broad_phase.fat_aabb(collider_b.proxy()));
            let mut manifold = if keep {
                collide(collider_a.shape(), body_a.transform(), collider_b.shape(), body_b.transform())
            } else {
                Manifold::default()
            };
            manifold.inherit_impulses(&contact.manifold);
            contact.manifold = manifold;

            match (was_touching, contact.is_touching()) {
                (false, true) => events.push(CollisionEvent::Started(pair)),
                (true, false) => events.push(CollisionEvent::Stopped(pair)),
                _ => {}
            }
            keep
        });
    }

    /// Forgets every contact involving `collider`, without reporting events.
    pub(crate) fn remove_collider(&mut self, collider: ColliderHandle) {
        self.contacts.retain(|_, contact| !contact.pair.involves_collider(collider));
    }

    pub(crate) fn touching(&self) -> impl Iterator<Item = &Contact> {
        self.contacts.values().filter(|contact| contact.is_touching())
    }

    pub(crate) fn get_mut(&mut self, pair: &ContactPair) -> Option<&mut Contact> {
        self.contacts.get_mut(&(pair.collider_a, pair.collider_b))
    }
}

/// Contacts only matter when at least one dynamic body can react to them.
fn should_collide(pair: &ContactPair, bodies: &Arena<Body>) -> bool {
    if pair.body_a == pair.body_b {
        return false;
    }
    let is_dynamic = |body: BodyHandle| bodies.get(body.0).is_some_and(|body| body.body_type() == BodyType::Dynamic);
    is_dynamic(pair.body_a) || is_dynamic(pair.body_b)
}
