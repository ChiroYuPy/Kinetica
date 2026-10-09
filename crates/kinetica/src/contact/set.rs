//! Tracking every pair of colliders that may touch, from step to step.

use std::collections::BTreeMap;

use super::{CollisionEvent, SensorEvent, SensorPair};
use super::{Contact, ContactPair, should_collide};
use crate::arena::Arena;
use crate::body::{Body, BodyHandle, BodyType};
use crate::collider::{Collider, ColliderHandle};
use crate::collision::broad_phase::{BroadPhase, CandidatePair};
use crate::collision::{Manifold, collide, separation_lower_bound};
use crate::joint::Joint;

/// Every pair of colliders whose fat bounding boxes overlap, touching or not, split between
/// solid contacts and sensor overlaps.
///
/// Ordered maps keep the iteration order, and therefore the simulation, deterministic.
#[derive(Debug, Clone, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub(crate) struct ContactSet {
    #[cfg_attr(feature = "serde", serde(with = "values_only"))]
    contacts: BTreeMap<CandidatePair, Contact>,
    #[cfg_attr(feature = "serde", serde(with = "values_only"))]
    sensors: BTreeMap<CandidatePair, SensorOverlap>,
}

/// A sensor and a collider whose fat boxes overlap, and whether the shapes do.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
struct SensorOverlap {
    pair: SensorPair,
    overlapping: bool,
}

impl ContactSet {
    /// Starts tracking new candidate pairs, given sorted. Their geometry is computed by the next
    /// [`update`](Self::update).
    pub(crate) fn add_pairs(
        &mut self,
        pairs: &[CandidatePair],
        colliders: &Arena<Collider>,
        bodies: &Arena<Body>,
        joints: &Arena<Joint>,
    ) {
        for key in self.unknown(pairs) {
            let (collider_a, collider_b) = key;
            let (Some(a), Some(b)) = (colliders.get(collider_a.0), colliders.get(collider_b.0)) else { continue };
            let pair = ContactPair { collider_a, collider_b, body_a: a.body(), body_b: b.body() };
            match (a.is_sensor(), b.is_sensor()) {
                (false, false) => {
                    if should_collide(&pair, bodies, joints) {
                        let contact = Contact { pair, manifold: Manifold::default(), enabled: true };
                        self.contacts.insert(key, contact);
                    }
                }
                (true, false) | (false, true) => {
                    if should_sense(&pair, bodies) {
                        let pair = SensorPair::new(&pair, a.is_sensor());
                        self.sensors.insert(key, SensorOverlap { pair, overlapping: false });
                    }
                }
                (true, true) => {}
            }
        }
    }

    /// The pairs, among the sorted `pairs`, that are not tracked yet.
    fn unknown(&self, pairs: &[CandidatePair]) -> Vec<CandidatePair> {
        debug_assert!(pairs.is_sorted(), "candidate pairs must be sorted");
        let is_known = |key: &CandidatePair| self.contacts.contains_key(key) || self.sensors.contains_key(key);
        let known_count = self.contacts.len() + self.sensors.len();
        if pairs.len() * 16 < known_count {
            return pairs.iter().copied().filter(|key| !is_known(key)).collect();
        }
        // Most pairs are already known. When there are many, walking them side by side with the
        // maps (all sorted by key) is cheaper than looking each one up.
        let mut contacts = self.contacts.keys().peekable();
        let mut sensors = self.sensors.keys().peekable();
        pairs
            .iter()
            .copied()
            .filter(|key| {
                while contacts.next_if(|known| *known < key).is_some() {}
                while sensors.next_if(|known| *known < key).is_some() {}
                contacts.peek() != Some(&key) && sensors.peek() != Some(&key)
            })
            .collect()
    }

    /// Recomputes every contact and sensor overlap, stops tracking the pairs whose fat boxes no
    /// longer overlap, and reports changes. Pairs where no body is awake are left as they are.
    pub(crate) fn update(
        &mut self,
        colliders: &Arena<Collider>,
        bodies: &Arena<Body>,
        broad_phase: &BroadPhase,
        collision_events: &mut Vec<CollisionEvent>,
        sensor_events: &mut Vec<SensorEvent>,
    ) {
        self.contacts.retain(|_, contact| {
            let pair = contact.pair;
            let (a, b) = ((pair.collider_a, pair.body_a), (pair.collider_b, pair.body_b));
            let Some(placed) = Placed::new(a, b, colliders, bodies) else { return false };
            if !placed.is_awake() {
                return true;
            }
            let keep = placed.fat_boxes_overlap(broad_phase);
            let event = update_contact(contact, &placed, keep);
            collision_events.extend(event);
            keep
        });

        self.sensors.retain(|_, overlap| {
            let pair = overlap.pair;
            let (sensor, visitor) = ((pair.sensor, pair.sensor_body), (pair.visitor, pair.visitor_body));
            let Some(placed) = Placed::new(sensor, visitor, colliders, bodies) else { return false };
            if !placed.is_awake() {
                return true;
            }
            let keep = placed.fat_boxes_overlap(broad_phase);
            let overlapping = keep && placed.interact() && placed.separation_lower_bound() < 0.0;
            match (overlap.overlapping, overlapping) {
                (false, true) => sensor_events.push(SensorEvent::Entered(pair)),
                (true, false) => sensor_events.push(SensorEvent::Exited(pair)),
                _ => {}
            }
            overlap.overlapping = overlapping;
            keep
        });
    }

    /// Forgets every contact between the two bodies, without reporting events.
    pub(crate) fn remove_between(&mut self, body: BodyHandle, other: BodyHandle) {
        self.contacts.retain(|_, contact| !(contact.pair.involves_body(body) && contact.pair.involves_body(other)));
    }

    /// Forgets every contact and sensor overlap involving `collider`, without reporting events.
    pub(crate) fn remove_collider(&mut self, collider: ColliderHandle) {
        self.contacts.retain(|_, contact| !contact.pair.involves_collider(collider));
        self.sensors.retain(|&(collider_a, collider_b), _| collider_a != collider && collider_b != collider);
    }

    /// Colliders currently overlapping `sensor`.
    pub(crate) fn sensor_overlaps(&self, sensor: ColliderHandle) -> impl Iterator<Item = &SensorPair> {
        self.sensors
            .values()
            .filter(move |overlap| overlap.overlapping && overlap.pair.sensor == sensor)
            .map(|overlap| &overlap.pair)
    }

    pub(crate) fn touching(&self) -> impl Iterator<Item = &Contact> {
        self.contacts.values().filter(|contact| contact.is_touching())
    }

    pub(crate) fn get_mut(&mut self, pair: &ContactPair) -> Option<&mut Contact> {
        self.contacts.get_mut(&(pair.collider_a, pair.collider_b))
    }
}

/// Recomputes the manifold of `contact` (empty when `keep` is false: the pair is about to be
/// dropped), and returns the event to report, if it started or stopped touching.
fn update_contact(contact: &mut Contact, placed: &Placed, keep: bool) -> Option<CollisionEvent> {
    let was_touching = contact.is_touching();
    let had_points = !contact.manifold.is_empty();
    let mut manifold = if keep && placed.interact() { placed.collide() } else { Manifold::default() };
    manifold.inherit_impulses(&contact.manifold);
    if !had_points && !manifold.is_empty() {
        // One-way platforms decide once, when the colliders start touching.
        let normal = manifold.normal();
        contact.enabled = placed.collider_a.blocks_along(placed.body_a.rotation(), normal)
            && placed.collider_b.blocks_along(placed.body_b.rotation(), -normal);
    }
    contact.manifold = manifold;

    let pair = contact.pair;
    match (was_touching, contact.is_touching()) {
        (false, true) => Some(CollisionEvent::Started { pair, impact_speed: placed.impact_speed(&contact.manifold) }),
        (true, false) => Some(CollisionEvent::Stopped { pair }),
        _ => None,
    }
}

/// Two colliders with their bodies, as they are now.
struct Placed<'a> {
    collider_a: &'a Collider,
    collider_b: &'a Collider,
    body_a: &'a Body,
    body_b: &'a Body,
}

impl<'a> Placed<'a> {
    /// Looks up two colliders and their bodies; `None` if one of them was removed.
    fn new(
        (collider_a, body_a): (ColliderHandle, BodyHandle),
        (collider_b, body_b): (ColliderHandle, BodyHandle),
        colliders: &'a Arena<Collider>,
        bodies: &'a Arena<Body>,
    ) -> Option<Self> {
        Some(Self {
            collider_a: colliders.get(collider_a.0)?,
            collider_b: colliders.get(collider_b.0)?,
            body_a: bodies.get(body_a.0)?,
            body_b: bodies.get(body_b.0)?,
        })
    }

    /// Pairs where both bodies sleep (or are fixed) cannot change.
    fn is_awake(&self) -> bool {
        self.body_a.is_active() || self.body_b.is_active()
    }

    /// Whether the pair is still worth tracking.
    fn fat_boxes_overlap(&self, broad_phase: &BroadPhase) -> bool {
        broad_phase.fat_aabb(self.collider_a.proxy()).overlaps(&broad_phase.fat_aabb(self.collider_b.proxy()))
    }

    /// Whether the collision layers let the colliders touch.
    fn interact(&self) -> bool {
        self.collider_a.layers().interacts_with(&self.collider_b.layers())
    }

    fn collide(&self) -> Manifold {
        collide(self.collider_a.shape(), self.body_a.transform(), self.collider_b.shape(), self.body_b.transform())
    }

    fn separation_lower_bound(&self) -> f32 {
        let (a, b) = (self.collider_a.shape(), self.collider_b.shape());
        separation_lower_bound(a, self.body_a.transform(), b, self.body_b.transform())
    }

    /// The fastest approach speed of the two bodies along the normal, at the contact points.
    fn impact_speed(&self, manifold: &Manifold) -> f32 {
        manifold
            .points()
            .iter()
            .map(|point| {
                let position = point.position();
                let relative_velocity =
                    self.body_b.velocity_at_point(position) - self.body_a.velocity_at_point(position);
                -relative_velocity.dot(manifold.normal())
            })
            .fold(0.0, f32::max)
    }
}

/// Sensors detect anything but their own body; two fixed bodies never move, so there is nothing
/// to detect between them.
fn should_sense(pair: &ContactPair, bodies: &Arena<Body>) -> bool {
    let (Some(body_a), Some(body_b)) = (bodies.get(pair.body_a.0), bodies.get(pair.body_b.0)) else { return false };
    pair.body_a != pair.body_b && !(body_a.body_type() == BodyType::Fixed && body_b.body_type() == BodyType::Fixed)
}

/// Values that know their own key in the contact maps.
#[cfg(feature = "serde")]
trait Keyed {
    fn key(&self) -> CandidatePair;
}

#[cfg(feature = "serde")]
impl Keyed for Contact {
    fn key(&self) -> CandidatePair {
        (self.pair.collider_a, self.pair.collider_b)
    }
}

#[cfg(feature = "serde")]
impl Keyed for SensorOverlap {
    fn key(&self) -> CandidatePair {
        let (sensor, visitor) = (self.pair.sensor, self.pair.visitor);
        if sensor < visitor { (sensor, visitor) } else { (visitor, sensor) }
    }
}

/// Serializes the contact maps as plain lists: their keys are pairs of handles, which formats
/// like JSON cannot use as map keys, and can be rebuilt from the values anyway.
#[cfg(feature = "serde")]
mod values_only {
    use std::collections::BTreeMap;

    use serde::{Deserialize, Deserializer, Serialize, Serializer};

    use super::{CandidatePair, Keyed};

    pub(super) fn serialize<S: Serializer, V: Serialize>(
        map: &BTreeMap<CandidatePair, V>,
        serializer: S,
    ) -> Result<S::Ok, S::Error> {
        serializer.collect_seq(map.values())
    }

    pub(super) fn deserialize<'de, D: Deserializer<'de>, V: Deserialize<'de> + Keyed>(
        deserializer: D,
    ) -> Result<BTreeMap<CandidatePair, V>, D::Error> {
        let values = Vec::<V>::deserialize(deserializer)?;
        Ok(values.into_iter().map(|value| (value.key(), value)).collect())
    }
}
