//! Contacts between colliders, sensor overlaps, and the events reporting when they start and
//! stop.

use std::collections::BTreeMap;

use crate::arena::Arena;
use crate::body::{Body, BodyHandle, BodyType};
use crate::collider::{Collider, ColliderHandle};
use crate::collision::broad_phase::{BroadPhase, CandidatePair};
use crate::collision::{Manifold, collide, separation_lower_bound};
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

/// Reported by [`World::collision_events`](crate::World::collision_events) when two colliders
/// start or stop touching.
///
/// Removing a collider or a body does not report `Stopped` for its contacts.
///
/// ```
/// # use kinetica::prelude::*;
/// # let world = World::new();
/// for event in world.collision_events() {
///     if let CollisionEvent::Started { pair, impact_speed } = event
///         && *impact_speed > 5.0
///     {
///         println!("{:?} hit {:?} hard", pair.body_a, pair.body_b);
///     }
/// }
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum CollisionEvent {
    /// The colliders started touching during the step.
    Started {
        /// The colliders and bodies concerned.
        pair: ContactPair,
        /// How fast the colliders were moving towards each other, in m/s: how hard they hit.
        impact_speed: f32,
    },
    /// The colliders stopped touching during the step.
    Stopped {
        /// The colliders and bodies concerned.
        pair: ContactPair,
    },
}

impl CollisionEvent {
    /// The colliders and bodies concerned.
    #[must_use]
    pub fn pair(&self) -> &ContactPair {
        match self {
            Self::Started { pair, .. } | Self::Stopped { pair } => pair,
        }
    }
}

/// A sensor and a collider overlapping it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct SensorPair {
    /// The sensor collider.
    pub sensor: ColliderHandle,
    /// Body of the sensor.
    pub sensor_body: BodyHandle,
    /// The collider inside the sensor; never a sensor itself.
    pub visitor: ColliderHandle,
    /// Body of the visitor.
    pub visitor_body: BodyHandle,
}

/// Reported by [`World::sensor_events`](crate::World::sensor_events) when a collider starts or
/// stops overlapping a sensor.
///
/// Sensors detect every non-sensor collider allowed by their [layers](crate::CollisionLayers),
/// except colliders of their own body, and colliders of fixed bodies when the sensor is fixed
/// too. Removing a collider or a body does not report `Exited`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum SensorEvent {
    /// The visitor started overlapping the sensor during the step.
    Entered(SensorPair),
    /// The visitor stopped overlapping the sensor during the step.
    Exited(SensorPair),
}

impl SensorEvent {
    /// The sensor and visitor concerned.
    #[must_use]
    pub fn pair(&self) -> &SensorPair {
        match self {
            Self::Entered(pair) | Self::Exited(pair) => pair,
        }
    }
}

#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
struct SensorOverlap {
    pair: SensorPair,
    overlapping: bool,
}

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

impl ContactSet {
    /// Creates contacts for new candidate pairs, given sorted. Their geometry is computed by the
    /// next [`update`](Self::update).
    pub(crate) fn add_pairs(
        &mut self,
        pairs: &[CandidatePair],
        colliders: &Arena<Collider>,
        bodies: &Arena<Body>,
        joints: &Arena<Joint>,
    ) {
        // Most pairs are already known. When there are many, walking them side by side with the
        // maps (all sorted by key) is cheaper than looking each one up.
        debug_assert!(pairs.is_sorted(), "candidate pairs must be sorted");
        let known_count = self.contacts.len() + self.sensors.len();
        let new_pairs: Vec<CandidatePair> = if pairs.len() * 16 < known_count {
            pairs
                .iter()
                .copied()
                .filter(|key| !self.contacts.contains_key(key) && !self.sensors.contains_key(key))
                .collect()
        } else {
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
        };
        for key in new_pairs {
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
                        let pair = if a.is_sensor() {
                            SensorPair {
                                sensor: collider_a,
                                sensor_body: pair.body_a,
                                visitor: collider_b,
                                visitor_body: pair.body_b,
                            }
                        } else {
                            SensorPair {
                                sensor: collider_b,
                                sensor_body: pair.body_b,
                                visitor: collider_a,
                                visitor_body: pair.body_a,
                            }
                        };
                        self.sensors.insert(key, SensorOverlap { pair, overlapping: false });
                    }
                }
                (true, true) => {}
            }
        }
    }

    /// Recomputes every contact and sensor overlap, drops the ones whose fat boxes no longer
    /// overlap, and reports changes. Pairs where no body is awake are left as they are.
    pub(crate) fn update(
        &mut self,
        colliders: &Arena<Collider>,
        bodies: &Arena<Body>,
        broad_phase: &BroadPhase,
        events: &mut Vec<CollisionEvent>,
        sensor_events: &mut Vec<SensorEvent>,
    ) {
        self.contacts.retain(|_, contact| {
            let pair = contact.pair;
            let (Some((collider_a, body_a)), Some((collider_b, body_b))) = (
                placed(pair.collider_a, pair.body_a, colliders, bodies),
                placed(pair.collider_b, pair.body_b, colliders, bodies),
            ) else {
                return false;
            };
            if !(body_a.is_active() || body_b.is_active()) {
                return true;
            }

            let was_touching = contact.is_touching();
            let had_points = !contact.manifold.is_empty();
            let keep = broad_phase.fat_aabb(collider_a.proxy()).overlaps(&broad_phase.fat_aabb(collider_b.proxy()));
            let mut manifold = if keep && collider_a.layers().interacts_with(&collider_b.layers()) {
                collide(collider_a.shape(), body_a.transform(), collider_b.shape(), body_b.transform())
            } else {
                Manifold::default()
            };
            manifold.inherit_impulses(&contact.manifold);
            if !had_points && !manifold.is_empty() {
                // One-way platforms decide once, when the colliders start touching.
                let normal = manifold.normal();
                contact.enabled = collider_a.blocks_along(body_a.rotation(), normal)
                    && collider_b.blocks_along(body_b.rotation(), -normal);
            }
            contact.manifold = manifold;

            match (was_touching, contact.is_touching()) {
                (false, true) => {
                    let impact_speed = impact_speed(&contact.manifold, body_a, body_b);
                    events.push(CollisionEvent::Started { pair, impact_speed });
                }
                (true, false) => events.push(CollisionEvent::Stopped { pair }),
                _ => {}
            }
            keep
        });

        self.sensors.retain(|_, overlap| {
            let pair = overlap.pair;
            let (Some((sensor, sensor_body)), Some((visitor, visitor_body))) = (
                placed(pair.sensor, pair.sensor_body, colliders, bodies),
                placed(pair.visitor, pair.visitor_body, colliders, bodies),
            ) else {
                return false;
            };
            if !(sensor_body.is_active() || visitor_body.is_active()) {
                return true;
            }

            let keep = broad_phase.fat_aabb(sensor.proxy()).overlaps(&broad_phase.fat_aabb(visitor.proxy()));
            let overlapping = keep
                && sensor.layers().interacts_with(&visitor.layers())
                && separation_lower_bound(
                    sensor.shape(),
                    sensor_body.transform(),
                    visitor.shape(),
                    visitor_body.transform(),
                ) < 0.0;
            match (overlap.overlapping, overlapping) {
                (false, true) => sensor_events.push(SensorEvent::Entered(overlap.pair)),
                (true, false) => sensor_events.push(SensorEvent::Exited(overlap.pair)),
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

/// A collider with its body, or `None` if either was removed.
fn placed<'a>(
    collider: ColliderHandle,
    body: BodyHandle,
    colliders: &'a Arena<Collider>,
    bodies: &'a Arena<Body>,
) -> Option<(&'a Collider, &'a Body)> {
    Some((colliders.get(collider.0)?, bodies.get(body.0)?))
}

/// Sensors detect anything but their own body; two fixed bodies never move, so there is nothing
/// to detect between them.
fn should_sense(pair: &ContactPair, bodies: &Arena<Body>) -> bool {
    let (Some(body_a), Some(body_b)) = (bodies.get(pair.body_a.0), bodies.get(pair.body_b.0)) else { return false };
    pair.body_a != pair.body_b && !(body_a.body_type() == BodyType::Fixed && body_b.body_type() == BodyType::Fixed)
}

/// Contacts only matter when at least one dynamic body can react to them, and joints may opt
/// their bodies out of colliding.
/// The fastest approach speed of the two bodies along the normal, at the contact points.
fn impact_speed(manifold: &Manifold, a: &Body, b: &Body) -> f32 {
    manifold
        .points()
        .iter()
        .map(|point| {
            let relative_velocity = b.velocity_at_point(point.position()) - a.velocity_at_point(point.position());
            -relative_velocity.dot(manifold.normal())
        })
        .fold(0.0, f32::max)
}

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
