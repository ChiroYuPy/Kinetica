//! Events reporting when contacts and sensor overlaps start and stop.

use super::ContactPair;
use crate::body::BodyHandle;
use crate::collider::ColliderHandle;

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

impl SensorPair {
    /// The pair seen from its sensor: collider A when `a_is_sensor`, collider B otherwise.
    pub(crate) fn new(pair: &ContactPair, a_is_sensor: bool) -> Self {
        if a_is_sensor {
            Self {
                sensor: pair.collider_a,
                sensor_body: pair.body_a,
                visitor: pair.collider_b,
                visitor_body: pair.body_b,
            }
        } else {
            Self {
                sensor: pair.collider_b,
                sensor_body: pair.body_b,
                visitor: pair.collider_a,
                visitor_body: pair.body_a,
            }
        }
    }
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
