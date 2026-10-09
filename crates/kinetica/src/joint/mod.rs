//! Joints: constraints connecting two bodies.
//!
//! Create a joint from one of the `*JointDesc` types with
//! [`World::add_joint`](crate::World::add_joint). By default, the two connected bodies stop
//! colliding with each other.
//!
//! | Joint | Keeps | Typical use |
//! |-------|-------|-------------|
//! | [`RevoluteJoint`] | a shared pivot | hinges, wheels, ragdoll limbs, pendulums |
//! | [`DistanceJoint`] | a distance | rods, springs, ropes |
//! | [`PrismaticJoint`] | a slide axis | pistons, elevators, sliding doors |
//! | [`WeldJoint`] | the relative placement | breakable or bendable structures |
//! | [`MouseJoint`] | a point near a target | dragging bodies with the mouse |

mod distance;
mod frame;
mod mouse;
mod prismatic;
mod revolute;
mod weld;

use std::ops::RangeInclusive;

pub use distance::{DistanceJoint, DistanceJointDesc};
use frame::JointFrame;
pub use mouse::{MouseJoint, MouseJointDesc};
pub use prismatic::{PrismaticJoint, PrismaticJointDesc};
pub use revolute::{RevoluteJoint, RevoluteJointDesc};
pub use weld::{WeldJoint, WeldJointDesc};

use crate::arena::{Arena, Index};
use crate::body::{Body, BodyHandle};
use crate::math::Vec2;
use crate::solver::SolverContext;
use crate::world::World;

/// Handle to a [`Joint`] stored in a [`World`].
///
/// A handle stays valid until its joint (or one of its bodies) is removed.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct JointHandle(pub(crate) Index);

/// Drives a joint towards a target speed with a bounded force.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Motor {
    /// Target speed: rad/s for revolute joints, m/s for prismatic joints.
    pub speed: f32,
    /// Strongest effort the motor may apply: a torque (N·m) for revolute joints, a force (N)
    /// for prismatic joints.
    pub max_force: f32,
}

/// A damped spring, described by its natural frequency and damping ratio.
///
/// A damping ratio of 1 settles as fast as possible without oscillating; lower values bounce.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Spring {
    /// Oscillation frequency, in Hz.
    pub hertz: f32,
    /// Damping ratio, usually between 0 and 1.
    pub damping_ratio: f32,
}

/// Description of any joint, accepted by [`World::add_joint`].
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum JointDesc {
    /// See [`RevoluteJointDesc`].
    Revolute(RevoluteJointDesc),
    /// See [`DistanceJointDesc`].
    Distance(DistanceJointDesc),
    /// See [`PrismaticJointDesc`].
    Prismatic(PrismaticJointDesc),
    /// See [`WeldJointDesc`].
    Weld(WeldJointDesc),
    /// See [`MouseJointDesc`].
    Mouse(MouseJointDesc),
}

impl JointDesc {
    pub(crate) fn bodies(&self) -> (BodyHandle, BodyHandle) {
        match self {
            Self::Revolute(desc) => (desc.body_a, desc.body_b),
            Self::Distance(desc) => (desc.body_a, desc.body_b),
            Self::Prismatic(desc) => (desc.body_a, desc.body_b),
            Self::Weld(desc) => (desc.body_a, desc.body_b),
            Self::Mouse(desc) => (desc.body_a, desc.body_b),
        }
    }

    fn collide_connected(&self) -> bool {
        match self {
            Self::Revolute(desc) => desc.collide_connected,
            Self::Distance(desc) => desc.collide_connected,
            Self::Prismatic(desc) => desc.collide_connected,
            Self::Weld(desc) => desc.collide_connected,
            // The reference body is not really connected.
            Self::Mouse(_) => true,
        }
    }
}

impl From<RevoluteJointDesc> for JointDesc {
    fn from(desc: RevoluteJointDesc) -> Self {
        Self::Revolute(desc)
    }
}

impl From<DistanceJointDesc> for JointDesc {
    fn from(desc: DistanceJointDesc) -> Self {
        Self::Distance(desc)
    }
}

impl From<PrismaticJointDesc> for JointDesc {
    fn from(desc: PrismaticJointDesc) -> Self {
        Self::Prismatic(desc)
    }
}

impl From<WeldJointDesc> for JointDesc {
    fn from(desc: WeldJointDesc) -> Self {
        Self::Weld(desc)
    }
}

impl From<MouseJointDesc> for JointDesc {
    fn from(desc: MouseJointDesc) -> Self {
        Self::Mouse(desc)
    }
}

/// The specific behaviour of a [`Joint`].
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum JointKind {
    /// See [`RevoluteJoint`].
    Revolute(RevoluteJoint),
    /// See [`DistanceJoint`].
    Distance(DistanceJoint),
    /// See [`PrismaticJoint`].
    Prismatic(PrismaticJoint),
    /// See [`WeldJoint`].
    Weld(WeldJoint),
    /// See [`MouseJoint`].
    Mouse(MouseJoint),
}

/// A constraint between two bodies, living in a [`World`].
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Joint {
    body_a: BodyHandle,
    body_b: BodyHandle,
    collide_connected: bool,
    kind: JointKind,
    frame: JointFrame,
    /// Converts the stored per-substep impulses to forces.
    inverse_substep: f32,
}

impl Joint {
    /// Builds the joint; both bodies must exist and be distinct.
    pub(crate) fn new(desc: &JointDesc, a: &Body, b: &Body) -> Self {
        let (body_a, body_b) = desc.bodies();
        let kind = match desc {
            JointDesc::Revolute(desc) => JointKind::Revolute(RevoluteJoint::new(desc)),
            JointDesc::Distance(desc) => JointKind::Distance(DistanceJoint::new(desc, a, b)),
            JointDesc::Prismatic(desc) => JointKind::Prismatic(PrismaticJoint::new(desc)),
            JointDesc::Weld(desc) => JointKind::Weld(WeldJoint::new(desc)),
            JointDesc::Mouse(desc) => JointKind::Mouse(MouseJoint::new(desc)),
        };
        Self {
            body_a,
            body_b,
            collide_connected: desc.collide_connected(),
            kind,
            frame: JointFrame::default(),
            inverse_substep: 0.0,
        }
    }

    /// First body.
    #[must_use]
    pub fn body_a(&self) -> BodyHandle {
        self.body_a
    }

    /// Second body.
    #[must_use]
    pub fn body_b(&self) -> BodyHandle {
        self.body_b
    }

    /// Given one body of the joint, returns the other one.
    #[must_use]
    pub fn other_body(&self, body: BodyHandle) -> Option<BodyHandle> {
        match body {
            _ if body == self.body_a => Some(self.body_b),
            _ if body == self.body_b => Some(self.body_a),
            _ => None,
        }
    }

    /// Whether the two bodies still collide with each other.
    #[must_use]
    pub fn collide_connected(&self) -> bool {
        self.collide_connected
    }

    /// The specific behaviour, to read its settings.
    #[must_use]
    pub fn kind(&self) -> &JointKind {
        &self.kind
    }

    /// The specific behaviour, to change its settings (motor, limits, target…).
    pub fn kind_mut(&mut self) -> &mut JointKind {
        &mut self.kind
    }

    /// Attachment points in the local frames of body A and body B.
    ///
    /// For a mouse joint, the first point is the target, in world space.
    #[must_use]
    pub fn local_anchors(&self) -> (Vec2, Vec2) {
        match &self.kind {
            JointKind::Revolute(joint) => (joint.local_anchor_a(), joint.local_anchor_b()),
            JointKind::Distance(joint) => (joint.local_anchor_a(), joint.local_anchor_b()),
            JointKind::Prismatic(joint) => (joint.local_anchor_a(), joint.local_anchor_b()),
            JointKind::Weld(joint) => (joint.local_anchor_a(), joint.local_anchor_b()),
            JointKind::Mouse(joint) => (joint.target(), joint.local_anchor_b()),
        }
    }

    /// Force the joint applied to body B during the last step, in newtons, in world space.
    /// Body A received the opposite force.
    ///
    /// Useful to break joints under load:
    ///
    /// ```
    /// # use kinetica::prelude::*;
    /// # let mut world = World::new();
    /// # let ceiling = world.add_body(BodyDesc::fixed());
    /// # let lamp = world.add_body(BodyDesc::dynamic().position(vec2(0.0, -1.0)));
    /// # world.add_collider(lamp, ColliderDesc::circle(0.2));
    /// let chain = world.add_joint(RevoluteJointDesc::at_world_point(&world, ceiling, lamp, Vec2::ZERO));
    /// world.step(1.0 / 60.0);
    /// if world.joint(chain).unwrap().constraint_force().length() > 1000.0 {
    ///     world.remove_joint(chain);
    /// }
    /// ```
    #[must_use]
    pub fn constraint_force(&self) -> Vec2 {
        self.inverse_substep * self.kind.reaction_impulse().0
    }

    /// Torque the joint applied to body B during the last step, in N·m, not counting the
    /// torque of [`constraint_force`](Self::constraint_force) about the anchor. Body A
    /// received the opposite torque.
    #[must_use]
    pub fn constraint_torque(&self) -> f32 {
        self.inverse_substep * self.kind.reaction_impulse().1
    }

    /// Returns `true` if the joint stops `body` and `other` from colliding.
    pub(crate) fn prevents_collision(&self, body: BodyHandle, other: BodyHandle) -> bool {
        !self.collide_connected && self.other_body(body) == Some(other)
    }

    pub(crate) fn prepare(&mut self, bodies: &Arena<Body>, context: &SolverContext) {
        let (Some(a), Some(b)) = (bodies.get(self.body_a.0), bodies.get(self.body_b.0)) else { return };
        let (local_anchor_a, local_anchor_b) = match &self.kind {
            // The target is not attached to body A.
            JointKind::Mouse(joint) => (a.local_center_of_mass(), joint.local_anchor_b()),
            _ => self.local_anchors(),
        };
        self.frame = JointFrame::new(a, b, local_anchor_a, local_anchor_b);
        self.inverse_substep = context.inverse_substep;
        match &mut self.kind {
            JointKind::Revolute(joint) => joint.prepare(&self.frame, a, b),
            JointKind::Distance(joint) => joint.prepare(&self.frame, context),
            JointKind::Prismatic(joint) => joint.prepare(&self.frame, a, b),
            JointKind::Weld(joint) => joint.prepare(&self.frame, a, b, context),
            JointKind::Mouse(joint) => joint.prepare(b, context),
        }
    }

    pub(crate) fn warm_start(&mut self, bodies: &mut Arena<Body>) {
        let Some((a, b)) = bodies.get2_mut(self.body_a.0, self.body_b.0) else { return };
        let frame = &self.frame;
        match &self.kind {
            JointKind::Revolute(joint) => joint.warm_start(frame, a, b),
            JointKind::Distance(joint) => joint.warm_start(frame, a, b),
            JointKind::Prismatic(joint) => joint.warm_start(frame, a, b),
            JointKind::Weld(joint) => joint.warm_start(frame, a, b),
            JointKind::Mouse(joint) => joint.warm_start(frame, b),
        }
    }

    pub(crate) fn solve(&mut self, bodies: &mut Arena<Body>, context: &SolverContext, use_bias: bool) {
        let Some((a, b)) = bodies.get2_mut(self.body_a.0, self.body_b.0) else { return };
        let frame = &self.frame;
        match &mut self.kind {
            JointKind::Revolute(joint) => joint.solve(frame, a, b, context, use_bias),
            JointKind::Distance(joint) => joint.solve(frame, a, b, context, use_bias),
            JointKind::Prismatic(joint) => joint.solve(frame, a, b, context, use_bias),
            JointKind::Weld(joint) => joint.solve(frame, a, b, context, use_bias),
            // A soft spring by design: identical with and without bias.
            JointKind::Mouse(joint) => joint.solve(frame, b, context),
        }
    }
}

impl JointKind {
    /// Linear and angular impulses applied to body B during the last substep.
    fn reaction_impulse(&self) -> (Vec2, f32) {
        match self {
            Self::Revolute(joint) => joint.reaction_impulse(),
            Self::Distance(joint) => joint.reaction_impulse(),
            Self::Prismatic(joint) => joint.reaction_impulse(),
            Self::Weld(joint) => joint.reaction_impulse(),
            Self::Mouse(joint) => joint.reaction_impulse(),
        }
    }
}

/// Looks up the two bodies a joint description refers to.
fn bodies_of(world: &World, body_a: BodyHandle, body_b: BodyHandle) -> (&Body, &Body) {
    let a = world.body(body_a).expect("joint body A was removed from the world");
    let b = world.body(body_b).expect("joint body B was removed from the world");
    (a, b)
}

/// Converts a range to `(lower, upper)`, checking it is not empty.
fn validated_range(range: RangeInclusive<f32>) -> (f32, f32) {
    let (lower, upper) = range.into_inner();
    assert!(lower <= upper, "joint limits {lower}..={upper} are empty");
    (lower, upper)
}
