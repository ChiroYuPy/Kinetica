//! Joints: constraints connecting two bodies.
//!
//! Describe a joint with one of the `*JointDesc` types, anchors given as world points, and add
//! it with [`World::add_joint`](crate::World::add_joint). By default, the two connected bodies
//! stop colliding with each other.
//!
//! | Joint | Keeps | Typical use |
//! |-------|-------|-------------|
//! | [`RevoluteJoint`] | a shared pivot | hinges, wheels, ragdoll limbs, pendulums |
//! | [`DistanceJoint`] | a distance | rods, springs, ropes |
//! | [`PrismaticJoint`] | a slide axis | pistons, elevators, sliding doors |
//! | [`WeldJoint`] | the relative placement | breakable or bendable structures |
//! | [`MouseJoint`] | a point near a target | dragging bodies with the mouse |
//!
//! The handle returned by [`World::add_joint`](crate::World::add_joint) knows the type of the
//! joint, so indexing the world with it gives that joint directly:
//!
//! ```
//! use kinetica::prelude::*;
//!
//! let mut world = World::new();
//! let ground = world.add_body(BodyDesc::fixed());
//! let wheel = world.add_body(BodyDesc::dynamic().collider(ColliderDesc::circle(0.5)));
//!
//! let axle = world.add_joint(RevoluteJointDesc::new(ground, wheel, Vec2::ZERO));
//! world[axle].set_motor(Some(Motor { speed: 2.0, max_force: 50.0 }));
//! world.step(1.0 / 60.0);
//!
//! // Every joint reports the force it applies, to break it under load.
//! if world[axle].constraint_force().length() > 1000.0 {
//!     world.remove_joint(axle);
//! }
//! ```

/// Methods shared by [`Joint`] and every joint type, which reach the common data through
/// `base()` and `reaction_impulse()`.
macro_rules! shared_joint_methods {
    () => {
        /// First body.
        #[must_use]
        pub fn body_a(&self) -> BodyHandle {
            self.base().body_a
        }

        /// Second body.
        #[must_use]
        pub fn body_b(&self) -> BodyHandle {
            self.base().body_b
        }

        /// Given one body of the joint, returns the other one.
        #[must_use]
        pub fn other_body(&self, body: BodyHandle) -> Option<BodyHandle> {
            let base = self.base();
            match body {
                _ if body == base.body_a => Some(base.body_b),
                _ if body == base.body_b => Some(base.body_a),
                _ => None,
            }
        }

        /// Whether the two bodies still collide with each other.
        #[must_use]
        pub fn collide_connected(&self) -> bool {
            self.base().collide_connected
        }

        /// Force the joint applied to body B during the last step, in newtons, in world space.
        /// Body A received the opposite force. Useful to break joints under load.
        #[must_use]
        pub fn constraint_force(&self) -> Vec2 {
            self.base().inverse_substep * self.reaction_impulse().0
        }

        /// Torque the joint applied to body B during the last step, in N·m, not counting the
        /// torque of [`constraint_force`](Self::constraint_force) about the anchor. Body A
        /// received the opposite torque.
        #[must_use]
        pub fn constraint_torque(&self) -> f32 {
            self.base().inverse_substep * self.reaction_impulse().1
        }
    };
}

mod distance;
mod frame;
mod mouse;
mod prismatic;
mod revolute;
mod weld;

use std::fmt;
use std::hash::{Hash, Hasher};
use std::marker::PhantomData;
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
use crate::solver::{Pass, SolverContext};

/// Handle to a joint stored in a [`World`](crate::World).
///
/// `J` is the type of the joint: [`World::add_joint`](crate::World::add_joint) returns, say,
/// a `JointHandle<RevoluteJoint>`, and `world[handle]` is then a [`RevoluteJoint`]. A plain
/// `JointHandle` refers to a joint of any type and gives a [`Joint`].
///
/// A handle stays valid until its joint (or one of its bodies) is removed.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize), serde(bound = ""))]
pub struct JointHandle<J = Joint>(pub(crate) Index, #[cfg_attr(feature = "serde", serde(skip))] PhantomData<fn() -> J>);

impl<J> JointHandle<J> {
    pub(crate) fn new(index: Index) -> Self {
        Self(index, PhantomData)
    }

    /// The same handle, forgetting the joint type, to store joints of different types together.
    #[must_use]
    pub fn untyped(self) -> JointHandle {
        JointHandle::new(self.0)
    }
}

// Written by hand: derives would require `J` itself to be `Clone`, `Eq`…
impl<J> Clone for JointHandle<J> {
    fn clone(&self) -> Self {
        *self
    }
}

impl<J> Copy for JointHandle<J> {}

impl<J> PartialEq for JointHandle<J> {
    fn eq(&self, other: &Self) -> bool {
        self.0 == other.0
    }
}

impl<J> Eq for JointHandle<J> {}

impl<J> PartialOrd for JointHandle<J> {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl<J> Ord for JointHandle<J> {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.0.cmp(&other.0)
    }
}

impl<J> Hash for JointHandle<J> {
    fn hash<H: Hasher>(&self, state: &mut H) {
        self.0.hash(state);
    }
}

impl<J> fmt::Debug for JointHandle<J> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_tuple("JointHandle").field(&self.0).finish()
    }
}

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

/// A joint of any type, living in a [`World`](crate::World).
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Joint {
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

impl Joint {
    shared_joint_methods!();

    /// Attachment points in the local frames of body A and body B.
    ///
    /// For a mouse joint, the first point is the target, in world space.
    #[must_use]
    pub fn local_anchors(&self) -> (Vec2, Vec2) {
        match self {
            Self::Revolute(joint) => (joint.local_anchor_a(), joint.local_anchor_b()),
            Self::Distance(joint) => (joint.local_anchor_a(), joint.local_anchor_b()),
            Self::Prismatic(joint) => (joint.local_anchor_a(), joint.local_anchor_b()),
            Self::Weld(joint) => (joint.local_anchor_a(), joint.local_anchor_b()),
            Self::Mouse(joint) => (joint.target(), joint.local_anchor_b()),
        }
    }

    fn base(&self) -> &JointBase {
        match self {
            Self::Revolute(joint) => &joint.base,
            Self::Distance(joint) => &joint.base,
            Self::Prismatic(joint) => &joint.base,
            Self::Weld(joint) => &joint.base,
            Self::Mouse(joint) => &joint.base,
        }
    }

    fn base_mut(&mut self) -> &mut JointBase {
        match self {
            Self::Revolute(joint) => &mut joint.base,
            Self::Distance(joint) => &mut joint.base,
            Self::Prismatic(joint) => &mut joint.base,
            Self::Weld(joint) => &mut joint.base,
            Self::Mouse(joint) => &mut joint.base,
        }
    }

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

    /// Returns `true` if the joint stops `body` and `other` from colliding.
    pub(crate) fn prevents_collision(&self, body: BodyHandle, other: BodyHandle) -> bool {
        !self.collide_connected() && self.other_body(body) == Some(other)
    }

    pub(crate) fn prepare(&mut self, bodies: &Arena<Body>, context: &SolverContext) {
        let (Some(a), Some(b)) = (bodies.get(self.body_a().0), bodies.get(self.body_b().0)) else { return };
        let (local_anchor_a, local_anchor_b) = match self {
            // The target is not attached to body A.
            Self::Mouse(joint) => (a.local_center_of_mass(), joint.local_anchor_b()),
            _ => self.local_anchors(),
        };
        let base = self.base_mut();
        base.frame = JointFrame::new(a, b, local_anchor_a, local_anchor_b);
        base.inverse_substep = context.inverse_substep;
        let frame = base.frame;
        match self {
            Self::Revolute(joint) => joint.prepare(&frame, a, b),
            Self::Distance(joint) => joint.prepare(&frame, context),
            Self::Prismatic(joint) => joint.prepare(&frame, a, b),
            Self::Weld(joint) => joint.prepare(&frame, a, b, context),
            Self::Mouse(joint) => joint.prepare(b, context),
        }
    }

    pub(crate) fn warm_start(&self, bodies: &mut Arena<Body>) {
        let Some((a, b)) = bodies.get2_mut(self.body_a().0, self.body_b().0) else { return };
        let frame = &self.base().frame;
        match self {
            Self::Revolute(joint) => joint.warm_start(frame, a, b),
            Self::Distance(joint) => joint.warm_start(frame, a, b),
            Self::Prismatic(joint) => joint.warm_start(frame, a, b),
            Self::Weld(joint) => joint.warm_start(frame, a, b),
            Self::Mouse(joint) => joint.warm_start(frame, b),
        }
    }

    pub(crate) fn solve(&mut self, bodies: &mut Arena<Body>, context: &SolverContext, pass: Pass) {
        let Some((a, b)) = bodies.get2_mut(self.body_a().0, self.body_b().0) else { return };
        let frame = self.base().frame;
        match self {
            Self::Revolute(joint) => joint.solve(&frame, a, b, context, pass),
            Self::Distance(joint) => joint.solve(&frame, a, b, context, pass),
            Self::Prismatic(joint) => joint.solve(&frame, a, b, context, pass),
            Self::Weld(joint) => joint.solve(&frame, a, b, context, pass),
            // A soft spring by design: identical with and without bias.
            Self::Mouse(joint) => joint.solve(&frame, b, context),
        }
    }
}

/// What every joint holds besides its own settings.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub(crate) struct JointBase {
    body_a: BodyHandle,
    body_b: BodyHandle,
    collide_connected: bool,
    frame: JointFrame,
    /// Converts the stored per-substep impulses to forces.
    inverse_substep: f32,
}

impl JointBase {
    fn new(body_a: BodyHandle, body_b: BodyHandle, collide_connected: bool) -> Self {
        Self { body_a, body_b, collide_connected, frame: JointFrame::default(), inverse_substep: 0.0 }
    }
}

/// A type of joint: [`Joint`] for any joint, or one of the specific types such as
/// [`RevoluteJoint`]. It is the `J` of a [`JointHandle<J>`].
pub trait JointType: sealed::JointType {}

/// A joint description, accepted by [`World::add_joint`](crate::World::add_joint).
pub trait JointDesc: sealed::JointDesc {
    /// The type of joint it creates.
    type Joint: JointType;
}

/// Keeps the methods of [`JointType`] and [`JointDesc`] internal, and the traits closed.
mod sealed {
    use super::Joint;
    use crate::body::{Body, BodyHandle};

    pub trait JointType: Sized {
        fn from_joint(joint: &Joint) -> Option<&Self>;
        fn from_joint_mut(joint: &mut Joint) -> Option<&mut Self>;
    }

    pub trait JointDesc {
        fn bodies(&self) -> (BodyHandle, BodyHandle);
        /// Builds the joint from the current placement of its bodies.
        fn create(self, a: &Body, b: &Body) -> Joint;
    }
}

impl sealed::JointType for Joint {
    fn from_joint(joint: &Joint) -> Option<&Self> {
        Some(joint)
    }

    fn from_joint_mut(joint: &mut Joint) -> Option<&mut Self> {
        Some(joint)
    }
}

impl JointType for Joint {}

/// Links a joint type to its variant of [`Joint`] and to its description, and gives it the
/// shared methods.
macro_rules! joint_type {
    ($joint:ident, $desc:ident, $variant:ident) => {
        impl $joint {
            shared_joint_methods!();

            fn base(&self) -> &JointBase {
                &self.base
            }
        }

        impl sealed::JointType for $joint {
            fn from_joint(joint: &Joint) -> Option<&Self> {
                if let Joint::$variant(joint) = joint { Some(joint) } else { None }
            }

            fn from_joint_mut(joint: &mut Joint) -> Option<&mut Self> {
                if let Joint::$variant(joint) = joint { Some(joint) } else { None }
            }
        }

        impl JointType for $joint {}

        impl sealed::JointDesc for $desc {
            fn bodies(&self) -> (BodyHandle, BodyHandle) {
                (self.body_a, self.body_b)
            }

            fn create(self, a: &Body, b: &Body) -> Joint {
                Joint::$variant($joint::new(&self, a, b))
            }
        }

        impl JointDesc for $desc {
            type Joint = $joint;
        }
    };
}

joint_type!(RevoluteJoint, RevoluteJointDesc, Revolute);
joint_type!(DistanceJoint, DistanceJointDesc, Distance);
joint_type!(PrismaticJoint, PrismaticJointDesc, Prismatic);
joint_type!(WeldJoint, WeldJointDesc, Weld);
joint_type!(MouseJoint, MouseJointDesc, Mouse);

/// Converts a range to `(lower, upper)`, checking it is not empty.
fn validated_range(range: RangeInclusive<f32>) -> (f32, f32) {
    let (lower, upper) = range.into_inner();
    assert!(lower <= upper, "joint limits {lower}..={upper} are empty");
    (lower, upper)
}
