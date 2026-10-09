//! Mouse joint: pulls a point of a body towards a target, like dragging it with the mouse.

use super::frame::JointFrame;
use super::{JointBase, Spring};
use crate::body::{Body, BodyHandle};
use crate::math::{SymmetricMatrix, Vec2};
use crate::solver::{Softness, SolverContext};

/// Description of a [`MouseJoint`].
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct MouseJointDesc {
    /// Reference body, not affected by the joint; usually a fixed body.
    pub body_a: BodyHandle,
    /// The dragged body.
    pub body_b: BodyHandle,
    /// World point of body B that is dragged, and the initial target.
    pub anchor: Vec2,
    /// Stiffness of the pull.
    pub spring: Spring,
    /// Strongest force the joint may apply, in N.
    pub max_force: f32,
}

impl MouseJointDesc {
    /// Describes dragging `body` by the world point `anchor`, which is also the initial target.
    /// `reference` is any body (typically fixed) and is not affected.
    ///
    /// The default pull is a 5 Hz spring with a damping ratio of 0.7 and no force limit.
    #[must_use]
    pub fn new(reference: BodyHandle, body: BodyHandle, anchor: Vec2) -> Self {
        Self {
            body_a: reference,
            body_b: body,
            anchor,
            spring: Spring { hertz: 5.0, damping_ratio: 0.7 },
            max_force: f32::INFINITY,
        }
    }

    /// Sets the stiffness of the pull.
    #[must_use]
    pub fn spring(mut self, hertz: f32, damping_ratio: f32) -> Self {
        self.spring = Spring { hertz, damping_ratio };
        self
    }

    /// Limits the force of the pull, in N.
    #[must_use]
    pub fn max_force(mut self, max_force: f32) -> Self {
        self.max_force = max_force;
        self
    }
}

/// Pulls a point of body B towards a movable world target with a damped spring.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct MouseJoint {
    pub(super) base: JointBase,
    target: Vec2,
    local_anchor_b: Vec2,
    spring: Spring,
    max_force: f32,

    linear_impulse: Vec2,
    softness: Softness,
    /// Center of mass of B minus the target, at the start of the step.
    offset: Vec2,
}

impl MouseJoint {
    pub(super) fn new(desc: &MouseJointDesc, _reference: &Body, b: &Body) -> Self {
        Self {
            // The reference body is not really connected.
            base: JointBase::new(desc.body_a, desc.body_b, true),
            target: desc.anchor,
            local_anchor_b: b.local_point(desc.anchor),
            spring: desc.spring,
            max_force: desc.max_force,
            linear_impulse: Vec2::ZERO,
            softness: Softness::new(0.0, 0.0, 1.0),
            offset: Vec2::ZERO,
        }
    }

    /// The point the body is pulled towards.
    #[must_use]
    pub fn target(&self) -> Vec2 {
        self.target
    }

    /// Moves the target.
    pub fn set_target(&mut self, target: Vec2) {
        self.target = target;
    }

    /// Dragged point in the local frame of body B.
    #[must_use]
    pub fn local_anchor_b(&self) -> Vec2 {
        self.local_anchor_b
    }

    /// Strongest force the joint may apply, in N.
    #[must_use]
    pub fn max_force(&self) -> f32 {
        self.max_force
    }

    /// Limits the force of the pull, in N.
    pub fn set_max_force(&mut self, max_force: f32) {
        self.max_force = max_force;
    }

    pub(crate) fn reaction_impulse(&self) -> (Vec2, f32) {
        (self.linear_impulse, 0.0)
    }

    pub(crate) fn prepare(&mut self, b: &Body, context: &SolverContext) {
        self.softness = Softness::new(self.spring.hertz, self.spring.damping_ratio, context.substep);
        self.offset = b.center_of_mass() - self.target;
    }

    pub(crate) fn warm_start(&self, frame: &JointFrame, b: &mut Body) {
        let anchor = b.delta_rotation().rotate(frame.anchor_b);
        b.apply_solver_impulse(self.linear_impulse, anchor);
    }

    pub(crate) fn solve(&mut self, frame: &JointFrame, b: &mut Body, context: &SolverContext) {
        let anchor = b.delta_rotation().rotate(frame.anchor_b);
        let velocity = b.linear_velocity() + b.angular_velocity() * anchor.perp();
        let error = b.delta_position() + anchor + self.offset;

        let (mass, inertia) = (frame.mass_b, frame.inertia_b);
        let inverse_mass = SymmetricMatrix {
            xx: mass + inertia * anchor.y * anchor.y,
            xy: -inertia * anchor.x * anchor.y,
            yy: mass + inertia * anchor.x * anchor.x,
        };
        let impulse = self.softness.impulse_2d(&inverse_mass, velocity, error, self.linear_impulse);

        let previous = self.linear_impulse;
        self.linear_impulse = (previous + impulse).clamp_length_max(self.max_force * context.substep);
        b.apply_solver_impulse(self.linear_impulse - previous, anchor);
    }
}
