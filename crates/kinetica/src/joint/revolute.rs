//! Revolute joint: a pivot.

use std::ops::RangeInclusive;

use super::frame::{
    JointFrame, apply_angular_impulse, apply_linear_impulse, limit_impulse, motor_impulse, relative_angle,
    relative_velocity, wrap_angle,
};
use super::{JointBase, Motor, validated_range};
use crate::body::{Body, BodyHandle};
use crate::math::Vec2;
use crate::solver::{Pass, SolverContext};

/// Description of a [`RevoluteJoint`].
///
/// ```
/// use kinetica::prelude::*;
///
/// let mut world = World::new();
/// let ground = world.add_body(BodyDesc::fixed());
/// let arm = world.add_body(BodyDesc::dynamic().position(vec2(1.0, 0.0)).collider(ColliderDesc::rectangle(2.0, 0.2)));
///
/// world.add_joint(RevoluteJointDesc::new(ground, arm, Vec2::ZERO).limits(-1.0..=1.0));
/// ```
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct RevoluteJointDesc {
    /// First body.
    pub body_a: BodyHandle,
    /// Second body.
    pub body_b: BodyHandle,
    /// Pivot, in world space.
    pub anchor: Vec2,
    /// Allowed range of the joint angle, in radians.
    pub limits: Option<RangeInclusive<f32>>,
    /// Drives the relative rotation.
    pub motor: Option<Motor>,
    /// Whether the two bodies still collide with each other.
    pub collide_connected: bool,
}

impl RevoluteJointDesc {
    /// Describes a pivot at the world point `anchor`. The joint angle is zero in the placement
    /// the bodies have when the joint is added.
    #[must_use]
    pub fn new(body_a: BodyHandle, body_b: BodyHandle, anchor: Vec2) -> Self {
        Self { body_a, body_b, anchor, limits: None, motor: None, collide_connected: false }
    }

    /// Restricts the joint angle to `angles`, in radians.
    ///
    /// # Panics
    ///
    /// Panics when the joint is added if the range is empty.
    #[must_use]
    pub fn limits(mut self, angles: RangeInclusive<f32>) -> Self {
        self.limits = Some(angles);
        self
    }

    /// Drives the joint at `speed` rad/s, with a torque of at most `max_torque` N·m.
    #[must_use]
    pub fn motor(mut self, speed: f32, max_torque: f32) -> Self {
        self.motor = Some(Motor { speed, max_force: max_torque });
        self
    }

    /// Lets the two bodies keep colliding with each other.
    #[must_use]
    pub fn collide_connected(mut self) -> Self {
        self.collide_connected = true;
        self
    }
}

/// A pivot: the anchors of both bodies stay together, the bodies rotate freely around it,
/// optionally within limits and driven by a motor.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct RevoluteJoint {
    pub(super) base: JointBase,
    local_anchor_a: Vec2,
    local_anchor_b: Vec2,
    reference_angle: f32,
    limits: Option<(f32, f32)>,
    motor: Option<Motor>,

    linear_impulse: Vec2,
    motor_impulse: f32,
    lower_impulse: f32,
    upper_impulse: f32,
    /// Joint angle at the start of the step.
    delta_angle: f32,
    axial_mass: f32,
}

impl RevoluteJoint {
    pub(super) fn new(desc: &RevoluteJointDesc, a: &Body, b: &Body) -> Self {
        Self {
            base: JointBase::new(desc.body_a, desc.body_b, desc.collide_connected),
            local_anchor_a: a.local_point(desc.anchor),
            local_anchor_b: b.local_point(desc.anchor),
            reference_angle: relative_angle(b.rotation(), a.rotation()),
            limits: desc.limits.clone().map(validated_range),
            motor: desc.motor,
            linear_impulse: Vec2::ZERO,
            motor_impulse: 0.0,
            lower_impulse: 0.0,
            upper_impulse: 0.0,
            delta_angle: 0.0,
            axial_mass: 0.0,
        }
    }

    /// Pivot in the local frame of body A.
    #[must_use]
    pub fn local_anchor_a(&self) -> Vec2 {
        self.local_anchor_a
    }

    /// Pivot in the local frame of body B.
    #[must_use]
    pub fn local_anchor_b(&self) -> Vec2 {
        self.local_anchor_b
    }

    /// Allowed range of the joint angle, if limited.
    #[must_use]
    pub fn limits(&self) -> Option<RangeInclusive<f32>> {
        self.limits.map(|(lower, upper)| lower..=upper)
    }

    /// Sets or removes the angle limits.
    ///
    /// # Panics
    ///
    /// Panics if the range is empty.
    pub fn set_limits(&mut self, angles: Option<RangeInclusive<f32>>) {
        self.limits = angles.map(validated_range);
        self.lower_impulse = 0.0;
        self.upper_impulse = 0.0;
    }

    /// The motor, if any.
    #[must_use]
    pub fn motor(&self) -> Option<Motor> {
        self.motor
    }

    /// Sets or removes the motor; `max_force` is a torque in N·m.
    pub fn set_motor(&mut self, motor: Option<Motor>) {
        self.motor = motor;
        if motor.is_none() {
            self.motor_impulse = 0.0;
        }
    }

    pub(crate) fn reaction_impulse(&self) -> (Vec2, f32) {
        (self.linear_impulse, self.motor_impulse + self.lower_impulse - self.upper_impulse)
    }

    pub(crate) fn prepare(&mut self, frame: &JointFrame, a: &Body, b: &Body) {
        self.delta_angle = wrap_angle(relative_angle(b.rotation(), a.rotation()) - self.reference_angle);
        self.axial_mass = frame.axial_mass();
    }

    pub(crate) fn warm_start(&self, frame: &JointFrame, a: &mut Body, b: &mut Body) {
        let (anchor_a, anchor_b) = frame.current_anchors(a, b);
        apply_linear_impulse(a, b, self.linear_impulse, anchor_a, anchor_b);
        apply_angular_impulse(a, b, self.motor_impulse + self.lower_impulse - self.upper_impulse);
    }

    pub(crate) fn solve(
        &mut self,
        frame: &JointFrame,
        a: &mut Body,
        b: &mut Body,
        context: &SolverContext,
        pass: Pass,
    ) {
        let angular_speed = |a: &Body, b: &Body| b.angular_velocity() - a.angular_velocity();

        if let Some(motor) = self.motor {
            let speed = angular_speed(a, b);
            let impulse = motor_impulse(motor, &mut self.motor_impulse, self.axial_mass, speed, context.substep);
            apply_angular_impulse(a, b, impulse);
        }

        if let Some((lower, upper)) = self.limits {
            let angle = self.delta_angle + relative_angle(b.delta_rotation(), a.delta_rotation());

            // Lower limit: keeps the angle above `lower`.
            let gap = angle - lower;
            let softness = context.limit_softness(gap, pass);
            let speed = angular_speed(a, b);
            let impulse = limit_impulse(&mut self.lower_impulse, self.axial_mass, speed, gap, softness);
            apply_angular_impulse(a, b, impulse);

            // Upper limit: keeps the angle below `upper`, pushing the other way.
            let gap = upper - angle;
            let softness = context.limit_softness(gap, pass);
            let speed = -angular_speed(a, b);
            let impulse = limit_impulse(&mut self.upper_impulse, self.axial_mass, speed, gap, softness);
            apply_angular_impulse(a, b, -impulse);
        }

        // The pivot itself, solved last so it wins over the motor and the limits.
        let (anchor_a, anchor_b) = frame.current_anchors(a, b);
        let velocity = relative_velocity(a, b, anchor_a, anchor_b);
        let separation = frame.separation(a, b, anchor_a, anchor_b);
        let inverse_mass = frame.inverse_point_mass(anchor_a, anchor_b);
        let impulse =
            context.equality_softness(pass).impulse_2d(&inverse_mass, velocity, separation, self.linear_impulse);
        self.linear_impulse += impulse;
        apply_linear_impulse(a, b, impulse, anchor_a, anchor_b);
    }
}
