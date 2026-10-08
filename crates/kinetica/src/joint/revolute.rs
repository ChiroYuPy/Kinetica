//! Revolute joint: a pivot.

use std::ops::RangeInclusive;

use super::frame::{
    BiasTerms, JointFrame, apply_angular_impulse, apply_linear_impulse, relative_angle, relative_velocity, wrap_angle,
};
use super::{Motor, validated_range};
use crate::body::{Body, BodyHandle};
use crate::math::Vec2;
use crate::solver::SolverContext;
use crate::world::World;

/// Description of a [`RevoluteJoint`].
///
/// ```
/// use kinetica::prelude::*;
///
/// let mut world = World::new();
/// let ground = world.add_body(BodyDesc::fixed());
/// let arm = world.add_body(BodyDesc::dynamic().position(vec2(1.0, 0.0)));
/// world.add_collider(arm, ColliderDesc::rectangle(1.0, 0.1));
///
/// let pivot = RevoluteJointDesc::at_world_point(&world, ground, arm, Vec2::ZERO).limits(-1.0..=1.0);
/// world.add_joint(pivot);
/// ```
#[derive(Debug, Clone, PartialEq)]
pub struct RevoluteJointDesc {
    /// First body.
    pub body_a: BodyHandle,
    /// Second body.
    pub body_b: BodyHandle,
    /// Pivot, in the local frame of body A.
    pub local_anchor_a: Vec2,
    /// Pivot, in the local frame of body B.
    pub local_anchor_b: Vec2,
    /// Angle of B relative to A at which the joint angle is zero.
    pub reference_angle: f32,
    /// Allowed range of the joint angle, in radians.
    pub limits: Option<RangeInclusive<f32>>,
    /// Drives the relative rotation.
    pub motor: Option<Motor>,
    /// Whether the two bodies still collide with each other.
    pub collide_connected: bool,
}

impl RevoluteJointDesc {
    /// Describes a pivot given in each body's local frame.
    #[must_use]
    pub fn new(body_a: BodyHandle, body_b: BodyHandle, local_anchor_a: Vec2, local_anchor_b: Vec2) -> Self {
        Self {
            body_a,
            body_b,
            local_anchor_a,
            local_anchor_b,
            reference_angle: 0.0,
            limits: None,
            motor: None,
            collide_connected: false,
        }
    }

    /// Describes a pivot at a world point, with the current relative angle as zero.
    ///
    /// # Panics
    ///
    /// Panics if either body was removed from `world`.
    #[must_use]
    pub fn at_world_point(world: &World, body_a: BodyHandle, body_b: BodyHandle, point: Vec2) -> Self {
        let (a, b) = super::bodies_of(world, body_a, body_b);
        Self {
            reference_angle: relative_angle(b.rotation(), a.rotation()),
            ..Self::new(body_a, body_b, a.local_point(point), b.local_point(point))
        }
    }

    /// Restricts the joint angle to `angles`, in radians.
    ///
    /// # Panics
    ///
    /// Panics if the range is empty.
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

    /// Sets whether the two bodies still collide with each other.
    #[must_use]
    pub fn collide_connected(mut self, collide: bool) -> Self {
        self.collide_connected = collide;
        self
    }
}

/// A pivot: the anchors of both bodies stay together, the bodies rotate freely around it,
/// optionally within limits and driven by a motor.
#[derive(Debug, Clone)]
pub struct RevoluteJoint {
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
    pub(crate) fn new(desc: &RevoluteJointDesc) -> Self {
        Self {
            local_anchor_a: desc.local_anchor_a,
            local_anchor_b: desc.local_anchor_b,
            reference_angle: desc.reference_angle,
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
        use_bias: bool,
    ) {
        if let Some(motor) = self.motor {
            let speed_error = b.angular_velocity() - a.angular_velocity() - motor.speed;
            let max_impulse = motor.max_force * context.substep;
            let previous = self.motor_impulse;
            self.motor_impulse = (previous - self.axial_mass * speed_error).clamp(-max_impulse, max_impulse);
            apply_angular_impulse(a, b, self.motor_impulse - previous);
        }

        if let Some((lower, upper)) = self.limits {
            let angle = self.delta_angle + relative_angle(b.delta_rotation(), a.delta_rotation());

            // Lower limit: keeps the angle above `lower`.
            let terms = BiasTerms::limit(angle - lower, context, use_bias);
            let speed = b.angular_velocity() - a.angular_velocity();
            let impulse =
                -self.axial_mass * terms.mass_scale * (speed + terms.bias) - terms.impulse_scale * self.lower_impulse;
            let previous = self.lower_impulse;
            self.lower_impulse = (previous + impulse).max(0.0);
            apply_angular_impulse(a, b, self.lower_impulse - previous);

            // Upper limit: same, mirrored.
            let terms = BiasTerms::limit(upper - angle, context, use_bias);
            let speed = a.angular_velocity() - b.angular_velocity();
            let impulse =
                -self.axial_mass * terms.mass_scale * (speed + terms.bias) - terms.impulse_scale * self.upper_impulse;
            let previous = self.upper_impulse;
            self.upper_impulse = (previous + impulse).max(0.0);
            apply_angular_impulse(a, b, previous - self.upper_impulse);
        }

        // The pivot itself, solved last so it wins over the motor and the limits.
        let (anchor_a, anchor_b) = frame.current_anchors(a, b);
        let velocity_error = relative_velocity(a, b, anchor_a, anchor_b);
        let (bias, mass_scale, impulse_scale) = if use_bias {
            let separation = frame.separation(a, b, anchor_a, anchor_b);
            let softness = context.joint_softness;
            (softness.bias_rate * separation, softness.mass_scale, softness.impulse_scale)
        } else {
            (Vec2::ZERO, 1.0, 0.0)
        };
        let mass = frame.point_mass(anchor_a, anchor_b);
        let impulse = -mass_scale * mass.solve(velocity_error + bias) - impulse_scale * self.linear_impulse;
        self.linear_impulse += impulse;
        apply_linear_impulse(a, b, impulse, anchor_a, anchor_b);
    }
}
