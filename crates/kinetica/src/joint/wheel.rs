//! Wheel joint: a wheel on a sprung suspension.

use std::ops::RangeInclusive;

use super::frame::{JointFrame, SlideGeometry, apply_impulses, motor_impulse, solve_slide_limits};
use super::{JointBase, Motor, Spring, validated_range};
use crate::body::{Body, BodyHandle};
use crate::math::Vec2;
use crate::solver::{Pass, Softness, SolverContext};

/// Description of a [`WheelJoint`].
///
/// ```
/// use kinetica::prelude::*;
///
/// let mut world = World::new();
/// let chassis = world.add_body(BodyDesc::dynamic().collider(ColliderDesc::rectangle(3.0, 0.5)));
/// let axle = vec2(1.0, -0.8);
/// let wheel = world.add_body(BodyDesc::dynamic().position(axle).collider(ColliderDesc::circle(0.4)));
///
/// // The wheel moves up and down on a 4 Hz spring, and a motor turns it.
/// let joint = world.add_joint(WheelJointDesc::new(chassis, wheel, axle, Vec2::Y).spring(4.0, 0.7).motor(-20.0, 50.0));
/// ```
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct WheelJointDesc {
    /// The body carrying the wheel, such as a chassis.
    pub body_a: BodyHandle,
    /// The wheel.
    pub body_b: BodyHandle,
    /// The axle, in world space: usually the wheel's center.
    pub anchor: Vec2,
    /// Direction the suspension moves along, in world space.
    pub axis: Vec2,
    /// Pushes the axle back to where it was when the joint was added. Without a spring, the
    /// axle slides freely along the axis.
    pub spring: Option<Spring>,
    /// Allowed range of the suspension travel along the axis, in meters.
    pub limits: Option<RangeInclusive<f32>>,
    /// Drives the wheel's rotation.
    pub motor: Option<Motor>,
    /// Whether the two bodies still collide with each other.
    pub collide_connected: bool,
}

impl WheelJointDesc {
    /// Describes a wheel turning about the world point `anchor`, which slides along the world
    /// direction `axis`. The suspension is at rest in the placement the bodies have when the
    /// joint is added.
    ///
    /// # Panics
    ///
    /// Panics if `axis` is zero.
    #[must_use]
    pub fn new(body_a: BodyHandle, body_b: BodyHandle, anchor: Vec2, axis: Vec2) -> Self {
        Self {
            body_a,
            body_b,
            anchor,
            axis: axis.try_normalize().expect("the suspension axis must not be zero"),
            spring: None,
            limits: None,
            motor: None,
            collide_connected: false,
        }
    }

    /// Holds the axle in place with a spring of the given frequency (Hz) and damping ratio.
    #[must_use]
    pub fn spring(mut self, hertz: f32, damping_ratio: f32) -> Self {
        self.spring = Some(Spring { hertz, damping_ratio });
        self
    }

    /// Restricts the suspension travel to `translations`, in meters along the axis.
    ///
    /// # Panics
    ///
    /// Panics when the joint is added if the range is empty.
    #[must_use]
    pub fn limits(mut self, translations: RangeInclusive<f32>) -> Self {
        self.limits = Some(translations);
        self
    }

    /// Turns the wheel at `speed` rad/s, with a torque of at most `max_torque` N·m.
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

/// A wheel on a suspension: body B turns freely about an axle that slides along an axis fixed
/// in body A, pushed back by a spring, optionally within limits and driven by a motor.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct WheelJoint {
    pub(super) base: JointBase,
    local_anchor_a: Vec2,
    local_anchor_b: Vec2,
    local_axis_a: Vec2,
    spring: Option<Spring>,
    limits: Option<(f32, f32)>,
    motor: Option<Motor>,

    /// Accumulated impulses.
    perp_impulse: f32,
    spring_impulse: f32,
    motor_impulse: f32,
    lower_impulse: f32,
    upper_impulse: f32,
    /// World axis at the start of the step, and the effective masses along it, across it and
    /// around the axle.
    axis: Vec2,
    axial_mass: f32,
    perp_mass: f32,
    angular_mass: f32,
    spring_softness: Softness,
}

impl WheelJoint {
    pub(super) fn new(desc: &WheelJointDesc, a: &Body, b: &Body) -> Self {
        Self {
            base: JointBase::new(desc.body_a, desc.body_b, desc.collide_connected),
            local_anchor_a: a.local_point(desc.anchor),
            local_anchor_b: b.local_point(desc.anchor),
            local_axis_a: a.local_vector(desc.axis),
            spring: desc.spring,
            limits: desc.limits.clone().map(validated_range),
            motor: desc.motor,
            perp_impulse: 0.0,
            spring_impulse: 0.0,
            motor_impulse: 0.0,
            lower_impulse: 0.0,
            upper_impulse: 0.0,
            axis: Vec2::Y,
            axial_mass: 0.0,
            perp_mass: 0.0,
            angular_mass: 0.0,
            spring_softness: Softness::RIGID,
        }
    }

    /// The axle in the local frame of body A.
    #[must_use]
    pub fn local_anchor_a(&self) -> Vec2 {
        self.local_anchor_a
    }

    /// The axle in the local frame of body B.
    #[must_use]
    pub fn local_anchor_b(&self) -> Vec2 {
        self.local_anchor_b
    }

    /// Suspension direction in the local frame of body A.
    #[must_use]
    pub fn local_axis_a(&self) -> Vec2 {
        self.local_axis_a
    }

    /// The suspension spring, if any.
    #[must_use]
    pub fn spring(&self) -> Option<Spring> {
        self.spring
    }

    /// Sets or removes the suspension spring.
    pub fn set_spring(&mut self, spring: Option<Spring>) {
        self.spring = spring;
        if spring.is_none() {
            self.spring_impulse = 0.0;
        }
    }

    /// Allowed range of the suspension travel, if limited.
    #[must_use]
    pub fn limits(&self) -> Option<RangeInclusive<f32>> {
        self.limits.map(|(lower, upper)| lower..=upper)
    }

    /// Sets or removes the suspension limits.
    ///
    /// # Panics
    ///
    /// Panics if the range is empty.
    pub fn set_limits(&mut self, translations: Option<RangeInclusive<f32>>) {
        self.limits = translations.map(validated_range);
        self.lower_impulse = 0.0;
        self.upper_impulse = 0.0;
    }

    /// The motor, if any.
    #[must_use]
    pub fn motor(&self) -> Option<Motor> {
        self.motor
    }

    /// Sets or removes the motor; `max_force` is a torque, in N·m.
    pub fn set_motor(&mut self, motor: Option<Motor>) {
        self.motor = motor;
        if motor.is_none() {
            self.motor_impulse = 0.0;
        }
    }

    /// Uses the axis at the start of the step, close enough for reporting.
    pub(crate) fn reaction_impulse(&self) -> (Vec2, f32) {
        let axial = self.spring_impulse + self.lower_impulse - self.upper_impulse;
        (axial * self.axis + self.perp_impulse * self.axis.perp(), self.motor_impulse)
    }

    pub(crate) fn prepare(&mut self, frame: &JointFrame, a: &Body, context: &SolverContext) {
        self.axis = a.world_vector(self.local_axis_a);
        self.axial_mass = frame.mass_along(self.axis);
        self.perp_mass = frame.mass_along(self.axis.perp());
        self.angular_mass = frame.axial_mass();
        if let Some(spring) = self.spring {
            self.spring_softness = Softness::new(spring.hertz, spring.damping_ratio, context.substep);
        }
    }

    pub(crate) fn warm_start(&self, frame: &JointFrame, a: &mut Body, b: &mut Body) {
        let geometry = SlideGeometry::new(self.axis, frame, a, b);
        let axial = self.spring_impulse + self.lower_impulse - self.upper_impulse;
        let linear = axial * geometry.axis + self.perp_impulse * geometry.perp;
        let angular_a = axial * geometry.axial_arm_a + self.perp_impulse * geometry.perp_arm_a + self.motor_impulse;
        let angular_b = axial * geometry.axial_arm_b + self.perp_impulse * geometry.perp_arm_b + self.motor_impulse;
        apply_impulses(a, b, linear, angular_a, angular_b);
    }

    pub(crate) fn solve(
        &mut self,
        frame: &JointFrame,
        a: &mut Body,
        b: &mut Body,
        context: &SolverContext,
        pass: Pass,
    ) {
        if let Some(motor) = self.motor {
            let speed = b.angular_velocity() - a.angular_velocity();
            let impulse = motor_impulse(motor, &mut self.motor_impulse, self.angular_mass, speed, context.substep);
            apply_impulses(a, b, Vec2::ZERO, impulse, impulse);
        }

        let geometry = SlideGeometry::new(self.axis, frame, a, b);
        // The spring is part of the physics, not an error to correct: it acts in both passes.
        if self.spring.is_some_and(|spring| spring.hertz > 0.0) {
            let speed = geometry.axial_speed(a, b);
            let impulse =
                self.spring_softness.impulse(self.axial_mass, speed, geometry.translation, self.spring_impulse);
            self.spring_impulse += impulse;
            geometry.apply_axial(a, b, impulse);
        }
        if let Some(limits) = self.limits {
            let impulses = (&mut self.lower_impulse, &mut self.upper_impulse);
            solve_slide_limits(&geometry, limits, impulses, self.axial_mass, a, b, context, pass);
        }

        // The axle stays on the axis.
        let speed = geometry.perp_speed(a, b);
        let error = geometry.perp.dot(geometry.separation);
        let impulse = context.equality_softness(pass).impulse(self.perp_mass, speed, error, self.perp_impulse);
        self.perp_impulse += impulse;
        geometry.apply_perp(a, b, impulse);
    }
}
