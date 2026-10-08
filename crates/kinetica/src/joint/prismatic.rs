//! Prismatic joint: a slider.

use std::ops::RangeInclusive;

use super::frame::{
    BiasTerms, JointFrame, SymmetricMatrix, apply_impulses, inverse_or_zero, relative_angle, wrap_angle,
};
use super::{Motor, validated_range};
use crate::body::{Body, BodyHandle};
use crate::math::{Vec2, vec2};
use crate::solver::SolverContext;
use crate::world::World;

/// Description of a [`PrismaticJoint`].
#[derive(Debug, Clone, PartialEq)]
pub struct PrismaticJointDesc {
    /// First body.
    pub body_a: BodyHandle,
    /// Second body.
    pub body_b: BodyHandle,
    /// Origin of the slide, in the local frame of body A.
    pub local_anchor_a: Vec2,
    /// Point of body B sliding along the axis, in its local frame.
    pub local_anchor_b: Vec2,
    /// Direction of the slide, in the local frame of body A.
    pub local_axis_a: Vec2,
    /// Angle of B relative to A, kept constant by the joint.
    pub reference_angle: f32,
    /// Allowed range of the translation along the axis, in meters.
    pub limits: Option<RangeInclusive<f32>>,
    /// Drives the translation.
    pub motor: Option<Motor>,
    /// Whether the two bodies still collide with each other.
    pub collide_connected: bool,
}

impl PrismaticJointDesc {
    /// Describes a slider given in each body's local frame.
    ///
    /// # Panics
    ///
    /// Panics if `local_axis_a` is zero.
    #[must_use]
    pub fn new(
        body_a: BodyHandle,
        body_b: BodyHandle,
        local_anchor_a: Vec2,
        local_anchor_b: Vec2,
        local_axis_a: Vec2,
    ) -> Self {
        Self {
            body_a,
            body_b,
            local_anchor_a,
            local_anchor_b,
            local_axis_a: local_axis_a.try_normalize().expect("the slide axis must not be zero"),
            reference_angle: 0.0,
            limits: None,
            motor: None,
            collide_connected: false,
        }
    }

    /// Describes a slider along `axis` (world direction) through a world point, starting at
    /// translation zero.
    ///
    /// # Panics
    ///
    /// Panics if either body was removed from `world`, or if `axis` is zero.
    #[must_use]
    pub fn at_world_point(world: &World, body_a: BodyHandle, body_b: BodyHandle, point: Vec2, axis: Vec2) -> Self {
        let (a, b) = super::bodies_of(world, body_a, body_b);
        Self {
            reference_angle: relative_angle(b.rotation(), a.rotation()),
            ..Self::new(body_a, body_b, a.local_point(point), b.local_point(point), a.local_vector(axis))
        }
    }

    /// Restricts the translation to `translations`, in meters.
    #[must_use]
    pub fn limits(mut self, translations: RangeInclusive<f32>) -> Self {
        self.limits = Some(translations);
        self
    }

    /// Drives the joint at `speed` m/s, with a force of at most `max_force` N.
    #[must_use]
    pub fn motor(mut self, speed: f32, max_force: f32) -> Self {
        self.motor = Some(Motor { speed, max_force });
        self
    }

    /// Sets whether the two bodies still collide with each other.
    #[must_use]
    pub fn collide_connected(mut self, collide: bool) -> Self {
        self.collide_connected = collide;
        self
    }
}

/// A slider: body B translates along an axis fixed in body A, without rotating relative to it.
#[derive(Debug, Clone)]
pub struct PrismaticJoint {
    local_anchor_a: Vec2,
    local_anchor_b: Vec2,
    local_axis_a: Vec2,
    reference_angle: f32,
    limits: Option<(f32, f32)>,
    motor: Option<Motor>,

    /// Accumulated impulses across the axis and around the joint.
    impulse: Vec2,
    motor_impulse: f32,
    lower_impulse: f32,
    upper_impulse: f32,
    /// World axis and joint angle at the start of the step.
    axis: Vec2,
    delta_angle: f32,
    axial_mass: f32,
}

impl PrismaticJoint {
    pub(crate) fn new(desc: &PrismaticJointDesc) -> Self {
        Self {
            local_anchor_a: desc.local_anchor_a,
            local_anchor_b: desc.local_anchor_b,
            local_axis_a: desc.local_axis_a,
            reference_angle: desc.reference_angle,
            limits: desc.limits.clone().map(validated_range),
            motor: desc.motor,
            impulse: Vec2::ZERO,
            motor_impulse: 0.0,
            lower_impulse: 0.0,
            upper_impulse: 0.0,
            axis: Vec2::X,
            delta_angle: 0.0,
            axial_mass: 0.0,
        }
    }

    /// Origin of the slide in the local frame of body A.
    #[must_use]
    pub fn local_anchor_a(&self) -> Vec2 {
        self.local_anchor_a
    }

    /// Sliding point in the local frame of body B.
    #[must_use]
    pub fn local_anchor_b(&self) -> Vec2 {
        self.local_anchor_b
    }

    /// Slide direction in the local frame of body A.
    #[must_use]
    pub fn local_axis_a(&self) -> Vec2 {
        self.local_axis_a
    }

    /// Allowed range of the translation, if limited.
    #[must_use]
    pub fn limits(&self) -> Option<RangeInclusive<f32>> {
        self.limits.map(|(lower, upper)| lower..=upper)
    }

    /// Sets or removes the translation limits.
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

    /// Sets or removes the motor; `max_force` is in N.
    pub fn set_motor(&mut self, motor: Option<Motor>) {
        self.motor = motor;
        if motor.is_none() {
            self.motor_impulse = 0.0;
        }
    }

    /// Uses the axis at the start of the step, close enough for reporting.
    pub(crate) fn reaction_impulse(&self) -> (Vec2, f32) {
        let axial = self.motor_impulse + self.lower_impulse - self.upper_impulse;
        (axial * self.axis + self.impulse.x * self.axis.perp(), self.impulse.y)
    }

    pub(crate) fn prepare(&mut self, frame: &JointFrame, a: &Body, b: &Body) {
        self.axis = a.world_vector(self.local_axis_a);
        self.delta_angle = wrap_angle(relative_angle(b.rotation(), a.rotation()) - self.reference_angle);
        let separation = frame.delta_center + frame.anchor_b - frame.anchor_a;
        let arm_a = (separation + frame.anchor_a).perp_dot(self.axis);
        let arm_b = frame.anchor_b.perp_dot(self.axis);
        self.axial_mass = inverse_or_zero(
            frame.mass_a + frame.mass_b + frame.inertia_a * arm_a * arm_a + frame.inertia_b * arm_b * arm_b,
        );
    }

    pub(crate) fn warm_start(&self, frame: &JointFrame, a: &mut Body, b: &mut Body) {
        let geometry = Geometry::new(self, frame, a, b);
        let axial = self.motor_impulse + self.lower_impulse - self.upper_impulse;
        let linear = axial * geometry.axis + self.impulse.x * geometry.perp;
        let angular_a = axial * geometry.axial_arm_a + self.impulse.x * geometry.perp_arm_a + self.impulse.y;
        let angular_b = axial * geometry.axial_arm_b + self.impulse.x * geometry.perp_arm_b + self.impulse.y;
        apply_impulses(a, b, linear, angular_a, angular_b);
    }

    pub(crate) fn solve(
        &mut self,
        frame: &JointFrame,
        a: &mut Body,
        b: &mut Body,
        context: &SolverContext,
        use_bias: bool,
    ) {
        let geometry = Geometry::new(self, frame, a, b);
        let apply_axial = |a: &mut Body, b: &mut Body, impulse: f32| {
            apply_impulses(
                a,
                b,
                impulse * geometry.axis,
                impulse * geometry.axial_arm_a,
                impulse * geometry.axial_arm_b,
            );
        };

        if let Some(motor) = self.motor {
            let speed_error = geometry.axial_speed(a, b) - motor.speed;
            let max_impulse = motor.max_force * context.substep;
            let previous = self.motor_impulse;
            self.motor_impulse = (previous - self.axial_mass * speed_error).clamp(-max_impulse, max_impulse);
            apply_axial(a, b, self.motor_impulse - previous);
        }

        if let Some((lower, upper)) = self.limits {
            let terms = BiasTerms::limit(geometry.translation - lower, context, use_bias);
            let impulse = -self.axial_mass * terms.mass_scale * (geometry.axial_speed(a, b) + terms.bias)
                - terms.impulse_scale * self.lower_impulse;
            let previous = self.lower_impulse;
            self.lower_impulse = (previous + impulse).max(0.0);
            apply_axial(a, b, self.lower_impulse - previous);

            let terms = BiasTerms::limit(upper - geometry.translation, context, use_bias);
            let impulse = -self.axial_mass * terms.mass_scale * (-geometry.axial_speed(a, b) + terms.bias)
                - terms.impulse_scale * self.upper_impulse;
            let previous = self.upper_impulse;
            self.upper_impulse = (previous + impulse).max(0.0);
            apply_axial(a, b, previous - self.upper_impulse);
        }

        // Across the axis and around the joint: solved together as a 2×2 block.
        let velocity_error = vec2(
            geometry.perp.dot(b.linear_velocity() - a.linear_velocity()) + geometry.perp_arm_b * b.angular_velocity()
                - geometry.perp_arm_a * a.angular_velocity(),
            b.angular_velocity() - a.angular_velocity(),
        );
        let (bias, mass_scale, impulse_scale) = if use_bias {
            let angle = self.delta_angle + relative_angle(b.delta_rotation(), a.delta_rotation());
            let error = vec2(geometry.perp.dot(geometry.separation), angle);
            let softness = context.joint_softness;
            (softness.bias_rate * error, softness.mass_scale, softness.impulse_scale)
        } else {
            (Vec2::ZERO, 1.0, 0.0)
        };
        let (ma, mb, ia, ib) = (frame.mass_a, frame.mass_b, frame.inertia_a, frame.inertia_b);
        let (s1, s2) = (geometry.perp_arm_a, geometry.perp_arm_b);
        let rotational = ia + ib;
        let mass = SymmetricMatrix {
            xx: ma + mb + ia * s1 * s1 + ib * s2 * s2,
            xy: ia * s1 + ib * s2,
            // Two bodies that cannot rotate: the angular row is irrelevant but must stay invertible.
            yy: if rotational == 0.0 { 1.0 } else { rotational },
        };
        let impulse = -mass_scale * mass.solve(velocity_error + bias) - impulse_scale * self.impulse;
        self.impulse += impulse;
        apply_impulses(a, b, impulse.x * geometry.perp, impulse.x * s1 + impulse.y, impulse.x * s2 + impulse.y);
    }
}

/// The joint's current axis and lever arms.
struct Geometry {
    axis: Vec2,
    perp: Vec2,
    separation: Vec2,
    translation: f32,
    axial_arm_a: f32,
    axial_arm_b: f32,
    perp_arm_a: f32,
    perp_arm_b: f32,
}

impl Geometry {
    fn new(joint: &PrismaticJoint, frame: &JointFrame, a: &Body, b: &Body) -> Self {
        let (anchor_a, anchor_b) = frame.current_anchors(a, b);
        let separation = frame.separation(a, b, anchor_a, anchor_b);
        let axis = a.delta_rotation().rotate(joint.axis);
        let perp = axis.perp();
        Self {
            axis,
            perp,
            separation,
            translation: axis.dot(separation),
            axial_arm_a: (separation + anchor_a).perp_dot(axis),
            axial_arm_b: anchor_b.perp_dot(axis),
            perp_arm_a: (separation + anchor_a).perp_dot(perp),
            perp_arm_b: anchor_b.perp_dot(perp),
        }
    }

    /// Relative speed of B along the axis.
    fn axial_speed(&self, a: &Body, b: &Body) -> f32 {
        self.axis.dot(b.linear_velocity() - a.linear_velocity()) + self.axial_arm_b * b.angular_velocity()
            - self.axial_arm_a * a.angular_velocity()
    }
}
