//! Distance joint: a rod, a spring or a rope between two anchors.

use std::ops::RangeInclusive;

use super::frame::{JointFrame, apply_linear_impulse, inverse_or_zero, limit_impulse, relative_velocity};
use super::{JointBase, Spring, validated_range};
use crate::body::{Body, BodyHandle};
use crate::math::Vec2;
use crate::solver::{Pass, Softness, SolverContext};
use kinetica_collision::LINEAR_SLOP;

/// Description of a [`DistanceJoint`].
///
/// Without a spring the joint is a rigid rod. With a spring, the distance oscillates around
/// `length`, and stays within `limits` if set. A rope is slack up to its length and taut
/// beyond:
///
/// ```
/// use kinetica::prelude::*;
///
/// let mut world = World::new();
/// let ceiling = world.add_body(BodyDesc::fixed().position(vec2(0.0, 5.0)));
/// let lamp = world.add_body(BodyDesc::dynamic().position(vec2(0.0, 3.0)).collider(ColliderDesc::circle(0.3)));
///
/// world.add_joint(DistanceJointDesc::new(ceiling, lamp, vec2(0.0, 5.0), vec2(0.0, 3.0)).rope(2.0));
/// ```
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct DistanceJointDesc {
    /// First body.
    pub body_a: BodyHandle,
    /// Second body.
    pub body_b: BodyHandle,
    /// Attachment point on body A, in world space.
    pub anchor_a: Vec2,
    /// Attachment point on body B, in world space.
    pub anchor_b: Vec2,
    /// Rest length; `None` uses the distance between the anchors when the joint is added.
    pub length: Option<f32>,
    /// Makes the joint elastic instead of rigid.
    pub spring: Option<Spring>,
    /// Allowed range of distances, only used with a spring.
    pub limits: Option<RangeInclusive<f32>>,
    /// Whether the two bodies still collide with each other.
    pub collide_connected: bool,
}

impl DistanceJointDesc {
    /// Describes a rigid rod between two world points, keeping their current distance.
    #[must_use]
    pub fn new(body_a: BodyHandle, body_b: BodyHandle, anchor_a: Vec2, anchor_b: Vec2) -> Self {
        Self { body_a, body_b, anchor_a, anchor_b, length: None, spring: None, limits: None, collide_connected: false }
    }

    /// Sets the rest length.
    #[must_use]
    pub fn length(mut self, length: f32) -> Self {
        self.length = Some(length);
        self
    }

    /// Makes the joint a spring of the given frequency (Hz) and damping ratio.
    /// A frequency of zero exerts no force.
    #[must_use]
    pub fn spring(mut self, hertz: f32, damping_ratio: f32) -> Self {
        self.spring = Some(Spring { hertz, damping_ratio });
        self
    }

    /// Restricts the distance to `lengths`; only used with a spring.
    ///
    /// # Panics
    ///
    /// Panics when the joint is added if the range is empty.
    #[must_use]
    pub fn limits(mut self, lengths: RangeInclusive<f32>) -> Self {
        self.limits = Some(lengths);
        self
    }

    /// Makes the joint a rope: no force while the anchors are closer than `max_length`, and
    /// they cannot get further apart.
    #[must_use]
    pub fn rope(self, max_length: f32) -> Self {
        self.spring(0.0, 0.0).limits(0.0..=max_length)
    }

    /// Lets the two bodies keep colliding with each other.
    #[must_use]
    pub fn collide_connected(mut self) -> Self {
        self.collide_connected = true;
        self
    }
}

/// Keeps two anchors at a given distance, rigidly or elastically.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct DistanceJoint {
    pub(super) base: JointBase,
    local_anchor_a: Vec2,
    local_anchor_b: Vec2,
    length: f32,
    spring: Option<Spring>,
    limits: Option<(f32, f32)>,

    impulse: f32,
    lower_impulse: f32,
    upper_impulse: f32,
    /// Direction from anchor A to anchor B at the last solve.
    axis: Vec2,
    axial_mass: f32,
    spring_softness: Softness,
}

impl DistanceJoint {
    pub(super) fn new(desc: &DistanceJointDesc, a: &Body, b: &Body) -> Self {
        let (local_anchor_a, local_anchor_b) = (a.local_point(desc.anchor_a), b.local_point(desc.anchor_b));
        let current_length = a.world_point(local_anchor_a).distance(b.world_point(local_anchor_b));
        Self {
            base: JointBase::new(desc.body_a, desc.body_b, desc.collide_connected),
            local_anchor_a,
            local_anchor_b,
            length: desc.length.unwrap_or(current_length).max(LINEAR_SLOP),
            spring: desc.spring,
            limits: desc.limits.clone().map(validated_range),
            impulse: 0.0,
            lower_impulse: 0.0,
            upper_impulse: 0.0,
            axis: Vec2::ZERO,
            axial_mass: 0.0,
            spring_softness: Softness::new(0.0, 0.0, 1.0),
        }
    }

    /// Attachment point in the local frame of body A.
    #[must_use]
    pub fn local_anchor_a(&self) -> Vec2 {
        self.local_anchor_a
    }

    /// Attachment point in the local frame of body B.
    #[must_use]
    pub fn local_anchor_b(&self) -> Vec2 {
        self.local_anchor_b
    }

    /// Rest length.
    #[must_use]
    pub fn length(&self) -> f32 {
        self.length
    }

    /// Sets the rest length.
    pub fn set_length(&mut self, length: f32) {
        self.length = length.max(LINEAR_SLOP);
    }

    /// The spring, if the joint is elastic.
    #[must_use]
    pub fn spring(&self) -> Option<Spring> {
        self.spring
    }

    /// Makes the joint elastic, or rigid with `None`.
    pub fn set_spring(&mut self, spring: Option<Spring>) {
        self.spring = spring;
        self.impulse = 0.0;
    }

    pub(crate) fn reaction_impulse(&self) -> (Vec2, f32) {
        ((self.impulse + self.lower_impulse - self.upper_impulse) * self.axis, 0.0)
    }

    pub(crate) fn prepare(&mut self, frame: &JointFrame, context: &SolverContext) {
        let axis = (frame.delta_center + frame.anchor_b - frame.anchor_a).normalize_or_zero();
        let (arm_a, arm_b) = (frame.anchor_a.perp_dot(axis), frame.anchor_b.perp_dot(axis));
        self.axial_mass = inverse_or_zero(
            frame.mass_a + frame.mass_b + frame.inertia_a * arm_a * arm_a + frame.inertia_b * arm_b * arm_b,
        );
        if let Some(spring) = self.spring {
            self.spring_softness = Softness::new(spring.hertz, spring.damping_ratio, context.substep);
        }
        if self.spring.is_none() || self.limits.is_none() {
            self.lower_impulse = 0.0;
            self.upper_impulse = 0.0;
        }
    }

    pub(crate) fn warm_start(&self, frame: &JointFrame, a: &mut Body, b: &mut Body) {
        let (anchor_a, anchor_b) = frame.current_anchors(a, b);
        let axis = frame.separation(a, b, anchor_a, anchor_b).normalize_or_zero();
        let impulse = self.impulse + self.lower_impulse - self.upper_impulse;
        apply_linear_impulse(a, b, impulse * axis, anchor_a, anchor_b);
    }

    pub(crate) fn solve(
        &mut self,
        frame: &JointFrame,
        a: &mut Body,
        b: &mut Body,
        context: &SolverContext,
        pass: Pass,
    ) {
        let (anchor_a, anchor_b) = frame.current_anchors(a, b);
        let separation = frame.separation(a, b, anchor_a, anchor_b);
        let length = separation.length();
        let axis = separation.normalize_or_zero();
        self.axis = axis;
        let axial_speed = |a: &Body, b: &Body| relative_velocity(a, b, anchor_a, anchor_b).dot(axis);

        // The rest length: rigid for a rod, elastic for a spring, absent for a rope.
        let softness = match self.spring {
            None => Some(context.equality_softness(pass)),
            Some(spring) if spring.hertz > 0.0 => Some(self.spring_softness),
            Some(_) => None,
        };
        if let Some(softness) = softness {
            let impulse = softness.impulse(self.axial_mass, axial_speed(a, b), length - self.length, self.impulse);
            self.impulse += impulse;
            apply_linear_impulse(a, b, impulse * axis, anchor_a, anchor_b);
        }

        // Limits only apply to springs and ropes: a rod already keeps its length.
        if let (Some(_), Some((min, max))) = (self.spring, self.limits) {
            // Shortest allowed distance: pushes the anchors apart.
            let gap = length - min;
            let softness = context.limit_softness(gap, pass);
            let impulse = limit_impulse(&mut self.lower_impulse, self.axial_mass, axial_speed(a, b), gap, softness);
            apply_linear_impulse(a, b, impulse * axis, anchor_a, anchor_b);

            // Longest allowed distance: pulls them together.
            let gap = max - length;
            let softness = context.limit_softness(gap, pass);
            let impulse = limit_impulse(&mut self.upper_impulse, self.axial_mass, -axial_speed(a, b), gap, softness);
            apply_linear_impulse(a, b, -impulse * axis, anchor_a, anchor_b);
        }
    }
}
