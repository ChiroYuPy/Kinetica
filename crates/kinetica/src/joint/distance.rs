//! Distance joint: a rod, a spring or a rope between two anchors.

use std::ops::RangeInclusive;

use super::frame::{BiasTerms, JointFrame, apply_linear_impulse, inverse_or_zero, relative_velocity};
use super::{Spring, validated_range};
use crate::body::{Body, BodyHandle};
use crate::constants::LINEAR_SLOP;
use crate::math::Vec2;
use crate::solver::{Softness, SolverContext};
use crate::world::World;

/// Description of a [`DistanceJoint`].
///
/// Without a spring the joint is a rigid rod. With a spring, the distance oscillates around
/// `length`, and stays within `limits` if set. A rope is a spring of zero stiffness limited to
/// `0..=length`:
///
/// ```
/// use kinetica::prelude::*;
///
/// let mut world = World::new();
/// let ceiling = world.add_body(BodyDesc::fixed().position(vec2(0.0, 5.0)));
/// let lamp = world.add_body(BodyDesc::dynamic().position(vec2(0.0, 3.0)));
/// world.add_collider(lamp, ColliderDesc::circle(0.3));
///
/// let rope = DistanceJointDesc::between_world_points(&world, ceiling, lamp, vec2(0.0, 5.0), vec2(0.0, 3.0))
///     .spring(0.0, 0.0)
///     .limits(0.0..=2.0);
/// world.add_joint(rope);
/// ```
#[derive(Debug, Clone, PartialEq)]
pub struct DistanceJointDesc {
    /// First body.
    pub body_a: BodyHandle,
    /// Second body.
    pub body_b: BodyHandle,
    /// Attachment point, in the local frame of body A.
    pub local_anchor_a: Vec2,
    /// Attachment point, in the local frame of body B.
    pub local_anchor_b: Vec2,
    /// Rest length; `None` uses the distance between the anchors when the joint is created.
    pub length: Option<f32>,
    /// Makes the joint elastic instead of rigid.
    pub spring: Option<Spring>,
    /// Allowed range of distances, only used with a spring.
    pub limits: Option<RangeInclusive<f32>>,
    /// Whether the two bodies still collide with each other.
    pub collide_connected: bool,
}

impl DistanceJointDesc {
    /// Describes a joint between anchors given in each body's local frame.
    #[must_use]
    pub fn new(body_a: BodyHandle, body_b: BodyHandle, local_anchor_a: Vec2, local_anchor_b: Vec2) -> Self {
        Self {
            body_a,
            body_b,
            local_anchor_a,
            local_anchor_b,
            length: None,
            spring: None,
            limits: None,
            collide_connected: false,
        }
    }

    /// Describes a joint between two world points, keeping their current distance.
    ///
    /// # Panics
    ///
    /// Panics if either body was removed from `world`.
    #[must_use]
    pub fn between_world_points(
        world: &World,
        body_a: BodyHandle,
        body_b: BodyHandle,
        point_a: Vec2,
        point_b: Vec2,
    ) -> Self {
        let (a, b) = super::bodies_of(world, body_a, body_b);
        Self::new(body_a, body_b, a.local_point(point_a), b.local_point(point_b))
    }

    /// Sets the rest length.
    #[must_use]
    pub fn length(mut self, length: f32) -> Self {
        self.length = Some(length);
        self
    }

    /// Makes the joint a spring of the given frequency (Hz) and damping ratio.
    /// A frequency of zero exerts no force, which is useful for ropes.
    #[must_use]
    pub fn spring(mut self, hertz: f32, damping_ratio: f32) -> Self {
        self.spring = Some(Spring { hertz, damping_ratio });
        self
    }

    /// Restricts the distance to `lengths`; only used with a spring.
    #[must_use]
    pub fn limits(mut self, lengths: RangeInclusive<f32>) -> Self {
        self.limits = Some(lengths);
        self
    }

    /// Sets whether the two bodies still collide with each other.
    #[must_use]
    pub fn collide_connected(mut self, collide: bool) -> Self {
        self.collide_connected = collide;
        self
    }
}

/// Keeps two anchors at a given distance, rigidly or elastically.
#[derive(Debug, Clone)]
pub struct DistanceJoint {
    local_anchor_a: Vec2,
    local_anchor_b: Vec2,
    length: f32,
    spring: Option<Spring>,
    limits: Option<(f32, f32)>,

    impulse: f32,
    lower_impulse: f32,
    upper_impulse: f32,
    axial_mass: f32,
    spring_softness: Softness,
}

impl DistanceJoint {
    pub(crate) fn new(desc: &DistanceJointDesc, a: &Body, b: &Body) -> Self {
        let current_length = a.world_point(desc.local_anchor_a).distance(b.world_point(desc.local_anchor_b));
        Self {
            local_anchor_a: desc.local_anchor_a,
            local_anchor_b: desc.local_anchor_b,
            length: desc.length.unwrap_or(current_length).max(LINEAR_SLOP),
            spring: desc.spring,
            limits: desc.limits.clone().map(validated_range),
            impulse: 0.0,
            lower_impulse: 0.0,
            upper_impulse: 0.0,
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
        use_bias: bool,
    ) {
        let (anchor_a, anchor_b) = frame.current_anchors(a, b);
        let separation = frame.separation(a, b, anchor_a, anchor_b);
        let length = separation.length();
        let axis = separation.normalize_or_zero();
        let axial_speed = |a: &Body, b: &Body| relative_velocity(a, b, anchor_a, anchor_b).dot(axis);

        let Some(spring) = self.spring else {
            // Rigid rod.
            let terms = BiasTerms::equality(length - self.length, context, use_bias);
            let impulse = -terms.mass_scale * self.axial_mass * (axial_speed(a, b) + terms.bias)
                - terms.impulse_scale * self.impulse;
            self.impulse += impulse;
            apply_linear_impulse(a, b, impulse * axis, anchor_a, anchor_b);
            return;
        };

        if spring.hertz > 0.0 {
            let terms = BiasTerms::soft(self.spring_softness, length - self.length);
            let impulse = -terms.mass_scale * self.axial_mass * (axial_speed(a, b) + terms.bias)
                - terms.impulse_scale * self.impulse;
            self.impulse += impulse;
            apply_linear_impulse(a, b, impulse * axis, anchor_a, anchor_b);
        }

        if let Some((min, max)) = self.limits {
            // Shortest allowed distance: pushes the anchors apart.
            let terms = BiasTerms::limit(length - min, context, use_bias);
            let impulse = -terms.mass_scale * self.axial_mass * (axial_speed(a, b) + terms.bias)
                - terms.impulse_scale * self.lower_impulse;
            let previous = self.lower_impulse;
            self.lower_impulse = (previous + impulse).max(0.0);
            apply_linear_impulse(a, b, (self.lower_impulse - previous) * axis, anchor_a, anchor_b);

            // Longest allowed distance: pulls them together.
            let terms = BiasTerms::limit(max - length, context, use_bias);
            let impulse = -terms.mass_scale * self.axial_mass * (-axial_speed(a, b) + terms.bias)
                - terms.impulse_scale * self.upper_impulse;
            let previous = self.upper_impulse;
            self.upper_impulse = (previous + impulse).max(0.0);
            apply_linear_impulse(a, b, (previous - self.upper_impulse) * axis, anchor_a, anchor_b);
        }
    }
}
