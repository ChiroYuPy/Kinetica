//! Weld joint: glues two bodies together, rigidly or elastically.

use super::frame::{
    BiasTerms, JointFrame, apply_angular_impulse, apply_linear_impulse, relative_angle, relative_velocity, wrap_angle,
};
use super::{JointBase, Spring};
use crate::body::{Body, BodyHandle};
use crate::math::Vec2;
use crate::solver::{Softness, SolverContext};

/// Description of a [`WeldJoint`].
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct WeldJointDesc {
    /// First body.
    pub body_a: BodyHandle,
    /// Second body.
    pub body_b: BodyHandle,
    /// Weld point, in world space.
    pub anchor: Vec2,
    /// Makes the weld point elastic instead of rigid.
    pub linear_spring: Option<Spring>,
    /// Makes the relative angle elastic instead of rigid.
    pub angular_spring: Option<Spring>,
    /// Whether the two bodies still collide with each other.
    pub collide_connected: bool,
}

impl WeldJointDesc {
    /// Describes a weld at the world point `anchor`, keeping the relative placement the bodies
    /// have when the joint is added.
    #[must_use]
    pub fn new(body_a: BodyHandle, body_b: BodyHandle, anchor: Vec2) -> Self {
        Self { body_a, body_b, anchor, linear_spring: None, angular_spring: None, collide_connected: false }
    }

    /// Makes the weld point elastic.
    #[must_use]
    pub fn linear_spring(mut self, hertz: f32, damping_ratio: f32) -> Self {
        self.linear_spring = Some(Spring { hertz, damping_ratio });
        self
    }

    /// Makes the relative angle elastic, like a bendable beam.
    #[must_use]
    pub fn angular_spring(mut self, hertz: f32, damping_ratio: f32) -> Self {
        self.angular_spring = Some(Spring { hertz, damping_ratio });
        self
    }

    /// Lets the two bodies keep colliding with each other.
    #[must_use]
    pub fn collide_connected(mut self) -> Self {
        self.collide_connected = true;
        self
    }
}

/// Keeps two bodies in the same relative placement.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct WeldJoint {
    pub(super) base: JointBase,
    local_anchor_a: Vec2,
    local_anchor_b: Vec2,
    reference_angle: f32,
    linear_spring: Option<Spring>,
    angular_spring: Option<Spring>,

    linear_impulse: Vec2,
    angular_impulse: f32,
    delta_angle: f32,
    axial_mass: f32,
    linear_softness: Option<Softness>,
    angular_softness: Option<Softness>,
}

impl WeldJoint {
    shared_joint_methods!();

    fn base(&self) -> &JointBase {
        &self.base
    }
    pub(super) fn new(desc: &WeldJointDesc, a: &Body, b: &Body) -> Self {
        Self {
            base: JointBase::new(desc.body_a, desc.body_b, desc.collide_connected),
            local_anchor_a: a.local_point(desc.anchor),
            local_anchor_b: b.local_point(desc.anchor),
            reference_angle: relative_angle(b.rotation(), a.rotation()),
            linear_spring: desc.linear_spring,
            angular_spring: desc.angular_spring,
            linear_impulse: Vec2::ZERO,
            angular_impulse: 0.0,
            delta_angle: 0.0,
            axial_mass: 0.0,
            linear_softness: None,
            angular_softness: None,
        }
    }

    /// Weld point in the local frame of body A.
    #[must_use]
    pub fn local_anchor_a(&self) -> Vec2 {
        self.local_anchor_a
    }

    /// Weld point in the local frame of body B.
    #[must_use]
    pub fn local_anchor_b(&self) -> Vec2 {
        self.local_anchor_b
    }

    pub(crate) fn reaction_impulse(&self) -> (Vec2, f32) {
        (self.linear_impulse, self.angular_impulse)
    }

    pub(crate) fn prepare(&mut self, frame: &JointFrame, a: &Body, b: &Body, context: &SolverContext) {
        self.delta_angle = wrap_angle(relative_angle(b.rotation(), a.rotation()) - self.reference_angle);
        self.axial_mass = frame.axial_mass();
        let soften = |spring: Spring| Softness::new(spring.hertz, spring.damping_ratio, context.substep);
        self.linear_softness = self.linear_spring.map(soften);
        self.angular_softness = self.angular_spring.map(soften);
    }

    pub(crate) fn warm_start(&self, frame: &JointFrame, a: &mut Body, b: &mut Body) {
        let (anchor_a, anchor_b) = frame.current_anchors(a, b);
        apply_linear_impulse(a, b, self.linear_impulse, anchor_a, anchor_b);
        apply_angular_impulse(a, b, self.angular_impulse);
    }

    pub(crate) fn solve(
        &mut self,
        frame: &JointFrame,
        a: &mut Body,
        b: &mut Body,
        context: &SolverContext,
        use_bias: bool,
    ) {
        // Angle.
        let angle = self.delta_angle + relative_angle(b.delta_rotation(), a.delta_rotation());
        let terms = match self.angular_softness {
            Some(softness) => BiasTerms::soft(softness, angle),
            None => BiasTerms::equality(angle, context, use_bias),
        };
        let speed = b.angular_velocity() - a.angular_velocity();
        let impulse =
            -self.axial_mass * terms.mass_scale * (speed + terms.bias) - terms.impulse_scale * self.angular_impulse;
        self.angular_impulse += impulse;
        apply_angular_impulse(a, b, impulse);

        // Weld point.
        let (anchor_a, anchor_b) = frame.current_anchors(a, b);
        let softness = match (self.linear_softness, use_bias) {
            (Some(softness), _) => Some(softness),
            (None, true) => Some(context.joint_softness),
            (None, false) => None,
        };
        let (bias, mass_scale, impulse_scale) = match softness {
            Some(softness) => {
                let separation = frame.separation(a, b, anchor_a, anchor_b);
                (softness.bias_rate * separation, softness.mass_scale, softness.impulse_scale)
            }
            None => (Vec2::ZERO, 1.0, 0.0),
        };
        let velocity_error = relative_velocity(a, b, anchor_a, anchor_b);
        let mass = frame.point_mass(anchor_a, anchor_b);
        let impulse = -mass_scale * mass.solve(velocity_error + bias) - impulse_scale * self.linear_impulse;
        self.linear_impulse += impulse;
        apply_linear_impulse(a, b, impulse, anchor_a, anchor_b);
    }
}
