//! Data and math shared by every joint.

use std::f32::consts::{PI, TAU};

use super::Motor;
use crate::body::Body;
use crate::math::{Rot, Vec2};
use crate::solver::SymmetricMatrix;
use crate::solver::{Pass, Softness, SolverContext};

/// Anchors and masses of a joint, captured at the start of each step.
///
/// Anchors are relative to each body's center of mass, in world orientation. During the step
/// they are rotated by how much each body turned, which keeps the joint accurate without
/// recomputing everything at each substep.
#[derive(Debug, Clone, Copy, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub(crate) struct JointFrame {
    pub(crate) anchor_a: Vec2,
    pub(crate) anchor_b: Vec2,
    /// `center_b - center_a` at the start of the step.
    pub(crate) delta_center: Vec2,
    pub(crate) mass_a: f32,
    pub(crate) mass_b: f32,
    pub(crate) inertia_a: f32,
    pub(crate) inertia_b: f32,
}

impl JointFrame {
    pub(crate) fn new(a: &Body, b: &Body, local_anchor_a: Vec2, local_anchor_b: Vec2) -> Self {
        Self {
            anchor_a: a.world_vector(local_anchor_a - a.local_center_of_mass()),
            anchor_b: b.world_vector(local_anchor_b - b.local_center_of_mass()),
            delta_center: b.center_of_mass() - a.center_of_mass(),
            mass_a: a.inverse_mass(),
            mass_b: b.inverse_mass(),
            inertia_a: a.inverse_inertia(),
            inertia_b: b.inverse_inertia(),
        }
    }

    /// Anchors rotated by each body's rotation since the step started.
    pub(crate) fn current_anchors(&self, a: &Body, b: &Body) -> (Vec2, Vec2) {
        (a.delta_rotation().rotate(self.anchor_a), b.delta_rotation().rotate(self.anchor_b))
    }

    /// Current vector from anchor A to anchor B.
    pub(crate) fn separation(&self, a: &Body, b: &Body, anchor_a: Vec2, anchor_b: Vec2) -> Vec2 {
        (b.delta_position() - a.delta_position()) + self.delta_center + (anchor_b - anchor_a)
    }

    /// Inverse of the effective mass matrix of a constraint keeping the anchors together.
    pub(crate) fn inverse_point_mass(&self, anchor_a: Vec2, anchor_b: Vec2) -> SymmetricMatrix {
        let (ma, mb, ia, ib) = (self.mass_a, self.mass_b, self.inertia_a, self.inertia_b);
        SymmetricMatrix {
            xx: ma + mb + anchor_a.y * anchor_a.y * ia + anchor_b.y * anchor_b.y * ib,
            xy: -anchor_a.y * anchor_a.x * ia - anchor_b.y * anchor_b.x * ib,
            yy: ma + mb + anchor_a.x * anchor_a.x * ia + anchor_b.x * anchor_b.x * ib,
        }
    }

    /// Effective mass of a constraint moving B's anchor along `direction` relative to A, at the
    /// start of the step: A turns about its center, through the line from its center to B's anchor.
    pub(crate) fn mass_along(&self, direction: Vec2) -> f32 {
        let separation = self.delta_center + self.anchor_b - self.anchor_a;
        let arm_a = (separation + self.anchor_a).perp_dot(direction);
        let arm_b = self.anchor_b.perp_dot(direction);
        inverse_or_zero(self.mass_a + self.mass_b + self.inertia_a * arm_a * arm_a + self.inertia_b * arm_b * arm_b)
    }

    /// Inverse of the summed rotational inertias, the effective mass of an angular constraint.
    pub(crate) fn axial_mass(&self) -> f32 {
        inverse_or_zero(self.inertia_a + self.inertia_b)
    }
}

/// The current slide axis of a joint and the lever arms along and across it, for joints whose
/// body B slides along an axis fixed in body A.
pub(crate) struct SlideGeometry {
    pub(crate) axis: Vec2,
    pub(crate) perp: Vec2,
    /// Current vector from anchor A to anchor B.
    pub(crate) separation: Vec2,
    /// How far B slid along the axis.
    pub(crate) translation: f32,
    pub(crate) axial_arm_a: f32,
    pub(crate) axial_arm_b: f32,
    pub(crate) perp_arm_a: f32,
    pub(crate) perp_arm_b: f32,
}

impl SlideGeometry {
    /// `axis` is the slide axis in world space at the start of the step.
    pub(crate) fn new(axis: Vec2, frame: &JointFrame, a: &Body, b: &Body) -> Self {
        let (anchor_a, anchor_b) = frame.current_anchors(a, b);
        let separation = frame.separation(a, b, anchor_a, anchor_b);
        let axis = a.delta_rotation().rotate(axis);
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
    pub(crate) fn axial_speed(&self, a: &Body, b: &Body) -> f32 {
        self.axis.dot(b.linear_velocity() - a.linear_velocity()) + self.axial_arm_b * b.angular_velocity()
            - self.axial_arm_a * a.angular_velocity()
    }

    /// Relative speed of B across the axis.
    pub(crate) fn perp_speed(&self, a: &Body, b: &Body) -> f32 {
        self.perp.dot(b.linear_velocity() - a.linear_velocity()) + self.perp_arm_b * b.angular_velocity()
            - self.perp_arm_a * a.angular_velocity()
    }

    /// Applies an impulse along the axis to B, and its opposite to A.
    pub(crate) fn apply_axial(&self, a: &mut Body, b: &mut Body, impulse: f32) {
        apply_impulses(a, b, impulse * self.axis, impulse * self.axial_arm_a, impulse * self.axial_arm_b);
    }

    /// Applies an impulse across the axis to B, and its opposite to A.
    pub(crate) fn apply_perp(&self, a: &mut Body, b: &mut Body, impulse: f32) {
        apply_impulses(a, b, impulse * self.perp, impulse * self.perp_arm_a, impulse * self.perp_arm_b);
    }
}

/// Keeps the translation of a slide within `lower..=upper`, accumulating the impulses of the
/// lower and upper limits.
#[allow(clippy::too_many_arguments, reason = "the state of both limits and of the step")]
pub(crate) fn solve_slide_limits(
    geometry: &SlideGeometry,
    (lower, upper): (f32, f32),
    (lower_impulse, upper_impulse): (&mut f32, &mut f32),
    mass: f32,
    a: &mut Body,
    b: &mut Body,
    context: &SolverContext,
    pass: Pass,
) {
    // Lower limit: keeps the translation above `lower`.
    let gap = geometry.translation - lower;
    let speed = geometry.axial_speed(a, b);
    let impulse = limit_impulse(lower_impulse, mass, speed, gap, context.limit_softness(gap, pass));
    geometry.apply_axial(a, b, impulse);

    // Upper limit: keeps the translation below `upper`, pushing the other way.
    let gap = upper - geometry.translation;
    let speed = -geometry.axial_speed(a, b);
    let impulse = limit_impulse(upper_impulse, mass, speed, gap, context.limit_softness(gap, pass));
    geometry.apply_axial(a, b, -impulse);
}

/// Accumulates the impulse of a motor driving `speed` towards `motor.speed`, and returns the
/// part to apply now. The total is bounded by the motor force.
pub(crate) fn motor_impulse(motor: Motor, accumulated: &mut f32, mass: f32, speed: f32, substep: f32) -> f32 {
    let max_impulse = motor.max_force * substep;
    let previous = *accumulated;
    *accumulated = (previous - mass * (speed - motor.speed)).clamp(-max_impulse, max_impulse);
    *accumulated - previous
}

/// Accumulates the impulse of a limit keeping `gap` from becoming negative, and returns the
/// part to apply now. `speed` is the rate of change of the gap. A limit can only push, so the
/// total stays positive.
pub(crate) fn limit_impulse(accumulated: &mut f32, mass: f32, speed: f32, gap: f32, softness: Softness) -> f32 {
    let previous = *accumulated;
    *accumulated = (previous + softness.impulse(mass, speed, gap, previous)).max(0.0);
    *accumulated - previous
}

/// Velocity of B's anchor relative to A's anchor.
pub(crate) fn relative_velocity(a: &Body, b: &Body, anchor_a: Vec2, anchor_b: Vec2) -> Vec2 {
    let velocity_a = a.linear_velocity() + a.angular_velocity() * anchor_a.perp();
    let velocity_b = b.linear_velocity() + b.angular_velocity() * anchor_b.perp();
    velocity_b - velocity_a
}

/// Applies `impulse` at B's anchor and its opposite at A's anchor.
pub(crate) fn apply_linear_impulse(a: &mut Body, b: &mut Body, impulse: Vec2, anchor_a: Vec2, anchor_b: Vec2) {
    a.apply_solver_impulse(-impulse, anchor_a);
    b.apply_solver_impulse(impulse, anchor_b);
}

/// Applies an angular impulse to B and its opposite to A.
pub(crate) fn apply_angular_impulse(a: &mut Body, b: &mut Body, impulse: f32) {
    a.apply_solver_angular_impulse(-impulse);
    b.apply_solver_angular_impulse(impulse);
}

/// Applies a linear impulse at the centers of mass, plus separate angular impulses.
pub(crate) fn apply_impulses(a: &mut Body, b: &mut Body, linear: Vec2, angular_a: f32, angular_b: f32) {
    a.apply_solver_impulse(-linear, Vec2::ZERO);
    a.apply_solver_angular_impulse(-angular_a);
    b.apply_solver_impulse(linear, Vec2::ZERO);
    b.apply_solver_angular_impulse(angular_b);
}

/// Angle of rotation `b` relative to rotation `a`, in `[-π, π]`.
pub(crate) fn relative_angle(b: Rot, a: Rot) -> f32 {
    (b * a.inverse()).angle()
}

/// Brings an angle into `[-π, π)`.
pub(crate) fn wrap_angle(angle: f32) -> f32 {
    (angle + PI).rem_euclid(TAU) - PI
}

pub(crate) fn inverse_or_zero(value: f32) -> f32 {
    if value > 0.0 { 1.0 / value } else { 0.0 }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn wrap_angle_stays_in_range() {
        for angle in [0.0, 3.0, -3.0, 7.0, -7.0, 100.0] {
            let wrapped = wrap_angle(angle);
            assert!((-PI..PI).contains(&wrapped), "{angle} -> {wrapped}");
            // Same direction: only whole turns were removed.
            assert!(Rot::from_angle(wrapped).rotate(Vec2::X).abs_diff_eq(Rot::from_angle(angle).rotate(Vec2::X), 1e-4));
        }
    }
}
