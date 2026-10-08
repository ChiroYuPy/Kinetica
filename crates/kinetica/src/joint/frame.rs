//! Data and math shared by every joint.

use std::f32::consts::{PI, TAU};

use crate::body::Body;
use crate::math::{Rot, Vec2, vec2};
use crate::solver::{Softness, SolverContext};

/// Anchors and masses of a joint, captured at the start of each step.
///
/// Anchors are relative to each body's center of mass, in world orientation. During the step
/// they are rotated by how much each body turned, which keeps the joint accurate without
/// recomputing everything at each substep.
#[derive(Debug, Clone, Copy, Default)]
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

    /// Inverse of the effective mass matrix of a point-to-point constraint.
    pub(crate) fn point_mass(&self, anchor_a: Vec2, anchor_b: Vec2) -> SymmetricMatrix {
        let (ma, mb, ia, ib) = (self.mass_a, self.mass_b, self.inertia_a, self.inertia_b);
        SymmetricMatrix {
            xx: ma + mb + anchor_a.y * anchor_a.y * ia + anchor_b.y * anchor_b.y * ib,
            xy: -anchor_a.y * anchor_a.x * ia - anchor_b.y * anchor_b.x * ib,
            yy: ma + mb + anchor_a.x * anchor_a.x * ia + anchor_b.x * anchor_b.x * ib,
        }
    }

    /// Inverse of the summed rotational inertias, the effective mass of an angular constraint.
    pub(crate) fn axial_mass(&self) -> f32 {
        inverse_or_zero(self.inertia_a + self.inertia_b)
    }
}

/// A symmetric 2×2 matrix.
#[derive(Debug, Clone, Copy)]
pub(crate) struct SymmetricMatrix {
    pub(crate) xx: f32,
    pub(crate) xy: f32,
    pub(crate) yy: f32,
}

impl SymmetricMatrix {
    /// Solves `self * x = b`; returns zero when the matrix is singular.
    pub(crate) fn solve(&self, b: Vec2) -> Vec2 {
        let determinant = self.xx * self.yy - self.xy * self.xy;
        if determinant == 0.0 {
            return Vec2::ZERO;
        }
        let inverse = 1.0 / determinant;
        vec2(inverse * (self.yy * b.x - self.xy * b.y), inverse * (self.xx * b.y - self.xy * b.x))
    }
}

/// The three terms that turn a constraint into a soft one, see [`Softness`].
#[derive(Debug, Clone, Copy)]
pub(crate) struct BiasTerms {
    /// Velocity added to correct the position error.
    pub(crate) bias: f32,
    pub(crate) mass_scale: f32,
    pub(crate) impulse_scale: f32,
}

impl BiasTerms {
    /// No position correction: only the velocity error is removed.
    pub(crate) const RIGID: Self = Self { bias: 0.0, mass_scale: 1.0, impulse_scale: 0.0 };

    pub(crate) fn soft(softness: Softness, position_error: f32) -> Self {
        Self {
            bias: softness.bias_rate * position_error,
            mass_scale: softness.mass_scale,
            impulse_scale: softness.impulse_scale,
        }
    }

    /// Terms for a one-sided limit whose remaining room is `gap` (negative when violated).
    ///
    /// While there is room, the limit only stops the motion that would cross it during this
    /// substep; once violated, it pushes back softly.
    pub(crate) fn limit(gap: f32, context: &SolverContext, use_bias: bool) -> Self {
        if gap > 0.0 {
            Self { bias: gap * context.inverse_substep, ..Self::RIGID }
        } else if use_bias {
            Self::soft(context.joint_softness, gap)
        } else {
            Self::RIGID
        }
    }

    /// Terms for an equality constraint with the given position error.
    pub(crate) fn equality(position_error: f32, context: &SolverContext, use_bias: bool) -> Self {
        if use_bias { Self::soft(context.joint_softness, position_error) } else { Self::RIGID }
    }
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
    fn symmetric_solve_inverts_multiplication() {
        let matrix = SymmetricMatrix { xx: 3.0, xy: 1.0, yy: 2.0 };
        let x = vec2(0.5, -1.5);
        let b = vec2(matrix.xx * x.x + matrix.xy * x.y, matrix.xy * x.x + matrix.yy * x.y);
        assert!(matrix.solve(b).abs_diff_eq(x, 1e-5));
        assert_eq!(SymmetricMatrix { xx: 1.0, xy: 1.0, yy: 1.0 }.solve(b), Vec2::ZERO);
    }

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
