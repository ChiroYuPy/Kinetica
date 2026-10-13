//! Soft constraints: a constraint behaving like a stiff, damped spring instead of a rigid rod.
//!
//! Softness removes the jitter of rigid constraints and makes the solver stable with few
//! iterations. See Erin Catto, "Solver2D" (2024) and "Soft Constraints" (GDC 2011).

use std::f32::consts::TAU;

use super::SymmetricMatrix;
use crate::math::Vec2;

/// Coefficients turning a spring (frequency, damping ratio) into solver terms for a substep.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub(crate) struct Softness {
    /// Fraction of the position error corrected per second.
    pub(crate) bias_rate: f32,
    /// Scales the effective mass; below 1 makes the constraint softer.
    pub(crate) mass_scale: f32,
    /// Fraction of the accumulated impulse removed at each iteration, preventing it from growing
    /// without bound.
    pub(crate) impulse_scale: f32,
}

impl Softness {
    /// A rigid constraint that only removes velocity errors, leaving position errors alone.
    pub(crate) const RIGID: Self = Self { bias_rate: 0.0, mass_scale: 1.0, impulse_scale: 0.0 };

    pub(crate) fn new(hertz: f32, damping_ratio: f32, substep: f32) -> Self {
        if hertz == 0.0 {
            return Self { bias_rate: 0.0, mass_scale: 1.0, impulse_scale: 0.0 };
        }
        let omega = TAU * hertz;
        let a1 = 2.0 * damping_ratio + substep * omega;
        let a2 = substep * omega * a1;
        let a3 = 1.0 / (1.0 + a2);
        Self { bias_rate: omega / a1, mass_scale: a2 * a3, impulse_scale: a3 }
    }

    /// A rigid constraint allowing a gap to close within one substep, but not further: used
    /// before contacts and limits are reached.
    pub(crate) fn closing_within(inverse_substep: f32) -> Self {
        Self { bias_rate: inverse_substep, ..Self::RIGID }
    }

    /// The impulse a one-dimensional constraint applies.
    ///
    /// `speed` is the rate of change of the constraint, `position_error` how far it is from
    /// being satisfied, `mass` its effective mass, and `accumulated` the impulse it already
    /// applied this step.
    pub(crate) fn impulse(&self, mass: f32, speed: f32, position_error: f32, accumulated: f32) -> f32 {
        -mass * self.mass_scale * (speed + self.bias_rate * position_error) - self.impulse_scale * accumulated
    }

    /// The impulse a two-dimensional constraint applies, like [`impulse`](Self::impulse) with
    /// an inverse effective mass matrix.
    pub(crate) fn impulse_2d(
        &self,
        inverse_mass: &SymmetricMatrix,
        velocity: Vec2,
        position_error: Vec2,
        accumulated: Vec2,
    ) -> Vec2 {
        -self.mass_scale * inverse_mass.solve(velocity + self.bias_rate * position_error)
            - self.impulse_scale * accumulated
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn zero_frequency_is_rigid_without_bias() {
        let softness = Softness::new(0.0, 1.0, 1.0 / 240.0);
        assert_eq!(softness, Softness { bias_rate: 0.0, mass_scale: 1.0, impulse_scale: 0.0 });
    }

    #[test]
    fn scales_add_up_to_one() {
        let softness = Softness::new(30.0, 10.0, 1.0 / 240.0);
        assert!((softness.mass_scale + softness.impulse_scale - 1.0).abs() < 1e-6);
        assert!(softness.bias_rate > 0.0);
    }
}
