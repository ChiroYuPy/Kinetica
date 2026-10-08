//! Tolerances shared by the collision and dynamics code, in meters.

/// Distance considered negligible by the simulation. Contacts are allowed to overlap by about
/// this much, which keeps resting bodies from jittering.
pub(crate) const LINEAR_SLOP: f32 = 0.005;

/// Contact points are created up to this distance *before* shapes actually touch, so the solver
/// can stop bodies before they overlap.
pub(crate) const SPECULATIVE_DISTANCE: f32 = 4.0 * LINEAR_SLOP;

/// Margin added around collider bounding boxes in the broad phase. A larger margin means fewer
/// tree updates for moving bodies, but more candidate pairs.
pub(crate) const AABB_MARGIN: f32 = 0.1;
