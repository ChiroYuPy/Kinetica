//! Tolerances shared by the shapes and the collision routines, in meters.

/// Distance considered negligible. Contacts are allowed to overlap by about this much, which
/// keeps resting bodies from jittering.
pub const LINEAR_SLOP: f32 = 0.005;

/// Contact points are created up to this distance *before* shapes actually touch, so a solver
/// can stop bodies before they overlap.
pub const SPECULATIVE_DISTANCE: f32 = 4.0 * LINEAR_SLOP;
