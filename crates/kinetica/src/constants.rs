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

/// Upper bound on the stiffness of contacts, in hertz. It is further limited to a quarter of the
/// substep rate, beyond which soft constraints become unstable.
pub(crate) const CONTACT_HERTZ: f32 = 30.0;

/// Damping of the contact springs; strongly over-damped so bodies settle without bouncing.
pub(crate) const CONTACT_DAMPING_RATIO: f32 = 10.0;

/// Maximum speed, in m/s, at which overlapping bodies are pushed apart. Keeps deep overlaps from
/// resolving explosively.
pub(crate) const MAX_CONTACT_PUSH_SPEED: f32 = 3.0;

/// Impacts slower than this, in m/s, do not bounce. Prevents resting bodies from jittering.
pub(crate) const RESTITUTION_THRESHOLD: f32 = 1.0;

/// Default number of substeps per step.
pub(crate) const DEFAULT_SUBSTEP_COUNT: u32 = 4;
