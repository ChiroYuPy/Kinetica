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

/// Stiffness of contacts, in hertz. Contacts are stiff springs, so tall stacks sink and lean a
/// little under their own weight.
pub(crate) const CONTACT_HERTZ: f32 = 30.0;

/// Shorter substeps allow stiffer contacts: up to this fraction of the substep rate, which is
/// 30 Hz at 60 steps per second with 4 substeps, and 60 Hz with 8.
pub(crate) const CONTACT_HERTZ_PER_SUBSTEP_RATE: f32 = 0.125;

/// Damping of the contact springs; strongly over-damped so bodies settle without bouncing.
pub(crate) const CONTACT_DAMPING_RATIO: f32 = 10.0;

/// Maximum speed, in m/s, at which overlapping bodies are pushed apart. Keeps deep overlaps from
/// resolving explosively.
pub(crate) const MAX_CONTACT_PUSH_SPEED: f32 = 3.0;

/// Impacts slower than this, in m/s, do not bounce. Prevents resting bodies from jittering.
pub(crate) const RESTITUTION_THRESHOLD: f32 = 1.0;

/// Joints are twice as stiff as contacts, and damped less.
pub(crate) const JOINT_HERTZ_RATIO: f32 = 2.0;
pub(crate) const JOINT_DAMPING_RATIO: f32 = 2.0;

/// Bodies slower than this, in m/s, are considered still and may fall asleep.
pub(crate) const SLEEP_LINEAR_SPEED: f32 = 0.05;

/// Bodies turning slower than this, in rad/s (about 2°/s), are considered still.
pub(crate) const SLEEP_ANGULAR_SPEED: f32 = 0.035;

/// How long, in seconds, a group of touching bodies must stay still before falling asleep.
pub(crate) const TIME_TO_SLEEP: f32 = 0.5;

/// A dynamic body moving more than this fraction of its thickness in one step is checked for
/// tunneling.
pub(crate) const CONTINUOUS_THRESHOLD: f32 = 0.5;

/// Default number of substeps per step.
pub(crate) const DEFAULT_SUBSTEP_COUNT: u32 = 4;
