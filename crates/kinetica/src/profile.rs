//! Timings of the stages of a step, to find out what makes a world slow.

use std::time::Duration;

/// How long each stage of the last [`World::step`](crate::World::step) took.
///
/// Only measured with the `profile` feature; every duration is zero otherwise, so the default
/// build does not pay for the clock (which does not exist on `wasm32-unknown-unknown`).
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct StepProfile {
    /// Contact and sensor geometry (the narrow phase).
    pub collide: Duration,
    /// Building the islands and waking the ones that were touched.
    pub islands: Duration,
    /// The constraint solver: joints, contacts and integration.
    pub solve: Duration,
    /// Moving collider bounding boxes and broad phase proxies.
    pub synchronize: Duration,
    /// Sweeping fast bodies (continuous collision).
    pub continuous: Duration,
    /// Putting resting islands to sleep.
    pub sleep: Duration,
    /// Finding the pairs of colliders that came close (the broad phase) and creating contacts.
    pub find_pairs: Duration,
}

impl StepProfile {
    /// The whole step.
    #[must_use]
    pub fn total(&self) -> Duration {
        self.collide + self.islands + self.solve + self.synchronize + self.continuous + self.sleep + self.find_pairs
    }
}

/// Measures the time between laps; does nothing without the `profile` feature.
pub(crate) struct Timer {
    #[cfg(feature = "profile")]
    last: std::time::Instant,
}

impl Timer {
    pub(crate) fn start() -> Self {
        Self {
            #[cfg(feature = "profile")]
            last: std::time::Instant::now(),
        }
    }

    /// Time since the start or the previous lap.
    #[allow(clippy::unused_self, reason = "the clock is only read with the `profile` feature")]
    pub(crate) fn lap(&mut self) -> Duration {
        #[cfg(feature = "profile")]
        {
            let now = std::time::Instant::now();
            let elapsed = now - self.last;
            self.last = now;
            elapsed
        }
        #[cfg(not(feature = "profile"))]
        Duration::ZERO
    }
}
