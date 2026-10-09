//! Fixed time step: the simulation advances by equal steps, whatever the frame rate.

/// Converts variable frame times into a whole number of fixed simulation steps.
///
/// Physics is only stable and reproducible with a constant step. Each frame, add the elapsed
/// time with [`advance`](Self::advance) and run the returned number of steps; the remainder is
/// kept for the next frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FixedTimestep {
    dt: f32,
    max_steps: u32,
    accumulator: f32,
}

impl FixedTimestep {
    /// Steps of `dt` seconds, at most `max_steps` per frame: after a very slow frame, the
    /// simulation slows down instead of freezing the game while it catches up.
    ///
    /// # Panics
    ///
    /// Panics if `dt` is not strictly positive or `max_steps` is zero.
    #[must_use]
    pub fn new(dt: f32, max_steps: u32) -> Self {
        assert!(dt > 0.0 && max_steps > 0, "a time step needs a positive duration and at least one step");
        Self { dt, max_steps, accumulator: 0.0 }
    }

    /// Duration of one step, in seconds.
    #[must_use]
    pub fn dt(&self) -> f32 {
        self.dt
    }

    /// Adds `frame_time` seconds and returns how many steps to run now.
    pub fn advance(&mut self, frame_time: f32) -> u32 {
        let max_time = self.max_steps as f32 * self.dt;
        self.accumulator = (self.accumulator + frame_time.max(0.0)).min(max_time);
        let mut steps = 0;
        while self.accumulator >= self.dt {
            self.accumulator -= self.dt;
            steps += 1;
        }
        steps
    }

    /// How far the simulation is into the next step, from 0 to 1: interpolating between the
    /// last two states with it gives smooth rendering at any frame rate.
    #[must_use]
    pub fn alpha(&self) -> f32 {
        self.accumulator / self.dt
    }

    /// Forgets the time owed to the simulation, for example after a pause.
    pub fn reset(&mut self) {
        self.accumulator = 0.0;
    }
}

impl Default for FixedTimestep {
    /// 60 steps per second, at most 4 per frame.
    fn default() -> Self {
        Self::new(1.0 / 60.0, 4)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn runs_whole_steps_and_keeps_the_rest() {
        let mut timestep = FixedTimestep::new(0.01, 10);
        assert_eq!(timestep.advance(0.025), 2);
        assert!((timestep.alpha() - 0.5).abs() < 1e-4);
        assert_eq!(timestep.advance(0.005), 1);
    }

    #[test]
    fn caps_the_steps_after_a_slow_frame() {
        let mut timestep = FixedTimestep::new(0.01, 4);
        assert_eq!(timestep.advance(1.0), 4);
        assert_eq!(timestep.advance(0.0), 0);
    }
}
