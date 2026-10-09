//! Game rules, independent of input and drawing so they can be tested.

use kinetica::prelude::*;

use crate::car::{Car, Controls};
use crate::track::{CHECKPOINTS, FINISH, PIT_DEPTH, Track};

pub const TIME_STEP: f32 = 1.0 / 60.0;
/// How long a crash shows before the car is put back at the last checkpoint, in seconds.
pub const CRASH_DELAY: f32 = 1.2;

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum State {
    Driving,
    /// The driver's head hit the ground, or the car fell in a pit, this many seconds ago.
    Crashed(f32),
    Finished,
}

#[derive(Clone)]
pub struct Game {
    pub world: World,
    pub track: Track,
    pub car: Car,
    pub state: State,
    /// Seconds since the start; it keeps running through crashes.
    pub time: f32,
    pub crashes: u32,
    /// Index in [`CHECKPOINTS`] of the last checkpoint passed.
    pub checkpoint: usize,
}

impl Game {
    pub fn new() -> Self {
        let mut world = World::new();
        let track = Track::new(&mut world);
        let car = Car::new(&mut world, Track::spawn_point(CHECKPOINTS[0]));
        Self { world, track, car, state: State::Driving, time: 0.0, crashes: 0, checkpoint: 0 }
    }

    pub fn step(&mut self, controls: Controls) {
        let controls = if self.state == State::Driving { controls } else { Controls::default() };
        self.car.drive(&mut self.world, controls);
        self.world.step(TIME_STEP);

        match self.state {
            State::Driving => {
                self.time += TIME_STEP;
                let x = self.car.position(&self.world).x;
                while self.checkpoint + 1 < CHECKPOINTS.len() && x > CHECKPOINTS[self.checkpoint + 1] {
                    self.checkpoint += 1;
                }
                if x > FINISH {
                    self.state = State::Finished;
                } else if self.has_crashed() {
                    self.crashes += 1;
                    self.state = State::Crashed(0.0);
                }
            }
            State::Crashed(since) if since > CRASH_DELAY => self.respawn(),
            State::Crashed(since) => {
                self.time += TIME_STEP;
                self.state = State::Crashed(since + TIME_STEP);
            }
            State::Finished => {}
        }
    }

    /// The head touches the ground or the bridge or seesaw planks (a crate falling on it is
    /// fine), or the car fell into a pit.
    fn has_crashed(&self) -> bool {
        let hurts = |body: BodyHandle| !self.car.bodies().contains(&body) && !self.track.crates.contains(&body);
        self.world.sensor_overlaps(self.car.head).any(|pair| hurts(pair.visitor_body))
            || self.car.position(&self.world).y < PIT_DEPTH + 2.0
    }

    /// Puts a new car at the last checkpoint.
    fn respawn(&mut self) {
        self.car.remove(&mut self.world);
        self.car = Car::new(&mut self.world, Track::spawn_point(CHECKPOINTS[self.checkpoint]));
        self.state = State::Driving;
    }

    /// How far along the track the car is, from 0 to 1.
    pub fn progress(&self) -> f32 {
        (self.car.position(&self.world).x / FINISH).clamp(0.0, 1.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const IDLE: Controls = Controls { throttle: 0.0 };

    /// Full throttle on the ground, easing off when the nose rears up; in the air, the pedals
    /// keep the car level.
    fn autopilot(game: &Game) -> Controls {
        let angle = game.world[game.car.chassis].angle();
        if game.car.on_ground(&game.world) {
            return Controls { throttle: if angle > 0.6 { 0.0 } else { 1.0 } };
        }
        Controls { throttle: (-3.0 * angle).clamp(-1.0, 1.0) }
    }

    #[test]
    fn car_rests_on_its_wheels() {
        let mut game = Game::new();
        for _ in 0..3 * 60 {
            game.step(IDLE);
        }
        let chassis = &game.world[game.car.chassis];
        assert!(chassis.angle().abs() < 0.02, "the car tilted to {}", chassis.angle());
        assert!(chassis.linear_velocity().length() < 0.05, "the car still moves");
        assert_eq!(game.state, State::Driving);
    }

    #[test]
    fn a_careful_driver_reaches_the_flag_without_crashing() {
        let mut game = Game::new();
        for _ in 0..120 * 60 {
            game.step(autopilot(&game));
            if game.state != State::Driving {
                break;
            }
        }
        assert_eq!(
            game.state,
            State::Finished,
            "stopped at x = {} after {:.1} s",
            game.car.position(&game.world).x,
            game.time
        );
    }

    #[test]
    fn driving_into_a_pit_crashes_and_respawns_at_the_checkpoint() {
        let mut game = Game::new();
        // Drop a car under the bridge, past the second checkpoint.
        game.car.remove(&mut game.world);
        game.car = Car::new(&mut game.world, vec2(138.0, -5.0));
        for _ in 0..60 {
            game.step(IDLE);
        }
        assert!(matches!(game.state, State::Crashed(_)), "{:?}", game.state);
        assert_eq!(game.crashes, 1);
        for _ in 0..2 * 60 {
            game.step(IDLE);
        }
        assert_eq!(game.state, State::Driving);
        let x = game.car.position(&game.world).x;
        assert!((x - CHECKPOINTS[1]).abs() < 1.0, "respawned at {x}");
    }
}
