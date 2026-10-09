//! Game rules, independent of input and drawing so they can be tested.

use kinetica::SensorEvent;
use kinetica::prelude::*;

use crate::car::{Car, Controls};
use crate::level::{self, Level, TRACK_LENGTH};

pub const TIME_STEP: f32 = 1.0 / 60.0;
pub const START: Vec2 = vec2(0.0, 1.5);
pub const FULL_TANK: f32 = 100.0;
/// Fuel burnt per second with the pedal down, and while idling.
const FUEL_PER_SECOND: f32 = 7.0;
const IDLE_FUEL_PER_SECOND: f32 = 1.0;
/// The run ends when the car stands still this long with an empty tank, in seconds.
const STALL_TIME: f32 = 2.0;
const STALL_SPEED: f32 = 0.3;
pub const COIN_VALUE: u32 = 10;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum State {
    Driving,
    /// The driver's head touched the ground.
    Crashed,
    OutOfFuel,
    /// Crossed the finish line.
    Finished,
}

#[derive(Clone)]
pub struct Game {
    pub world: World,
    pub level: Level,
    pub car: Car,
    pub fuel: f32,
    pub coins: u32,
    pub state: State,
    pub time: f32,
    /// Farthest distance reached, in meters.
    pub distance: f32,
    /// How long the car has been standing still without fuel.
    stalled_for: f32,
}

impl Game {
    pub fn new() -> Self {
        let mut world = World::new();
        let level = level::build(&mut world);
        let car = Car::new(&mut world, START);
        Self {
            world,
            level,
            car,
            fuel: FULL_TANK,
            coins: 0,
            state: State::Driving,
            time: 0.0,
            distance: 0.0,
            stalled_for: 0.0,
        }
    }

    pub fn step(&mut self, controls: Controls) {
        let controls = if self.state == State::Driving && self.fuel > 0.0 { controls } else { Controls::default() };
        self.car.drive(&mut self.world, controls);
        self.world.step(TIME_STEP);
        if self.state != State::Driving {
            return;
        }

        self.time += TIME_STEP;
        let burn = if controls.throttle == 0.0 { IDLE_FUEL_PER_SECOND } else { FUEL_PER_SECOND };
        self.fuel = (self.fuel - burn * TIME_STEP).max(0.0);
        self.distance = self.distance.max(self.car.position(&self.world).x - START.x);
        self.handle_sensors();

        if self.fuel == 0.0 && self.car.speed(&self.world) < STALL_SPEED {
            self.stalled_for += TIME_STEP;
            if self.stalled_for > STALL_TIME {
                self.state = State::OutOfFuel;
            }
        } else {
            self.stalled_for = 0.0;
        }
    }

    fn handle_sensors(&mut self) {
        let car_bodies = self.car.bodies(&self.world);
        let entered: Vec<_> = self
            .world
            .sensor_events()
            .iter()
            .filter_map(|event| match event {
                SensorEvent::Entered(pair) => Some(*pair),
                SensorEvent::Exited(_) => None,
            })
            .collect();
        for pair in entered {
            if pair.sensor == self.car.head {
                // The head is a sensor too: anything solid that is not part of the car hurts.
                if !car_bodies.contains(&pair.visitor_body) {
                    self.state = State::Crashed;
                }
            } else if !car_bodies.contains(&pair.visitor_body) {
                // Pickups only react to the car.
            } else if let Some(index) = self.level.fuel_cans.iter().position(|&can| can == pair.sensor_body) {
                self.world.remove_body(self.level.fuel_cans.remove(index));
                self.fuel = FULL_TANK;
            } else if let Some(index) = self.level.coins.iter().position(|&coin| coin == pair.sensor_body) {
                self.world.remove_body(self.level.coins.swap_remove(index));
                self.coins += 1;
            } else if pair.sensor == self.level.finish {
                self.state = State::Finished;
            }
        }
    }

    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss, reason = "distances are positive and small")]
    pub fn score(&self) -> u32 {
        self.distance as u32 + COIN_VALUE * self.coins
    }

    /// Progress along the track, between 0 and 1.
    pub fn progress(&self) -> f32 {
        (self.distance / TRACK_LENGTH).clamp(0.0, 1.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Drives like a careful player: full throttle on the ground, and in the air keeps the car
    /// parallel to the ground ahead.
    fn autopilot(game: &Game) -> Controls {
        let world = &game.world;
        let chassis = &world[game.car.chassis];
        if game.car.on_ground(world) {
            return Controls { throttle: 1.0 };
        }
        let x = chassis.position().x;
        let slope = (level::height(x + 1.0) - level::height(x - 1.0)) / 2.0;
        let error = chassis.angle() - slope.atan();
        let throttle = if error > 0.15 {
            -1.0
        } else if error < -0.15 {
            1.0
        } else {
            0.0
        };
        Controls { throttle }
    }

    fn drive(game: &mut Game, seconds: usize, controls: impl Fn(&Game) -> Controls) {
        for _ in 0..seconds * 60 {
            let controls = controls(game);
            game.step(controls);
        }
    }

    const IDLE: Controls = Controls { throttle: 0.0 };
    const FULL_THROTTLE: Controls = Controls { throttle: 1.0 };

    #[test]
    fn car_rests_on_its_wheels() {
        let mut game = Game::new();
        drive(&mut game, 3, |_| IDLE);
        let world = &game.world;
        assert!(game.car.on_ground(world), "the wheels should touch the ground");
        assert_eq!(game.state, State::Driving);
        let chassis = &world[game.car.chassis];
        assert!(
            chassis.angle().abs() < 0.02 && chassis.linear_velocity().length() < 0.05,
            "the car should stand still"
        );
        for &wheel in &game.car.wheels {
            assert!(!world.body_contacts(wheel).any(|contact| contact.pair().involves_body(game.car.chassis)));
        }
    }

    #[test]
    fn careful_driver_gets_far_and_collects_coins() {
        let mut game = Game::new();
        game.fuel = f32::INFINITY;
        drive(&mut game, 35, autopilot);
        assert_eq!(game.state, State::Driving, "crashed at {:?}", game.car.position(&game.world));
        assert!(game.distance > 300.0, "only reached {} m", game.distance);
        assert!(game.coins > 20, "only {} coins", game.coins);
    }

    #[test]
    fn fuel_cans_refill_the_tank() {
        let mut game = Game::new();
        // The first can is 90 m away: about 9 s of driving, so the tank is below half by then.
        drive(&mut game, 6, autopilot);
        assert!(game.fuel < 60.0, "fuel {}", game.fuel);
        drive(&mut game, 4, autopilot);
        assert!(game.distance > 95.0, "the car should be past the first can: {} m", game.distance);
        assert!(game.fuel > 80.0, "the tank should have been refilled: {}", game.fuel);
    }

    #[test]
    fn empty_tank_ends_the_run() {
        let mut game = Game::new();
        game.fuel = 5.0;
        drive(&mut game, 10, |_| FULL_THROTTLE);
        assert_eq!(game.state, State::OutOfFuel);
        assert!(game.distance < 30.0, "the car went {} m on almost no fuel", game.distance);
    }

    #[test]
    fn landing_on_the_head_crashes() {
        let mut game = Game::new();
        // Turn the whole car upside down around its chassis, a little above the ground.
        let pivot = game.car.position(&game.world);
        for body in game.car.bodies(&game.world) {
            let body = &mut game.world[body];
            let offset = body.position() - pivot;
            body.set_position(pivot + vec2(0.0, 1.0) - offset);
            body.set_angle(body.angle() + std::f32::consts::PI);
        }
        drive(&mut game, 2, |_| IDLE);
        assert_eq!(game.state, State::Crashed);
    }
}
