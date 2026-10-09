//! Game rules, independent of input and drawing so they can be tested.

use kinetica::SensorEvent;
use kinetica::prelude::*;

use crate::level::{self, FALL_LIMIT, Level};
use crate::player::{Input, Player};

pub const TIME_STEP: f32 = 1.0 / 60.0;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum State {
    Playing,
    /// Reached the goal with every coin.
    Won,
}

#[derive(Clone)]
pub struct Game {
    pub world: World,
    pub level: Level,
    pub player: Player,
    pub coins_collected: usize,
    pub coin_count: usize,
    pub deaths: u32,
    pub time: f32,
    pub state: State,
    /// The player touched the goal without every coin, this long ago.
    pub missing_coins_hint: Option<f32>,
}

impl Game {
    pub fn new() -> Self {
        let mut world = World::new();
        let level = level::build(&mut world);
        let player = Player::new(&mut world, level.spawn);
        let coin_count = level.coins.len();
        Self {
            world,
            level,
            player,
            coins_collected: 0,
            coin_count,
            deaths: 0,
            time: 0.0,
            state: State::Playing,
            missing_coins_hint: None,
        }
    }

    pub fn step(&mut self, input: Input) {
        let input = if self.state == State::Playing { input } else { Input::default() };
        self.player.update(&mut self.world, input, TIME_STEP);
        for platform in &mut self.level.moving_platforms {
            platform.update(&mut self.world);
        }
        self.world.step(TIME_STEP);
        if self.state == State::Playing {
            self.time += TIME_STEP;
        }
        if let Some(age) = &mut self.missing_coins_hint {
            *age += TIME_STEP;
        }
        self.handle_sensors();
        if self.player.position(&self.world).y < FALL_LIMIT {
            self.die();
        }
    }

    fn handle_sensors(&mut self) {
        let player = self.player.body;
        let entered: Vec<_> = self
            .world
            .sensor_events()
            .iter()
            .filter_map(|event| match event {
                SensorEvent::Entered(pair) if pair.visitor_body == player => Some(*pair),
                _ => None,
            })
            .collect();
        for pair in entered {
            if let Some(index) = self.level.coins.iter().position(|&coin| coin == pair.sensor_body) {
                self.world.remove_body(self.level.coins.swap_remove(index));
                self.coins_collected += 1;
            } else if self.level.spikes.contains(&pair.sensor) {
                self.die();
            } else if self.level.goal == Some(pair.sensor) {
                if self.level.coins.is_empty() {
                    self.state = State::Won;
                } else {
                    self.missing_coins_hint = Some(0.0);
                }
            }
        }
    }

    fn die(&mut self) {
        self.deaths += 1;
        self.player.respawn(&mut self.world, self.level.spawn);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn run(game: &mut Game, steps: usize, input: Input) {
        for _ in 0..steps {
            game.step(input);
        }
    }

    fn position(game: &Game) -> Vec2 {
        game.player.position(&game.world)
    }

    const RIGHT: Input = Input { run: 1.0, jump_pressed: false, jump_held: false };
    const IDLE: Input = Input { run: 0.0, jump_pressed: false, jump_held: false };

    fn jump(game: &mut Game) {
        game.step(Input { jump_pressed: true, jump_held: true, ..IDLE });
    }

    #[test]
    fn player_stands_still_and_runs() {
        let mut game = Game::new();
        run(&mut game, 60, IDLE);
        let start = position(&game);
        assert!(game.player.ground(&game.world).is_some(), "the player should stand on the ground");
        run(&mut game, 60, IDLE);
        assert!(position(&game).distance(start) < 0.01, "the player slides while idle");

        run(&mut game, 30, RIGHT);
        let speed = game.world.body(game.player.body).unwrap().linear_velocity().x;
        assert!((speed - 7.0).abs() < 0.1, "running speed {speed}");
    }

    #[test]
    fn jumps_reach_the_first_platform_and_land_on_it() {
        let mut game = Game::new();
        // Under the first platform (top at y = 2.62), then jump straight up and hold.
        game.player.respawn(&mut game.world, vec2(-2.0, 0.7));
        run(&mut game, 30, IDLE);
        jump(&mut game);
        run(&mut game, 120, Input { jump_held: true, ..IDLE });
        let y = position(&game).y;
        assert!((y - (2.62 + 0.6)).abs() < 0.05, "the player should stand on the platform, at y = {y}");
    }

    #[test]
    fn short_jumps_are_lower() {
        let apex = |hold_steps: usize| {
            let mut game = Game::new();
            run(&mut game, 30, IDLE);
            let floor = position(&game).y;
            jump(&mut game);
            let mut highest = floor;
            for step in 0..90 {
                game.step(Input { jump_held: step < hold_steps, ..IDLE });
                highest = highest.max(position(&game).y);
            }
            highest - floor
        };
        let (short, full) = (apex(3), apex(90));
        assert!((full - 3.7).abs() < 0.3, "full jump height {full}");
        assert!(short < 0.6 * full, "short jump {short} vs full {full}");
    }

    #[test]
    fn ferry_carries_the_player_over_the_pit() {
        let mut game = Game::new();
        // Drop onto the ferry at its starting point.
        game.player.respawn(&mut game.world, vec2(9.5, 2.0));
        run(&mut game, 120, IDLE);
        let x = position(&game).x;
        assert!(x > 12.0, "the player should ride the ferry, it is at x = {x}");
        assert_eq!(game.deaths, 0, "the player fell into the pit");
    }

    #[test]
    fn coins_spikes_and_goal() {
        let mut game = Game::new();
        // Walk into the first coin from below it.
        game.player.respawn(&mut game.world, vec2(-2.0, 3.0));
        run(&mut game, 10, IDLE);
        assert_eq!(game.coins_collected, 1);

        // The spikes send the player back to the start.
        game.player.respawn(&mut game.world, vec2(12.0, -2.5));
        run(&mut game, 60, IDLE);
        assert_eq!(game.deaths, 1);
        assert!(position(&game).distance(game.level.spawn) < 1.0);

        // The goal only counts with every coin.
        game.player.respawn(&mut game.world, vec2(50.0, 1.5));
        run(&mut game, 10, IDLE);
        assert_eq!(game.state, State::Playing);
        assert!(game.missing_coins_hint.is_some());
        for coin in std::mem::take(&mut game.level.coins) {
            game.world.remove_body(coin);
        }
        game.player.respawn(&mut game.world, vec2(46.0, 1.5));
        run(&mut game, 10, IDLE);
        game.player.respawn(&mut game.world, vec2(50.0, 1.5));
        run(&mut game, 10, IDLE);
        assert_eq!(game.state, State::Won);
    }
}
