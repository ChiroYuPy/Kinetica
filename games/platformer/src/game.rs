//! Game rules, independent of input and drawing so they can be tested.

use kinetica::prelude::*;

use crate::level::{CHECKPOINTS, DEATH_HEIGHT, GOAL, Level};
use crate::player::{Input, Player};

pub const TIME_STEP: f32 = 1.0 / 60.0;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum State {
    Playing,
    Won,
}

#[derive(Clone)]
pub struct Game {
    pub world: World,
    pub level: Level,
    pub player: Player,
    pub state: State,
    pub time: f32,
    pub coins: usize,
    pub deaths: u32,
    /// Index in [`CHECKPOINTS`] of the last checkpoint passed.
    pub checkpoint: usize,
}

impl Game {
    pub fn new() -> Self {
        let mut world = World::new();
        let level = Level::new(&mut world);
        let player = Player::new(&mut world, CHECKPOINTS[0]);
        Self { world, level, player, state: State::Playing, time: 0.0, coins: 0, deaths: 0, checkpoint: 0 }
    }

    pub fn total_coins(&self) -> usize {
        self.coins + self.level.coins.len()
    }

    pub fn step(&mut self, input: Input) {
        let input = if self.state == State::Playing { input } else { Input::default() };
        for mover in &self.level.movers {
            mover.update(&mut self.world, self.time, TIME_STEP);
        }
        self.player.update(&mut self.world, input, TIME_STEP);
        self.world.step(TIME_STEP);
        self.time += TIME_STEP;
        if self.state != State::Playing {
            return;
        }

        self.collect_coins();
        let position = self.player.position(&self.world);
        while self.checkpoint + 1 < CHECKPOINTS.len() && position.x > CHECKPOINTS[self.checkpoint + 1].x {
            self.checkpoint += 1;
        }
        if position.x > GOAL {
            self.state = State::Won;
        } else if position.y < DEATH_HEIGHT || self.on_spikes() {
            self.deaths += 1;
            self.player.respawn(&mut self.world, CHECKPOINTS[self.checkpoint]);
        }
    }

    fn collect_coins(&mut self) {
        let player = self.player.body;
        let touched: Vec<BodyHandle> = self
            .world
            .sensor_events()
            .iter()
            .filter_map(|event| match event {
                SensorEvent::Entered(pair) if pair.visitor_body == player => Some(pair.sensor_body),
                _ => None,
            })
            .filter(|body| self.level.coins.contains(body))
            .collect();
        for coin in touched {
            self.level.coins.retain(|&other| other != coin);
            self.world.remove_body(coin);
            self.coins += 1;
        }
    }

    fn on_spikes(&self) -> bool {
        let player = self.player.body;
        self.level.pits.iter().any(|pit| self.world.sensor_overlaps(pit.sensor).any(|pair| pair.visitor_body == player))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const IDLE: Input = Input { run: 0.0, jump_pressed: false, jump_held: false };
    const RIGHT: Input = Input { run: 1.0, ..IDLE };

    fn run(game: &mut Game, steps: usize, input: Input) {
        for _ in 0..steps {
            game.step(input);
        }
    }

    fn position(game: &Game) -> Vec2 {
        game.player.position(&game.world)
    }

    /// Puts the player at `position`, at rest, and lets it land.
    fn place(game: &mut Game, position: Vec2) {
        game.player.respawn(&mut game.world, position);
        run(game, 30, IDLE);
    }

    /// Jumps holding the button, and returns how high the feet went above where they started.
    fn jump_height(game: &mut Game) -> f32 {
        let start = position(game).y;
        game.step(Input { jump_pressed: true, jump_held: true, ..IDLE });
        let mut highest = start;
        for _ in 0..90 {
            game.step(Input { jump_held: true, ..IDLE });
            highest = highest.max(position(game).y);
        }
        highest - start
    }

    #[test]
    fn player_stands_still_on_the_ground() {
        let mut game = Game::new();
        run(&mut game, 60, IDLE);
        let start = position(&game);
        run(&mut game, 60, IDLE);
        assert!(position(&game).distance(start) < 1e-3, "drifted from {start} to {}", position(&game));
        assert!(game.player.ground(&game.world).is_some());
    }

    #[test]
    fn a_held_jump_is_higher_than_a_tap_but_cannot_reach_the_ledge() {
        let mut game = Game::new();
        run(&mut game, 30, IDLE);
        let held = jump_height(&mut game);
        let start = position(&game).y;
        game.step(Input { jump_pressed: true, jump_held: true, ..IDLE });
        let mut tap = 0.0_f32;
        for _ in 0..90 {
            game.step(IDLE);
            tap = tap.max(position(&game).y - start);
        }
        assert!(held > 2.5 && held < 3.3, "a held jump rose {held} m");
        assert!(tap < 0.6 * held, "a tap rose {tap} m, a held jump {held} m");
    }

    #[test]
    fn one_way_platforms_are_jumped_through_and_stood_on() {
        let mut game = Game::new();
        // Under the first platform, whose top is at 2.2 m.
        place(&mut game, vec2(10.0, 1.2));
        jump_height(&mut game);
        run(&mut game, 30, IDLE);
        let y = position(&game).y;
        assert!(y > 2.2, "the player should stand on the platform, it is at {y}");
    }

    #[test]
    fn the_moving_platform_carries_the_player_over_the_pit() {
        let mut game = Game::new();
        // Wait at the edge for the platform, step on, and ride it.
        place(&mut game, vec2(27.0, 1.2));
        while game.world[game.level.movers[0].body].position().x > 29.6 {
            game.step(IDLE);
        }
        run(&mut game, 20, RIGHT);
        run(&mut game, 150, IDLE);
        run(&mut game, 40, RIGHT);
        assert_eq!(game.deaths, 0);
        assert!(position(&game).x > 38.0, "the player is at {}", position(&game));
    }

    #[test]
    fn the_box_is_the_way_up_the_ledge() {
        let mut game = Game::new();
        place(&mut game, vec2(41.0, 1.2));
        // Push the box against the ledge, back off, climb on it, then jump to the ledge.
        run(&mut game, 90, RIGHT);
        let box_x = game.world[game.level.boxes[0]].position().x;
        assert!(box_x > 48.5, "the box only went to {box_x}");
        run(&mut game, 25, Input { run: -1.0, ..IDLE });
        // Two jumps to the right, each from the ground: onto the box, then onto the ledge.
        let jump_right = Input { run: 1.0, jump_pressed: true, jump_held: true };
        for _ in 0..2 {
            game.step(jump_right);
            run(&mut game, 10, Input { jump_pressed: false, ..jump_right });
            while game.player.ground(&game.world).is_none() {
                game.step(Input { jump_pressed: false, ..jump_right });
            }
        }
        assert!(position(&game).y > 3.4, "the player did not reach the ledge: {}", position(&game));
    }

    #[test]
    fn spikes_kill_and_the_player_restarts_at_the_checkpoint() {
        let mut game = Game::new();
        place(&mut game, vec2(33.0, -1.0));
        assert_eq!(game.deaths, 1);
        assert!(position(&game).distance(CHECKPOINTS[0]) < 1.0, "respawned at {}", position(&game));
    }

    #[test]
    fn coins_are_collected_once() {
        let mut game = Game::new();
        let total = game.total_coins();
        place(&mut game, vec2(116.0, 1.2));
        run(&mut game, 30, IDLE);
        assert_eq!(game.coins, 1);
        assert_eq!(game.total_coins(), total);
    }
}
