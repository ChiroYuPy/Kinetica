//! Game rules, independent of input and drawing so they can be tested.

use kinetica::prelude::*;

use crate::table::{BALL_START, Table};

pub const TIME_STEP: f32 = 1.0 / 60.0;
pub const BALLS_PER_GAME: u32 = 3;
const ROLLOVER_POINTS: u32 = 500;
/// Scored when all three rollovers are lit; they then go dark again.
pub const ROLLOVER_BONUS: u32 = 5000;
/// How long a kicker stays lit after a hit, in seconds.
pub const FLASH_DURATION: f32 = 0.15;

/// What the player holds down this step.
#[derive(Debug, Clone, Copy, Default)]
pub struct Input {
    pub left_flipper: bool,
    pub right_flipper: bool,
    pub plunger: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum State {
    Playing,
    /// Every ball drained.
    GameOver,
}

/// A kicker hit recently, drawn lit.
#[derive(Debug, Clone, Copy)]
pub struct Flash {
    pub collider: ColliderHandle,
    /// Seconds since the hit.
    pub age: f32,
}

#[derive(Clone)]
pub struct Game {
    pub world: World,
    pub table: Table,
    /// The ball in play; `None` once the game is over.
    pub ball: Option<BodyHandle>,
    /// Number of the ball in play, from 1.
    pub ball_number: u32,
    pub score: u32,
    pub rollovers_lit: [bool; 3],
    pub flashes: Vec<Flash>,
    pub state: State,
}

impl Game {
    pub fn new() -> Self {
        let mut world = World::new();
        let table = Table::new(&mut world);
        let ball = Table::add_ball(&mut world, BALL_START);
        Self {
            world,
            table,
            ball: Some(ball),
            ball_number: 1,
            score: 0,
            rollovers_lit: [false; 3],
            flashes: Vec::new(),
            state: State::Playing,
        }
    }

    pub fn step(&mut self, input: Input) {
        let input = if self.state == State::Playing { input } else { Input::default() };
        let [left, right] = &self.table.flippers;
        left.actuate(&mut self.world, input.left_flipper);
        right.actuate(&mut self.world, input.right_flipper);
        self.table.plunger.pull(&mut self.world, input.plunger);

        self.world.step(TIME_STEP);
        self.kick_the_ball();
        self.handle_sensors();

        for flash in &mut self.flashes {
            flash.age += TIME_STEP;
        }
        self.flashes.retain(|flash| flash.age < FLASH_DURATION);
    }

    /// Bumpers and slingshots hit by the ball send it away, and score.
    fn kick_the_ball(&mut self) {
        let Some(ball) = self.ball else { return };
        let hits: Vec<ColliderHandle> = self
            .world
            .collision_events()
            .iter()
            .filter_map(|event| match event {
                CollisionEvent::Started { pair, .. } => pair.other_collider(ball),
                CollisionEvent::Stopped { .. } => None,
            })
            .collect();
        for collider in hits {
            let Some(kicker) = self.table.kickers.iter().find(|kicker| kicker.collider == collider) else { continue };
            kicker.kick(&mut self.world, ball);
            self.score += kicker.points;
            self.flashes.push(Flash { collider, age: 0.0 });
        }
    }

    fn handle_sensors(&mut self) {
        let Some(ball) = self.ball else { return };
        let entered: Vec<ColliderHandle> = self
            .world
            .sensor_events()
            .iter()
            .filter_map(|event| match event {
                SensorEvent::Entered(pair) if pair.visitor_body == ball => Some(pair.sensor),
                _ => None,
            })
            .collect();
        for sensor in entered {
            if sensor == self.table.drain {
                self.lose_ball(ball);
            } else if let Some(index) = self.table.rollovers.iter().position(|&rollover| rollover == sensor) {
                self.light_rollover(index);
            }
        }
    }

    fn light_rollover(&mut self, index: usize) {
        if self.rollovers_lit[index] {
            return;
        }
        self.rollovers_lit[index] = true;
        self.score += ROLLOVER_POINTS;
        if self.rollovers_lit.iter().all(|&lit| lit) {
            self.score += ROLLOVER_BONUS;
            self.rollovers_lit = [false; 3];
        }
    }

    /// Removes the drained ball and puts the next one on the plunger, if any is left.
    fn lose_ball(&mut self, ball: BodyHandle) {
        self.world.remove_body(ball);
        if self.ball_number < BALLS_PER_GAME {
            self.ball_number += 1;
            self.ball = Some(Table::add_ball(&mut self.world, BALL_START));
        } else {
            self.ball = None;
            self.state = State::GameOver;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::table::BUMPER_POINTS;

    const IDLE: Input = Input { left_flipper: false, right_flipper: false, plunger: false };
    const PULL: Input = Input { plunger: true, ..IDLE };

    /// Runs `steps` steps of 1/60 s.
    fn run(game: &mut Game, steps: usize, input: Input) {
        for _ in 0..steps {
            game.step(input);
        }
    }

    fn ball(game: &Game) -> &kinetica::Body {
        &game.world[game.ball.expect("a ball in play")]
    }

    /// Puts the ball at `position`, at rest.
    fn place_ball(game: &mut Game, position: Vec2) {
        let ball = &mut game.world[game.ball.unwrap()];
        ball.set_position(position);
        ball.set_linear_velocity(Vec2::ZERO);
    }

    /// Pulls the plunger for `pull_steps`, releases it, and returns the highest point the ball
    /// reached in the next `steps`, and where it is then.
    fn launch(game: &mut Game, pull_steps: usize, steps: usize) -> (f32, Vec2) {
        run(game, pull_steps, PULL);
        let mut highest = f32::MIN;
        for _ in 0..steps {
            game.step(IDLE);
            highest = highest.max(ball(game).position().y);
        }
        (highest, ball(game).position())
    }

    #[test]
    fn ball_waits_on_the_plunger() {
        let mut game = Game::new();
        run(&mut game, 120, IDLE);
        assert!(ball(&game).position().distance(BALL_START) < 0.05, "the ball moved to {}", ball(&game).position());
        for flipper in &game.table.flippers {
            assert!(!flipper.is_up(&game.world));
        }
    }

    #[test]
    fn full_pull_launches_the_ball_round_into_the_playfield() {
        let mut game = Game::new();
        let (highest, position) = launch(&mut game, 60, 90);
        assert!(highest > 18.0, "the ball only rose to {highest}");
        assert!(position.x < 4.4, "the ball should have come round into the playfield, it is at {position}");
    }

    #[test]
    fn weak_pull_falls_back_into_the_lane() {
        let mut game = Game::new();
        let (highest, position) = launch(&mut game, 9, 180);
        assert!(highest < 13.0, "a weak pull sent the ball to {highest}");
        assert!(position.x > 4.5 && position.y < 2.5, "the ball should be back on the plunger, it is at {position}");
    }

    #[test]
    fn flippers_swing_up_while_held() {
        let mut game = Game::new();
        let up = |game: &Game| game.table.flippers.each_ref().map(|flipper| flipper.is_up(&game.world));
        run(&mut game, 12, Input { left_flipper: true, ..IDLE });
        assert_eq!(up(&game), [true, false]);
        run(&mut game, 18, Input { right_flipper: true, ..IDLE });
        assert_eq!(up(&game), [false, true]);
    }

    #[test]
    fn a_flipper_sends_the_ball_up_the_table() {
        let mut game = Game::new();
        // Drop the ball on the middle of the resting left flipper.
        place_ball(&mut game, vec2(-1.7, 3.5));
        run(&mut game, 18, IDLE);
        let mut highest = f32::MIN;
        for _ in 0..60 {
            game.step(Input { left_flipper: true, ..IDLE });
            highest = highest.max(ball(&game).position().y);
        }
        assert!(highest > 10.0, "the flipper only sent the ball to {highest}");
    }

    #[test]
    fn bumpers_kick_and_score() {
        let mut game = Game::new();
        // Just above the bottom bumper, a little off center.
        place_ball(&mut game, vec2(0.2, 11.6));
        let mut fastest: f32 = 0.0;
        for _ in 0..30 {
            game.step(IDLE);
            fastest = fastest.max(ball(&game).linear_velocity().length());
        }
        assert!(game.score >= BUMPER_POINTS, "score {}", game.score);
        assert!(fastest > 8.0, "the bumper barely kicked: {fastest} m/s");
    }

    #[test]
    fn three_lit_rollovers_score_a_bonus() {
        let mut game = Game::new();
        for x in [-1.2, 0.0, 1.2] {
            place_ball(&mut game, vec2(x, 18.6));
            run(&mut game, 30, IDLE);
        }
        assert!(game.score >= ROLLOVER_BONUS + 3 * ROLLOVER_POINTS, "score {}", game.score);
        assert_eq!(game.rollovers_lit, [false; 3], "the rollovers should go dark after the bonus");
    }

    #[test]
    fn drained_balls_end_the_game() {
        let mut game = Game::new();
        for number in 1..=BALLS_PER_GAME {
            assert_eq!(game.ball_number, number);
            // Straight down between the flippers.
            place_ball(&mut game, vec2(0.0, 4.0));
            run(&mut game, 90, IDLE);
        }
        assert_eq!(game.state, State::GameOver);
        assert!(game.ball.is_none());
    }

    /// A small deterministic pseudo-random generator, to play without a player.
    struct Dice(u32);

    impl Dice {
        fn roll(&mut self) -> bool {
            self.0 ^= self.0 << 13;
            self.0 ^= self.0 >> 17;
            self.0 ^= self.0 << 5;
            self.0.is_multiple_of(7)
        }
    }

    #[test]
    fn the_ball_never_leaves_the_table() {
        let mut game = Game::new();
        let mut dice = Dice(12345);
        let (mut left, mut right, mut pulling) = (false, false, false);
        for step in 0..120 * 60 {
            // Flip at random, and launch every ball waiting on the plunger with a full pull.
            if dice.roll() {
                left = !left;
            }
            if dice.roll() {
                right = !right;
            }
            if ball(&game).position().distance(BALL_START) < 0.1 {
                pulling = true;
            } else if game.table.plunger.tension(&game.world) > 0.95 {
                pulling = false;
            }
            game.step(Input { left_flipper: left, right_flipper: right, plunger: pulling });
            if game.state == State::GameOver {
                game.state = State::Playing;
                game.ball_number = 1;
                game.ball = Some(Table::add_ball(&mut game.world, BALL_START));
            }
            let position = ball(&game).position();
            assert!(
                (-4.5..5.3).contains(&position.x) && (-3.0..21.0).contains(&position.y),
                "the ball escaped to {position} at step {step}"
            );
        }
    }
}
