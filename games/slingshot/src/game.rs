//! Game rules, independent of input and drawing so they can be tested.

use kinetica::prelude::*;

use crate::level::{self, LEVEL_COUNT, Level};

/// Where the projectile is launched from.
pub const SLING: Vec2 = vec2(0.0, 2.2);
/// The projectile can be pulled at most this far from the sling.
pub const MAX_PULL: f32 = 2.5;
/// Launch speed per meter of pull, in m/s.
const LAUNCH_SPEED_PER_METER: f32 = 9.0;
pub const PROJECTILE_RADIUS: f32 = 0.3;
/// A target hit at this speed, in m/s, pops.
const POP_IMPACT_SPEED: f32 = 2.0;
/// A target below this height fell off the world, and pops too.
const FALL_LIMIT: f32 = -5.0;
/// How long a popped target stays drawn, in seconds.
pub const POP_DURATION: f32 = 0.5;
const TIME_STEP: f32 = 1.0 / 60.0;
/// Below this speed, in m/s, everything counts as settled.
const SETTLED_SPEED: f32 = 0.1;
const POINTS_PER_TARGET: u32 = 500;
const POINTS_PER_SPARE_SHOT: u32 = 1000;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum State {
    /// Waiting for the player to shoot, or for the scene to settle.
    Playing,
    /// Every target popped.
    Won,
    /// No shot left and targets remain.
    Lost,
}

/// Where a target popped, for a small effect.
#[derive(Debug, Clone, Copy)]
pub struct Pop {
    pub position: Vec2,
    /// Seconds since it popped.
    pub age: f32,
}

#[derive(Clone)]
pub struct Game {
    pub world: World,
    pub level_index: usize,
    pub level: Level,
    pub projectiles: Vec<BodyHandle>,
    pub shots_left: u32,
    pub score: u32,
    pub state: State,
    /// Targets popped recently.
    pub pops: Vec<Pop>,
}

impl Game {
    pub fn new(level_index: usize) -> Self {
        let mut world = World::new();
        let level = level::build(&mut world, level_index);
        let shots_left = level.shots;
        Self {
            world,
            level_index,
            level,
            projectiles: Vec::new(),
            shots_left,
            score: 0,
            state: State::Playing,
            pops: Vec::new(),
        }
    }

    pub fn has_next_level(&self) -> bool {
        self.level_index + 1 < LEVEL_COUNT
    }

    /// The next level, keeping the score; after the last one, a new game from the first.
    pub fn next_level(&self) -> Self {
        if !self.has_next_level() {
            return Self::new(0);
        }
        Self { score: self.score, ..Self::new(self.level_index + 1) }
    }

    /// Launch velocity for the projectile pulled to `pull_point`.
    pub fn launch_velocity(pull_point: Vec2) -> Vec2 {
        Self::clamp_pull(pull_point - SLING) * -LAUNCH_SPEED_PER_METER
    }

    /// Where the projectile sits when pulled towards `point`.
    pub fn pulled_position(point: Vec2) -> Vec2 {
        SLING + Self::clamp_pull(point - SLING)
    }

    fn clamp_pull(pull: Vec2) -> Vec2 {
        pull.clamp_length_max(MAX_PULL)
    }

    pub fn can_shoot(&self) -> bool {
        self.state == State::Playing && self.shots_left > 0
    }

    /// Releases the sling pulled to `pull_point`.
    pub fn shoot(&mut self, pull_point: Vec2) {
        if !self.can_shoot() {
            return;
        }
        self.shots_left -= 1;
        let projectile = BodyDesc::dynamic()
            .position(Self::pulled_position(pull_point))
            .linear_velocity(Self::launch_velocity(pull_point))
            .angular_damping(1.0)
            .bullet()
            .collider(ColliderDesc::circle(PROJECTILE_RADIUS).density(4.0).friction(0.6));
        self.projectiles.push(self.world.add_body(projectile));
    }

    pub fn step(&mut self) {
        self.world.step(TIME_STEP);
        self.pop_hit_targets();
        for pop in &mut self.pops {
            pop.age += TIME_STEP;
        }
        self.pops.retain(|pop| pop.age < POP_DURATION);
        self.update_state();
    }

    /// Targets hit hard, or fallen off the world, pop.
    fn pop_hit_targets(&mut self) {
        let hard_hits: Vec<ContactPair> = self
            .world
            .collision_events()
            .iter()
            .filter_map(|event| match event {
                CollisionEvent::Started { pair, impact_speed } if *impact_speed > POP_IMPACT_SPEED => Some(*pair),
                _ => None,
            })
            .collect();
        let Self { world, level, pops, score, .. } = self;
        level.targets.retain(|&target| {
            let position = world[target].position();
            let popped = hard_hits.iter().any(|pair| pair.involves_body(target)) || position.y < FALL_LIMIT;
            if popped {
                pops.push(Pop { position, age: 0.0 });
                world.remove_body(target);
                *score += POINTS_PER_TARGET;
            }
            !popped
        });
    }

    fn update_state(&mut self) {
        if self.state != State::Playing {
            return;
        }
        if self.level.targets.is_empty() {
            self.state = State::Won;
            self.score += POINTS_PER_SPARE_SHOT * self.shots_left;
        } else if self.shots_left == 0 && self.is_settled() {
            self.state = State::Lost;
        }
    }

    /// Whether everything stopped moving, so the last shot had its full effect.
    pub fn is_settled(&self) -> bool {
        self.world
            .bodies()
            .filter(|(_, body)| body.body_type() == BodyType::Dynamic)
            .all(|(_, body)| body.is_sleeping() || body.linear_velocity().length() < SETTLED_SPEED)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn run(game: &mut Game, seconds: u32) {
        for _ in 0..seconds * 60 {
            game.step();
        }
    }

    #[test]
    fn levels_stand_still_until_shot() {
        for index in 0..LEVEL_COUNT {
            let mut game = Game::new(index);
            let targets = game.level.targets.len();
            run(&mut game, 6);
            assert_eq!(game.level.targets.len(), targets, "level {index} collapses on its own");
            assert!(game.is_settled(), "level {index} is still moving");
        }
    }

    /// Searches a grid of pulls for the shot popping the most targets.
    fn best_shot(game: &Game) -> (Vec2, usize) {
        let mut best = (Vec2::ZERO, 0);
        for angle_step in 0..24 {
            for strength_step in 1..=5 {
                // Pull down and to the left, to shoot up and to the right.
                let angle = std::f32::consts::PI + 0.05 * angle_step as f32;
                let pull = SLING + MAX_PULL * strength_step as f32 / 5.0 * Vec2::from_angle(angle);
                let mut trial = game.clone();
                let before = trial.level.targets.len();
                trial.shoot(pull);
                run(&mut trial, 4);
                let popped = before - trial.level.targets.len();
                if popped > best.1 {
                    best = (pull, popped);
                }
            }
        }
        best
    }

    #[test]
    fn every_level_can_be_won() {
        for index in 0..LEVEL_COUNT {
            let mut game = Game::new(index);
            run(&mut game, 1);
            while game.state == State::Playing && game.shots_left > 0 {
                let (pull, popped) = best_shot(&game);
                assert!(popped > 0, "level {index}: no shot pops anything, {} targets left", game.level.targets.len());
                game.shoot(pull);
                run(&mut game, 4);
            }
            assert_eq!(game.state, State::Won, "level {index} could not be won");
        }
    }
}
