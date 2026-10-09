//! Game rules, independent of input and drawing so they can be tested.

use kinetica::prelude::*;

use crate::level::{Level, Material, TARGET_RADIUS, levels};

pub const TIME_STEP: f32 = 1.0 / 60.0;
/// Where the sling's pouch rests.
pub const SLING: Vec2 = vec2(-4.0, 3.2);
/// How far the pouch can be pulled from its rest position, in meters.
pub const MAX_PULL: f32 = 2.5;
pub const PROJECTILE_RADIUS: f32 = 0.35;
pub const SHOTS_PER_LEVEL: u32 = 4;
/// Launch speed per meter of pull, in m/s.
const LAUNCH_SPEED_PER_METER: f32 = 9.0;
/// Impact speed that pops a target, in m/s.
const TARGET_TOUGHNESS: f32 = 2.5;
const TARGET_POINTS: u32 = 500;
const UNUSED_SHOT_POINTS: u32 = 1000;
/// A shot ends when everything is still, or after this long.
const LONGEST_SHOT: f32 = 8.0;
/// Bodies slower than this, in m/s, count as still.
const STILL_SPEED: f32 = 0.15;
/// How long debris flies before vanishing, in seconds.
pub const DEBRIS_LIFETIME: f32 = 1.2;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum State {
    /// A stone waits in the sling.
    Aiming,
    /// The stone flies and things fall; the next shot waits for everything to settle.
    Shot,
    Won,
    /// Out of stones with targets left.
    Lost,
}

/// What a scrap of paper flying from a broken block or a popped target was.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Scrap {
    Block(Material),
    Target,
}

/// A scrap of paper flying away, only drawn: it does not collide.
#[derive(Debug, Clone, Copy)]
pub struct Debris {
    pub kind: Scrap,
    pub position: Vec2,
    pub velocity: Vec2,
    pub angle: f32,
    pub spin: f32,
    pub size: f32,
    /// Seconds since it broke off.
    pub age: f32,
}

#[derive(Clone)]
pub struct Game {
    pub world: World,
    pub level_index: usize,
    pub level_name: &'static str,
    pub blocks: Vec<(BodyHandle, Material)>,
    pub targets: Vec<BodyHandle>,
    pub projectiles: Vec<BodyHandle>,
    pub debris: Vec<Debris>,
    pub shots_left: u32,
    pub score: u32,
    pub state: State,
    /// Seconds since the last shot.
    shot_time: f32,
    /// Seeds the debris scatter.
    random: u32,
}

impl Game {
    pub fn new(level_index: usize, score: u32) -> Self {
        let level = &levels()[level_index];
        let mut world = World::new();
        world.add_body(BodyDesc::fixed().position(vec2(0.0, -0.5)).collider(ColliderDesc::rectangle(80.0, 1.0)));
        let Level { name, blocks, targets } = level;
        let blocks = blocks.iter().map(|block| (world.add_body(block.desc()), block.material)).collect();
        let targets = targets
            .iter()
            .map(|&center| {
                let ball = ColliderDesc::circle(TARGET_RADIUS).density(0.5).friction(0.7);
                world.add_body(BodyDesc::dynamic().position(center).collider(ball))
            })
            .collect();
        Self {
            world,
            level_index,
            level_name: name,
            blocks,
            targets,
            projectiles: Vec::new(),
            debris: Vec::new(),
            shots_left: SHOTS_PER_LEVEL,
            score,
            state: State::Aiming,
            shot_time: 0.0,
            random: 0x1234_5678,
        }
    }

    pub fn level_count() -> usize {
        levels().len()
    }

    pub fn is_last_level(&self) -> bool {
        self.level_index + 1 == Self::level_count()
    }

    /// The pouch position for a pull towards `point`, kept within reach.
    pub fn pouch(point: Vec2) -> Vec2 {
        SLING + (point - SLING).clamp_length_max(MAX_PULL)
    }

    /// Launch velocity for the pouch pulled to `pouch`: away from the pull, faster the further.
    pub fn launch_velocity(pouch: Vec2) -> Vec2 {
        (SLING - pouch) * LAUNCH_SPEED_PER_METER
    }

    /// Where a stone launched from `pouch` will be at regular times, ignoring what it hits.
    pub fn trajectory(&self, pouch: Vec2) -> impl Iterator<Item = Vec2> {
        let velocity = Self::launch_velocity(pouch);
        let gravity = self.world.gravity();
        (1..16).map(move |i| {
            let t = 0.07 * i as f32;
            pouch + velocity * t + 0.5 * gravity * t * t
        })
    }

    /// Releases the sling pulled to `pouch`.
    pub fn launch(&mut self, pouch: Vec2) {
        if self.state != State::Aiming {
            return;
        }
        let pouch = Self::pouch(pouch);
        let stone = ColliderDesc::circle(PROJECTILE_RADIUS).density(8.0).friction(0.7);
        let desc = BodyDesc::dynamic().position(pouch).linear_velocity(Self::launch_velocity(pouch)).bullet();
        self.projectiles.push(self.world.add_body(desc.collider(stone)));
        self.shots_left -= 1;
        self.shot_time = 0.0;
        self.state = State::Shot;
    }

    pub fn step(&mut self) {
        self.world.step(TIME_STEP);
        self.shot_time += TIME_STEP;
        self.break_what_was_hit();
        self.update_debris();

        if self.state == State::Shot && (self.everything_still() || self.shot_time > LONGEST_SHOT) {
            self.state = if self.targets.is_empty() {
                self.score += self.shots_left * UNUSED_SHOT_POINTS;
                State::Won
            } else if self.shots_left == 0 {
                State::Lost
            } else {
                State::Aiming
            };
        }
    }

    /// Pops the targets and breaks the blocks hit harder than they can take.
    fn break_what_was_hit(&mut self) {
        let mut broken = Vec::new();
        for event in self.world.collision_events() {
            let CollisionEvent::Started { pair, impact_speed } = *event else { continue };
            for body in [pair.body_a, pair.body_b] {
                let toughness = if self.targets.contains(&body) {
                    TARGET_TOUGHNESS
                } else if let Some(&(_, material)) = self.blocks.iter().find(|(block, _)| *block == body) {
                    material.strength()
                } else {
                    continue;
                };
                if impact_speed > toughness && !broken.contains(&body) {
                    broken.push(body);
                }
            }
        }
        for body in broken {
            let kind = if let Some(index) = self.targets.iter().position(|&target| target == body) {
                self.targets.remove(index);
                self.score += TARGET_POINTS;
                Scrap::Target
            } else {
                let index = self.blocks.iter().position(|(block, _)| *block == body).expect("a block");
                let (_, material) = self.blocks.remove(index);
                self.score += material.points();
                Scrap::Block(material)
            };
            self.scatter(body, kind);
            self.world.remove_body(body);
        }
    }

    /// Throws scraps of paper out of a broken body.
    fn scatter(&mut self, body: BodyHandle, kind: Scrap) {
        let body = &self.world[body];
        let (center, velocity) = (body.position(), body.linear_velocity());
        let extent = body.colliders().iter().map(|&collider| self.world[collider].aabb().half_extents()).next();
        let extent = extent.unwrap_or(Vec2::splat(0.3));
        for _ in 0..8 {
            let offset = vec2(self.random(-1.0, 1.0) * extent.x, self.random(-1.0, 1.0) * extent.y);
            let burst = vec2(self.random(-3.0, 3.0), self.random(1.0, 5.0));
            let (angle, spin, size) = (self.random(0.0, 6.0), self.random(-8.0, 8.0), self.random(0.12, 0.3));
            let position = center + offset;
            self.debris.push(Debris { kind, position, velocity: 0.3 * velocity + burst, angle, spin, size, age: 0.0 });
        }
    }

    fn update_debris(&mut self) {
        let gravity = self.world.gravity();
        for debris in &mut self.debris {
            debris.velocity += gravity * TIME_STEP;
            debris.position += debris.velocity * TIME_STEP;
            debris.angle += debris.spin * TIME_STEP;
            debris.age += TIME_STEP;
        }
        self.debris.retain(|debris| debris.age < DEBRIS_LIFETIME);
    }

    fn everything_still(&self) -> bool {
        self.world.bodies().all(|(_, body)| {
            body.is_sleeping()
                || (body.linear_velocity().length() < STILL_SPEED && body.angular_velocity().abs() < STILL_SPEED)
        })
    }

    /// A pseudo-random number between `low` and `high`, the same every game.
    fn random(&mut self, low: f32, high: f32) -> f32 {
        self.random ^= self.random << 13;
        self.random ^= self.random >> 17;
        self.random ^= self.random << 5;
        low + (high - low) * (self.random % 10_000) as f32 / 10_000.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Steps until the shot is over: a shot never lasts longer than `LONGEST_SHOT`.
    fn settle(game: &mut Game) {
        while game.state == State::Shot {
            game.step();
        }
    }

    #[test]
    fn every_level_stands_until_hit() {
        for index in 0..Game::level_count() {
            let mut game = Game::new(index, 0);
            let (blocks, targets) = (game.blocks.len(), game.targets.len());
            for _ in 0..10 * 60 {
                game.step();
            }
            assert_eq!(game.blocks.len(), blocks, "{}: a block broke on its own", game.level_name);
            assert_eq!(game.targets.len(), targets, "{}: a target popped on its own", game.level_name);
            assert_eq!(game.state, State::Aiming);
        }
    }

    /// Plays the shot that pops the most targets among a grid of pulls, `shots` times.
    fn play_greedily(mut game: Game) -> Game {
        while game.state == State::Aiming {
            let mut best: Option<(usize, u32, Game)> = None;
            for angle_step in 0..24 {
                for pull_step in 0..4 {
                    // Pull down and to the left, at angles from 0 to 69° below horizontal.
                    let angle = std::f32::consts::PI + 0.05 * angle_step as f32;
                    let pull = MAX_PULL * (1.0 - 0.15 * pull_step as f32);
                    let mut trial = game.clone();
                    trial.launch(SLING + pull * Vec2::from_angle(angle));
                    settle(&mut trial);
                    let key = (usize::MAX - trial.targets.len(), trial.score);
                    if best.as_ref().is_none_or(|(left, score, _)| key > (*left, *score)) {
                        best = Some((key.0, key.1, trial));
                    }
                }
            }
            game = best.expect("some shot was tried").2;
        }
        game
    }

    #[test]
    fn every_level_can_be_won() {
        for index in 0..Game::level_count() {
            let game = play_greedily(Game::new(index, 0));
            assert_eq!(game.state, State::Won, "{}: {} targets left", game.level_name, game.targets.len());
        }
    }

    #[test]
    fn hard_hits_break_glass_but_not_stone() {
        for (material, breaks) in [(Material::Glass, true), (Material::Stone, false)] {
            let mut world = World::new();
            world.add_body(BodyDesc::fixed().position(vec2(0.0, -0.5)).collider(ColliderDesc::rectangle(80.0, 1.0)));
            let mut game = Game::new(0, 0);
            game.world = world;
            let block = crate::level::Block { center: vec2(5.0, 0.5), size: Vec2::ONE, material };
            game.blocks = vec![(game.world.add_body(block.desc()), material)];
            game.targets.clear();
            game.launch(SLING + vec2(-MAX_PULL, 0.0));
            // Aim straight at the block instead.
            let stone = game.projectiles[0];
            game.world[stone].set_position(vec2(2.0, 0.5));
            game.world[stone].set_linear_velocity(vec2(15.0, 0.0));
            for _ in 0..60 {
                game.step();
            }
            assert_eq!(game.blocks.is_empty(), breaks, "{material:?}");
        }
    }
}
