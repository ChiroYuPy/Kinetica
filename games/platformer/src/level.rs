//! The level: terrain, platforms, crates, coins, spikes and the goal.

use kinetica::prelude::*;

/// A kinematic platform going back and forth between two points.
#[derive(Debug, Clone)]
pub struct MovingPlatform {
    pub body: BodyHandle,
    pub from: Vec2,
    pub to: Vec2,
    /// Travel speed, in m/s.
    pub speed: f32,
    /// Whether it currently heads to `to`.
    pub forward: bool,
}

impl MovingPlatform {
    /// Turns around at the ends of the path.
    pub fn update(&mut self, world: &mut World) {
        let body = &mut world[self.body];
        let target = if self.forward { self.to } else { self.from };
        if (target - body.position()).dot(self.direction()) <= 0.0 {
            self.forward = !self.forward;
        }
        body.set_linear_velocity(self.speed * self.direction());
    }

    /// The unit vector it currently moves along.
    fn direction(&self) -> Vec2 {
        let forward = (self.to - self.from).normalize();
        if self.forward { forward } else { -forward }
    }
}

#[derive(Debug, Clone)]
pub struct Level {
    pub spawn: Vec2,
    /// The ground outline, to draw it filled.
    pub terrain: Vec<Vec2>,
    pub platforms: Vec<ColliderHandle>,
    pub moving_platforms: Vec<MovingPlatform>,
    pub crates: Vec<BodyHandle>,
    /// Each coin is a fixed body holding a sensor.
    pub coins: Vec<BodyHandle>,
    pub spikes: Vec<ColliderHandle>,
    pub goal: ColliderHandle,
}

pub const COIN_RADIUS: f32 = 0.3;
pub const GOAL_HEIGHT: f32 = 3.0;
/// Below this height, the player fell out of the level.
pub const FALL_LIMIT: f32 = -8.0;

pub fn build(world: &mut World) -> Level {
    let fixed = world.add_body(BodyDesc::fixed());

    // Terrain, listed left to right so it faces up, walled at both ends. A pit lined with
    // spikes splits it in two.
    let terrain = vec![
        vec2(-10.0, 12.0),
        vec2(-10.0, 0.0),
        vec2(2.0, 0.0),
        vec2(5.0, 1.0),
        vec2(8.0, 1.0),
        vec2(8.0, -4.0),
        vec2(16.0, -4.0),
        vec2(16.0, 1.0),
        vec2(22.0, 1.0),
        vec2(26.0, 3.0),
        vec2(34.0, 3.0),
        vec2(40.0, 0.5),
        vec2(52.0, 0.5),
        vec2(52.0, 12.0),
    ];
    world.add_chain(fixed, ChainDesc::new(terrain.clone()).friction(0.8));
    let spikes = vec![world.add_collider(fixed, ColliderDesc::rectangle(8.0, 0.6).offset(vec2(12.0, -3.7)).sensor())];

    // One-way platforms.
    let platforms = [
        (vec2(-2.0, 2.5), 3.0),
        (vec2(1.5, 4.5), 3.0),
        (vec2(-3.0, 6.5), 2.4),
        (vec2(30.0, 6.0), 4.0),
        (vec2(36.0, 4.5), 3.0),
    ]
    .map(|(center, width)| {
        let platform = ColliderDesc::rectangle(width, 0.24).offset(center).one_way(Vec2::Y).friction(0.8);
        world.add_collider(fixed, platform)
    });

    // A ferry over the pit, and an elevator up to the high platform.
    let moving_platforms =
        [(vec2(9.5, 1.0), vec2(14.5, 1.0), 2.0), (vec2(24.0, 3.5), vec2(24.0, 7.0), 1.5)].map(|(from, to, speed)| {
            let platform = ColliderDesc::rectangle(2.4, 0.3).friction(0.8);
            let body = world.add_body(BodyDesc::kinematic().position(from).collider(platform));
            MovingPlatform { body, from, to, speed, forward: true }
        });

    // Crates to push around or climb on.
    let crates = [vec2(19.0, 1.5), vec2(19.0, 2.5), vec2(44.0, 1.0)].map(|position| {
        let crate_box = ColliderDesc::square(0.9).friction(0.8);
        world.add_body(BodyDesc::dynamic().position(position).collider(crate_box))
    });

    let coin_positions = [
        vec2(-2.0, 3.5),
        vec2(1.5, 5.5),
        vec2(-3.0, 7.5),
        vec2(12.0, 2.8),
        vec2(19.0, 4.0),
        vec2(24.0, 9.0),
        vec2(30.0, 7.0),
        vec2(36.0, 5.5),
        vec2(46.0, 1.5),
    ];
    let coins = coin_positions.map(|position| {
        world.add_body(BodyDesc::fixed().position(position).collider(ColliderDesc::circle(COIN_RADIUS).sensor()))
    });

    let goal = ColliderDesc::rectangle(1.0, GOAL_HEIGHT).offset(vec2(50.0, 0.5 + GOAL_HEIGHT / 2.0)).sensor();
    let goal = world.add_collider(fixed, goal);

    Level {
        spawn: vec2(-6.0, 1.0),
        terrain,
        platforms: platforms.to_vec(),
        moving_platforms: moving_platforms.to_vec(),
        crates: crates.to_vec(),
        coins: coins.to_vec(),
        spikes,
        goal,
    }
}
