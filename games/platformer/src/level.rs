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
        let Some(body) = world.body_mut(self.body) else { return };
        let target = if self.forward { self.to } else { self.from };
        let offset = target - body.position();
        let direction = (self.to - self.from).normalize() * if self.forward { 1.0 } else { -1.0 };
        if offset.dot(direction) <= 0.0 {
            self.forward = !self.forward;
        }
        let direction = (self.to - self.from).normalize() * if self.forward { 1.0 } else { -1.0 };
        let velocity = self.speed * direction;
        if velocity != body.linear_velocity() {
            body.set_linear_velocity(velocity);
        }
    }
}

#[derive(Debug, Clone, Default)]
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
    pub goal: Option<ColliderHandle>,
}

pub const COIN_RADIUS: f32 = 0.3;
/// Below this height, the player fell out of the level.
pub const FALL_LIMIT: f32 = -8.0;

pub fn build(world: &mut World) -> Level {
    let mut level = Level { spawn: vec2(-6.0, 1.0), ..Level::default() };
    let fixed = world.add_body(BodyDesc::fixed());

    // Terrain, listed left to right so it faces up, walled at both ends. A pit lined with
    // spikes splits it in two.
    level.terrain = vec![
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
    world.add_chain(fixed, &ChainDesc::new(level.terrain.clone()).friction(0.8));
    level
        .spikes
        .push(world.add_collider(fixed, ColliderDesc::offset_rectangle(4.0, 0.3, vec2(12.0, -3.7), 0.0).sensor(true)));

    // One-way platforms.
    for (center, half_width) in [
        (vec2(-2.0, 2.5), 1.5),
        (vec2(1.5, 4.5), 1.5),
        (vec2(-3.0, 6.5), 1.2),
        (vec2(30.0, 6.0), 2.0),
        (vec2(36.0, 4.5), 1.5),
    ] {
        let platform = ColliderDesc::offset_rectangle(half_width, 0.12, center, 0.0).one_way(Vec2::Y).friction(0.8);
        level.platforms.push(world.add_collider(fixed, platform));
    }

    // A ferry over the pit, and an elevator up to the high platform.
    for (from, to, speed) in [(vec2(9.5, 1.0), vec2(14.5, 1.0), 2.0), (vec2(24.0, 3.5), vec2(24.0, 7.0), 1.5)] {
        let body = world.add_body(BodyDesc::kinematic().position(from));
        world.add_collider(body, ColliderDesc::rectangle(1.2, 0.15).friction(0.8));
        level.moving_platforms.push(MovingPlatform { body, from, to, speed, forward: true });
    }

    // Crates to push around or climb on.
    for position in [vec2(19.0, 1.5), vec2(19.0, 2.5), vec2(44.0, 1.0)] {
        let crate_body = world.add_body(BodyDesc::dynamic().position(position));
        world.add_collider(crate_body, ColliderDesc::rectangle(0.45, 0.45).density(1.0).friction(0.8));
        level.crates.push(crate_body);
    }

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
    for position in coin_positions {
        let coin = world.add_body(BodyDesc::fixed().position(position));
        world.add_collider(coin, ColliderDesc::circle(COIN_RADIUS).sensor(true));
        level.coins.push(coin);
    }

    level.goal =
        Some(world.add_collider(fixed, ColliderDesc::offset_rectangle(0.5, 1.5, vec2(50.0, 2.0), 0.0).sensor(true)));
    level
}
