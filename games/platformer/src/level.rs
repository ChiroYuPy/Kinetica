//! The level: ground with pits and ledges, one-way platforms, a moving platform and an elevator,
//! a box to push, a plank bridge, coins and spikes.

use kinetica::prelude::*;

/// Crossing this x wins.
pub const GOAL: f32 = 118.0;
/// Where the player starts again after dying: the start, then after each obstacle.
pub const CHECKPOINTS: [Vec2; 4] = [vec2(0.0, 1.2), vec2(40.0, 1.2), vec2(70.0, 8.2), vec2(94.0, 8.2)];
/// Falling below this height kills.
pub const DEATH_HEIGHT: f32 = -5.0;
pub const PLATFORM_THICKNESS: f32 = 0.3;
pub const BOX_SIZE: f32 = 1.2;
pub const COIN_RADIUS: f32 = 0.3;
/// Height of the spikes at the bottom of the pits.
pub const SPIKE_HEIGHT: f32 = 0.5;
const PIT_FLOOR: f32 = -3.0;
const BRIDGE: (f32, f32) = (78.0, 90.0);
const BRIDGE_HEIGHT: f32 = 7.0;
const BRIDGE_PLANKS: usize = 12;

/// The ground line from left to right, walls included; solid below it.
const SURFACE: [Vec2; 22] = [
    vec2(-6.0, 0.0),
    vec2(28.0, 0.0),
    vec2(28.0, PIT_FLOOR),
    vec2(38.0, PIT_FLOOR),
    vec2(38.0, 0.0),
    vec2(50.0, 0.0),
    // A ledge too high to jump on without the box.
    vec2(50.0, 3.4),
    vec2(62.0, 3.4),
    vec2(62.0, 0.0),
    // The elevator shaft, then a cliff.
    vec2(66.0, 0.0),
    vec2(66.0, BRIDGE_HEIGHT),
    vec2(BRIDGE.0, BRIDGE_HEIGHT),
    vec2(BRIDGE.0, PIT_FLOOR),
    vec2(BRIDGE.1, PIT_FLOOR),
    vec2(BRIDGE.1, BRIDGE_HEIGHT),
    vec2(100.0, BRIDGE_HEIGHT),
    // A slope down to the goal.
    vec2(104.0, 5.5),
    vec2(108.0, 2.5),
    vec2(112.0, 0.5),
    vec2(114.0, 0.0),
    vec2(125.0, 0.0),
    vec2(125.0, 12.0),
];

const ONE_WAY_PLATFORMS: [(f32, f32, f32); 3] = [(8.0, 12.0, 2.2), (14.0, 18.0, 4.2), (20.0, 24.0, 2.2)];
const COINS: [Vec2; 13] = [
    vec2(10.0, 3.0),
    vec2(16.0, 5.0),
    vec2(22.0, 3.0),
    vec2(33.0, 1.6),
    vec2(45.0, 3.2),
    vec2(56.0, 4.4),
    vec2(59.0, 4.4),
    vec2(64.0, 4.0),
    vec2(72.0, 8.0),
    vec2(84.0, 7.8),
    vec2(96.0, 8.0),
    vec2(106.0, 5.0),
    vec2(116.0, 1.0),
];

/// A kinematic platform going back and forth between two points.
#[derive(Debug, Clone)]
pub struct Mover {
    pub body: BodyHandle,
    from: Vec2,
    to: Vec2,
    /// Seconds for a return trip.
    period: f32,
}

impl Mover {
    /// Where the platform is at `time`: easing in and out at both ends.
    fn position_at(&self, time: f32) -> Vec2 {
        let along = 0.5 - 0.5 * (std::f32::consts::TAU * time / self.period).cos();
        self.from.lerp(self.to, along)
    }

    /// Sets the velocity that brings the platform where it must be after this step.
    pub fn update(&self, world: &mut World, time: f32, dt: f32) {
        let body = &mut world[self.body];
        let velocity = (self.position_at(time + dt) - body.position()) / dt;
        body.set_linear_velocity(velocity);
    }
}

/// A pit with a row of spikes at the bottom.
#[derive(Debug, Clone)]
pub struct Pit {
    pub left: f32,
    pub right: f32,
    /// Height of the ground on both sides.
    pub rim: f32,
    pub floor: f32,
    /// Covers the spikes.
    pub sensor: ColliderHandle,
}

#[derive(Debug, Clone)]
pub struct Level {
    pub surface: Vec<Vec2>,
    pub one_way: Vec<BodyHandle>,
    pub movers: Vec<Mover>,
    pub planks: Vec<BodyHandle>,
    pub boxes: Vec<BodyHandle>,
    pub coins: Vec<BodyHandle>,
    pub pits: Vec<Pit>,
}

impl Level {
    pub fn new(world: &mut World) -> Self {
        let ground = world.add_body(BodyDesc::fixed());
        // A wall behind the start, then the ground.
        let mut outline = vec![vec2(SURFACE[0].x, 12.0)];
        outline.extend(SURFACE);
        world.add_chain(ground, ChainDesc::new(outline).friction(0.6));

        let one_way = ONE_WAY_PLATFORMS
            .iter()
            .map(|&(left, right, top)| {
                let center = vec2(f32::midpoint(left, right), top - PLATFORM_THICKNESS / 2.0);
                let shape = ColliderDesc::rectangle(right - left, PLATFORM_THICKNESS).one_way(Vec2::Y);
                world.add_body(BodyDesc::fixed().position(center).collider(shape))
            })
            .collect();

        let platform = |world: &mut World, from: Vec2, to: Vec2, width: f32, period: f32| {
            let shape = ColliderDesc::rectangle(width, PLATFORM_THICKNESS).friction(0.8);
            let body = world.add_body(BodyDesc::kinematic().position(from).collider(shape));
            Mover { body, from, to, period }
        };
        let top = -PLATFORM_THICKNESS / 2.0;
        let movers = vec![
            // Across the first pit.
            platform(world, vec2(29.4, top), vec2(36.6, top), 2.6, 5.0),
            // Up the cliff.
            platform(world, vec2(64.0, top), vec2(64.0, BRIDGE_HEIGHT + top), 3.0, 7.0),
        ];

        let shape = ColliderDesc::square(BOX_SIZE).friction(0.6);
        let boxes = vec![world.add_body(BodyDesc::dynamic().position(vec2(44.0, BOX_SIZE / 2.0)).collider(shape))];

        let coins = COINS
            .iter()
            .map(|&center| {
                world.add_body(BodyDesc::fixed().position(center).collider(ColliderDesc::circle(COIN_RADIUS).sensor()))
            })
            .collect();

        let pits = [(28.0, 38.0, 0.0), (BRIDGE.0, BRIDGE.1, BRIDGE_HEIGHT)]
            .into_iter()
            .map(|(left, right, rim)| {
                let strip = ColliderDesc::rectangle(right - left, SPIKE_HEIGHT)
                    .offset(vec2(f32::midpoint(left, right), PIT_FLOOR + SPIKE_HEIGHT / 2.0))
                    .sensor();
                Pit { left, right, rim, floor: PIT_FLOOR, sensor: world.add_collider(ground, strip) }
            })
            .collect();

        Self { surface: SURFACE.to_vec(), one_way, movers, planks: add_bridge(world, ground), boxes, coins, pits }
    }
}

/// Planks hinged end to end across the second pit, hooked to the ground on both sides.
fn add_bridge(world: &mut World, ground: BodyHandle) -> Vec<BodyHandle> {
    let (left, right) = BRIDGE;
    let length = (right - left) / BRIDGE_PLANKS as f32;
    let mut previous = ground;
    let mut planks = Vec::new();
    for i in 0..BRIDGE_PLANKS {
        let start = left + length * i as f32;
        let center = vec2(start + length / 2.0, BRIDGE_HEIGHT - PLATFORM_THICKNESS / 2.0);
        let shape = ColliderDesc::rectangle(length, PLATFORM_THICKNESS).friction(0.8);
        let plank = world.add_body(BodyDesc::dynamic().position(center).collider(shape));
        world.add_joint(RevoluteJointDesc::new(previous, plank, vec2(start, BRIDGE_HEIGHT)));
        previous = plank;
        planks.push(plank);
    }
    world.add_joint(RevoluteJointDesc::new(previous, ground, vec2(right, BRIDGE_HEIGHT)));
    planks
}
