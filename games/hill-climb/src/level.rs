//! The track: endless-looking hills that grow steeper with distance, fuel cans and coins.

use kinetica::prelude::*;

/// Where the track ends, in meters from the start.
pub const TRACK_LENGTH: f32 = 1000.0;
/// Distance between the points of the ground chain, in meters.
const POINT_SPACING: f32 = 1.0;
/// Points of the ground chain: the whole track, from a little behind the start to a little
/// past the finish.
const POINT_COUNT: usize = 1020;
/// Flat run-up before the hills start.
const FLAT_START: f32 = 20.0;
/// Distance over which the hills grow to their full height.
const RAMP_UP: f32 = 500.0;
/// The first fuel can, and how much farther each next one is.
const FIRST_FUEL: f32 = 90.0;
const FUEL_SPACING_GROWTH: f32 = 15.0;
const FUEL_SPACING: f32 = 90.0;
const COIN_SPACING: f32 = 1.2;
const COINS_PER_ROW: usize = 5;
const COIN_ROW_SPACING: f32 = 35.0;
pub const COIN_RADIUS: f32 = 0.3;
pub const FUEL_SIZE: f32 = 0.7;

/// Ground height at `x`: a few sine waves, flat at the start and steeper farther on.
pub fn height(x: f32) -> f32 {
    if x <= FLAT_START {
        return 0.0;
    }
    let distance = x - FLAT_START;
    // Hills grow smoothly from nothing, so the ground has no kink where they start.
    let growth = (distance / RAMP_UP).min(1.0);
    let amplitude = 0.3 + 0.7 * growth;
    let fade_in = (distance / 30.0).min(1.0);
    // Long hills, bumps on them, and small ripples; all start at zero height.
    let waves = 6.0 * (0.05 * distance).sin() + 2.0 * (0.13 * distance).sin() + 0.6 * (0.31 * distance).sin();
    fade_in * amplitude * waves
}

#[derive(Debug, Clone)]
pub struct Level {
    /// The ground line, to draw it.
    pub terrain: Vec<Vec2>,
    pub fuel_cans: Vec<BodyHandle>,
    pub coins: Vec<BodyHandle>,
    pub finish: ColliderHandle,
}

pub fn build(world: &mut World) -> Level {
    let ground = world.add_body(BodyDesc::fixed());
    let terrain: Vec<Vec2> = (0..POINT_COUNT)
        .map(|i| {
            let x = -10.0 + POINT_SPACING * i as f32;
            vec2(x, height(x))
        })
        .collect();
    // A wall behind the start, going down to the ground, then the ground left to right so it
    // faces up.
    let mut outline = vec![terrain[0] + vec2(0.0, 10.0)];
    outline.extend(&terrain);
    world.add_chain(ground, ChainDesc::new(outline).friction(0.9));

    let mut fuel_cans = Vec::new();
    let mut x = FIRST_FUEL;
    let mut spacing = FUEL_SPACING;
    while x < TRACK_LENGTH {
        fuel_cans.push(add_pickup(world, vec2(x, height(x) + 1.0), ColliderDesc::square(FUEL_SIZE)));
        x += spacing;
        spacing += FUEL_SPACING_GROWTH;
    }

    let mut coins = Vec::new();
    let mut row = 30.0;
    while row < TRACK_LENGTH {
        for i in 0..COINS_PER_ROW {
            let x = row + COIN_SPACING * i as f32;
            coins.push(add_pickup(world, vec2(x, height(x) + 1.2), ColliderDesc::circle(COIN_RADIUS)));
        }
        row += COIN_ROW_SPACING;
    }

    let finish_line = world.add_body(BodyDesc::fixed().position(vec2(TRACK_LENGTH, height(TRACK_LENGTH) + 5.0)));
    let finish = world.add_collider(finish_line, ColliderDesc::rectangle(0.4, 10.0).sensor());
    Level { terrain, fuel_cans, coins, finish }
}

/// A fixed sensor: things pass through it, and the game notices.
fn add_pickup(world: &mut World, position: Vec2, collider: ColliderDesc) -> BodyHandle {
    world.add_body(BodyDesc::fixed().position(position).collider(collider.sensor()))
}
