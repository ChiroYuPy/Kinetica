//! The track: rolling hills, a plank bridge over a pit, a seesaw, crates and a jump, from the
//! start to the flag.

use kinetica::prelude::*;

/// Where the flag stands.
pub const FINISH: f32 = 400.0;
/// Where the car starts again after a crash: the start, then each section's beginning.
pub const CHECKPOINTS: [f32; 5] = [0.0, 118.0, 196.0, 262.0, 304.0];
/// Bottom of the pits.
pub const PIT_DEPTH: f32 = -8.0;
/// The pit the bridge crosses, and the one to jump over.
const BRIDGE_GAP: (f32, f32) = (130.0, 146.0);
const JUMP_GAP: (f32, f32) = (282.0, 290.0);
/// Both pits, from left to right edge.
pub const PITS: [(f32, f32); 2] = [BRIDGE_GAP, JUMP_GAP];
const BRIDGE_PLANKS: usize = 16;
const JUMP_HEIGHT: f32 = 2.4;
const LANDING_HEIGHT: f32 = 1.2;
const SEESAW_PIVOT: Vec2 = vec2(211.0, 0.9);
pub const SEESAW_LENGTH: f32 = 12.0;
pub const PLANK_THICKNESS: f32 = 0.25;
/// Taller than the gap under the car, so a box is never caught between the wheels.
pub const CRATE_SIZE: f32 = 1.0;

/// Ground height at `x`, outside the pits.
pub fn ground_height(x: f32) -> f32 {
    // Hills rising and falling back to flat ground over `start..end`.
    let hills = |start: f32, end: f32, amplitude: f32| {
        let along = (x - start) / (end - start);
        let envelope = (std::f32::consts::PI * along).sin();
        let waves = (0.11 * x).sin() + 0.5 * (0.27 * x + 1.0).sin() + 0.2 * (0.63 * x).sin();
        amplitude * envelope * envelope * waves
    };
    match x {
        x if x < 20.0 => 0.0,
        x if x < 120.0 => hills(20.0, 120.0, 2.5),
        x if x < 152.0 => 0.0,
        x if x < 196.0 => hills(152.0, 196.0, 1.5),
        x if x < 222.0 => 0.0,
        // Three small bumps.
        x if x < 240.0 => 0.15 * (1.0 - (std::f32::consts::TAU * (x - 222.0) / 6.0).cos()),
        x if x < 270.0 => 0.0,
        // The kicker up to the jump.
        x if x <= JUMP_GAP.0 => JUMP_HEIGHT * (x - 270.0) / (JUMP_GAP.0 - 270.0),
        // The landing slope after it.
        x if x < 300.0 => LANDING_HEIGHT * (300.0 - x) / (300.0 - JUMP_GAP.1),
        x if x < 304.0 => 0.0,
        x if x < 390.0 => hills(304.0, 390.0, 2.2),
        _ => 0.0,
    }
}

#[derive(Debug, Clone)]
pub struct Track {
    /// The ground line from left to right, pits included, to draw it.
    pub surface: Vec<Vec2>,
    pub planks: Vec<BodyHandle>,
    pub seesaw: BodyHandle,
    pub pivot: BodyHandle,
    pub crates: Vec<BodyHandle>,
}

impl Track {
    pub fn new(world: &mut World) -> Self {
        let surface = surface();
        let ground = world.add_body(BodyDesc::fixed());
        // A wall behind the start, then the ground: listed left to right, it is solid below.
        let mut outline = vec![vec2(surface[0].x, 20.0)];
        outline.extend(&surface);
        world.add_chain(ground, ChainDesc::new(outline).friction(0.9));
        let (seesaw, pivot) = add_seesaw(world);
        let planks = add_bridge(world, ground);
        Self { surface, planks, seesaw, pivot, crates: add_crates(world) }
    }

    /// Where the car is placed at a checkpoint: above the ground.
    pub fn spawn_point(checkpoint: f32) -> Vec2 {
        vec2(checkpoint, ground_height(checkpoint) + 1.4)
    }
}

/// The ground line, a point per meter, down into the pits and back up.
fn surface() -> Vec<Vec2> {
    let mut points = Vec::new();
    for x in (-10..=410).map(|x| x as f32) {
        let in_pit = |(left, right): (f32, f32)| x > left && x < right;
        if in_pit(BRIDGE_GAP) || in_pit(JUMP_GAP) {
            continue;
        }
        let point = vec2(x, ground_height(x));
        #[allow(clippy::float_cmp, reason = "the gaps start and end on whole meters")]
        if x == BRIDGE_GAP.1 || x == JUMP_GAP.1 {
            points.push(vec2(x, PIT_DEPTH));
        }
        points.push(point);
        #[allow(clippy::float_cmp, reason = "the gaps start and end on whole meters")]
        if x == BRIDGE_GAP.0 || x == JUMP_GAP.0 {
            points.push(vec2(x, PIT_DEPTH));
        }
    }
    points
}

/// Planks hinged end to end across the first pit, hooked to the ground on both banks.
fn add_bridge(world: &mut World, ground: BodyHandle) -> Vec<BodyHandle> {
    let (left, right) = BRIDGE_GAP;
    let length = (right - left) / BRIDGE_PLANKS as f32;
    let mut previous = ground;
    let mut planks = Vec::new();
    for i in 0..BRIDGE_PLANKS {
        let start = left + length * i as f32;
        let plank = world.add_body(
            BodyDesc::dynamic()
                .position(vec2(start + length / 2.0, -PLANK_THICKNESS / 2.0))
                .collider(ColliderDesc::rectangle(length, PLANK_THICKNESS).density(2.0).friction(0.9)),
        );
        world.add_joint(RevoluteJointDesc::new(previous, plank, vec2(start, 0.0)));
        previous = plank;
        planks.push(plank);
    }
    world.add_joint(RevoluteJointDesc::new(previous, ground, vec2(right, 0.0)));
    planks
}

/// A long plank on a pivot, its left end down on the ground; returns the plank and the pivot.
///
/// The pivot is off the plank's middle, so the left end stays down until the car drives past
/// the pivot and tips it over.
fn add_seesaw(world: &mut World) -> (BodyHandle, BodyHandle) {
    let offset = 1.0;
    let tilt = (SEESAW_PIVOT.y / (SEESAW_LENGTH / 2.0 + offset)).asin();
    let center = SEESAW_PIVOT - offset * Vec2::from_angle(tilt);
    let plank = world.add_body(
        BodyDesc::dynamic()
            .position(center)
            .angle(tilt)
            .collider(ColliderDesc::rectangle(SEESAW_LENGTH, PLANK_THICKNESS).density(0.6).friction(0.9)),
    );
    let half = 0.7;
    let pivot = ColliderDesc::convex_hull(&[vec2(-half, 0.0), vec2(half, 0.0), vec2(0.0, SEESAW_PIVOT.y)])
        .expect("a triangle is convex");
    let pivot = world.add_body(BodyDesc::fixed().position(vec2(SEESAW_PIVOT.x, 0.0)).collider(pivot));
    world.add_joint(RevoluteJointDesc::new(pivot, plank, SEESAW_PIVOT));
    (plank, pivot)
}

/// Light cardboard boxes to knock out of the way.
fn add_crates(world: &mut World) -> Vec<BodyHandle> {
    let half = CRATE_SIZE / 2.0;
    [vec2(250.0, half), vec2(250.0 + CRATE_SIZE, half), vec2(250.0 + half, 3.0 * half), vec2(256.0, half)]
        .into_iter()
        .map(|center| {
            let shape = ColliderDesc::square(CRATE_SIZE).density(0.15).friction(0.4);
            world.add_body(BodyDesc::dynamic().position(center).collider(shape))
        })
        .collect()
}
