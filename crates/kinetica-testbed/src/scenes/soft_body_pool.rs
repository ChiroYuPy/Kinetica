use std::f32::consts::TAU;

use kinetica::prelude::*;
use macroquad::color::Color;

use super::soft_bodies::{area, fill_ring};
use super::{Build, Category, Frame, Labels, Scene, SceneUpdate};
use kinetica_macroquad::DebugRenderer;

pub const SCENE: Scene = Scene {
    name: "Soft body pool",
    category: Category::SoftStructures,
    description: "Soft bodies pile up in a pool: skins of welded capsules filled with gas, which flatten under the \
                  pile without collapsing.",
    camera_center: vec2(0.0, 6.5),
    view_height: 17.0,
    build: Build::Animated(build),
};

const BLOB_COUNT: usize = 28;
const THROW_PERIOD: f32 = 0.4;
const RADIUS: f32 = 1.0;
/// Capsules around each body.
const SEGMENT_COUNT: usize = 14;
/// Gas pressure at the rest area, in N/m: the outward force on each meter of skin.
const PRESSURE: f32 = 40.0;
/// Air drag. It also calms the skin, which the pressure, updated once per step, would otherwise
/// keep shaking.
const DAMPING: f32 = 1.0;

const COLORS: [Color; 5] = [
    Color::new(0.95, 0.45, 0.7, 1.0),
    Color::new(0.45, 0.75, 0.95, 1.0),
    Color::new(0.55, 0.85, 0.45, 1.0),
    Color::new(1.0, 0.75, 0.35, 1.0),
    Color::new(0.7, 0.55, 0.95, 1.0),
];

fn build(world: &mut World, _labels: &mut Labels) -> Box<dyn SceneUpdate> {
    Box::new(build_pool(world))
}

fn build_pool(world: &mut World) -> Pool {
    // Down the left wall, along a bumpy floor, up the right wall: the chain faces inward.
    let pool = [
        vec2(-12.0, 14.0),
        vec2(-12.0, 4.0),
        vec2(-9.0, 1.0),
        vec2(-5.5, 0.3),
        vec2(-3.0, 1.6),
        vec2(-1.0, 0.0),
        vec2(3.0, -0.3),
        vec2(5.5, 1.2),
        vec2(8.5, 0.6),
        vec2(12.0, 5.0),
        vec2(12.0, 14.0),
    ];
    let ground = world.add_body(BodyDesc::fixed());
    world.add_chain(ground, ChainDesc::new(pool).friction(0.6));
    Pool { blobs: Vec::new() }
}

struct Blob {
    /// The skin, counter-clockwise.
    skin: Vec<BodyHandle>,
    rest_area: f32,
}

struct Pool {
    blobs: Vec<Blob>,
}

impl SceneUpdate for Pool {
    /// Inflates every body, then throws a new one every [`THROW_PERIOD`], from each side in turn.
    fn update(&mut self, world: &mut World, frame: &Frame) {
        for blob in &self.blobs {
            inflate(world, blob);
        }
        let thrown = self.blobs.len();
        if thrown >= BLOB_COUNT || frame.time < THROW_PERIOD * thrown as f32 {
            return;
        }
        // Golden ratio steps vary the throws without ever repeating one.
        let scatter = (thrown as f32 * 0.618_034).fract();
        let side = if thrown.is_multiple_of(2) { -1.0 } else { 1.0 };
        let position = vec2(side * 9.5, 12.0);
        let velocity = vec2(-side * (3.0 + 6.0 * scatter), 2.0 + 4.0 * scatter);
        self.blobs.push(add_blob(world, position, velocity));
    }

    fn draw(&self, world: &World, _frame: &Frame, drawer: &mut DebugRenderer) {
        for (i, blob) in self.blobs.iter().enumerate() {
            let skin: Vec<Vec2> = blob.skin.iter().map(|&body| world[body].position()).collect();
            let middle = skin.iter().sum::<Vec2>() / skin.len() as f32;
            fill_ring(drawer, middle, &skin, COLORS[i % COLORS.len()]);
        }
    }
}

/// Pushes every piece of skin outwards, harder as the body is squeezed (pressure times area
/// stays constant). Forces last one step, so this runs before each one.
fn inflate(world: &mut World, blob: &Blob) {
    let pressure = PRESSURE * blob.rest_area / area(world, &blob.skin).max(0.1 * blob.rest_area);
    let count = blob.skin.len();
    for i in 0..count {
        let (a, b) = (blob.skin[i], blob.skin[(i + 1) % count]);
        let edge = world[b].position() - world[a].position();
        // Counter-clockwise: outwards is to the right of each edge.
        let force = pressure * vec2(edge.y, -edge.x);
        world[a].apply_force(0.5 * force);
        world[b].apply_force(0.5 * force);
    }
}

/// A round soft body moving at `velocity`: capsules welded end to end around `center`.
fn add_blob(world: &mut World, center: Vec2, velocity: Vec2) -> Blob {
    let corner = |i: usize| center + RADIUS * Vec2::from_angle(TAU * i as f32 / SEGMENT_COUNT as f32);
    let skin: Vec<BodyHandle> = (0..SEGMENT_COUNT)
        .map(|i| {
            let (start, end) = (corner(i), corner(i + 1));
            let middle = (start + end) / 2.0;
            let capsule = ColliderDesc::capsule(start - middle, end - middle, 0.12).friction(0.6);
            let body = BodyDesc::dynamic().position(middle).linear_velocity(velocity).linear_damping(DAMPING);
            world.add_body(body.collider(capsule))
        })
        .collect();
    for i in 0..SEGMENT_COUNT {
        // The angular spring keeps the body round when nothing presses on it.
        let weld = WeldJointDesc::new(skin[i], skin[(i + 1) % SEGMENT_COUNT], corner(i + 1)).angular_spring(4.0, 0.5);
        world.add_joint(weld);
    }
    let rest_area = area(world, &skin);
    Blob { skin, rest_area }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn piled_up_bodies_keep_their_shape() {
        let mut world = World::new();
        let mut scene = build_pool(&mut world);
        for step in 0..30 * 60 {
            let frame = Frame { time: step as f32 / 60.0, ..Frame::default() };
            scene.update(&mut world, &frame);
            world.step(1.0 / 60.0);
        }
        assert_eq!(scene.blobs.len(), BLOB_COUNT);
        for blob in &scene.blobs {
            let squash = area(&world, &blob.skin) / blob.rest_area;
            assert!(squash > 0.5 && squash < 1.2, "a body kept {squash} of its area");
            for &piece in &blob.skin {
                let position = world[piece].position();
                assert!(position.x.abs() < 12.0 && position.y > -0.5 && position.y < 14.0, "skin at {position}");
                assert!(world[piece].linear_velocity().length() < 0.5, "the pile is still shaking");
            }
        }
    }
}
