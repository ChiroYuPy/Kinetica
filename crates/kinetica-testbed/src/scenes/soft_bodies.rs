use std::f32::consts::TAU;

use kinetica::prelude::*;
use macroquad::color::Color;

use super::{Build, Category, Frame, Labels, Scene, SceneUpdate, add_ground};
use kinetica_macroquad::DebugRenderer;

pub const SCENE: Scene = Scene {
    name: "Soft bodies",
    category: Category::SoftStructures,
    description: "Four ways to make a soft body from rigid parts: balls on springs, a skin inflated by \
                  pressure, a lattice of springs, and capsules welded end to end.",
    camera_center: vec2(0.0, 4.0),
    view_height: 15.0,
    build: Build::Animated(build),
};

const DROP_HEIGHT: f32 = 6.5;
const RADIUS: f32 = 1.1;
/// Balls around the blob and the balloon, capsules around the tire.
const RING_COUNT: usize = 20;
/// Gas pressure of the balloon at its rest area, in N/m: the outward force on each meter of skin.
const BALLOON_PRESSURE: f32 = 40.0;
/// Air drag on the light balloon skin. It also calms the skin, which the pressure, updated once
/// per step, would otherwise keep shaking.
const BALLOON_DAMPING: f32 = 2.0;

const BLOB_COLOR: Color = Color::new(0.95, 0.45, 0.7, 1.0);
const BALLOON_COLOR: Color = Color::new(0.45, 0.75, 0.95, 1.0);
const JELLY_COLOR: Color = Color::new(0.55, 0.85, 0.45, 1.0);

fn build(world: &mut World, labels: &mut Labels) -> Box<dyn SceneUpdate> {
    Box::new(build_soft_bodies(world, labels))
}

fn build_soft_bodies(world: &mut World, labels: &mut Labels) -> SoftBodies {
    add_ground(world, 40.0);
    let walls = world.add_body(BodyDesc::fixed());
    for x in [-10.5, 10.5] {
        world.add_collider(walls, ColliderDesc::rectangle(0.5, 4.0).offset(vec2(x, 2.0)));
    }
    for (x, text) in [(-7.5, "Blob"), (-2.5, "Balloon"), (2.5, "Jelly"), (7.5, "Tire")] {
        labels.at(vec2(x, DROP_HEIGHT + 2.0), text);
    }

    let blob = add_blob(world, vec2(-7.5, DROP_HEIGHT));
    let balloon = add_ring(world, vec2(-2.5, DROP_HEIGHT), BALLOON_DAMPING, DistanceJointDesc::rope);
    let jelly = add_jelly(world, vec2(2.5, DROP_HEIGHT));
    add_tire(world, vec2(7.5, DROP_HEIGHT));
    let rest_area = area(world, &balloon);
    SoftBodies { blob, balloon, rest_area, jelly }
}

struct SoftBodies {
    /// The blob's center, then its ring.
    blob: (BodyHandle, Vec<BodyHandle>),
    balloon: Vec<BodyHandle>,
    rest_area: f32,
    /// The jelly's particles, row by row.
    jelly: Vec<Vec<BodyHandle>>,
}

impl SceneUpdate for SoftBodies {
    /// The gas in the balloon pushes every piece of skin outwards, harder as it is squeezed
    /// (pressure times area stays constant). Forces last one step, so this runs before each one.
    fn update(&mut self, world: &mut World, _frame: &Frame) {
        let pressure = BALLOON_PRESSURE * self.rest_area / area(world, &self.balloon).max(0.1 * self.rest_area);
        let count = self.balloon.len();
        for i in 0..count {
            let (a, b) = (self.balloon[i], self.balloon[(i + 1) % count]);
            let edge = world[b].position() - world[a].position();
            // The ring turns counter-clockwise: outwards is to the right of each edge.
            let force = pressure * vec2(edge.y, -edge.x);
            world[a].apply_force(0.5 * force);
            world[b].apply_force(0.5 * force);
        }
    }

    fn draw(&self, world: &World, _frame: &Frame, drawer: &mut DebugRenderer) {
        let position = |body: BodyHandle| world[body].position();
        let (center, ring) = &self.blob;
        fill_ring(drawer, position(*center), &ring.iter().map(|&body| position(body)).collect::<Vec<_>>(), BLOB_COLOR);
        let skin: Vec<Vec2> = self.balloon.iter().map(|&body| position(body)).collect();
        let middle = skin.iter().sum::<Vec2>() / skin.len() as f32;
        fill_ring(drawer, middle, &skin, BALLOON_COLOR);
        for rows in self.jelly.windows(2) {
            for column in 0..rows[0].len() - 1 {
                let cell = [rows[1][column], rows[1][column + 1], rows[0][column + 1], rows[0][column]].map(position);
                drawer.draw_polygon(&cell, JELLY_COLOR);
            }
        }
    }
}

/// Fills a ring with triangles from a point inside it, which works however much it is squashed.
pub(super) fn fill_ring(drawer: &mut DebugRenderer, inside: Vec2, ring: &[Vec2], color: Color) {
    for i in 0..ring.len() {
        drawer.draw_polygon(&[inside, ring[i], ring[(i + 1) % ring.len()]], color);
    }
}

/// Area enclosed by a ring of bodies, positive when counter-clockwise.
pub(super) fn area(world: &World, ring: &[BodyHandle]) -> f32 {
    let points: Vec<Vec2> = ring.iter().map(|&body| world[body].position()).collect();
    (0..points.len()).map(|i| points[i].perp_dot(points[(i + 1) % points.len()])).sum::<f32>() / 2.0
}

/// Ties `a` to `b` with a distance joint of their current distance, shaped by `kind`.
fn tie(world: &mut World, a: BodyHandle, b: BodyHandle, kind: impl Fn(DistanceJointDesc, f32) -> DistanceJointDesc) {
    let (point_a, point_b) = (world[a].position(), world[b].position());
    world.add_joint(kind(DistanceJointDesc::new(a, b, point_a, point_b), point_a.distance(point_b)));
}

fn spring(hertz: f32) -> impl Fn(DistanceJointDesc, f32) -> DistanceJointDesc {
    move |desc, _| desc.spring(hertz, 0.4)
}

/// A ring of touching balls around `center`, counter-clockwise, each tied to the next by `kind`.
fn add_ring(
    world: &mut World,
    center: Vec2,
    linear_damping: f32,
    kind: impl Fn(DistanceJointDesc, f32) -> DistanceJointDesc,
) -> Vec<BodyHandle> {
    let ball_radius = 0.9 * RADIUS * (TAU / RING_COUNT as f32) / 2.0;
    let ring: Vec<BodyHandle> = (0..RING_COUNT)
        .map(|i| {
            let position = center + RADIUS * Vec2::from_angle(TAU * i as f32 / RING_COUNT as f32);
            let ball = ColliderDesc::circle(ball_radius).friction(0.8);
            world.add_body(BodyDesc::dynamic().position(position).linear_damping(linear_damping).collider(ball))
        })
        .collect();
    for i in 0..RING_COUNT {
        tie(world, ring[i], ring[(i + 1) % RING_COUNT], &kind);
    }
    ring
}

/// A ring with a stiff skin, some resistance to bending, and springy spokes to a center body.
fn add_blob(world: &mut World, center: Vec2) -> (BodyHandle, Vec<BodyHandle>) {
    let ring = add_ring(world, center, 0.0, spring(12.0));
    let hub = world.add_body(BodyDesc::dynamic().position(center).collider(ColliderDesc::circle(0.3)));
    for i in 0..RING_COUNT {
        tie(world, ring[i], ring[(i + 2) % RING_COUNT], spring(3.0));
        tie(world, hub, ring[i], spring(3.0));
    }
    (hub, ring)
}

/// A square grid of balls tied to their neighbors and across each cell by springs.
fn add_jelly(world: &mut World, center: Vec2) -> Vec<Vec<BodyHandle>> {
    const SIDE: usize = 5;
    let spacing = 2.0 * RADIUS / (SIDE - 1) as f32;
    let corner = center - Vec2::splat(RADIUS);
    let grid: Vec<Vec<BodyHandle>> = (0..SIDE)
        .map(|row| {
            (0..SIDE)
                .map(|column| {
                    let position = corner + spacing * vec2(column as f32, row as f32);
                    let ball = ColliderDesc::circle(0.45 * spacing).friction(0.8);
                    world.add_body(BodyDesc::dynamic().position(position).collider(ball))
                })
                .collect()
        })
        .collect();
    for row in 0..SIDE {
        for column in 0..SIDE {
            let here = grid[row][column];
            let mut link = |other: BodyHandle| tie(world, here, other, spring(6.0));
            if column + 1 < SIDE {
                link(grid[row][column + 1]);
            }
            if row + 1 < SIDE {
                link(grid[row + 1][column]);
                if column + 1 < SIDE {
                    link(grid[row + 1][column + 1]);
                }
                if column > 0 {
                    link(grid[row + 1][column - 1]);
                }
            }
        }
    }
    grid
}

/// Capsules around a circle, welded end to end: the welds keep the rim from stretching, and
/// their angular springs let it bend and spring back.
fn add_tire(world: &mut World, center: Vec2) {
    let corner = |i: usize| center + RADIUS * Vec2::from_angle(TAU * i as f32 / RING_COUNT as f32);
    let segments: Vec<BodyHandle> = (0..RING_COUNT)
        .map(|i| {
            let (start, end) = (corner(i), corner(i + 1));
            let middle = (start + end) / 2.0;
            let capsule = ColliderDesc::capsule(start - middle, end - middle, 0.12).friction(0.9);
            world.add_body(BodyDesc::dynamic().position(middle).collider(capsule))
        })
        .collect();
    for i in 0..RING_COUNT {
        let weld =
            WeldJointDesc::new(segments[i], segments[(i + 1) % RING_COUNT], corner(i + 1)).angular_spring(20.0, 0.5);
        world.add_joint(weld);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn soft_bodies_land_and_keep_their_shape() {
        let mut world = World::new();
        let mut scene = build_soft_bodies(&mut world, &mut Labels::default());
        let blob_rest = area(&world, &scene.blob.1);
        for step in 0..8 * 60 {
            let frame = Frame { time: step as f32 / 60.0, ..Frame::default() };
            scene.update(&mut world, &frame);
            world.step(1.0 / 60.0);
        }
        for (_, body) in world.bodies().filter(|(_, body)| body.body_type() == BodyType::Dynamic) {
            let position = body.position();
            assert!(position.is_finite() && position.y > -0.1 && position.y < 8.0, "a piece flew to {position}");
        }
        // Resting on the ground, each one sags a little but holds its shape.
        let blob = area(&world, &scene.blob.1) / blob_rest;
        let balloon = area(&world, &scene.balloon) / scene.rest_area;
        assert!(blob > 0.6 && blob < 1.1, "the blob kept {blob} of its area");
        assert!(balloon > 0.8 && balloon < 1.1, "the balloon kept {balloon} of its area");
        let shaking = scene.balloon.iter().map(|&body| world[body].linear_velocity().length()).fold(0.0, f32::max);
        assert!(shaking < 0.2, "the balloon skin still moves at {shaking} m/s");
        let jelly: Vec<Vec2> = scene.jelly.iter().flatten().map(|&body| world[body].position()).collect();
        let (min, max) = jelly.iter().fold((Vec2::MAX, Vec2::MIN), |(min, max), &p| (min.min(p), max.max(p)));
        let ratio = (max.x - min.x) / (max.y - min.y);
        assert!(ratio > 0.8 && ratio < 1.4, "the jelly is {ratio} times wider than tall");
    }
}
