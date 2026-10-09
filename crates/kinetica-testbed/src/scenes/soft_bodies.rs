use std::f32::consts::TAU;

use kinetica::prelude::*;
use kinetica::{Color, DebugDraw};

use super::{Build, Category, Frame, Labels, Scene, SceneUpdate, add_ground};
use kinetica_macroquad::DebugRenderer;

pub const SCENE: Scene = Scene {
    name: "Soft bodies",
    category: Category::SoftStructures,
    description: "Each blob is a ring of balls tied to its center by springs: the softer the springs, the \
                  squishier the blob. From soft (pink) to firm (blue). Grab and throw them.",
    camera_center: vec2(0.0, 5.0),
    view_height: 14.0,
    build: Build::Animated(build),
};

const RING_PARTICLES: usize = 16;
/// Spring frequency of the spokes, from squishy to firm, with each blob's color.
const BLOBS: [(f32, Color); 4] = [
    (2.5, Color::rgb(0.95, 0.45, 0.7)),
    (4.0, Color::rgb(0.95, 0.65, 0.35)),
    (7.0, Color::rgb(0.55, 0.85, 0.45)),
    (12.0, Color::rgb(0.45, 0.65, 0.95)),
];
const SPAWN_PERIOD: f32 = 1.2;
const MAX_BLOBS: usize = 10;

/// A ring of particles around a center body.
struct Blob {
    center: BodyHandle,
    ring: Vec<BodyHandle>,
    color: Color,
    stiffness: f32,
}

impl Blob {
    fn new(world: &mut World, position: Vec2, radius: f32, stiffness: f32, color: Color) -> Self {
        let center = world.add_body(BodyDesc::dynamic().position(position));
        world.add_collider(center, ColliderDesc::circle(0.3 * radius));
        // Particles just touch their neighbors.
        let particle_radius = 0.9 * radius * (TAU / RING_PARTICLES as f32) / 2.0;
        let ring: Vec<BodyHandle> = (0..RING_PARTICLES)
            .map(|i| {
                let angle = TAU * i as f32 / RING_PARTICLES as f32;
                let particle =
                    world.add_body(BodyDesc::dynamic().position(position + radius * Vec2::from_angle(angle)));
                world.add_collider(particle, ColliderDesc::circle(particle_radius).friction(0.8));
                particle
            })
            .collect();

        let link = |world: &mut World, a: BodyHandle, b: BodyHandle, hertz: f32| {
            let (point_a, point_b) = (world.body(a).unwrap().position(), world.body(b).unwrap().position());
            world.add_joint(DistanceJointDesc::between_world_points(world, a, b, point_a, point_b).spring(hertz, 0.4));
        };
        for i in 0..RING_PARTICLES {
            // A stiff skin, some resistance to bending, and the springy spokes.
            link(world, ring[i], ring[(i + 1) % RING_PARTICLES], 4.0 * stiffness);
            link(world, ring[i], ring[(i + 2) % RING_PARTICLES], stiffness);
            link(world, center, ring[i], stiffness);
        }
        Self { center, ring, color, stiffness }
    }

    /// Fills the blob with triangles from its center, which works however much it is squashed.
    fn draw(&self, world: &World, drawer: &mut DebugRenderer) {
        let position = |body: BodyHandle| world.body(body).map(kinetica::Body::position);
        let Some(center) = position(self.center) else { return };
        for i in 0..self.ring.len() {
            if let (Some(a), Some(b)) = (position(self.ring[i]), position(self.ring[(i + 1) % self.ring.len()])) {
                drawer.draw_polygon(&[center, a, b], self.color);
            }
        }
    }
}

fn build(world: &mut World, labels: &mut Labels) -> Box<dyn SceneUpdate> {
    add_ground(world, 20.0);
    let level = world.add_body(BodyDesc::fixed());
    // A ramp down to the right, pegs below it, and walls around a pit.
    world.add_collider(level, ColliderDesc::offset_rectangle(4.0, 0.2, vec2(-4.0, 6.0), -0.35));
    for (x, y) in [(1.0, 3.5), (3.0, 3.0), (5.0, 3.5), (2.0, 1.5), (4.0, 1.5)] {
        world.add_collider(level, ColliderDesc::new(kinetica::shape::Circle::new(0.3).with_center(vec2(x, y))));
    }
    for x in [-9.0, 9.0] {
        world.add_collider(level, ColliderDesc::offset_rectangle(0.3, 5.0, vec2(x, 5.0), 0.0));
    }
    labels.at(vec2(0.0, 10.5), "A ring of balls tied to its center by springs; label: spring frequency");
    Box::new(Blobs { blobs: Vec::new(), next_spawn: 0.0 })
}

struct Blobs {
    blobs: Vec<Blob>,
    next_spawn: f32,
}

impl SceneUpdate for Blobs {
    fn update(&mut self, world: &mut World, frame: &Frame) {
        if self.blobs.len() >= MAX_BLOBS || frame.time < self.next_spawn {
            return;
        }
        self.next_spawn += SPAWN_PERIOD;
        let i = self.blobs.len();
        let (stiffness, color) = BLOBS[i % BLOBS.len()];
        let radius = 0.7 + 0.3 * (i as f32 * 0.618_034).fract();
        self.blobs.push(Blob::new(world, vec2(-6.0, 11.0), radius, stiffness, color));
    }

    fn draw(&self, world: &World, _frame: &Frame, drawer: &mut DebugRenderer) {
        for blob in &self.blobs {
            blob.draw(world, drawer);
        }
    }

    fn labels(&self, world: &World, _frame: &Frame, labels: &mut Labels) {
        for blob in &self.blobs {
            // Above the highest particle of the ring.
            let top = blob
                .ring
                .iter()
                .filter_map(|&body| world.body(body))
                .map(kinetica::Body::position)
                .max_by(|a, b| a.y.total_cmp(&b.y));
            if let Some(top) = top {
                labels.at(vec2(top.x, top.y + 0.3), format!("{} Hz", blob.stiffness));
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Area enclosed by the ring, positive when counter-clockwise.
    fn area(world: &World, blob: &Blob) -> f32 {
        let points: Vec<Vec2> = blob.ring.iter().map(|&body| world.body(body).unwrap().position()).collect();
        (0..points.len()).map(|i| points[i].perp_dot(points[(i + 1) % points.len()])).sum::<f32>() / 2.0
    }

    #[test]
    fn blobs_squash_on_impact_and_recover() {
        let mut world = World::new();
        let mut scene = Blobs { blobs: Vec::new(), next_spawn: 0.0 };
        build(&mut world, &mut Labels::default());
        let mut first_areas = Vec::new();
        for step in 0..20 * 60 {
            let frame = Frame { time: step as f32 / 60.0, ..Frame::default() };
            scene.update(&mut world, &frame);
            world.step(1.0 / 60.0);
            first_areas.push(area(&world, &scene.blobs[0]));
            for blob in &scene.blobs {
                // Squashed under a pile, a soft blob flattens but never turns inside out.
                assert!(area(&world, blob) > 0.2, "a blob turned inside out at step {step}");
            }
        }

        // The first blob (radius 0.7 m, about 1.5 m² at rest) hits the ramp at about 1 s.
        let rest = first_areas[0];
        let squashed = first_areas[..90].iter().copied().fold(f32::INFINITY, f32::min);
        assert!(squashed < 0.6 * rest, "the softest blob barely squashed: {squashed} of {rest}");
        assert!(first_areas[150] > 0.9 * rest, "the blob did not recover its shape: {}", first_areas[150]);

        for blob in &scene.blobs {
            let center = world.body(blob.center).unwrap().position();
            assert!(center.x.abs() < 9.0 && (0.0..10.0).contains(&center.y), "a blob escaped to {center}");
        }
    }
}
