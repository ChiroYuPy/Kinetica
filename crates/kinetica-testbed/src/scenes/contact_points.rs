use kinetica::prelude::*;
use kinetica_macroquad::DebugRenderer;
use macroquad::color::Color;

use super::{Build, Category, Frame, Labels, Scene, SceneUpdate, add_ground};

pub const SCENE: Scene = Scene {
    name: "Contact points",
    category: Category::Collisions,
    description: "Touching colliders get one or two contact points and a normal, drawn here as long as the force \
                  it carries: the bottom box holds the whole stack.",
    camera_center: vec2(0.0, 2.5),
    view_height: 8.0,
    build: Build::Animated(build),
};

/// Drawn length of a contact normal per newton of contact force, in meters.
const LENGTH_PER_NEWTON: f32 = 0.03;
const MIN_NORMAL_LENGTH: f32 = 0.1;
const POINT_COLOR: Color = Color::new(0.95, 0.25, 0.25, 1.0);
const NORMAL_COLOR: Color = Color::new(0.35, 0.85, 0.45, 1.0);

fn build(world: &mut World, _labels: &mut Labels) -> Box<dyn SceneUpdate> {
    add_ground(world, 14.0);
    for level in 0..4 {
        world
            .add_body(BodyDesc::dynamic().position(vec2(-3.0, 0.5 + level as f32)).collider(ColliderDesc::square(1.0)));
    }
    world.add_body(BodyDesc::dynamic().position(vec2(0.0, 0.6)).collider(ColliderDesc::circle(0.6)));
    // A plank leaning on a box.
    world.add_body(BodyDesc::dynamic().position(vec2(4.0, 0.6)).collider(ColliderDesc::square(1.2)));
    world.add_body(
        BodyDesc::dynamic()
            .position(vec2(2.6, 0.9))
            .angle(0.6)
            .collider(ColliderDesc::rectangle(3.0, 0.2).friction(0.9)),
    );
    Box::new(ContactForces)
}

struct ContactForces;

impl SceneUpdate for ContactForces {
    fn update(&mut self, _world: &mut World, _frame: &Frame) {}

    fn draw(&self, world: &World, _frame: &Frame, drawer: &mut DebugRenderer) {
        // Impulses are those of the last substep.
        let inverse_substep = 60.0 * world.substep_count() as f32;
        for contact in world.contacts() {
            let normal = contact.manifold().normal();
            for point in contact.manifold().points() {
                let position = point.position();
                let force = point.normal_impulse() * inverse_substep;
                let length = (force * LENGTH_PER_NEWTON).max(MIN_NORMAL_LENGTH);
                drawer.draw_segment(position, position + length * normal, NORMAL_COLOR);
                drawer.draw_point(position, POINT_COLOR);
            }
        }
    }
}
