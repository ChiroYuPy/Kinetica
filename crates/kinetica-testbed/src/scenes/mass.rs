use kinetica::prelude::*;
use kinetica_macroquad::DebugRenderer;
use macroquad::color::Color;
use macroquad::input::KeyCode;

use super::{Build, Category, Frame, Labels, Scene, SceneUpdate, add_ground};

pub const SCENE: Scene = Scene {
    name: "Mass and center of mass",
    category: Category::Bodies,
    description: "A body's mass comes from its colliders: density times area. A body can have several colliders; \
                  it then turns around their common center of mass, drawn as a cross. Press T to throw the \
                  hammer again.",
    camera_center: vec2(0.0, 4.0),
    view_height: 15.0,
    build: Build::Animated(build),
};

const PLANK_LENGTH: f32 = 5.0;
const HAMMER_START: Vec2 = vec2(2.8, 1.0);
const CENTER_COLOR: Color = Color::new(1.0, 0.9, 0.3, 1.0);

fn build(world: &mut World, labels: &mut Labels) -> Box<dyn SceneUpdate> {
    add_ground(world, 40.0);

    // Same mass from different sizes: the seesaw stays level.
    let small_dense = ColliderDesc::square(0.5).density(4.0);
    add_seesaw(world, labels, -7.0, ColliderDesc::square(1.0), small_dense);
    labels.at(vec2(-7.0, -1.2), "Same mass:\n1 m² × density 1 = 0.25 m² × density 4");
    // Same size, different densities: it tips.
    let heavy = ColliderDesc::square(1.0).density(3.0);
    add_seesaw(world, labels, -1.5, ColliderDesc::square(1.0), heavy);
    labels.at(vec2(-1.5, -1.2), "Same size:\ndensity 1 and 3");

    // A hammer: one body, a light handle and a heavy head.
    let handle = ColliderDesc::rectangle(1.6, 0.16);
    let head = ColliderDesc::rectangle(0.3, 0.7).offset(vec2(0.8, 0.0)).density(6.0);
    let hammer = world.add_body(BodyDesc::dynamic().position(HAMMER_START).collider(handle).collider(head));
    throw(world, hammer);
    labels.above(hammer, "One body, two colliders:\nit spins around the cross");

    // Round-bottomed toys: where the mass sits decides whether they stand up.
    add_tumbler_toy(world, labels, vec2(6.3, 0.6), true, "Weighted bottom:\nstands back up");
    add_tumbler_toy(world, labels, vec2(9.3, 0.6), false, "Weighted top:\nfalls over");

    Box::new(MassDemo { hammer })
}

/// A plank on a pivot with a box near each end.
fn add_seesaw(world: &mut World, labels: &mut Labels, x: f32, left: ColliderDesc, right: ColliderDesc) {
    // The post stops short of the plank, which only rests on its pivot.
    let post = world.add_body(BodyDesc::fixed().position(vec2(x, 0.25)).collider(ColliderDesc::rectangle(0.24, 0.5)));
    let pivot = vec2(x, 0.7);
    let plank = world.add_body(
        BodyDesc::dynamic().position(pivot).collider(ColliderDesc::rectangle(PLANK_LENGTH, 0.2).friction(0.9)),
    );
    // Pinned to the post, not the ground: jointed bodies do not collide, and the plank must hit the ground.
    world.add_joint(RevoluteJointDesc::new(post, plank, pivot));

    for (side, collider) in [(-1.0, left), (1.0, right)] {
        let half_height = collider.shape.compute_aabb(&Transform::IDENTITY).max.y;
        let position = pivot + vec2(side * (PLANK_LENGTH / 2.0 - 0.6), 0.1 + half_height);
        let body = world.add_body(BodyDesc::dynamic().position(position).collider(collider.friction(0.9)));
        labels.above(body, format!("{:.1} kg", world[body].mass()));
    }
}

/// A ball with a post on top, tilted, with a weight low in the ball or at the top of the post.
fn add_tumbler_toy(world: &mut World, labels: &mut Labels, position: Vec2, weighted_bottom: bool, text: &str) {
    // Colliders of a body may overlap: their masses simply add up.
    let weight_center = if weighted_bottom { vec2(0.0, -0.3) } else { vec2(0.0, 1.3) };
    world.add_body(
        BodyDesc::dynamic()
            .position(position)
            .angle(0.9)
            .angular_damping(1.5)
            .collider(ColliderDesc::circle(0.6).density(0.5))
            .collider(ColliderDesc::rectangle(0.4, 1.2).offset(vec2(0.0, 0.9)).density(0.5))
            .collider(ColliderDesc::circle(0.25).offset(weight_center).density(20.0)),
    );
    labels.at(position + vec2(0.0, 2.6), text);
}

/// Sends the hammer up from its starting point, spinning.
fn throw(world: &mut World, hammer: BodyHandle) {
    let body = &mut world[hammer];
    body.set_position(HAMMER_START);
    body.set_angle(0.0);
    body.set_linear_velocity(vec2(0.0, 9.0));
    body.set_angular_velocity(7.0);
}

struct MassDemo {
    hammer: BodyHandle,
}

impl SceneUpdate for MassDemo {
    fn update(&mut self, world: &mut World, frame: &Frame) {
        if frame.is_pressed(KeyCode::T) {
            throw(world, self.hammer);
        }
    }

    fn draw(&self, world: &World, _frame: &Frame, drawer: &mut DebugRenderer) {
        for (_, body) in world.bodies().filter(|(_, body)| body.body_type() == BodyType::Dynamic) {
            let center = body.center_of_mass();
            let size = 0.15;
            drawer.draw_segment(center - vec2(size, 0.0), center + vec2(size, 0.0), CENTER_COLOR);
            drawer.draw_segment(center - vec2(0.0, size), center + vec2(0.0, size), CENTER_COLOR);
        }
    }
}
