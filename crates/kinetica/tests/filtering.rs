//! Collision layers, sensors and one-way platforms.

use kinetica::prelude::*;
use kinetica::{SensorEvent, SensorPair};

const DT: f32 = 1.0 / 60.0;
const SECOND: usize = 60;

fn world_with_ground() -> (World, BodyHandle) {
    let mut world = World::new();
    let ground = world.add_body(BodyDesc::fixed().position(vec2(0.0, -0.5)));
    world.add_collider(ground, ColliderDesc::rectangle(50.0, 0.5));
    (world, ground)
}

fn run(world: &mut World, steps: usize) {
    for _ in 0..steps {
        world.step(DT);
    }
}

fn position(world: &World, body: BodyHandle) -> Vec2 {
    world.body(body).unwrap().position()
}

// --- Layers ---

const GROUND: u32 = 1 << 0;
const RED: u32 = 1 << 1;
const BLUE: u32 = 1 << 2;

#[test]
fn layers_choose_what_collides() {
    let (mut world, ground) = world_with_ground();
    let ground_collider = world.body(ground).unwrap().colliders()[0];
    world.collider_mut(ground_collider).unwrap().set_layers(CollisionLayers::new(GROUND, u32::MAX));

    // Red boxes collide with the ground and red boxes, blue ones with the ground only.
    let add = |world: &mut World, y: f32, layers: CollisionLayers| {
        let body = world.add_body(BodyDesc::dynamic().position(vec2(0.0, y)));
        world.add_collider(body, ColliderDesc::rectangle(0.5, 0.5).layers(layers));
        body
    };
    let red = CollisionLayers::new(RED, GROUND | RED);
    let blue = CollisionLayers::new(BLUE, GROUND);
    let red_bottom = add(&mut world, 0.5, red);
    let red_top = add(&mut world, 1.5, red);
    let blue_top = add(&mut world, 2.5, blue);
    run(&mut world, 2 * SECOND);

    assert!((position(&world, red_bottom).y - 0.5).abs() < 0.02);
    assert!((position(&world, red_top).y - 1.5).abs() < 0.02, "red boxes stack");
    // The blue box falls through both red boxes, down to the ground.
    assert!((position(&world, blue_top).y - 0.5).abs() < 0.02, "blue box at {}", position(&world, blue_top));
}

#[test]
fn changing_layers_takes_effect_at_the_next_step() {
    let (mut world, _) = world_with_ground();
    let block = world.add_body(BodyDesc::dynamic().position(vec2(0.0, 0.5)));
    let collider = world.add_collider(block, ColliderDesc::rectangle(0.5, 0.5));
    run(&mut world, SECOND);
    assert_eq!(world.body_contacts(block).count(), 1);

    world.collider_mut(collider).unwrap().set_layers(CollisionLayers::NONE);
    world.body_mut(block).unwrap().wake_up();
    world.step(DT);
    assert!(world.collision_events().iter().any(|event| matches!(event, CollisionEvent::Stopped(_))));
    run(&mut world, SECOND);
    assert!(position(&world, block).y < -1.0, "the box should fall through the ground");

    world.collider_mut(collider).unwrap().set_layers(CollisionLayers::ALL);
    let fall = position(&world, block).y;
    world.body_mut(block).unwrap().set_position(vec2(0.0, 2.0));
    run(&mut world, 2 * SECOND);
    assert!((position(&world, block).y - 0.5).abs() < 0.02, "collides again after falling {fall}");
}

#[test]
fn queries_filter_by_layer() {
    let mut world = World::new();
    let body = world.add_body(BodyDesc::fixed());
    world.add_collider(body, ColliderDesc::circle(1.0).layers(CollisionLayers::new(RED, u32::MAX)));
    let hit = |filter: QueryFilter| world.cast_ray(vec2(-5.0, 0.0), vec2(10.0, 0.0), &filter).is_some();
    assert!(hit(QueryFilter::new()));
    assert!(hit(QueryFilter::new().layers(RED | BLUE)));
    assert!(!hit(QueryFilter::new().layers(BLUE)));
}

// --- Sensors ---

fn add_ball(world: &mut World, position: Vec2) -> BodyHandle {
    let ball = world.add_body(BodyDesc::dynamic().position(position));
    world.add_collider(ball, ColliderDesc::circle(0.25));
    ball
}

#[test]
fn sensor_reports_entering_and_exiting_without_pushing() {
    let mut world = World::new();
    let zone = world.add_body(BodyDesc::fixed());
    let sensor = world.add_collider(zone, ColliderDesc::rectangle(2.0, 1.0).sensor(true));
    let ball = add_ball(&mut world, vec2(0.0, 3.0));

    let mut events = Vec::new();
    let mut inside_steps = 0;
    for _ in 0..SECOND {
        world.step(DT);
        events.extend_from_slice(world.sensor_events());
        if world.sensor_overlaps(sensor).count() > 0 {
            inside_steps += 1;
        }
    }

    let pair =
        SensorPair { sensor, sensor_body: zone, visitor: world.body(ball).unwrap().colliders()[0], visitor_body: ball };
    assert_eq!(events, [SensorEvent::Entered(pair), SensorEvent::Exited(pair)]);
    assert!(inside_steps > 5);
    assert_eq!(world.contacts().count(), 0, "sensors make no contacts");
    // Free fall for one second: v = g t.
    let speed = world.body(ball).unwrap().linear_velocity().y;
    assert!((speed + 9.81).abs() < 0.01, "the sensor slowed the ball down: {speed}");
}

#[test]
fn sensors_have_no_mass() {
    let mut world = World::new();
    let body = world.add_body(BodyDesc::dynamic());
    world.add_collider(body, ColliderDesc::circle(0.5));
    let mass = world.body(body).unwrap().mass();
    world.add_collider(body, ColliderDesc::offset_rectangle(2.0, 2.0, vec2(3.0, 0.0), 0.0).sensor(true));
    let body = world.body(body).unwrap();
    assert!((body.mass() - mass).abs() < f32::EPSILON);
    assert_eq!(body.local_center_of_mass(), Vec2::ZERO);
}

#[test]
fn sensors_ignore_their_own_body_and_fixed_bodies_when_fixed() {
    let (mut world, _) = world_with_ground();
    let zone = world.add_body(BodyDesc::fixed());
    let fixed_sensor = world.add_collider(zone, ColliderDesc::rectangle(1.0, 1.0).sensor(true));
    let player = world.add_body(BodyDesc::dynamic().position(vec2(5.0, 0.5)));
    world.add_collider(player, ColliderDesc::rectangle(0.5, 0.5));
    let feet = world.add_collider(player, ColliderDesc::offset_rectangle(0.4, 0.1, vec2(0.0, -0.5), 0.0).sensor(true));
    run(&mut world, 10);

    assert_eq!(world.sensor_overlaps(fixed_sensor).count(), 0, "a fixed sensor ignores the fixed ground");
    let touched: Vec<_> = world.sensor_overlaps(feet).map(|pair| pair.visitor_body).collect();
    assert_eq!(touched.len(), 1, "the feet sensor touches the ground, not the player itself");
    assert_ne!(touched[0], player);
}

#[test]
fn queries_skip_sensors_unless_asked() {
    let mut world = World::new();
    let zone = world.add_body(BodyDesc::fixed());
    world.add_collider(zone, ColliderDesc::circle(1.0).sensor(true));
    assert_eq!(world.colliders_at_point(Vec2::ZERO, &QueryFilter::new()), []);
    assert_eq!(world.colliders_at_point(Vec2::ZERO, &QueryFilter::new().include_sensors()).len(), 1);
}

#[test]
fn sleeping_visitor_stays_inside_without_events() {
    let (mut world, _) = world_with_ground();
    let zone = world.add_body(BodyDesc::fixed());
    let sensor = world.add_collider(zone, ColliderDesc::rectangle(2.0, 2.0).sensor(true));
    let ball = add_ball(&mut world, vec2(0.0, 1.0));
    run(&mut world, 3 * SECOND);
    assert!(world.body(ball).unwrap().is_sleeping());
    assert_eq!(world.sensor_overlaps(sensor).count(), 1);
    assert_eq!(world.sensor_events(), []);
}

// --- One-way platforms ---

/// A thin platform at y = 3 that only blocks from above.
fn add_platform(world: &mut World) {
    let platform = world.add_body(BodyDesc::fixed().position(vec2(0.0, 3.0)));
    world.add_collider(platform, ColliderDesc::rectangle(2.0, 0.1).one_way(Vec2::Y));
}

fn add_box(world: &mut World, desc: BodyDesc) -> BodyHandle {
    let block = world.add_body(desc);
    world.add_collider(block, ColliderDesc::rectangle(0.3, 0.3));
    block
}

#[test]
fn one_way_platform_holds_what_lands_on_it() {
    let (mut world, _) = world_with_ground();
    add_platform(&mut world);
    let block = add_box(&mut world, BodyDesc::dynamic().position(vec2(0.0, 5.0)));
    run(&mut world, 2 * SECOND);
    assert!((position(&world, block).y - 3.4).abs() < 0.02, "box at {}", position(&world, block));
}

#[test]
fn one_way_platform_lets_bodies_jump_through_from_below() {
    let (mut world, _) = world_with_ground();
    add_platform(&mut world);
    // Fast enough to rise about 4.6 m, well above the platform, then land on it.
    let block = add_box(&mut world, BodyDesc::dynamic().position(vec2(0.0, 0.3)).linear_velocity(vec2(0.0, 9.5)));
    run(&mut world, 3 * SECOND);
    assert!((position(&world, block).y - 3.4).abs() < 0.02, "box at {}", position(&world, block));
}

#[test]
fn one_way_platform_lets_bodies_through_from_the_side() {
    let (mut world, _) = world_with_ground();
    add_platform(&mut world);
    let block = add_box(
        &mut world,
        BodyDesc::dynamic().position(vec2(-4.0, 3.0)).linear_velocity(vec2(8.0, 0.0)).gravity_scale(0.0),
    );
    run(&mut world, SECOND);
    assert!(position(&world, block).x > 3.0, "box stopped at {}", position(&world, block));
}

#[test]
fn half_way_jump_falls_back_through() {
    let (mut world, _) = world_with_ground();
    add_platform(&mut world);
    // Rises about 2.9 m: its top enters the platform, but it does not get above.
    let block = add_box(&mut world, BodyDesc::dynamic().position(vec2(0.0, 0.3)).linear_velocity(vec2(0.0, 7.6)));
    run(&mut world, 3 * SECOND);
    assert!((position(&world, block).y - 0.3).abs() < 0.02, "box at {}", position(&world, block));
}
