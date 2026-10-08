//! Physical behaviour of the contact solver: resting, stacking, friction, restitution.

use kinetica::prelude::*;

const DT: f32 = 1.0 / 60.0;
/// Number of steps in one simulated second.
const SECOND: usize = 60;

fn world_with_ground() -> World {
    let mut world = World::new();
    let ground = world.add_body(BodyDesc::fixed().position(vec2(0.0, -0.5)));
    world.add_collider(ground, ColliderDesc::rectangle(50.0, 0.5));
    world
}

fn run(world: &mut World, steps: usize) {
    for _ in 0..steps {
        world.step(DT);
    }
}

fn body(world: &World, handle: BodyHandle) -> &kinetica::Body {
    world.body(handle).unwrap()
}

#[test]
fn box_comes_to_rest_on_the_ground() {
    let mut world = world_with_ground();
    let block = world.add_body(BodyDesc::dynamic().position(vec2(0.0, 3.0)));
    world.add_collider(block, ColliderDesc::rectangle(0.5, 0.5));

    run(&mut world, 3 * SECOND);

    let block = body(&world, block);
    assert!(block.linear_velocity().length() < 1e-3, "still moving: {:?}", block.linear_velocity());
    assert!(block.angular_velocity().abs() < 1e-3);
    // Resting boxes sink into the ground by about the linear slop at most.
    assert!((block.position().y - 0.5).abs() < 0.01, "rests at {}", block.position().y);
    assert!(block.angle().abs() < 1e-3);
}

#[test]
fn circle_comes_to_rest_on_the_ground() {
    let mut world = world_with_ground();
    let ball = world.add_body(BodyDesc::dynamic().position(vec2(0.0, 2.0)));
    world.add_collider(ball, ColliderDesc::circle(0.5));

    run(&mut world, 3 * SECOND);

    let ball = body(&world, ball);
    assert!(ball.linear_velocity().length() < 1e-3);
    assert!((ball.position().y - 0.5).abs() < 0.01, "rests at {}", ball.position().y);
}

#[test]
fn tall_stack_stays_upright() {
    let mut world = world_with_ground();
    let height = 10;
    let boxes: Vec<_> = (0..height)
        .map(|i| {
            let block = world.add_body(BodyDesc::dynamic().position(vec2(0.0, 0.5 + i as f32)));
            world.add_collider(block, ColliderDesc::rectangle(0.5, 0.5));
            block
        })
        .collect();

    run(&mut world, 10 * SECOND);

    for (i, &block) in boxes.iter().enumerate() {
        let block = body(&world, block);
        assert!(block.position().x.abs() < 0.01, "box {i} drifted to x = {}", block.position().x);
        assert!((block.position().y - (0.5 + i as f32)).abs() < 0.05, "box {i} at y = {}", block.position().y);
        assert!(block.linear_velocity().length() < 0.01, "box {i} still moving");
    }
}

#[test]
fn pyramid_stays_standing() {
    let mut world = world_with_ground();
    let base = 10;
    let mut top = None;
    for row in 0..base {
        for column in 0..base - row {
            let x = (column as f32 - (base - row - 1) as f32 / 2.0) * 1.05;
            let block = world.add_body(BodyDesc::dynamic().position(vec2(x, 0.5 + row as f32)));
            world.add_collider(block, ColliderDesc::rectangle(0.5, 0.5));
            top = Some(block);
        }
    }

    run(&mut world, 10 * SECOND);

    let top = body(&world, top.unwrap());
    assert!(top.position().x.abs() < 0.05, "top drifted to x = {}", top.position().x);
    assert!((top.position().y - (base as f32 - 0.5)).abs() < 0.1, "top at y = {}", top.position().y);
}

/// A box on a fixed slope of `angle` radians, starting at rest.
fn box_on_slope(angle: f32, friction: f32) -> (World, BodyHandle) {
    let mut world = World::new();
    let slope = world.add_body(BodyDesc::fixed().angle(angle));
    world.add_collider(slope, ColliderDesc::rectangle(20.0, 0.5).friction(friction));
    let up = Rot::from_angle(angle).rotate(Vec2::Y);
    let block = world.add_body(BodyDesc::dynamic().position(up * 1.0).angle(angle));
    world.add_collider(block, ColliderDesc::rectangle(0.5, 0.5).friction(friction));
    (world, block)
}

#[test]
fn friction_holds_a_box_on_a_gentle_slope() {
    // tan(20°) ≈ 0.36 < 0.6: static friction wins.
    let (mut world, block) = box_on_slope(20f32.to_radians(), 0.6);
    run(&mut world, 3 * SECOND);
    let block = body(&world, block);
    assert!(block.position().length() - 1.0 < 0.02, "slid to {:?}", block.position());
    assert!(block.linear_velocity().length() < 0.01);
}

#[test]
fn frictionless_box_slides_down_with_the_expected_acceleration() {
    let angle = 20f32.to_radians();
    let (mut world, block) = box_on_slope(angle, 0.0);
    run(&mut world, SECOND);

    // Without friction the acceleration along the slope is g·sin(angle); this is the speed
    // after one second.
    let expected_speed = 9.81 * angle.sin();
    let speed = body(&world, block).linear_velocity().length();
    assert!((speed - expected_speed).abs() < 0.05 * expected_speed, "speed {speed}, expected {expected_speed}");
}

#[test]
fn bouncy_ball_bounces_back_near_its_drop_height() {
    let mut world = world_with_ground();
    let drop_height = 5.0;
    let ball = world.add_body(BodyDesc::dynamic().position(vec2(0.0, drop_height)));
    world.add_collider(ball, ColliderDesc::circle(0.5).restitution(1.0));

    // Fall for about a second, then track the highest point after the bounce.
    run(&mut world, SECOND + SECOND / 5);
    assert!(body(&world, ball).linear_velocity().y > 0.0, "the ball should be going up after the bounce");
    let mut peak = f32::NEG_INFINITY;
    for _ in 0..120 {
        world.step(DT);
        peak = peak.max(body(&world, ball).position().y);
    }
    assert!(peak > 0.9 * drop_height, "bounced up to {peak} only");
    assert!(peak < 1.02 * drop_height, "gained energy: bounced up to {peak}");
}

#[test]
fn dead_ball_does_not_bounce() {
    let mut world = world_with_ground();
    let ball = world.add_body(BodyDesc::dynamic().position(vec2(0.0, 5.0)));
    world.add_collider(ball, ColliderDesc::circle(0.5).restitution(0.0));

    let mut highest_after_impact = f32::NEG_INFINITY;
    let mut landed = false;
    for _ in 0..180 {
        world.step(DT);
        landed |= world.body_contacts(ball).next().is_some();
        if landed {
            highest_after_impact = highest_after_impact.max(body(&world, ball).position().y);
        }
    }
    assert!(highest_after_impact < 0.6, "rose to {highest_after_impact} after landing");
}

#[test]
fn elastic_head_on_collision_swaps_velocities() {
    let mut world = World::new();
    world.set_gravity(Vec2::ZERO);
    let left = world.add_body(BodyDesc::dynamic().position(vec2(-2.0, 0.0)).linear_velocity(vec2(3.0, 0.0)));
    world.add_collider(left, ColliderDesc::circle(0.5).restitution(1.0).friction(0.0));
    let right = world.add_body(BodyDesc::dynamic().position(vec2(2.0, 0.0)));
    world.add_collider(right, ColliderDesc::circle(0.5).restitution(1.0).friction(0.0));

    run(&mut world, 2 * SECOND);

    let (left, right) = (body(&world, left).linear_velocity(), body(&world, right).linear_velocity());
    assert!(left.length() < 0.1, "left kept {left:?}");
    assert!((right.x - 3.0).abs() < 0.1 && right.y.abs() < 1e-4, "right moves at {right:?}");
}

#[test]
fn momentum_is_conserved_in_collisions() {
    let mut world = World::new();
    world.set_gravity(Vec2::ZERO);
    let heavy = world.add_body(BodyDesc::dynamic().position(vec2(-2.0, 0.1)).linear_velocity(vec2(2.0, 0.0)));
    world.add_collider(heavy, ColliderDesc::rectangle(0.5, 0.5).density(4.0));
    let light = world.add_body(BodyDesc::dynamic().position(vec2(1.0, 0.0)).linear_velocity(vec2(-1.0, 0.5)));
    world.add_collider(light, ColliderDesc::circle(0.4).restitution(0.5));

    let momentum = |world: &World| -> Vec2 { world.bodies().map(|(_, b)| b.mass() * b.linear_velocity()).sum() };
    let before = momentum(&world);
    run(&mut world, 2 * SECOND);
    assert!(momentum(&world).abs_diff_eq(before, 1e-3), "{before:?} became {:?}", momentum(&world));
}

#[test]
fn kinematic_platform_carries_a_box() {
    let mut world = World::new();
    let platform = world.add_body(BodyDesc::kinematic().linear_velocity(vec2(1.0, 0.0)));
    world.add_collider(platform, ColliderDesc::rectangle(3.0, 0.25));
    let block = world.add_body(BodyDesc::dynamic().position(vec2(0.0, 0.75)));
    world.add_collider(block, ColliderDesc::rectangle(0.5, 0.5));

    run(&mut world, 2 * SECOND);

    let block = body(&world, block);
    assert!((block.linear_velocity().x - 1.0).abs() < 0.01, "box moves at {:?}", block.linear_velocity());
    assert!((block.position().y - 0.75).abs() < 0.02);
}
