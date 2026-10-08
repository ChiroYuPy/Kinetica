//! Continuous collision detection: fast bodies do not tunnel through thin obstacles.

use kinetica::prelude::*;

const DT: f32 = 1.0 / 60.0;
const SECOND: usize = 60;

fn run(world: &mut World, steps: usize) {
    for _ in 0..steps {
        world.step(DT);
    }
}

/// A world without gravity with a thin fixed wall at x = 5.
fn world_with_wall() -> World {
    let mut world = World::new();
    world.set_gravity(Vec2::ZERO);
    let wall = world.add_body(BodyDesc::fixed().position(vec2(5.0, 0.0)));
    world.add_collider(wall, ColliderDesc::rectangle(0.05, 3.0));
    world
}

/// Shoots a small ball towards +X at `speed` (m/s).
fn shoot(world: &mut World, speed: f32, desc: BodyDesc) -> BodyHandle {
    let ball = world.add_body(desc.linear_velocity(vec2(speed, 0.0)));
    world.add_collider(ball, ColliderDesc::circle(0.1));
    ball
}

fn x(world: &World, body: BodyHandle) -> f32 {
    world.body(body).unwrap().position().x
}

#[test]
fn fast_ball_tunnels_without_continuous_collision() {
    let mut world = world_with_wall();
    world.set_continuous_enabled(false);
    let ball = shoot(&mut world, 200.0, BodyDesc::dynamic());
    run(&mut world, SECOND / 2);
    assert!(x(&world, ball) > 5.0, "expected tunneling, the test setup is too easy");
}

#[test]
fn fast_ball_is_stopped_by_a_thin_wall() {
    for speed in [50.0, 200.0, 1000.0] {
        let mut world = world_with_wall();
        let ball = shoot(&mut world, speed, BodyDesc::dynamic());
        run(&mut world, SECOND / 2);
        assert!(x(&world, ball) < 4.95, "at {speed} m/s the ball went through: x = {}", x(&world, ball));
    }
}

#[test]
fn fast_falling_box_lands_on_thin_ground() {
    let mut world = World::new();
    let ground = world.add_body(BodyDesc::fixed());
    world.add_collider(ground, ColliderDesc::rectangle(10.0, 0.05));
    let block = world.add_body(BodyDesc::dynamic().position(vec2(0.0, 20.0)).linear_velocity(vec2(0.0, -120.0)));
    world.add_collider(block, ColliderDesc::rectangle(0.2, 0.2));
    run(&mut world, 2 * SECOND);
    let y = world.body(block).unwrap().position().y;
    assert!((y - 0.25).abs() < 0.02, "box at y = {y}");
}

#[test]
fn only_bullets_are_stopped_by_thin_dynamic_bodies() {
    for bullet in [false, true] {
        let mut world = World::new();
        world.set_gravity(Vec2::ZERO);
        let plank = world.add_body(BodyDesc::dynamic().position(vec2(5.0, 0.0)));
        world.add_collider(plank, ColliderDesc::rectangle(0.05, 1.0));
        let ball = shoot(&mut world, 300.0, BodyDesc::dynamic().bullet(bullet));
        run(&mut world, SECOND / 4);
        let went_through = x(&world, ball) > x(&world, plank);
        assert_eq!(went_through, !bullet, "bullet: {bullet}");
        if bullet {
            assert!(world.body(plank).unwrap().linear_velocity().x > 1.0, "the bullet should push the plank");
        }
    }
}

#[test]
fn fast_body_sliding_on_the_ground_keeps_its_speed() {
    let mut world = World::new();
    world.set_gravity(vec2(0.0, -9.81));
    let ground = world.add_body(BodyDesc::fixed());
    // Several boxes end to end, so the ball crosses seams.
    for i in 0..20 {
        world.add_collider(ground, ColliderDesc::offset_rectangle(2.5, 0.5, vec2(-45.0 + 5.0 * i as f32, -0.5), 0.0));
    }
    let ball = world.add_body(BodyDesc::dynamic().position(vec2(-45.0, 0.25)).linear_velocity(vec2(40.0, 0.0)));
    world.add_collider(ball, ColliderDesc::circle(0.25).friction(0.0));
    run(&mut world, SECOND);
    let ball = world.body(ball).unwrap();
    assert!(ball.linear_velocity().x > 39.0, "the ball slowed down to {}", ball.linear_velocity().x);
    assert!(ball.position().y > 0.2, "the ball sank to {}", ball.position().y);
}

#[test]
fn sensors_and_one_way_platforms_do_not_stop_fast_bodies() {
    let mut world = World::new();
    world.set_gravity(Vec2::ZERO);
    let walls = world.add_body(BodyDesc::fixed());
    world.add_collider(walls, ColliderDesc::offset_rectangle(0.05, 3.0, vec2(3.0, 0.0), 0.0).sensor(true));
    // Blocks only things coming from +X, i.e. moving towards -X.
    world.add_collider(walls, ColliderDesc::offset_rectangle(0.05, 3.0, vec2(6.0, 0.0), 0.0).one_way(Vec2::X));
    let ball = shoot(&mut world, 200.0, BodyDesc::dynamic());
    run(&mut world, SECOND / 4);
    assert!(x(&world, ball) > 10.0, "the ball stopped at {}", x(&world, ball));

    // Coming back, the one-way wall blocks it.
    world.body_mut(ball).unwrap().set_linear_velocity(vec2(-200.0, 0.0));
    run(&mut world, SECOND / 4);
    assert!(x(&world, ball) > 6.0, "the ball went back through the one-way wall: {}", x(&world, ball));
}
