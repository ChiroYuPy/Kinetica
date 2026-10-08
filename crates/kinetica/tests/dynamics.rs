//! Motion of bodies without contacts: gravity, forces, impulses, damping.

use kinetica::prelude::*;

const DT: f32 = 1.0 / 60.0;

fn single_body(desc: BodyDesc) -> (World, BodyHandle) {
    let mut world = World::new();
    let body = world.add_body(desc);
    world.add_collider(body, ColliderDesc::rectangle(0.5, 0.5));
    (world, body)
}

#[test]
fn free_fall_matches_semi_implicit_euler() {
    let start_height = 100.0;
    let (mut world, body) = single_body(BodyDesc::dynamic().position(vec2(0.0, start_height)));
    let gravity = world.gravity().y;

    let steps = 120;
    for _ in 0..steps {
        world.step(DT);
    }

    // Semi-implicit Euler: v_n = n·g·dt and y_n = y_0 + g·dt²·n(n+1)/2.
    let n = steps as f32;
    let body = world.body(body).unwrap();
    assert!((body.linear_velocity().y - n * gravity * DT).abs() < 1e-3);
    let expected_height = start_height + gravity * DT * DT * n * (n + 1.0) / 2.0;
    assert!((body.position().y - expected_height).abs() < 1e-3);
}

#[test]
fn fixed_body_ignores_everything() {
    let (mut world, body) = single_body(BodyDesc::fixed().linear_velocity(vec2(5.0, 0.0)));
    let fixed = world.body_mut(body).unwrap();
    fixed.apply_force(vec2(100.0, 100.0));
    fixed.apply_linear_impulse(vec2(100.0, 100.0));
    fixed.set_angular_velocity(3.0);

    world.step(DT);

    let fixed = world.body(body).unwrap();
    assert_eq!(fixed.position(), Vec2::ZERO);
    assert_eq!(fixed.angle(), 0.0);
    assert_eq!(fixed.mass(), 0.0);
}

#[test]
fn kinematic_body_moves_at_constant_velocity() {
    let velocity = vec2(2.0, -1.0);
    let (mut world, body) = single_body(BodyDesc::kinematic().linear_velocity(velocity));

    for _ in 0..60 {
        world.step(DT);
    }

    let kinematic = world.body(body).unwrap();
    assert_eq!(kinematic.linear_velocity(), velocity, "gravity must not affect kinematic bodies");
    assert!(kinematic.position().abs_diff_eq(velocity, 1e-4));
}

#[test]
fn gravity_scale_zero_floats() {
    let (mut world, body) = single_body(BodyDesc::dynamic().gravity_scale(0.0));
    world.step(DT);
    assert_eq!(world.body(body).unwrap().linear_velocity(), Vec2::ZERO);
}

#[test]
fn force_is_cleared_after_step() {
    let mut world = World::new();
    world.set_gravity(Vec2::ZERO);
    let body = world.add_body(BodyDesc::dynamic());
    world.add_collider(body, ColliderDesc::rectangle(0.5, 0.5)); // 1 kg

    world.body_mut(body).unwrap().apply_force(vec2(60.0, 0.0));
    world.step(DT);
    world.step(DT);

    assert!((world.body(body).unwrap().linear_velocity().x - 1.0).abs() < 1e-5);
}

#[test]
fn off_center_impulse_makes_body_spin() {
    let (mut world, body) = single_body(BodyDesc::dynamic());
    let hit = world.body_mut(body).unwrap();
    hit.apply_linear_impulse_at_point(vec2(0.0, 1.0), vec2(0.5, 0.0));
    assert!(hit.angular_velocity() > 0.0, "pushing up on the right side spins counter-clockwise");

    let fixed_rotation = world.add_body(BodyDesc::dynamic().fixed_rotation(true));
    world.add_collider(fixed_rotation, ColliderDesc::rectangle(0.5, 0.5));
    let hit = world.body_mut(fixed_rotation).unwrap();
    hit.apply_linear_impulse_at_point(vec2(0.0, 1.0), vec2(0.5, 0.0));
    assert_eq!(hit.angular_velocity(), 0.0);
}

#[test]
fn spinning_body_rotates_around_its_center_of_mass() {
    let mut world = World::new();
    world.set_gravity(Vec2::ZERO);
    let body = world.add_body(BodyDesc::dynamic().angular_velocity(1.0));
    // A circle away from the origin puts the center of mass at (2, 0).
    world.add_collider(body, ColliderDesc::new(kinetica::shape::Circle::new(0.5).with_center(vec2(2.0, 0.0))));

    for _ in 0..90 {
        world.step(DT);
    }

    let body = world.body(body).unwrap();
    assert!(body.center_of_mass().abs_diff_eq(vec2(2.0, 0.0), 1e-4));
    assert!((body.position().distance(vec2(2.0, 0.0)) - 2.0).abs() < 1e-4, "the origin orbits the center of mass");
    assert!(body.angle() > 1.0, "the body should have turned");
}

#[test]
fn damping_slows_bodies_down() {
    let (mut world, body) =
        single_body(BodyDesc::dynamic().gravity_scale(0.0).linear_velocity(vec2(10.0, 0.0)).linear_damping(1.0));
    for _ in 0..60 {
        world.step(DT);
    }
    let speed = world.body(body).unwrap().linear_velocity().x;
    assert!(speed > 0.0 && speed < 4.0, "speed after 1 s with damping 1: {speed}");
}

#[test]
fn invalid_time_steps_are_ignored() {
    let (mut world, body) = single_body(BodyDesc::dynamic());
    for dt in [0.0, -1.0, f32::NAN, f32::INFINITY] {
        world.step(dt);
    }
    assert_eq!(world.body(body).unwrap().linear_velocity(), Vec2::ZERO);
}
