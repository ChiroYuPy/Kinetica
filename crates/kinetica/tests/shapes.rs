//! Capsules, segments and chains.

use std::f32::consts::PI;

use kinetica::prelude::*;
use kinetica::shape::{Capsule, Shape};

const DT: f32 = 1.0 / 60.0;
const SECOND: usize = 60;

fn run(world: &mut World, steps: usize) {
    for _ in 0..steps {
        world.step(DT);
    }
}

fn body(world: &World, handle: BodyHandle) -> &kinetica::Body {
    world.body(handle).unwrap()
}

/// A flat chain from x = -`half_width` to `half_width` at y = 0, made of `segments` pieces.
fn chain_ground(world: &mut World, half_width: f32, segments: usize) {
    let ground = world.add_body(BodyDesc::fixed());
    let points: Vec<Vec2> =
        (0..=segments).map(|i| vec2(-half_width + 2.0 * half_width * i as f32 / segments as f32, 0.0)).collect();
    world.add_chain(ground, &ChainDesc::new(points));
}

// --- Capsule geometry ---

#[test]
fn capsule_mass_properties_match_numerical_integration() {
    let capsule = Capsule::new(vec2(-0.6, 0.2), vec2(0.4, 0.5), 0.3);
    let shape = Shape::from(capsule);
    let properties = shape.mass_properties(2.0);

    // Integrate over a fine grid of small squares.
    let cell = 0.004;
    let (mut mass, mut moment, mut first) = (0.0_f64, Vec2::ZERO.as_dvec2(), 0.0_f64);
    let mut points = Vec::new();
    for i in -300..300 {
        for j in -300..300 {
            let point = vec2(i as f32 * cell, j as f32 * cell);
            if shape.contains_point(&Transform::IDENTITY, point) {
                points.push(point);
            }
        }
    }
    let cell_mass = 2.0 * f64::from(cell * cell);
    for point in &points {
        mass += cell_mass;
        moment += cell_mass * point.as_dvec2();
    }
    let center = moment / mass;
    for point in &points {
        first += cell_mass * point.as_dvec2().distance_squared(center);
    }

    let expected_mass = 2.0 * (2.0 * 0.3 * capsule.start().distance(capsule.end()) + PI * 0.09);
    assert!((properties.mass - expected_mass).abs() < 1e-4);
    assert!((f64::from(properties.mass) - mass).abs() / mass < 0.01, "{} vs {mass}", properties.mass);
    assert!(properties.center.as_dvec2().distance(center) < 0.005);
    assert!((f64::from(properties.inertia) - first).abs() / first < 0.01, "{} vs {first}", properties.inertia);
}

#[test]
fn rays_hit_capsules_where_brute_force_does() {
    let shape = Shape::from(Capsule::new(vec2(-1.0, 0.0), vec2(1.0, 0.5), 0.4));
    let transform = Transform::new(vec2(0.5, -0.3), Rot::from_angle(0.7));
    for i in 0..60 {
        let angle = i as f32 * 0.37;
        let origin = 4.0 * Vec2::from_angle(angle);
        let translation = 8.0 * (vec2(0.3 * (i as f32).sin(), 0.2) - origin).normalize();
        let hit = shape.cast_ray(&transform, origin, translation, 1.0);

        // March along the ray until the first point inside.
        let steps = 20_000;
        let brute = (0..=steps)
            .map(|k| k as f32 / steps as f32)
            .find(|&t| shape.contains_point(&transform, origin + t * translation));
        match (hit, brute) {
            (Some(hit), Some(t)) => {
                assert!((hit.fraction - t).abs() < 2e-4, "ray {i}: {} vs {t}", hit.fraction);
                // The normal points out of the surface, against the ray.
                assert!(hit.normal.dot(translation) < 0.0);
            }
            (None, None) => {}
            (hit, brute) => panic!("ray {i}: cast {hit:?}, brute force {brute:?}"),
        }
    }
}

// --- Capsules in the world ---

#[test]
fn capsule_lying_and_standing_rest_at_the_right_height() {
    let mut world = World::new();
    let ground = world.add_body(BodyDesc::fixed().position(vec2(0.0, -0.5)));
    world.add_collider(ground, ColliderDesc::rectangle(20.0, 0.5));
    let lying = world.add_body(BodyDesc::dynamic().position(vec2(-3.0, 1.0)));
    world.add_collider(lying, ColliderDesc::capsule(vec2(-1.0, 0.0), vec2(1.0, 0.0), 0.25));
    let standing = world.add_body(BodyDesc::dynamic().position(vec2(3.0, 1.5)).fixed_rotation(true));
    world.add_collider(standing, ColliderDesc::vertical_capsule(0.5, 0.25));
    run(&mut world, 3 * SECOND);

    assert!((body(&world, lying).position().y - 0.25).abs() < 0.01, "{}", body(&world, lying).position());
    assert!(body(&world, lying).angle().abs() < 0.01);
    assert!((body(&world, standing).position().y - 0.75).abs() < 0.01, "{}", body(&world, standing).position());
    assert!(body(&world, lying).is_sleeping() && body(&world, standing).is_sleeping());
}

#[test]
fn standing_capsule_tips_over_and_lies_down() {
    let mut world = World::new();
    chain_ground(&mut world, 10.0, 10);
    let capsule = world.add_body(BodyDesc::dynamic().position(vec2(0.0, 0.76)).angle(0.05));
    world.add_collider(capsule, ColliderDesc::vertical_capsule(0.5, 0.25));
    run(&mut world, 5 * SECOND);
    let capsule = body(&world, capsule);
    assert!((capsule.angle().abs() - PI / 2.0).abs() < 0.02, "angle {}", capsule.angle());
    assert!((capsule.position().y - 0.25).abs() < 0.01);
}

#[test]
fn capsules_stack() {
    let mut world = World::new();
    chain_ground(&mut world, 10.0, 4);
    let capsules: Vec<BodyHandle> = (0..5)
        .map(|i| {
            let capsule = world.add_body(BodyDesc::dynamic().position(vec2(0.0, 0.25 + 0.5 * i as f32)));
            world.add_collider(capsule, ColliderDesc::capsule(vec2(-1.0, 0.0), vec2(1.0, 0.0), 0.25));
            capsule
        })
        .collect();
    run(&mut world, 4 * SECOND);
    for (i, &capsule) in capsules.iter().enumerate() {
        let position = body(&world, capsule).position();
        assert!((position.y - (0.25 + 0.5 * i as f32)).abs() < 0.03, "capsule {i} at {position}");
        assert!(position.x.abs() < 0.05, "capsule {i} slid to {position}");
    }
}

// --- Segments ---

#[test]
fn segments_collide_on_both_sides() {
    let mut world = World::new();
    let shelf = world.add_body(BodyDesc::fixed());
    world.add_collider(shelf, ColliderDesc::segment(vec2(-3.0, 2.0), vec2(3.0, 2.0)));
    let above = world.add_body(BodyDesc::dynamic().position(vec2(0.0, 4.0)));
    world.add_collider(above, ColliderDesc::rectangle(0.5, 0.5));
    let below = world.add_body(BodyDesc::dynamic().position(vec2(1.0, 0.0)).linear_velocity(vec2(0.0, 8.0)));
    world.add_collider(below, ColliderDesc::circle(0.3));
    run(&mut world, 2 * SECOND);

    assert!((body(&world, above).position().y - 2.5).abs() < 0.01, "{}", body(&world, above).position());
    // The ball bounced off the underside and fell away.
    assert!(body(&world, below).position().y < 1.7);
    assert_eq!(world.colliders_at_point(vec2(0.0, 2.0), &QueryFilter::new().exclude_dynamic()), []);
}

// --- Chains ---

#[test]
fn box_slides_across_chain_joints_without_catching() {
    let mut world = World::new();
    // Short segments: the box crosses a joint every 0.25 m.
    chain_ground(&mut world, 20.0, 160);
    let block = world.add_body(BodyDesc::dynamic().position(vec2(-18.0, 0.25)).linear_velocity(vec2(6.0, 0.0)));
    world.add_collider(block, ColliderDesc::rectangle(0.25, 0.25).friction(0.0));
    // Let it settle onto the chain first.
    run(&mut world, SECOND / 4);
    for _ in 0..4 * SECOND {
        world.step(DT);
        let block = body(&world, block);
        assert!(block.linear_velocity().x > 5.95, "the box caught on a joint at {}", block.position());
        assert!(block.linear_velocity().y.abs() < 0.05, "the box bumped at {}", block.position());
        assert!(block.angle().abs() < 0.01);
    }
}

#[test]
fn chains_are_one_sided() {
    let mut world = World::new();
    chain_ground(&mut world, 5.0, 5);
    // Thrown up from below: it passes through, then lands on top.
    let ball = world.add_body(BodyDesc::dynamic().position(vec2(0.0, -1.0)).linear_velocity(vec2(0.0, 8.0)));
    world.add_collider(ball, ColliderDesc::circle(0.25));
    run(&mut world, 3 * SECOND);
    assert!((body(&world, ball).position().y - 0.25).abs() < 0.01, "{}", body(&world, ball).position());
}

#[test]
fn closed_chain_keeps_balls_inside() {
    let mut world = World::new();
    let bowl = world.add_body(BodyDesc::fixed());
    // Counter-clockwise: the inside collides.
    let outline: Vec<Vec2> = (0..24).map(|i| 4.0 * Vec2::from_angle(i as f32 / 24.0 * 2.0 * PI)).collect();
    world.add_chain(bowl, &ChainDesc::new(outline).closed(true).restitution(0.5));
    let balls: Vec<BodyHandle> = (0..20)
        .map(|i| {
            let position = vec2((i % 5) as f32 - 2.0, (i / 5) as f32 - 1.5);
            let ball = world.add_body(BodyDesc::dynamic().position(position).linear_velocity(vec2(5.0, 3.0)));
            world.add_collider(ball, ColliderDesc::circle(0.2));
            ball
        })
        .collect();
    run(&mut world, 5 * SECOND);
    for ball in balls {
        assert!(body(&world, ball).position().length() < 3.85, "a ball escaped to {}", body(&world, ball).position());
    }
}

#[test]
fn circle_rolls_over_hills_and_into_the_valley() {
    let mut world = World::new();
    let ground = world.add_body(BodyDesc::fixed());
    let hills: Vec<Vec2> = (0..=60)
        .map(|i| {
            let x = -15.0 + i as f32 * 0.5;
            vec2(x, (0.5 * x).cos())
        })
        .collect();
    // The lowest point of the chain, near the valley of the curve at x = -2π.
    let lowest = hills.iter().filter(|point| point.x < 0.0).min_by(|a, b| a.y.total_cmp(&b.y)).unwrap().x;
    world.add_chain(ground, &ChainDesc::new(hills));
    // Damped, so it stops rolling back and forth.
    let ball = world.add_body(BodyDesc::dynamic().position(vec2(-5.0, 2.0)).linear_damping(0.5).angular_damping(0.5));
    world.add_collider(ball, ColliderDesc::circle(0.3));
    run(&mut world, 20 * SECOND);
    let x = body(&world, ball).position().x;
    assert!((x - lowest).abs() < 0.05, "the ball rests at x = {x}, not in the valley at {lowest}");
}

#[test]
fn fast_ball_does_not_tunnel_through_a_chain() {
    let mut world = World::new();
    world.set_gravity(Vec2::ZERO);
    let wall = world.add_body(BodyDesc::fixed());
    // Going up, so it faces -X and blocks things coming from the left.
    world.add_chain(wall, &ChainDesc::new([vec2(5.0, -3.0), vec2(5.0, 3.0)]));
    let ball = world.add_body(BodyDesc::dynamic().linear_velocity(vec2(300.0, 0.0)));
    world.add_collider(ball, ColliderDesc::circle(0.1));
    run(&mut world, SECOND / 4);
    assert!(body(&world, ball).position().x < 5.0, "{}", body(&world, ball).position());
}

#[test]
fn rays_hit_segments_and_chains_from_both_sides() {
    let mut world = World::new();
    let walls = world.add_body(BodyDesc::fixed());
    world.add_collider(walls, ColliderDesc::segment(vec2(2.0, -1.0), vec2(2.0, 1.0)));
    world.add_chain(walls, &ChainDesc::new([vec2(-2.0, -1.0), vec2(-2.0, 1.0)]));
    let filter = QueryFilter::new();
    for (direction, wall_x) in [(1.0, 2.0), (-1.0, -2.0)] {
        let hit = world.cast_ray(Vec2::ZERO, vec2(10.0 * direction, 0.0), &filter).unwrap();
        assert!((hit.point.x - wall_x).abs() < 1e-5);
        assert!(hit.normal.abs_diff_eq(vec2(-direction, 0.0), 1e-5));
    }
}
