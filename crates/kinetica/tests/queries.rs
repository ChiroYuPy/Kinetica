//! Spatial queries: rays, shape casts, point, box and overlap queries.

use std::f32::consts::FRAC_PI_4;

use kinetica::math::{Aabb, Transform};
use kinetica::prelude::*;
use kinetica::shape::{Circle, Polygon, Shape};
use kinetica::{BodyType, RayHit};

fn add(world: &mut World, body_type: BodyType, position: Vec2, collider: ColliderDesc) -> BodyHandle {
    world.add_body(BodyDesc::new(body_type).position(position).collider(collider))
}

/// A deterministic pseudo-random scatter of shapes, for comparisons with brute force.
fn scattered_world() -> World {
    let mut world = World::new();
    let mut seed = 12345_u32;
    let mut random = move || {
        seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        (seed >> 8) as f32 / (1 << 24) as f32
    };
    for i in 0..150 {
        let position = vec2(random() * 40.0 - 20.0, random() * 40.0 - 20.0);
        let body = world.add_body(BodyDesc::dynamic().position(position).angle(random() * 6.0));
        let collider = match i % 3 {
            0 => ColliderDesc::circle(0.2 + random()),
            1 => ColliderDesc::rectangle(0.4 + 2.0 * random(), 0.4 + 2.0 * random()),
            _ => ColliderDesc::regular_polygon(3 + i % 5, 0.3 + random()),
        };
        world.add_collider(body, collider);
    }
    world
}

#[test]
fn ray_hits_a_circle_at_the_expected_point() {
    let mut world = World::new();
    let ball = add(&mut world, BodyType::Fixed, vec2(5.0, 0.0), ColliderDesc::circle(1.0));

    let hit = world.cast_ray(Vec2::ZERO, vec2(10.0, 0.0), QueryFilter::new()).unwrap();
    assert_eq!(hit.body, ball);
    assert!(hit.point.abs_diff_eq(vec2(4.0, 0.0), 1e-5));
    assert!(hit.normal.abs_diff_eq(-Vec2::X, 1e-5));
    assert!((hit.fraction - 0.4).abs() < 1e-6);
}

#[test]
fn ray_hits_a_rotated_box_on_the_right_face() {
    let mut world = World::new();
    world.add_body(BodyDesc::fixed().position(vec2(0.0, 5.0)).angle(FRAC_PI_4).collider(ColliderDesc::square(2.0)));

    // Straight up into a diamond: hits its bottom corner.
    let hit = world.cast_ray(Vec2::ZERO, vec2(0.0, 10.0), QueryFilter::new()).unwrap();
    assert!((hit.point.y - (5.0 - std::f32::consts::SQRT_2)).abs() < 1e-4);

    // Slightly to the side: hits the lower-left face, whose normal points down-left.
    let hit = world.cast_ray(vec2(-0.5, 0.0), vec2(0.0, 10.0), QueryFilter::new()).unwrap();
    assert!(hit.normal.abs_diff_eq(vec2(-1.0, -1.0).normalize(), 1e-4));
}

#[test]
fn rays_starting_inside_or_too_short_miss() {
    let mut world = World::new();
    add(&mut world, BodyType::Fixed, Vec2::ZERO, ColliderDesc::square(2.0));
    let filter = QueryFilter::new();

    assert!(world.cast_ray(Vec2::ZERO, vec2(5.0, 0.0), filter).is_none(), "starts inside");
    assert!(world.cast_ray(vec2(-5.0, 0.0), vec2(3.0, 0.0), filter).is_none(), "stops short");
    assert!(world.cast_ray(vec2(-5.0, 2.0), vec2(10.0, 0.0), filter).is_none(), "passes above");
    assert!(world.cast_ray(vec2(-5.0, 0.0), vec2(-10.0, 0.0), filter).is_none(), "points away");
}

#[test]
fn closest_hit_and_all_hits() {
    let mut world = World::new();
    let near = add(&mut world, BodyType::Fixed, vec2(3.0, 0.0), ColliderDesc::circle(0.5));
    let middle = add(&mut world, BodyType::Dynamic, vec2(6.0, 0.0), ColliderDesc::square(1.0));
    let far = add(&mut world, BodyType::Kinematic, vec2(9.0, 0.0), ColliderDesc::circle(0.5));
    add(&mut world, BodyType::Fixed, vec2(6.0, 3.0), ColliderDesc::circle(0.5));

    let translation = vec2(20.0, 0.0);
    let filter = QueryFilter::new();
    assert_eq!(world.cast_ray(Vec2::ZERO, translation, filter).unwrap().body, near);

    let all: Vec<_> = world.cast_ray_all(Vec2::ZERO, translation, filter).iter().map(|hit| hit.body).collect();
    assert_eq!(all, [near, middle, far]);

    let skip_near = QueryFilter::new().exclude_body(near);
    assert_eq!(world.cast_ray(Vec2::ZERO, translation, skip_near).unwrap().body, middle);
    let only_kinematic = QueryFilter::new().exclude_fixed().exclude_dynamic();
    assert_eq!(world.cast_ray(Vec2::ZERO, translation, only_kinematic).unwrap().body, far);
}

#[test]
fn ray_casts_match_brute_force() {
    let world = scattered_world();
    let filter = QueryFilter::new();
    let mut hit_count = 0;
    for i in 0..100 {
        // From the left edge of the field, fanning out across it.
        let origin = vec2(-25.0, (i as f32 * 0.43) % 30.0 - 15.0);
        let angle = -0.4 + (0.8 * 0.61 * i as f32) % 0.8;
        let translation = 60.0 * Vec2::from_angle(angle);

        let brute_force = world
            .colliders()
            .filter_map(|(handle, collider)| {
                let body = &world[collider.body()];
                let intersection = collider.shape().cast_ray(body.transform(), origin, translation, 1.0)?;
                Some((intersection.fraction, handle))
            })
            .min_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));

        let found = world.cast_ray(origin, translation, filter).map(|hit: RayHit| (hit.fraction, hit.collider));
        assert_eq!(found, brute_force, "ray {i}");
        hit_count += usize::from(found.is_some());
    }
    assert!(hit_count > 50, "only {hit_count} rays hit something: the test would prove little");
}

#[test]
fn point_and_box_queries_match_brute_force() {
    let world = scattered_world();
    let filter = QueryFilter::new();
    let area = Aabb::new(vec2(-5.0, -3.0), vec2(6.0, 4.0));
    let expected: Vec<_> =
        world.colliders().filter(|(_, collider)| collider.aabb().overlaps(&area)).map(|(handle, _)| handle).collect();
    assert_eq!(world.colliders_in_aabb(&area, filter), expected);

    for (handle, collider) in world.colliders() {
        let center = world[collider.body()].center_of_mass();
        assert!(world.colliders_at_point(center, filter).contains(&handle), "a shape contains its center");
    }
}

#[test]
fn overlap_query_ignores_merely_touching_shapes() {
    let mut world = World::new();
    let block = add(&mut world, BodyType::Fixed, Vec2::ZERO, ColliderDesc::square(2.0));
    let probe = Shape::from(Circle::new(0.5));
    let filter = QueryFilter::new();
    let overlapping = |x: f32| world.colliders_overlapping(probe.clone(), vec2(x, 0.0), filter);

    assert_eq!(overlapping(1.4).len(), 1, "overlaps by 10 cm");
    assert!(overlapping(1.6).is_empty(), "10 cm away");
    let block_collider = world[block].colliders()[0];
    assert_eq!(overlapping(0.0), [block_collider], "fully inside");
}

#[test]
fn circle_cast_stops_where_the_circle_touches() {
    let mut world = World::new();
    let wall = add(&mut world, BodyType::Fixed, vec2(10.0, 0.0), ColliderDesc::rectangle(2.0, 10.0));
    let ball = Shape::from(Circle::new(0.5));

    let hit = world.cast_shape(ball.clone(), Transform::IDENTITY, vec2(20.0, 0.0), QueryFilter::new()).unwrap();
    assert_eq!(hit.body, wall);
    // The ball's center stops 0.5 before the wall face at x = 9.
    let stop = 20.0 * hit.fraction;
    assert!((stop - 8.5).abs() < 0.005, "stopped at {stop}");
    assert!(hit.normal.abs_diff_eq(-Vec2::X, 1e-3));
    assert!((hit.point.x - 9.0).abs() < 0.005);
}

#[test]
fn box_cast_hits_a_corner_and_respects_the_filter() {
    let mut world = World::new();
    // A diamond whose left corner is at x = 5 - √2.
    let diamond =
        world.add_body(BodyDesc::fixed().position(vec2(5.0, 0.0)).angle(FRAC_PI_4).collider(ColliderDesc::square(2.0)));
    let crate_shape = Shape::from(Polygon::rectangle(1.0, 1.0));

    let hit = world.cast_shape(crate_shape.clone(), Transform::IDENTITY, vec2(10.0, 0.0), QueryFilter::new()).unwrap();
    let stop = 10.0 * hit.fraction;
    let expected = 5.0 - std::f32::consts::SQRT_2 - 0.5;
    assert!((stop - expected).abs() < 0.005, "stopped at {stop}, expected {expected}");

    let ignore_diamond = QueryFilter::new().exclude_body(diamond);
    assert!(world.cast_shape(crate_shape.clone(), Transform::IDENTITY, vec2(10.0, 0.0), ignore_diamond).is_none());
}

#[test]
fn corner_to_corner_cast_converges() {
    // The hardest case for conservative advancement: the closest features are two vertices,
    // so the distance bound from edge normals is only about 70% of the real distance.
    let mut world = World::new();
    world.add_body(BodyDesc::fixed().position(vec2(5.0, 0.0)).angle(FRAC_PI_4).collider(ColliderDesc::square(2.0)));
    let diamond = Shape::from(Polygon::rectangle(2.0, 2.0));
    let start = Transform::new(Vec2::ZERO, Rot::from_angle(FRAC_PI_4));

    let hit = world.cast_shape(diamond.clone(), start, vec2(10.0, 0.0), QueryFilter::new()).unwrap();
    let expected = 5.0 - 2.0 * std::f32::consts::SQRT_2;
    assert!((10.0 * hit.fraction - expected).abs() < 0.005, "stopped at {}", 10.0 * hit.fraction);
}

#[test]
fn shape_cast_starting_in_contact_hits_at_zero() {
    let mut world = World::new();
    add(&mut world, BodyType::Fixed, Vec2::ZERO, ColliderDesc::square(2.0));
    let ball = Shape::from(Circle::new(0.5));
    let start = Transform::new(vec2(1.2, 0.0), Rot::IDENTITY);

    let hit = world.cast_shape(ball.clone(), start, vec2(5.0, 0.0), QueryFilter::new()).unwrap();
    assert_eq!(hit.fraction, 0.0);
}

#[test]
fn shape_cast_misses_what_is_not_in_the_way() {
    let mut world = World::new();
    add(&mut world, BodyType::Fixed, vec2(5.0, 2.0), ColliderDesc::circle(1.0));
    let ball = Shape::from(Circle::new(0.5));
    // Passes 0.5 m below the obstacle.
    assert!(world.cast_shape(ball.clone(), Transform::IDENTITY, vec2(10.0, 0.0), QueryFilter::new()).is_none());
    // Too short to reach it.
    let aimed = Transform::new(vec2(0.0, 2.0), Rot::IDENTITY);
    assert!(world.cast_shape(ball.clone(), aimed, vec2(3.0, 0.0), QueryFilter::new()).is_none());
}
