//! World bookkeeping: handles, mass updates and reproducibility.

use kinetica::prelude::*;

#[test]
fn removing_a_body_removes_its_colliders() {
    let mut world = World::new();
    let body = world.add_body(BodyDesc::dynamic());
    let collider = world.add_collider(body, ColliderDesc::circle(1.0));

    assert!(world.remove_body(body).is_some());
    assert!(world.body(body).is_none());
    assert!(world.collider(collider).is_none());
    assert_eq!(world.collider_count(), 0);
}

#[test]
fn stale_handle_does_not_alias_a_new_body() {
    let mut world = World::new();
    let old = world.add_body(BodyDesc::dynamic());
    world.remove_body(old);
    let new = world.add_body(BodyDesc::fixed());

    assert_ne!(old, new);
    assert!(world.body(old).is_none());
    assert!(world.body(new).is_some());
}

#[test]
fn mass_follows_attached_colliders() {
    let mut world = World::new();
    let body = world.add_body(BodyDesc::dynamic());
    let left = world.add_collider(body, ColliderDesc::convex_hull(&square_at(vec2(-1.0, 0.0))).unwrap());
    world.add_collider(body, ColliderDesc::convex_hull(&square_at(vec2(1.0, 0.0))).unwrap().density(3.0));

    let compound = world.body(body).unwrap();
    assert!((compound.mass() - 4.0).abs() < 1e-5);
    // Weighted average of the two square centers: (-1·1 + 1·3) / 4.
    assert!(compound.center_of_mass().abs_diff_eq(vec2(0.5, 0.0), 1e-5));

    world.remove_collider(left);
    let single = world.body(body).unwrap();
    assert!((single.mass() - 3.0).abs() < 1e-5);
    assert!(single.center_of_mass().abs_diff_eq(vec2(1.0, 0.0), 1e-5));
}

#[test]
fn collider_aabb_follows_its_body() {
    let mut world = World::new();
    let body = world.add_body(BodyDesc::kinematic().linear_velocity(vec2(1.0, 0.0)));
    let collider = world.add_collider(body, ColliderDesc::circle(0.5));

    world.step(1.0);

    let aabb = world.collider(collider).unwrap().aabb();
    assert!(aabb.center().abs_diff_eq(vec2(1.0, 0.0), 1e-5));
}

#[test]
fn identical_worlds_stay_bit_identical() {
    let mut original = build_scene();
    for _ in 0..30 {
        original.step(1.0 / 60.0);
    }
    let mut copy = original.clone();

    for _ in 0..120 {
        original.step(1.0 / 60.0);
        copy.step(1.0 / 60.0);
    }

    let states = |world: &World| -> Vec<_> {
        world.bodies().map(|(_, b)| (b.position(), b.angle(), b.linear_velocity(), b.angular_velocity())).collect()
    };
    assert_eq!(states(&original), states(&copy));
}

fn square_at(center: Vec2) -> [Vec2; 4] {
    [vec2(-0.5, -0.5), vec2(0.5, -0.5), vec2(0.5, 0.5), vec2(-0.5, 0.5)].map(|corner| corner + center)
}

fn build_scene() -> World {
    let mut world = World::new();
    for i in 0..10 {
        let i = i as f32;
        let body = world
            .add_body(BodyDesc::dynamic().position(vec2(i, 2.0 * i)).angular_velocity(0.3 * i).linear_damping(0.1));
        world.add_collider(body, ColliderDesc::rectangle(0.5, 0.25));
    }
    world
}
