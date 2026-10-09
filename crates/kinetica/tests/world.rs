//! World bookkeeping: handles, mass updates and reproducibility.

use kinetica::prelude::*;

#[test]
fn removing_a_body_removes_its_colliders() {
    let mut world = World::new();
    let body = world.add_body(BodyDesc::dynamic());
    let collider = world.add_collider(body, ColliderDesc::circle(1.0));

    assert!(world.remove_body(body).is_some());
    assert!(world.get_body(body).is_none());
    assert!(world.get_collider(collider).is_none());
    assert_eq!(world.collider_count(), 0);
}

#[test]
fn stale_handle_does_not_alias_a_new_body() {
    let mut world = World::new();
    let old = world.add_body(BodyDesc::dynamic());
    world.remove_body(old);
    let new = world.add_body(BodyDesc::fixed());

    assert_ne!(old, new);
    assert!(world.get_body(old).is_none());
    assert!(world.get_body(new).is_some());
}

#[test]
#[should_panic(expected = "was removed from the world")]
fn indexing_a_removed_body_panics() {
    let mut world = World::new();
    let body = world.add_body(BodyDesc::dynamic());
    world.remove_body(body);
    let _ = world[body].position();
}

#[test]
fn bodies_are_created_with_their_colliders() {
    let mut world = World::new();
    let body = world.add_body(
        BodyDesc::dynamic()
            .position(vec2(1.0, 2.0))
            .collider(ColliderDesc::square(1.0))
            .collider(ColliderDesc::circle(0.5).offset(vec2(2.0, 0.0)).sensor()),
    );
    let colliders = world[body].colliders();
    assert_eq!(colliders.len(), 2);
    assert!(world[colliders[1]].is_sensor());
    // The sensor has no mass: the body is the 1 kg square alone.
    assert!((world[body].mass() - 1.0).abs() < 1e-5);
    assert!(world[colliders[1]].aabb().center().abs_diff_eq(vec2(3.0, 2.0), 1e-5));
}

#[test]
fn sizes_are_full_sizes() {
    let mut world = World::new();
    let body = world.add_body(
        BodyDesc::fixed()
            .collider(ColliderDesc::rectangle(4.0, 1.0))
            .collider(ColliderDesc::vertical_capsule(3.0, 0.5).offset(vec2(10.0, 0.0))),
    );
    let [rectangle, capsule] = [0, 1].map(|i| world[world[body].colliders()[i]].aabb());
    assert!(rectangle.half_extents().abs_diff_eq(vec2(2.0, 0.5), 1e-5));
    assert!(capsule.half_extents().abs_diff_eq(vec2(0.5, 1.5), 1e-5));
}

#[test]
fn collider_offset_and_angle_place_the_shape() {
    let mut world = World::new();
    // A 2 x 1 bar turned upright, its center 3 m to the right of the body origin.
    let shape = ColliderDesc::rectangle(2.0, 1.0).offset(vec2(3.0, 0.0)).angle(std::f32::consts::FRAC_PI_2);
    let body = world.add_body(BodyDesc::fixed().position(vec2(0.0, 1.0)).collider(shape));
    let aabb = world[world[body].colliders()[0]].aabb();
    assert!(aabb.center().abs_diff_eq(vec2(3.0, 1.0), 1e-5));
    assert!(aabb.half_extents().abs_diff_eq(vec2(0.5, 1.0), 1e-5));
}

#[test]
fn mass_follows_attached_colliders() {
    let mut world = World::new();
    let body = world.add_body(BodyDesc::dynamic());
    let left = world.add_collider(body, ColliderDesc::convex_hull(&square_at(vec2(-1.0, 0.0))).unwrap());
    world.add_collider(body, ColliderDesc::convex_hull(&square_at(vec2(1.0, 0.0))).unwrap().density(3.0));

    let compound = &world[body];
    assert!((compound.mass() - 4.0).abs() < 1e-5);
    // Weighted average of the two square centers: (-1·1 + 1·3) / 4.
    assert!(compound.center_of_mass().abs_diff_eq(vec2(0.5, 0.0), 1e-5));

    world.remove_collider(left);
    let single = &world[body];
    assert!((single.mass() - 3.0).abs() < 1e-5);
    assert!(single.center_of_mass().abs_diff_eq(vec2(1.0, 0.0), 1e-5));
}

#[test]
fn collider_aabb_follows_its_body() {
    let mut world = World::new();
    let body = world.add_body(BodyDesc::kinematic().linear_velocity(vec2(1.0, 0.0)));
    let collider = world.add_collider(body, ColliderDesc::circle(0.5));

    world.step(1.0);

    let aabb = world[collider].aabb();
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
        world.add_collider(body, ColliderDesc::rectangle(1.0, 0.5));
    }
    world
}
