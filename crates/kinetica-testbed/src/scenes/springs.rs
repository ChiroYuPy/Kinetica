use kinetica::prelude::*;

use super::{Build, Scene, add_ground};

pub const SCENE: Scene = Scene {
    name: "Springs",
    description: "Left: a jelly of balls linked by springs. Right: 1 Hz springs with damping ratio 0, 0.1, 0.3 and 1.",
    camera_center: vec2(0.0, 5.0),
    view_height: 13.0,
    build: Build::Static(build),
};

fn build(world: &mut World) {
    add_ground(world, 40.0);
    add_jelly(world, vec2(-7.0, 5.0));

    for (i, damping_ratio) in [0.0, 0.1, 0.3, 1.0].into_iter().enumerate() {
        let anchor_position = vec2(2.0 + 1.8 * i as f32, 10.0);
        let anchor = world.add_body(BodyDesc::fixed().position(anchor_position));
        // Start 1.5 m below the attachment point, with a 3 m rest length: the spring pushes down first.
        let weight_position = anchor_position - vec2(0.0, 1.5);
        let weight = world.add_body(BodyDesc::dynamic().position(weight_position));
        world.add_collider(weight, ColliderDesc::rectangle(0.4, 0.4));
        world.add_joint(
            DistanceJointDesc::between_world_points(world, anchor, weight, anchor_position, weight_position)
                .length(3.0)
                .spring(1.0, damping_ratio),
        );
    }
}

/// A square grid of small balls, each linked to its neighbours (including diagonals) by springs.
fn add_jelly(world: &mut World, corner: Vec2) {
    let size = 6;
    let spacing = 0.5;
    let mut grid = Vec::new();
    for row in 0..size {
        for column in 0..size {
            let position = corner + spacing * vec2(column as f32, row as f32);
            let ball = world.add_body(BodyDesc::dynamic().position(position));
            world.add_collider(ball, ColliderDesc::circle(0.15).friction(0.4));
            grid.push((ball, position));
        }
    }
    let at = |row: usize, column: usize| grid[row * size + column];
    for row in 0..size {
        for column in 0..size {
            let neighbours =
                [(row, column + 1), (row + 1, column), (row + 1, column + 1), (row + 1, column.wrapping_sub(1))];
            for (other_row, other_column) in neighbours {
                if other_row < size && other_column < size {
                    let ((a, position_a), (b, position_b)) = (at(row, column), at(other_row, other_column));
                    world.add_joint(
                        DistanceJointDesc::between_world_points(world, a, b, position_a, position_b).spring(6.0, 0.3),
                    );
                }
            }
        }
    }
}
