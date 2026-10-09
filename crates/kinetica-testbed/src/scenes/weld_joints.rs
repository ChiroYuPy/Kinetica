use kinetica::prelude::*;

use super::{Build, Category, Frame, Labels, Scene, SceneUpdate, add_ground};

pub const SCENE: Scene = Scene {
    name: "Weld joints",
    category: Category::Joints,
    description: "A weld joint glues two bodies together. Left, two beams of welded segments: rigid welds on top \
                  (joints are slightly soft, so it sags a little), and welds with angular springs below. Right, a \
                  shelf reads the torque its weld carries every step and breaks it when overloaded.",
    camera_center: vec2(0.5, 4.0),
    view_height: 12.0,
    build: Build::Animated(build),
};

const SEGMENT_COUNT: usize = 7;
const SEGMENT_LENGTH: f32 = 1.0;
const WALL_X: f32 = 7.0;
const SHELF_LENGTH: f32 = 2.4;
const SHELF_Y: f32 = 3.0;
/// The shelf breaks when its average torque exceeds this, in N·m.
const BREAKING_TORQUE: f32 = 100.0;
/// The torque is averaged over about this long, so short impacts do not break the shelf.
const AVERAGING_TIME: f32 = 0.2;
const DROP_PERIOD: f32 = 1.5;

fn build(world: &mut World, labels: &mut Labels) -> Box<dyn SceneUpdate> {
    add_ground(world, 60.0);

    let wall = world.add_body(BodyDesc::fixed().position(vec2(-6.0, 4.0)).collider(ColliderDesc::rectangle(0.5, 8.0)));
    add_beam(world, wall, vec2(-5.75, 6.5), None);
    labels.at(vec2(-2.0, 7.3), "Rigid welds");
    add_beam(world, wall, vec2(-5.75, 3.0), Some((20.0, 0.5)));
    labels.at(vec2(-2.0, 3.8), "Angular springs, 20 Hz");

    let right_wall = world
        .add_body(BodyDesc::fixed().position(vec2(WALL_X + 0.25, 4.0)).collider(ColliderDesc::rectangle(0.5, 8.0)));
    let board = ColliderDesc::rectangle(SHELF_LENGTH, 0.2).friction(0.8);
    let shelf =
        world.add_body(BodyDesc::dynamic().position(vec2(WALL_X - SHELF_LENGTH / 2.0, SHELF_Y)).collider(board));
    let weld = world.add_joint(WeldJointDesc::new(right_wall, shelf, vec2(WALL_X, SHELF_Y)));

    Box::new(Shelf { weld: Some(weld), torque: 0.0, drops: 0 })
}

/// A horizontal beam welded to `wall` at `start`, rigid or with angular springs.
fn add_beam(world: &mut World, wall: BodyHandle, start: Vec2, spring: Option<(f32, f32)>) {
    let mut previous = wall;
    let mut weld_point = start;
    for _ in 0..SEGMENT_COUNT {
        let bar = ColliderDesc::rectangle(SEGMENT_LENGTH, 0.24);
        let segment =
            world.add_body(BodyDesc::dynamic().position(weld_point + vec2(SEGMENT_LENGTH / 2.0, 0.0)).collider(bar));
        let mut weld = WeldJointDesc::new(previous, segment, weld_point);
        if let Some((hertz, damping_ratio)) = spring {
            weld = weld.angular_spring(hertz, damping_ratio);
        }
        world.add_joint(weld);
        previous = segment;
        weld_point += vec2(SEGMENT_LENGTH, 0.0);
    }
}

struct Shelf {
    weld: Option<JointHandle<WeldJoint>>,
    /// Torque carried by the weld, averaged over the last few steps.
    torque: f32,
    drops: usize,
}

impl SceneUpdate for Shelf {
    fn update(&mut self, world: &mut World, frame: &Frame) {
        if let Some(weld) = self.weld {
            // The bending torque at the wall.
            let torque = world[weld].constraint_torque().abs();
            self.torque += (torque - self.torque) * (1.0 / 60.0 / AVERAGING_TIME);
            if self.torque > BREAKING_TORQUE {
                world.remove_joint(weld);
                self.weld = None;
            }
        }
        // Drop a box on the shelf every few seconds while it holds.
        if self.weld.is_some() && frame.time >= DROP_PERIOD * self.drops as f32 {
            let x = WALL_X - 0.6 - 0.35 * (self.drops % 4) as f32;
            let heavy_box = ColliderDesc::square(0.5).density(4.0);
            world.add_body(BodyDesc::dynamic().position(vec2(x, SHELF_Y + 1.0)).collider(heavy_box));
            self.drops += 1;
        }
    }

    fn labels(&self, _world: &World, _frame: &Frame, labels: &mut Labels) {
        let text = if self.weld.is_some() {
            format!("Torque at the wall: {:.0} / {BREAKING_TORQUE:.0} N·m", self.torque)
        } else {
            "Broken!".to_owned()
        };
        labels.at(vec2(WALL_X - 1.5, SHELF_Y - 1.0), text);
    }
}
