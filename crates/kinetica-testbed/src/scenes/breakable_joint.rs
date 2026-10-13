use kinetica::prelude::*;

use super::{Build, Category, Frame, Labels, Scene, SceneUpdate, add_ground};

pub const SCENE: Scene = Scene {
    name: "Breakable joint",
    category: Category::Joints,
    description: "Every joint tells the force and torque it carries. This scene reads the torque of the shelf's \
                  weld each step, and removes the weld when it is too high.",
    camera_center: vec2(0.0, 3.0),
    view_height: 9.0,
    build: Build::Animated(build),
};

const WALL_X: f32 = 1.5;
const SHELF_LENGTH: f32 = 2.4;
const SHELF_Y: f32 = 3.0;
/// The shelf breaks when its average torque exceeds this, in N·m.
const BREAKING_TORQUE: f32 = 100.0;
/// The torque is averaged over about this long, so short impacts do not break the shelf.
const AVERAGING_TIME: f32 = 0.2;
const DROP_PERIOD: f32 = 1.5;

fn build(world: &mut World, _labels: &mut Labels) -> Box<dyn SceneUpdate> {
    add_ground(world, 40.0);
    let wall = world
        .add_body(BodyDesc::fixed().position(vec2(WALL_X + 0.25, 3.0)).collider(ColliderDesc::rectangle(0.5, 6.0)));
    let board = ColliderDesc::rectangle(SHELF_LENGTH, 0.2).friction(0.8);
    let shelf =
        world.add_body(BodyDesc::dynamic().position(vec2(WALL_X - SHELF_LENGTH / 2.0, SHELF_Y)).collider(board));
    let weld = world.add_joint(WeldJointDesc::new(wall, shelf, vec2(WALL_X, SHELF_Y)));
    Box::new(Shelf { weld: Some(weld), torque: 0.0, drops: 0 })
}

struct Shelf {
    weld: Option<JointHandle<WeldJoint>>,
    /// Torque carried by the weld, averaged over the last few steps.
    torque: f32,
    drops: usize,
}

impl SceneUpdate for Shelf {
    fn update(&mut self, world: &mut World, frame: &Frame) {
        let Some(weld) = self.weld else { return };
        let torque = world[weld].constraint_torque().abs();
        self.torque += (torque - self.torque) * (1.0 / 60.0 / AVERAGING_TIME);
        if self.torque > BREAKING_TORQUE {
            world.remove_joint(weld);
            self.weld = None;
        } else if frame.time >= DROP_PERIOD * self.drops as f32 {
            // One more heavy box on the shelf.
            let x = WALL_X - 0.6 - 0.35 * (self.drops % 4) as f32;
            let heavy_box = ColliderDesc::square(0.5).density(4.0);
            world.add_body(BodyDesc::dynamic().position(vec2(x, SHELF_Y + 1.0)).collider(heavy_box));
            self.drops += 1;
        }
    }

    fn labels(&self, _world: &World, _frame: &Frame, labels: &mut Labels) {
        let text = if self.weld.is_some() {
            format!("{:.0} / {BREAKING_TORQUE:.0} N·m", self.torque)
        } else {
            "Broken".to_owned()
        };
        labels.at(vec2(WALL_X - SHELF_LENGTH / 2.0, SHELF_Y - 1.0), text);
    }
}
