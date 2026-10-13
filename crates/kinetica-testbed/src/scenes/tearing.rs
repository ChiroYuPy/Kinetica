use kinetica::math::Rot;
use kinetica::prelude::*;
use kinetica_macroquad::DebugRenderer;
use macroquad::color::Color;
use macroquad::input::KeyCode;

use super::cloth::Cloth;
use super::{Build, Category, Frame, Labels, Scene, SceneUpdate, add_ground};

pub const SCENE: Scene = Scene {
    name: "Tearing cloth",
    category: Category::SoftStructures,
    description: "Each link of this curtain is removed when it pulls too hard. Hold Shift and move the mouse to \
                  cut it: past a point, the heavy bar tears the rest.",
    camera_center: vec2(0.0, 5.5),
    view_height: 13.0,
    build: Build::Animated(build),
};

const CURTAIN_COLOR: Color = Color::new(0.9, 0.75, 0.35, 1.0);
const CURTAIN_COLUMNS: usize = 24;
const CURTAIN_ROWS: usize = 22;
const CURTAIN_TOP_LEFT: Vec2 = vec2(-3.45, 10.5);
const SPACING: f32 = 0.3;
const BAR_MASS: f32 = 400.0;
/// Force a link can bear, in newtons. Intact, the curtain pulls at most about 400 N on a link.
const STRENGTH: f32 = 700.0;
const CUT_RADIUS: f32 = 0.15;
const CUT_COLOR: Color = Color::new(0.95, 0.95, 0.95, 1.0);
const BAR_COLOR: Color = Color::new(0.45, 0.45, 0.5, 1.0);

fn build(world: &mut World, _labels: &mut Labels) -> Box<dyn SceneUpdate> {
    add_ground(world, 40.0);
    let (curtain, bar) = add_curtain(world);
    let links = world.joint_count();
    Box::new(Curtain { curtain, bar, links, cutting: false })
}

/// A curtain hanging from its top row, holding a heavy bar hinged to every particle of its hem,
/// which spreads the weight over the whole width.
fn add_curtain(world: &mut World) -> (Cloth, BodyHandle) {
    let curtain = Cloth::new(
        world,
        CURTAIN_TOP_LEFT,
        (CURTAIN_COLUMNS, CURTAIN_ROWS),
        (SPACING, 1.0),
        |_, row| row == 0,
        CURTAIN_COLOR,
    );
    let hem_y = CURTAIN_TOP_LEFT.y - SPACING * (CURTAIN_ROWS - 1) as f32;
    let (width, height) = (SPACING * (CURTAIN_COLUMNS - 1) as f32, 0.3);
    let center = vec2(CURTAIN_TOP_LEFT.x + width / 2.0, hem_y - 0.2);
    let shape = ColliderDesc::rectangle(width, height).density(BAR_MASS / (width * height));
    let bar = world.add_body(BodyDesc::dynamic().position(center).collider(shape));
    for &particle in curtain.bottom_row() {
        world.add_joint(RevoluteJointDesc::new(bar, particle, world[particle].position()));
    }
    (curtain, bar)
}

struct Curtain {
    curtain: Cloth,
    bar: BodyHandle,
    /// Joints at the start, to count the broken links.
    links: usize,
    cutting: bool,
}

impl SceneUpdate for Curtain {
    fn update(&mut self, world: &mut World, frame: &Frame) {
        self.cutting = frame.is_down(KeyCode::LeftShift) || frame.is_down(KeyCode::RightShift);
        if self.cutting {
            self.curtain.cut(world, frame.mouse, CUT_RADIUS);
        }
        self.curtain.tear(world, STRENGTH);
    }

    fn draw(&self, world: &World, frame: &Frame, drawer: &mut DebugRenderer) {
        self.curtain.draw(world, drawer);
        drawer.draw_body(world, self.bar, BAR_COLOR);
        if self.cutting {
            drawer.draw_circle(frame.mouse, CUT_RADIUS, Rot::IDENTITY, CUT_COLOR);
        }
    }

    fn labels(&self, world: &World, _frame: &Frame, labels: &mut Labels) {
        let broken = self.links.saturating_sub(world.joint_count());
        labels.at(vec2(0.0, 2.4), format!("{BAR_MASS:.0} kg bar, {broken} links broken"));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The curtain with its bar above the ground.
    fn curtain() -> (World, Cloth, BodyHandle) {
        let mut world = World::new();
        add_ground(&mut world, 40.0);
        let (curtain, bar) = add_curtain(&mut world);
        (world, curtain, bar)
    }

    fn run(world: &mut World, curtain: &mut Cloth, steps: usize) {
        for _ in 0..steps {
            curtain.tear(world, STRENGTH);
            world.step(1.0 / 60.0);
        }
    }

    /// Cuts across the curtain at mid height, through `columns`.
    fn cut(world: &mut World, curtain: &mut Cloth, columns: std::ops::Range<usize>) {
        for column in columns {
            curtain.cut(world, vec2(CURTAIN_TOP_LEFT.x + SPACING * column as f32, 7.15), CUT_RADIUS);
        }
    }

    #[test]
    fn intact_curtain_holds_the_bar() {
        let (mut world, mut curtain, bar) = curtain();
        let joints = world.joint_count();
        run(&mut world, &mut curtain, 10 * 60);
        assert_eq!(world.joint_count(), joints, "the intact curtain tore");
        assert!(world[bar].position().y > 3.5, "the bar dropped to {}", world[bar].position().y);
    }

    #[test]
    fn cutting_half_the_curtain_makes_it_tear_through() {
        let (mut world, mut curtain, bar) = curtain();
        run(&mut world, &mut curtain, 2 * 60);
        cut(&mut world, &mut curtain, 0..CURTAIN_COLUMNS / 2);
        run(&mut world, &mut curtain, 3 * 60);
        assert!(world[bar].position().y < 0.5, "the curtain still holds the bar at {}", world[bar].position().y);
    }

    #[test]
    fn a_small_cut_does_not_spread() {
        let (mut world, mut curtain, bar) = curtain();
        run(&mut world, &mut curtain, 2 * 60);
        cut(&mut world, &mut curtain, 10..13);
        let joints = world.joint_count();
        run(&mut world, &mut curtain, 3 * 60);
        assert!(world.joint_count() + 10 > joints, "a small cut spread: {} links tore", joints - world.joint_count());
        assert!(world[bar].position().y > 3.5);
    }
}
