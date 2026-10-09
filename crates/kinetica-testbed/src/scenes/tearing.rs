use kinetica::prelude::*;
use kinetica::{Color, DebugDraw};
use macroquad::input::KeyCode;

use super::cloth::Cloth;
use super::{Build, Category, Frame, Labels, Scene, SceneUpdate, add_ground};
use kinetica_macroquad::DebugRenderer;

pub const SCENE: Scene = Scene {
    name: "Tearing cloth",
    category: Category::SoftStructures,
    description: "A curtain holds a heavy bar, and each link breaks when it pulls harder than its strength. Hold \
                  Shift and sweep the mouse across the curtain to cut it: once enough is cut, the remaining \
                  links are overloaded and the tear spreads on its own.",
    camera_center: vec2(0.0, 5.5),
    view_height: 13.0,
    build: Build::Animated(build),
};

const COLUMNS: usize = 24;
const ROWS: usize = 22;
const SPACING: f32 = 0.3;
const TOP_LEFT: Vec2 = vec2(-3.45, 10.5);
const BAR_MASS: f32 = 400.0;
/// Force a link can bear, in newtons. Intact, the curtain pulls at most about 400 N on a link.
const STRENGTH: f32 = 700.0;
const CUT_RADIUS: f32 = 0.15;
const CUT_COLOR: Color = Color::rgb(0.95, 0.95, 0.95);
const BAR_COLOR: Color = Color::rgb(0.45, 0.45, 0.5);

fn build(world: &mut World, labels: &mut Labels) -> Box<dyn SceneUpdate> {
    labels.at(vec2(0.0, TOP_LEFT.y + 0.4), "Hold Shift and move the mouse across the curtain to cut it");
    Box::new(build_curtain(world))
}

fn build_curtain(world: &mut World) -> Tearing {
    add_ground(world, 20.0);
    let curtain =
        Cloth::new(world, TOP_LEFT, (COLUMNS, ROWS), (SPACING, 1.0), |_, row| row == 0, Color::rgb(0.9, 0.75, 0.35));

    // A heavy bar hinged to every particle of the hem spreads its weight over the whole width.
    let hem_y = TOP_LEFT.y - SPACING * (ROWS - 1) as f32;
    let half_width = SPACING * (COLUMNS - 1) as f32 / 2.0;
    let center = vec2(TOP_LEFT.x + half_width, hem_y - 0.2);
    let bar = world.add_body(BodyDesc::dynamic().position(center));
    let half_height = 0.15;
    let density = BAR_MASS / (4.0 * half_width * half_height);
    world.add_collider(bar, ColliderDesc::rectangle(half_width, half_height).density(density));
    for &particle in curtain.bottom_row() {
        let point = world.body(particle).map_or(center, kinetica::Body::position);
        world.add_joint(RevoluteJointDesc::at_world_point(world, bar, particle, point));
    }
    Tearing { curtain, bar, cutting: false, links: world.joint_count() }
}

struct Tearing {
    curtain: Cloth,
    bar: BodyHandle,
    cutting: bool,
    /// Joints at the start, to count the broken links.
    links: usize,
}

impl SceneUpdate for Tearing {
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
        labels.at(vec2(0.0, 2.4), format!("{BAR_MASS:.0} kg bar\nLinks break above {STRENGTH:.0} N: {broken} broken"));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn run(world: &mut World, scene: &mut Tearing, steps: usize) {
        for _ in 0..steps {
            scene.curtain.tear(world, STRENGTH);
            world.step(1.0 / 60.0);
        }
    }

    fn bar_height(world: &World, scene: &Tearing) -> f32 {
        world.body(scene.bar).unwrap().position().y
    }

    #[test]
    fn intact_curtain_holds_the_bar() {
        let mut world = World::new();
        let mut scene = build_curtain(&mut world);
        let joints = world.joint_count();
        run(&mut world, &mut scene, 10 * 60);
        assert_eq!(world.joint_count(), joints, "the intact curtain tore");
        assert!(bar_height(&world, &scene) > 3.5, "the bar dropped to {}", bar_height(&world, &scene));
    }

    #[test]
    fn cutting_half_the_curtain_makes_it_tear_through() {
        let mut world = World::new();
        let mut scene = build_curtain(&mut world);
        run(&mut world, &mut scene, 2 * 60);
        // Sweep the cutter across the left half, at mid height.
        for i in 0..COLUMNS / 2 {
            scene.curtain.cut(&mut world, vec2(TOP_LEFT.x + SPACING * i as f32, 7.15), CUT_RADIUS);
        }
        run(&mut world, &mut scene, 3 * 60);
        assert!(bar_height(&world, &scene) < 0.5, "the curtain still holds the bar at {}", bar_height(&world, &scene));
    }

    #[test]
    fn a_small_cut_does_not_spread() {
        let mut world = World::new();
        let mut scene = build_curtain(&mut world);
        run(&mut world, &mut scene, 2 * 60);
        for i in 10..13 {
            scene.curtain.cut(&mut world, vec2(TOP_LEFT.x + SPACING * i as f32, 7.15), CUT_RADIUS);
        }
        let joints = world.joint_count();
        run(&mut world, &mut scene, 3 * 60);
        assert!(world.joint_count() + 10 > joints, "a small cut spread: {} links tore", joints - world.joint_count());
        assert!(bar_height(&world, &scene) > 3.5);
    }
}
