//! Cloth made of particles linked by ropes that can tear, and the scene showing it.

use kinetica::prelude::*;
use macroquad::color::Color;
use macroquad::input::KeyCode;

use super::{Build, Category, Frame, Labels, Scene, SceneUpdate, add_ground};
use kinetica_macroquad::DebugRenderer;

pub const SCENE: Scene = Scene {
    name: "Cloth",
    category: Category::SoftStructures,
    description: "Cloth is a grid of small balls linked by rope joints: it resists stretching but folds freely. \
                  Left, a curtain holds a heavy bar, and each link breaks when it pulls harder than its strength: \
                  cut it with Shift and the mouse, and once enough is cut the tear spreads on its own. Right, a \
                  hammock catching whatever falls on it.",
    camera_center: vec2(0.0, 6.0),
    view_height: 16.0,
    build: Build::Animated(build),
};

/// Particles are heavy for their size, so the cloth is not crushed by what lands on it.
const PARTICLE_DENSITY: f32 = 20.0;
const CURTAIN_COLOR: Color = Color::new(0.9, 0.75, 0.35, 1.0);
const CURTAIN_COLUMNS: usize = 24;
const CURTAIN_ROWS: usize = 22;
const CURTAIN_TOP_LEFT: Vec2 = vec2(-10.5, 10.5);
const SPACING: f32 = 0.3;
const BAR_MASS: f32 = 400.0;
/// Force a link can bear, in newtons. Intact, the curtain pulls at most about 400 N on a link.
const STRENGTH: f32 = 700.0;
const CUT_RADIUS: f32 = 0.15;
const CUT_COLOR: Color = Color::new(0.95, 0.95, 0.95, 1.0);
const BAR_COLOR: Color = Color::new(0.45, 0.45, 0.5, 1.0);
const HAMMOCK_COLOR: Color = Color::new(0.35, 0.7, 0.9, 1.0);
const DROP_PERIOD: f32 = 1.5;
const MAX_DROPS: usize = 8;

/// A rectangular grid of particles; neighbors are linked by ropes that may be torn.
struct Cloth {
    columns: usize,
    rows: usize,
    /// Longest distance allowed between neighbors.
    link_length: f32,
    /// Row by row, the first row at the top.
    particles: Vec<BodyHandle>,
    /// Links between a particle and its right neighbor, row by row; `None` once torn.
    horizontal: Vec<Option<JointHandle<DistanceJoint>>>,
    /// Links between a particle and the one below it, row by row.
    vertical: Vec<Option<JointHandle<DistanceJoint>>>,
    color: Color,
}

impl Cloth {
    /// Builds a cloth whose top-left particle is at `top_left`, with particles `spacing` apart.
    /// Links may stretch to `slack` times the spacing before pulling, so a cloth pinned on
    /// both sides can sag. `pinned(column, row)` tells which particles are fixed in place.
    fn new(
        world: &mut World,
        top_left: Vec2,
        (columns, rows): (usize, usize),
        (spacing, slack): (f32, f32),
        pinned: impl Fn(usize, usize) -> bool,
        color: Color,
    ) -> Self {
        let mut particles = Vec::with_capacity(columns * rows);
        for row in 0..rows {
            for column in 0..columns {
                let position = top_left + spacing * vec2(column as f32, -(row as f32));
                let desc = if pinned(column, row) { BodyDesc::fixed() } else { BodyDesc::dynamic() };
                let ball = ColliderDesc::circle(0.3 * spacing).density(PARTICLE_DENSITY).friction(0.3);
                particles.push(world.add_body(desc.position(position).linear_damping(0.1).collider(ball)));
            }
        }
        let mut cloth = Self {
            columns,
            rows,
            link_length: slack * spacing,
            particles,
            horizontal: Vec::new(),
            vertical: Vec::new(),
            color,
        };
        for row in 0..rows {
            for column in 0..columns {
                if column + 1 < columns {
                    let link = cloth.link(world, (column, row), (column + 1, row));
                    cloth.horizontal.push(Some(link));
                }
                if row + 1 < rows {
                    let link = cloth.link(world, (column, row), (column, row + 1));
                    cloth.vertical.push(Some(link));
                }
            }
        }
        cloth
    }

    fn particle(&self, column: usize, row: usize) -> BodyHandle {
        self.particles[row * self.columns + column]
    }

    /// A rope: the particles may get closer, never further than the link length.
    fn link(&self, world: &mut World, a: (usize, usize), b: (usize, usize)) -> JointHandle<DistanceJoint> {
        let (a, b) = (self.particle(a.0, a.1), self.particle(b.0, b.1));
        let rope = DistanceJointDesc::new(a, b, world[a].position(), world[b].position()).rope(self.link_length);
        world.add_joint(rope)
    }

    /// The particles of the bottom row, from left to right.
    fn bottom_row(&self) -> &[BodyHandle] {
        &self.particles[(self.rows - 1) * self.columns..]
    }

    /// Breaks the links that pulled harder than `max_force` (in newtons) during the last step.
    /// Links to pinned particles are reinforced and never break.
    fn tear(&mut self, world: &mut World, max_force: f32) {
        let is_pinned = |body: BodyHandle| world[body].body_type() == BodyType::Fixed;
        let mut torn = Vec::new();
        for link in self.horizontal.iter_mut().chain(&mut self.vertical) {
            if let Some(joint) = link.map(|joint| &world[joint])
                && !is_pinned(joint.body_a())
                && !is_pinned(joint.body_b())
                && joint.constraint_force().length() > max_force
            {
                torn.extend(link.take());
            }
        }
        for joint in torn {
            world.remove_joint(joint);
        }
    }

    /// Cuts the links passing within `radius` of `point`.
    fn cut(&mut self, world: &mut World, point: Vec2, radius: f32) {
        let mut cut = Vec::new();
        for link in self.horizontal.iter_mut().chain(&mut self.vertical) {
            let Some(joint) = link.map(|joint| &world[joint]) else { continue };
            let (a, b) = (world[joint.body_a()].position(), world[joint.body_b()].position());
            if distance_to_segment(point, a, b) < radius {
                cut.extend(link.take());
            }
        }
        for joint in cut {
            world.remove_joint(joint);
        }
    }

    /// Fills every cell whose four sides are still linked.
    fn draw(&self, world: &World, drawer: &mut DebugRenderer) {
        let position = |column, row| world[self.particle(column, row)].position();
        for row in 0..self.rows - 1 {
            for column in 0..self.columns - 1 {
                let top = self.horizontal[row * (self.columns - 1) + column];
                let bottom = self.horizontal[(row + 1) * (self.columns - 1) + column];
                let left = self.vertical[row * self.columns + column];
                let right = self.vertical[row * self.columns + column + 1];
                if [top, bottom, left, right].iter().any(Option::is_none) {
                    continue;
                }
                let corners = [
                    position(column, row),
                    position(column, row + 1),
                    position(column + 1, row + 1),
                    position(column + 1, row),
                ];
                drawer.draw_polygon(&corners, self.color);
            }
        }
    }
}

fn build(world: &mut World, labels: &mut Labels) -> Box<dyn SceneUpdate> {
    add_ground(world, 60.0);
    let (curtain, bar) = add_curtain(world);
    let hammock =
        Cloth::new(world, vec2(1.0, 7.0), (34, 4), (0.3, 1.15), |column, _| column == 0 || column == 33, HAMMOCK_COLOR);
    labels.at(
        vec2(CURTAIN_TOP_LEFT.x + 3.45, CURTAIN_TOP_LEFT.y + 0.4),
        "Hold Shift and move the mouse\nacross the curtain to cut it",
    );
    labels.at(vec2(6.0, 7.5), "Hammock: fixed at both ends,\nlinks 15% longer than the spacing");
    let links = world.joint_count();
    Box::new(ClothDemo { curtain, bar, hammock, links, cutting: false, drops: Vec::new(), next_drop: 1.0 })
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

struct ClothDemo {
    curtain: Cloth,
    bar: BodyHandle,
    hammock: Cloth,
    /// Joints at the start, to count the broken links.
    links: usize,
    cutting: bool,
    drops: Vec<BodyHandle>,
    next_drop: f32,
}

impl SceneUpdate for ClothDemo {
    fn update(&mut self, world: &mut World, frame: &Frame) {
        self.cutting = frame.is_down(KeyCode::LeftShift) || frame.is_down(KeyCode::RightShift);
        if self.cutting {
            self.curtain.cut(world, frame.mouse, CUT_RADIUS);
        }
        self.curtain.tear(world, STRENGTH);

        if frame.time >= self.next_drop && self.drops.len() < MAX_DROPS {
            self.next_drop += DROP_PERIOD;
            let i = self.drops.len();
            let x = 4.0 + 4.0 * (i as f32 * 0.618_034).fract();
            let shape = if i.is_multiple_of(2) { ColliderDesc::circle(0.5) } else { ColliderDesc::square(0.9) };
            let drop = BodyDesc::dynamic().position(vec2(x, 12.0)).angular_velocity(1.0).collider(shape.density(2.0));
            self.drops.push(world.add_body(drop));
        }
    }

    fn draw(&self, world: &World, frame: &Frame, drawer: &mut DebugRenderer) {
        self.curtain.draw(world, drawer);
        drawer.draw_body(world, self.bar, BAR_COLOR);
        self.hammock.draw(world, drawer);
        if self.cutting {
            drawer.draw_circle(frame.mouse, CUT_RADIUS, Rot::IDENTITY, CUT_COLOR);
        }
    }

    fn labels(&self, world: &World, _frame: &Frame, labels: &mut Labels) {
        let broken = self.links.saturating_sub(world.joint_count());
        let below_bar = vec2(CURTAIN_TOP_LEFT.x + 3.45, 2.4);
        labels.at(below_bar, format!("{BAR_MASS:.0} kg bar\nLinks break above {STRENGTH:.0} N: {broken} broken"));
    }
}

fn distance_to_segment(point: Vec2, start: Vec2, end: Vec2) -> f32 {
    let along = end - start;
    let t = ((point - start).dot(along) / along.length_squared().max(f32::EPSILON)).clamp(0.0, 1.0);
    point.distance(start + t * along)
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
