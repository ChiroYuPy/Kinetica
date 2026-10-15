//! The picture at the top of the README: the name of the engine, written with bodies and joints.
//!
//! Every letter is a handful of bars welded together, standing on the ground while a few
//! shapes land around them. The scene runs for a moment, then the window is saved as a PNG.
//!
//! ```sh
//! cargo run --release -p kinetica-macroquad --example logo            # writes docs/kinetica.png
//! cargo run --release -p kinetica-macroquad --example logo -- out.png
//! ```

use kinetica::CollisionLayers;
use kinetica::prelude::*;
use kinetica_macroquad::paper::{INK, PAPER};
use kinetica_macroquad::{Paper, PaperRenderer, WorldCamera};
use macroquad::prelude::*;

const WIDTH: i32 = 1600;
const HEIGHT: i32 = 480;
/// Height of the picture, in meters.
const VIEW_HEIGHT: f32 = 6.2;
/// Thickness of every bar.
const BAR: f32 = 0.28;
/// Height of the lowercase letters, outer edge to outer edge.
const X_HEIGHT: f32 = 2.25;
/// Gap between two letters.
const GAP: f32 = 0.45;
/// Angle between two vertices of an arc, in degrees. Arcs of round letters start 15° off an axis so
/// that a flat bar lies at the bottom, wide enough for the letter to stand steadily.
const ARC_STEP: f32 = 30.0;
/// Steps simulated before the picture is taken.
const STEPS: u32 = 60 * 4;

const GRASS: Color = Color::new(0.5, 0.7, 0.38, 1.0);
const DIRT: Color = Color::new(0.72, 0.56, 0.4, 1.0);
const SHAPES: [Color; 4] = [
    Color::new(0.98, 0.8, 0.3, 1.0),
    Color::new(0.35, 0.65, 0.7, 1.0),
    Color::new(0.9, 0.35, 0.3, 1.0),
    Color::new(0.82, 0.62, 0.4, 1.0),
];
/// One color per letter of "Kinetica".
const LETTERS: [Color; 8] = [
    Color::new(0.88, 0.32, 0.28, 1.0),
    Color::new(0.98, 0.8, 0.3, 1.0),
    Color::new(0.35, 0.65, 0.7, 1.0),
    Color::new(0.5, 0.7, 0.38, 1.0),
    Color::new(0.9, 0.55, 0.3, 1.0),
    Color::new(0.55, 0.45, 0.78, 1.0),
    Color::new(0.35, 0.55, 0.85, 1.0),
    Color::new(0.85, 0.45, 0.6, 1.0),
];

/// A letter: strokes as lines of points (a closed stroke repeats its first point), in meters,
/// from the bottom left of the letter, and the center of a dot above it if it has one.
struct Letter {
    strokes: Vec<Vec<Vec2>>,
    dot: Option<Vec2>,
}

/// Points of an arc around `center`, from `from` to `to` degrees (counter-clockwise).
#[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)] // a small positive whole number
fn arc(center: Vec2, radius: f32, from: f32, to: f32) -> Vec<Vec2> {
    let count = ((to - from) / ARC_STEP).round() as usize;
    (0..=count).map(|i| center + radius * Vec2::from_angle((from + ARC_STEP * i as f32).to_radians())).collect()
}

/// Radius of a round letter, so that its outer edge is `X_HEIGHT` tall.
fn round_radius() -> f32 {
    (X_HEIGHT - BAR) / 2.0 / (ARC_STEP / 2.0).to_radians().cos()
}

/// Center of a round letter standing on the ground, on a flat bar.
fn round_center() -> Vec2 {
    vec2(round_radius() + BAR / 2.0, X_HEIGHT / 2.0)
}

fn letter(character: char) -> Letter {
    let r = round_radius();
    let o = round_center();
    match character {
        'K' => Letter {
            strokes: vec![
                vec![vec2(0.0, 0.0), vec2(0.0, 3.3)],
                vec![vec2(1.75, 3.3 - BAR), vec2(0.0, 1.35)],
                vec![vec2(1.75, 3.3 - BAR).lerp(vec2(0.0, 1.35), 0.7), vec2(1.85, 0.0)],
            ],
            dot: None,
        },
        'i' => {
            Letter { strokes: vec![vec![vec2(0.0, 0.0), vec2(0.0, X_HEIGHT)]], dot: Some(vec2(0.0, X_HEIGHT + 0.7)) }
        }
        'n' => {
            let x = r + BAR / 2.0;
            let arch_center = vec2(x, X_HEIGHT - BAR / 2.0 - r);
            // The left stem, then the arch from it over to the right and down.
            let mut arch = arc(arch_center, r, 0.0, 180.0);
            arch.reverse();
            arch.push(vec2(x + r, 0.0));
            Letter { strokes: vec![vec![vec2(BAR / 2.0, 0.0), vec2(BAR / 2.0, X_HEIGHT)], arch], dot: None }
        }
        'e' => {
            // A bar from the left to the right of the ring, a bit above its center.
            let at = |degrees: f32| o + r * Vec2::from_angle(degrees.to_radians());
            Letter { strokes: vec![arc(o, r, 15.0, 315.0), vec![at(165.0), at(15.0)]], dot: None }
        }
        't' => {
            let bar_height = X_HEIGHT - BAR / 2.0;
            Letter {
                strokes: vec![
                    vec![vec2(0.0, 0.0), vec2(0.0, 3.1)],
                    vec![vec2(-0.6, bar_height), vec2(0.0, bar_height), vec2(0.6, bar_height)],
                ],
                dot: None,
            }
        }
        'c' => Letter { strokes: vec![arc(o, r, 45.0, 345.0)], dot: None },
        'a' => {
            let stem = o.x + r;
            Letter { strokes: vec![arc(o, r, 15.0, 375.0), vec![vec2(stem, 0.0), vec2(stem, X_HEIGHT)]], dot: None }
        }
        _ => unreachable!("only the letters of Kinetica are drawn"),
    }
}

/// Everything the picture draws.
struct Scene {
    world: World,
    letters: Vec<(Vec<BodyHandle>, Color)>,
    /// The welds, drawn as rivets.
    welds: Vec<JointHandle<WeldJoint>>,
    shapes: Vec<(BodyHandle, Color)>,
    /// Left and right edge of the word, in meters.
    extent: (f32, f32),
}

/// Distance from `point` to the segment from `a` to `b`.
fn distance_to_segment(point: Vec2, a: Vec2, b: Vec2) -> f32 {
    let along = b - a;
    let t = ((point - a).dot(along) / along.length_squared()).clamp(0.0, 1.0);
    point.distance(a + t * along)
}

fn build() -> Scene {
    let mut world = World::new();
    world.add_body(
        BodyDesc::fixed().position(vec2(0.0, -0.5)).collider(ColliderDesc::rectangle(80.0, 1.0).friction(0.9)),
    );

    let mut letters = Vec::new();
    let mut welds = Vec::new();
    // Left and right edge of every letter.
    let mut spans: Vec<(f32, f32)> = Vec::new();
    let mut x = 0.0;
    for (index, character) in "Kinetica".chars().enumerate() {
        let shape = letter(character);
        // Letters are described around their stem: shift so that the letter starts at `x`.
        let left = shape.strokes.iter().flatten().map(|point| point.x).fold(f32::MAX, f32::min) - BAR / 2.0;
        let right = shape.strokes.iter().flatten().map(|point| point.x).fold(f32::MIN, f32::max) + BAR / 2.0;
        let offset = vec2(x - left, 0.02);

        // The bars of a letter overlap where they meet: they do not collide with each other, only
        // with the ground, the shapes and the other letters.
        let membership = 1 << (index + 1);
        let own_layer = CollisionLayers::new(membership, !membership);

        // One bar per piece of every stroke.
        let mut bars: Vec<(BodyHandle, Vec2, Vec2)> = Vec::new();
        let mut stroke_bars: Vec<(usize, usize, bool)> = Vec::new();
        for stroke in &shape.strokes {
            let closed = stroke.len() > 2 && stroke[0].distance(stroke[stroke.len() - 1]) < 1e-3;
            let first = bars.len();
            for (i, pair) in stroke.windows(2).enumerate() {
                let (start, end) = (pair[0] + offset, pair[1] + offset);
                let direction = (end - start).normalize();
                // Bars overlap at the corners inside a stroke, but stay flat at its ends.
                let before = if i > 0 || closed { BAR * 0.3 } else { 0.0 };
                let after = if i + 2 < stroke.len() || closed { BAR * 0.3 } else { 0.0 };
                let (from, to) = (start - before * direction, end + after * direction);
                let body = world.add_body(
                    BodyDesc::dynamic().position((from + to) / 2.0).angle(direction.y.atan2(direction.x)).collider(
                        ColliderDesc::rectangle(from.distance(to), BAR).friction(0.8).density(4.0).layers(own_layer),
                    ),
                );
                bars.push((body, start, end));
            }
            stroke_bars.push((first, bars.len(), closed));
        }

        // Weld the bars of a stroke at its corners, and the strokes together where they touch.
        for &(first, end, closed) in &stroke_bars {
            for i in first..end {
                let next = if i + 1 < end {
                    i + 1
                } else if closed {
                    first
                } else {
                    continue;
                };
                welds.push(world.add_joint(WeldJointDesc::new(bars[i].0, bars[next].0, bars[i].2)));
            }
            for i in first..end {
                for vertex in [bars[i].1, bars[i].2] {
                    let touched = bars.iter().enumerate().find(|&(other, &(_, a, b))| {
                        !(first..end).contains(&other) && distance_to_segment(vertex, a, b) < 0.05
                    });
                    if let Some((other, _)) = touched {
                        welds.push(world.add_joint(WeldJointDesc::new(bars[i].0, bars[other].0, vertex)));
                    }
                }
            }
        }

        let mut handles: Vec<BodyHandle> = bars.iter().map(|&(body, ..)| body).collect();
        if let Some(dot) = shape.dot {
            // The dot of the 'i' floats over its stem, held by a weld.
            let dot = dot + offset;
            let body = world.add_body(
                BodyDesc::dynamic()
                    .position(dot)
                    .collider(ColliderDesc::circle(0.27).friction(0.8).restitution(0.2).layers(own_layer)),
            );
            let stem_top = bars[0].2;
            welds.push(world.add_joint(WeldJointDesc::new(bars[0].0, body, (stem_top + dot) / 2.0)));
            handles.push(body);
        }
        letters.push((handles, LETTERS[index]));
        spans.push((x, x + right - left));
        x += right - left + GAP;
    }
    let extent = (0.0, x - GAP);

    // Shapes landing around the word and on a few of its letters. Some letters roll or topple
    // under a hit, so only the steadiest ones receive shapes.
    let middle = |index: usize| f32::midpoint(spans[index].0, spans[index].1);
    let drops = [
        vec2(-0.9, 0.8),
        vec2(-1.5, 0.8),
        vec2(-1.2, 2.0),
        vec2(middle(2) + 0.5, 4.6),
        vec2(middle(7), 4.4),
        vec2(extent.1 + 0.8, 0.8),
        vec2(extent.1 + 1.4, 0.8),
        vec2(extent.1 + 1.1, 2.0),
    ];
    let mut shapes = Vec::new();
    for (i, position) in drops.into_iter().enumerate() {
        let t = i as f32;
        let collider = match i % 3 {
            0 => ColliderDesc::circle(0.3 + 0.1 * ((t * 0.37) % 1.0)),
            1 => ColliderDesc::rectangle(0.5, 0.5),
            _ => ColliderDesc::capsule(vec2(-0.2, 0.0), vec2(0.2, 0.0), 0.2),
        };
        let desc = BodyDesc::dynamic().position(position).angle(t * 1.3);
        let body = world.add_body(desc.collider(collider.friction(0.6).restitution(0.3).density(0.6)));
        shapes.push((body, SHAPES[i % SHAPES.len()]));
    }

    Scene { world, letters, welds, shapes, extent }
}

fn window_conf() -> Conf {
    Conf {
        window_title: "Kinetica logo".to_owned(),
        window_width: WIDTH,
        window_height: HEIGHT,
        high_dpi: false,
        sample_count: 4,
        ..Conf::default()
    }
}

fn draw(scene: &Scene, paper: &Paper, camera: &WorldCamera) {
    let world = &scene.world;
    paper.draw_sheet(PAPER);
    camera.apply();
    let pen = PaperRenderer::new(camera);

    let half_width = 40.0;
    let surface = [vec2(-half_width, 0.0), vec2(half_width, 0.0)];
    pen.draw_ground(&surface, -10.0, DIRT);
    pen.draw_grass(&surface, 0.35, GRASS);

    for &(body, color) in &scene.shapes {
        pen.draw_body(world, body, color);
    }
    for (bodies, color) in &scene.letters {
        for &body in bodies {
            pen.draw_body(world, body, *color);
        }
    }
    // The joints, as ink rivets.
    for &weld in &scene.welds {
        let joint = &world[weld];
        let anchor = world[joint.body_a()].transform().transform_point(joint.local_anchor_a());
        pen.draw_circle(anchor, 0.075, INK);
    }
    set_default_camera();
}

#[macroquad::main(window_conf)]
async fn main() {
    let output = std::env::args().nth(1).unwrap_or_else(|| "docs/kinetica.png".to_owned());
    let mut scene = build();
    let paper = Paper::new();
    let (left, right) = scene.extent;
    let camera = WorldCamera::new(vec2(f32::midpoint(left, right), VIEW_HEIGHT / 2.0 - 0.9), VIEW_HEIGHT);

    for _ in 0..STEPS {
        scene.world.step(1.0 / 60.0);
    }
    // The first frames of a window can come out black: draw a few before saving.
    for frame in 0..10 {
        draw(&scene, &paper, &camera);
        if frame == 9 {
            get_screen_data().export_png(&output);
            println!("saved {output}");
        }
        next_frame().await;
    }
}
