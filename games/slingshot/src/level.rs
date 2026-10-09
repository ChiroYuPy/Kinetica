//! The levels: towers of wood, glass and stone, with targets to knock down.

use kinetica::prelude::*;

pub const TARGET_RADIUS: f32 = 0.45;
/// Thickness of posts and beams.
const THICKNESS: f32 = 0.4;

/// What a block is made of: how heavy it is, and how hard it must be hit to break.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Material {
    Wood,
    Glass,
    Stone,
}

impl Material {
    pub fn density(self) -> f32 {
        match self {
            Self::Wood => 1.0,
            Self::Glass => 0.8,
            Self::Stone => 3.0,
        }
    }

    /// Impact speed, in m/s, that breaks the block; stone never breaks.
    pub fn strength(self) -> f32 {
        match self {
            Self::Wood => 7.0,
            Self::Glass => 3.5,
            Self::Stone => f32::INFINITY,
        }
    }

    pub fn points(self) -> u32 {
        match self {
            Self::Wood => 100,
            Self::Glass => 50,
            Self::Stone => 0,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct Block {
    pub center: Vec2,
    pub size: Vec2,
    pub material: Material,
}

impl Block {
    pub fn desc(&self) -> BodyDesc {
        let shape = ColliderDesc::rectangle(self.size.x, self.size.y).density(self.material.density()).friction(0.7);
        BodyDesc::dynamic().position(self.center).collider(shape)
    }
}

#[derive(Debug, Clone, Default)]
pub struct Level {
    pub name: &'static str,
    pub blocks: Vec<Block>,
    /// Centers of the targets.
    pub targets: Vec<Vec2>,
}

impl Level {
    fn new(name: &'static str) -> Self {
        Self { name, ..Self::default() }
    }

    /// A vertical post standing on `bottom`; returns the height of its top.
    fn post(&mut self, x: f32, bottom: f32, height: f32, material: Material) -> f32 {
        let size = vec2(THICKNESS, height);
        self.blocks.push(Block { center: vec2(x, bottom + height / 2.0), size, material });
        bottom + height
    }

    /// A horizontal beam from `left` to `right` lying on `bottom`; returns the height of its top.
    fn beam(&mut self, left: f32, right: f32, bottom: f32, material: Material) -> f32 {
        let size = vec2(right - left, THICKNESS);
        self.blocks.push(Block { center: vec2(f32::midpoint(left, right), bottom + THICKNESS / 2.0), size, material });
        bottom + THICKNESS
    }

    /// A square block of side `size` sitting on `bottom`; returns the height of its top.
    fn cube(&mut self, x: f32, bottom: f32, size: f32, material: Material) -> f32 {
        self.blocks.push(Block { center: vec2(x, bottom + size / 2.0), size: Vec2::splat(size), material });
        bottom + size
    }

    /// Two posts and a beam on top, `width` wide and centered on `x`; returns the roof height.
    fn hut(&mut self, x: f32, bottom: f32, width: f32, height: f32, material: Material) -> f32 {
        let half = (width - THICKNESS) / 2.0;
        self.post(x - half, bottom, height, material);
        self.post(x + half, bottom, height, material);
        self.beam(x - width / 2.0, x + width / 2.0, bottom + height, material)
    }

    fn target(&mut self, x: f32, bottom: f32) {
        self.targets.push(vec2(x, bottom + TARGET_RADIUS));
    }
}

pub fn levels() -> Vec<Level> {
    vec![two_huts(), glass_tower(), stone_fort(), pyramid()]
}

/// Two wooden huts, a target in each and one on a roof.
fn two_huts() -> Level {
    let mut level = Level::new("Two huts");
    level.hut(6.0, 0.0, 2.4, 1.8, Material::Wood);
    level.target(6.0, 0.0);
    let roof = level.hut(11.0, 0.0, 2.4, 1.8, Material::Wood);
    level.target(11.0, 0.0);
    level.target(11.0, roof);
    level
}

/// Three floors, the lower walls glass, a target on each floor.
fn glass_tower() -> Level {
    let mut level = Level::new("Glass tower");
    let mut floor = 0.0;
    for (storey, material) in [Material::Glass, Material::Wood, Material::Glass].into_iter().enumerate() {
        let width = 3.2 - 0.6 * storey as f32;
        level.target(9.0, floor);
        floor = level.hut(9.0, floor, width, 1.6, material);
    }
    level.target(9.0, floor);
    level
}

/// Stone walls protect the targets; the wooden roof and the glass shelves are the weak spots.
fn stone_fort() -> Level {
    let mut level = Level::new("Stone fort");
    for x in [5.0, 9.0, 13.0] {
        level.post(x, 0.0, 2.8, Material::Stone);
    }
    for room in [7.0, 11.0] {
        let shelf = level.hut(room, 0.0, 1.6, 1.0, Material::Glass);
        level.target(room, shelf);
    }
    level.beam(4.8, 9.0, 2.8, Material::Wood);
    let roof = level.beam(9.0, 13.2, 2.8, Material::Wood);
    level.target(9.0, roof);
    level
}

/// A pyramid of crates with targets hidden in it and one on top.
fn pyramid() -> Level {
    let mut level = Level::new("Pyramid");
    let size = 0.9;
    let rows = 5;
    let mut top = 0.0;
    for row in 0..rows {
        let count = rows - row;
        let bottom = row as f32 * size;
        for column in 0..count {
            let x = 9.0 + (column as f32 - (count - 1) as f32 / 2.0) * (size + 0.02);
            let material = if (row + column) % 3 == 0 { Material::Glass } else { Material::Wood };
            top = level.cube(x, bottom, size, material);
        }
    }
    level.target(9.0, top);
    level.target(14.5, 0.0);
    level.hut(14.5, 0.0, 2.0, 1.2, Material::Stone);
    level
}
