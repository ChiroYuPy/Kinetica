//! The levels, as data: where the sweet hangs, the ropes and springs holding it, the basket,
//! the stars, and the bouncy pads and windmills on the way.

use kinetica::prelude::*;

/// A rope from a pin to the sweet.
#[derive(Debug, Clone, Copy)]
pub struct Rope {
    pub pin: Vec2,
    /// Rope length relative to the distance from the pin to the sweet: above 1, it starts slack.
    pub slack: f32,
}

/// An elastic band from a pin to the sweet, pulling it towards the pin.
#[derive(Debug, Clone, Copy)]
pub struct Elastic {
    pub pin: Vec2,
    pub hertz: f32,
}

/// A fixed pad that bounces the sweet off.
#[derive(Debug, Clone, Copy)]
pub struct Pad {
    pub center: Vec2,
    pub angle: f32,
    pub width: f32,
}

/// A paddle spinning about its center.
#[derive(Debug, Clone, Copy)]
pub struct Windmill {
    pub center: Vec2,
    pub length: f32,
    /// In rad/s, counter-clockwise.
    pub speed: f32,
}

#[derive(Debug, Clone)]
pub struct Level {
    pub name: &'static str,
    pub sweet: Vec2,
    pub ropes: Vec<Rope>,
    pub elastics: Vec<Elastic>,
    /// Middle of the basket's floor.
    pub basket: Vec2,
    pub stars: [Vec2; 3],
    pub pads: Vec<Pad>,
    pub windmills: Vec<Windmill>,
}

impl Level {
    fn new(name: &'static str, sweet: Vec2, basket: Vec2, stars: [Vec2; 3]) -> Self {
        Self {
            name,
            sweet,
            ropes: Vec::new(),
            elastics: Vec::new(),
            basket,
            stars,
            pads: Vec::new(),
            windmills: Vec::new(),
        }
    }

    fn rope(mut self, pin: Vec2, slack: f32) -> Self {
        self.ropes.push(Rope { pin, slack });
        self
    }

    fn elastic(mut self, pin: Vec2, hertz: f32) -> Self {
        self.elastics.push(Elastic { pin, hertz });
        self
    }

    fn pad(mut self, center: Vec2, angle: f32, width: f32) -> Self {
        self.pads.push(Pad { center, angle, width });
        self
    }

    fn windmill(mut self, center: Vec2, length: f32, speed: f32) -> Self {
        self.windmills.push(Windmill { center, length, speed });
        self
    }
}

pub fn levels() -> Vec<Level> {
    vec![
        Level::new("First snip", vec2(12.0, 9.0), vec2(12.0, 0.5), [vec2(12.0, 7.0), vec2(12.0, 5.0), vec2(12.0, 3.0)])
            .rope(vec2(12.0, 12.5), 1.0),
        Level::new("Swing", vec2(7.0, 8.0), vec2(14.5, 0.5), [vec2(8.5, 7.4), vec2(11.0, 6.8), vec2(13.5, 4.5)])
            .rope(vec2(3.5, 12.0), 1.0)
            .rope(vec2(10.0, 12.5), 1.0),
        // The pad, tilted by 22.5°, turns the fall into a 45° throw towards the basket.
        Level::new("Bounce", vec2(5.0, 10.0), vec2(16.4, 0.5), [vec2(5.0, 7.0), vec2(10.5, 6.6), vec2(13.7, 4.9)])
            .rope(vec2(5.0, 13.0), 1.0)
            .pad(vec2(5.5, 3.0), -0.39, 2.5),
        Level::new("Windmill", vec2(6.0, 9.0), vec2(20.0, 0.5), [vec2(9.0, 7.5), vec2(14.0, 9.0), vec2(18.0, 5.0)])
            .rope(vec2(3.0, 12.5), 1.0)
            .elastic(vec2(9.0, 12.5), 1.5)
            .windmill(vec2(11.0, 5.0), 4.0, 2.0),
    ]
}
