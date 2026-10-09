use std::collections::VecDeque;

use kinetica::prelude::*;
use kinetica::{Color, SensorEvent};

use super::{Build, Category, Frame, Labels, Scene, SceneUpdate, add_ground};
use kinetica_macroquad::DebugRenderer;

pub const SCENE: Scene = Scene {
    name: "Sensors",
    category: Category::GameFeatures,
    description: "Sensors detect what overlaps them without pushing it. The zones light up while occupied, the \
                  sweeping arm's tip too, and flash when something enters. Bodies inside a sensor are outlined.",
    camera_center: vec2(0.0, 5.0),
    view_height: 15.0,
    build: Build::Animated(build),
};

const IDLE_COLOR: Color = Color::rgb(0.5, 0.5, 0.6);
const OCCUPIED_COLOR: Color = Color::rgb(0.3, 0.9, 0.6);
const ENTERED_COLOR: Color = Color::rgb(1.0, 1.0, 0.5);
const VISITOR_COLOR: Color = Color::rgb(0.3, 0.9, 0.6);
/// How long a sensor flashes after something enters it, in seconds.
const FLASH_TIME: f32 = 0.15;
const SPAWN_PERIOD: f32 = 0.35;
const MAX_BODIES: usize = 40;

fn build(world: &mut World, labels: &mut Labels) -> Box<dyn SceneUpdate> {
    add_ground(world, 20.0);

    let zones = world.add_body(BodyDesc::fixed());
    let mut sensors = Vec::new();
    for center in [vec2(-4.0, 6.0), vec2(4.0, 3.0)] {
        sensors.push(world.add_collider(zones, ColliderDesc::offset_rectangle(2.5, 1.2, center, 0.0).sensor(true)));
    }

    // A kinematic arm sweeping a sensor disc through the rain.
    let arm = world.add_body(BodyDesc::kinematic().position(vec2(0.0, 4.5)).angular_velocity(0.8));
    world.add_collider(arm, ColliderDesc::rectangle(3.0, 0.08).sensor(true));
    sensors.push(world.add_collider(
        arm,
        ColliderDesc::new(kinetica::shape::Circle::new(0.8).with_center(vec2(3.5, 0.0))).sensor(true),
    ));

    labels.above(arm, "Sensor on a kinematic arm");

    let flashes = vec![f32::NEG_INFINITY; sensors.len()];
    Box::new(Zones { sensors, flashes, bodies: VecDeque::new(), spawned: 0, next_spawn: 0.0, time: 0.0 })
}

struct Zones {
    sensors: Vec<ColliderHandle>,
    /// When each sensor last saw something enter.
    flashes: Vec<f32>,
    bodies: VecDeque<BodyHandle>,
    spawned: usize,
    next_spawn: f32,
    time: f32,
}

impl SceneUpdate for Zones {
    fn update(&mut self, world: &mut World, frame: &Frame) {
        // Events of the previous step.
        for event in world.sensor_events() {
            if let SensorEvent::Entered(pair) = event
                && let Some(index) = self.sensors.iter().position(|&sensor| sensor == pair.sensor)
            {
                self.flashes[index] = frame.time;
            }
        }
        self.time = frame.time;

        if frame.time < self.next_spawn {
            return;
        }
        self.next_spawn += SPAWN_PERIOD;
        let i = self.spawned;
        self.spawned += 1;
        let x = ((i * 7) % 17) as f32 - 8.0;
        let body = world.add_body(BodyDesc::dynamic().position(vec2(x, 13.0)).angular_velocity(1.0));
        let collider =
            if i.is_multiple_of(2) { ColliderDesc::circle(0.35) } else { ColliderDesc::rectangle(0.35, 0.35) };
        world.add_collider(body, collider);
        self.bodies.push_back(body);
        if self.bodies.len() > MAX_BODIES
            && let Some(oldest) = self.bodies.pop_front()
        {
            world.remove_body(oldest);
        }
    }

    fn draw(&self, world: &World, _frame: &Frame, drawer: &mut DebugRenderer) {
        for (&sensor, &flash) in self.sensors.iter().zip(&self.flashes) {
            let color = if self.time - flash < FLASH_TIME {
                ENTERED_COLOR
            } else if world.sensor_overlaps(sensor).next().is_some() {
                OCCUPIED_COLOR
            } else {
                IDLE_COLOR
            };
            drawer.draw_collider(world, sensor, color.with_alpha(0.5));
            for pair in world.sensor_overlaps(sensor) {
                drawer.draw_collider(world, pair.visitor, VISITOR_COLOR);
            }
        }
    }

    fn labels(&self, world: &World, _frame: &Frame, labels: &mut Labels) {
        // The fixed zones; the arm's tip moves too much to carry a label.
        for &sensor in &self.sensors[..2] {
            let Some(collider) = world.collider(sensor) else { continue };
            let aabb = collider.aabb();
            let inside = world.sensor_overlaps(sensor).count();
            labels.at(
                vec2(f32::midpoint(aabb.min.x, aabb.max.x), aabb.max.y + 0.15),
                format!("Fixed sensor: {inside} inside"),
            );
        }
    }
}
