use kinetica::core::World;

pub trait Scene {
    fn name(&self) -> &str;
    fn setup(&self, world: &mut World);
}

pub mod scenes {
    use super::Scene;
    use kinetica::collisions::NaiveCollisionDetector;
    use kinetica::core::{RigidBody, Shape, World};
    use kinetica::forces::LinearGravity;
    use kinetica::math::Vec2;

    pub struct FallingBalls;
    pub struct BoxDrop;
    pub struct Pyramid;

    impl Scene for FallingBalls {
        fn name(&self) -> &str { "1. Falling balls" }

        fn setup(&self, world: &mut World) {
            world.add_force_generator(Box::new(LinearGravity { acceleration: Vec2::new(0.0, 200.0) }));
            world.collision_detector = Some(Box::new(NaiveCollisionDetector::new()));

            for i in 0..20 {
                world.add_body(RigidBody::new(
                    Vec2::new(100.0 + i as f32 * 20.0, 100.0),
                    1.0,
                    Shape::Circle(10.0),
                ));
            }
        }
    }

    impl Scene for BoxDrop {
        fn name(&self) -> &str { "2. Box drop" }

        fn setup(&self, world: &mut World) {
            world.add_force_generator(Box::new(LinearGravity { acceleration: Vec2::new(0.0, 500.0) }));
            world.collision_detector = Some(Box::new(NaiveCollisionDetector::new()));

            // Floor
            for i in 0..60 {
                world.add_body(RigidBody::static_body(
                    Vec2::new(i as f32 * 20.0, 700.0),
                    Shape::Circle(10.0),
                ));
            }

            // Boxes
            for i in 0..10 {
                world.add_body(RigidBody::new(
                    Vec2::new(100.0 + i as f32 * 30.0, 100.0),
                    2.0,
                    Shape::Rectangle(Vec2::new(25.0, 25.0)),
                ));
            }
        }
    }

    impl Scene for Pyramid {
        fn name(&self) -> &str { "3. Pyramid" }

        fn setup(&self, world: &mut World) {
            world.add_force_generator(Box::new(LinearGravity { acceleration: Vec2::new(0.0, 500.0) }));
            world.collision_detector = Some(Box::new(NaiveCollisionDetector::new()));

            let size = 8;
            for row in 0..size {
                for col in 0..size - row {
                    world.add_body(RigidBody::new(
                        Vec2::new(640.0 + col as f32 * 22.0, 100.0 + row as f32 * 22.0),
                        1.0,
                        Shape::Circle(10.0),
                    ));
                }
            }
        }
    }
}
