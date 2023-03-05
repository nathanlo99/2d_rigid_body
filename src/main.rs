extern crate find_folder;
extern crate opengl_graphics;
extern crate piston_window;

mod bounding_box;
mod intersects;

use bounding_box::BoundingBox;
use glam::*;
use intersects::{ConvexPolygon, Support};
use opengl_graphics::{GlGraphics, OpenGL};
use piston_window::*;
use rand::{thread_rng, Rng};

#[derive(Copy, Clone)]
struct RectangleShape {
    width: f64,
    height: f64,
    density: f64,

    colour: [f32; 4],
}

impl RectangleShape {
    fn new(width: f64, height: f64, density: f64, colour: [f32; 4]) -> RectangleShape {
        RectangleShape {
            width,
            height,
            density,
            colour,
        }
    }

    fn corners(&self) -> [DVec2; 4] {
        [
            dvec2(-self.width / 2.0, -self.height / 2.0),
            dvec2(self.width / 2.0, -self.height / 2.0),
            dvec2(self.width / 2.0, self.height / 2.0),
            dvec2(-self.width / 2.0, self.height / 2.0),
        ]
    }
}

struct RigidBody {
    inv_mass: f64,
    inv_moment_of_inertia: f64,

    position: DVec2,
    velocity: DVec2,
    force: DVec2,

    angle: f64,
    angular_velocity: f64,
    torque: f64,

    shape: RectangleShape,
    fixed: bool,
}

impl RigidBody {
    fn new(shape: RectangleShape, position: DVec2, angle: f64, fixed: bool) -> RigidBody {
        let mass = shape.width * shape.height * shape.density;
        let moment_of_inertia =
            mass * (shape.width * shape.width + shape.height * shape.height) / 12.0;
        RigidBody {
            inv_mass: 1.0 / mass,
            inv_moment_of_inertia: 1.0 / moment_of_inertia,
            position,
            velocity: DVec2::ZERO,
            force: DVec2::ZERO,
            angle,
            angular_velocity: 0.0,
            torque: 0.0,
            shape,
            fixed,
        }
    }

    fn center(&self) -> DVec2 {
        self.position
    }

    fn corners(&self) -> ConvexPolygon {
        let rotate_vector = dvec2(self.angle.cos(), self.angle.sin());
        let vertices = self
            .shape
            .corners()
            .map(|corner| self.position + corner.rotate(rotate_vector))
            .to_vec();
        ConvexPolygon { vertices }
    }

    fn bounding_box(&self) -> bounding_box::BoundingBox {
        let corners = self.corners().vertices;
        // TODO: Figure out how to do this with .iter().reduce()
        let mut min = corners[0];
        let mut max = corners[0];
        for transformed_corner in &corners[1..] {
            min = min.min(*transformed_corner);
            max = max.max(*transformed_corner);
        }
        bounding_box::BoundingBox { min, max }
    }

    fn intersects(&self, other: &Self) -> bool {
        self.bounding_box().intersects(&other.bounding_box())
            && intersects::intersects(&self.corners(), &other.corners()).is_some()
    }

    fn clear_forces(&mut self) {
        self.force = DVec2::ZERO;
        self.torque = 0.0;
    }
}

struct Simulation {
    objects: Vec<RigidBody>,

    gl: GlGraphics,
}

impl Simulation {
    fn new(width: f64, height: f64, gl: GlGraphics) -> Self {
        let shape = RectangleShape::new(width, height, 1000000.0, [0.5, 0.5, 0.5, 1.0]);
        let top = RigidBody::new(shape, dvec2(width / 2.0, -height / 2.0), 0.0, true);
        let bottom = RigidBody::new(shape, dvec2(width / 2.0, height * 1.5), 0.0, true);
        let left = RigidBody::new(shape, dvec2(-width / 2.0, height / 2.0), 0.0, true);
        let right = RigidBody::new(shape, dvec2(width * 1.5, height / 2.0), 0.0, true);
        let objects = vec![top, bottom, left, right];
        Simulation { objects, gl }
    }

    fn compute_forces(&mut self, _dt: f64) {
        self.objects.iter_mut().for_each(RigidBody::clear_forces);

        // Apply gravity
        self.objects.iter_mut().for_each(|object| {
            object.force += 500.0 / object.inv_mass * DVec2::Y;
        });
    }

    fn update(&mut self, args: &UpdateArgs) {
        let dt = args.dt;
        let num_steps = 16;
        for _ in 0..num_steps {
            self.step(dt / num_steps as f64);
        }
    }

    fn colliding_objects(&self) -> Vec<(usize, usize, DVec2)> {
        let mut result = Vec::new();
        let n = self.objects.len();
        for i in 0..n {
            for j in i + 1..n {
                let object1 = &self.objects[i];
                let object2 = &self.objects[j];

                if self.objects[i].fixed && self.objects[j].fixed {
                    continue;
                }
                let object1_corners = object1.corners();
                let object2_corners = object2.corners();
                if let Some(direction) =
                    intersects::intersection_direction(&object1_corners, &object2_corners)
                {
                    if direction.length() > 0.0001 {
                        result.push((i, j, direction));
                    }
                }
            }
        }
        result
    }

    fn resolve_collisions(&mut self) {
        for (i, j, direction) in self.colliding_objects() {
            let object1 = &self.objects[i];
            let object2 = &self.objects[j];

            let object1_move_factor = if object2.fixed {
                1.0
            } else {
                1.0 / (1.0 + object2.inv_mass / object1.inv_mass)
            };
            let object2_move_factor = if object1.fixed {
                1.0
            } else {
                1.0 / (1.0 + object1.inv_mass / object2.inv_mass)
            };

            if !self.objects[i].fixed {
                self.objects[i].position -= direction * object1_move_factor;
            }
            if !self.objects[j].fixed {
                self.objects[j].position += direction * object2_move_factor;
            }

            let n = -direction.normalize();
            let contact_point = self.objects[j].corners().support_point(n);

            let inv_mass1 = self.objects[i].inv_mass;
            let inv_mass2 = self.objects[j].inv_mass;
            let velocity1 = self.objects[i].velocity;
            let velocity2 = self.objects[j].velocity;

            let rap = contact_point - self.objects[i].center();
            let rap_perp = rap.rotate(dvec2(0.0, 1.0));
            let rap_perp_dot_n = rap_perp.dot(n);

            let rbp = contact_point - self.objects[j].center();
            let rbp_perp = rbp.rotate(dvec2(0.0, 1.0));
            let rbp_perp_dot_n = rbp_perp.dot(n);
            let epsilon = 0.20;

            let angular_contribution1 =
                rap_perp_dot_n * rap_perp_dot_n * self.objects[i].inv_moment_of_inertia;
            let angular_contribution2 =
                rbp_perp_dot_n * rbp_perp_dot_n * self.objects[j].inv_moment_of_inertia;

            let num = -(1.0 + epsilon) * (velocity2 - velocity1).dot(n);
            let denom = n.length_squared() * (inv_mass1 + inv_mass2)
                + angular_contribution1
                + angular_contribution2;

            let impulse = num / denom;
            let impulse = if impulse.is_nan() { 0.0 } else { impulse };
            if !self.objects[i].fixed {
                self.objects[i].velocity -= impulse * inv_mass1 * n;
                self.objects[i].angular_velocity -=
                    rap_perp.dot(n) * impulse * self.objects[i].inv_moment_of_inertia;
            }

            if !self.objects[j].fixed {
                self.objects[j].velocity += impulse * inv_mass2 * n;
                self.objects[j].angular_velocity +=
                    rbp_perp.dot(n) * impulse * self.objects[j].inv_moment_of_inertia;
            }
        }
    }

    fn step(&mut self, dt: f64) {
        self.compute_forces(dt);

        for object in &mut self.objects {
            if object.fixed {
                continue;
            }

            object.velocity += object.force * object.inv_mass * dt;
            object.position += object.velocity * dt;

            object.angular_velocity += object.torque * object.inv_moment_of_inertia * dt;
            object.angle += object.angular_velocity * dt;
        }
        self.resolve_collisions();
    }

    fn render(&mut self, args: &RenderArgs) {
        self.gl.draw(args.viewport(), |context, graphics| {
            clear([1.0; 4], graphics);

            for object in &self.objects {
                let width = object.shape.width;
                let height = object.shape.height;
                let x = object.position.x;
                let y = object.position.y;

                let shape = rectangle::square(-0.5, -0.5, 1.0);
                let transform = context
                    .transform
                    .trans(x, y)
                    .rot_rad(object.angle)
                    .scale(width, height);
                rectangle(object.shape.colour, shape, transform, graphics);
            }
        });
    }
}

fn main() {
    let opengl = OpenGL::V3_2;
    let (width, height) = (1024, 768);
    let window: PistonWindow = WindowSettings::new("2D Rigid Body Simulation", [width, height])
        .exit_on_esc(true)
        .graphics_api(opengl)
        .build()
        .unwrap();

    let mut simulation = Simulation::new(width as f64, height as f64, GlGraphics::new(opengl));

    let window_bounding_box = BoundingBox {
        min: dvec2(0.0, 0.0),
        max: dvec2(width as f64, height as f64),
    };
    let mut rng = thread_rng();

    let num_boxes = 1;
    while simulation.objects.len() < 4 + num_boxes {
        let position = dvec2(
            rng.gen_range(0.0..width as f64),
            rng.gen_range(0.0..height as f64),
        );
        let angle = rng.gen_range(0.0..360.0);

        let colour: [f32; 4] = [
            rng.gen_range(0.5..0.9),
            rng.gen_range(0.5..0.9),
            rng.gen_range(0.5..0.9),
            1.0,
        ];
        let width = rng.gen_range(50.0..100.0);
        let height = rng.gen_range(50.0..100.0);
        let shape = RectangleShape::new(width, height, 1.0, colour);
        let rectangle = RigidBody::new(shape, position, angle, false);

        let intersects_existing = simulation
            .objects
            .iter()
            .any(|object| rectangle.intersects(object));
        let completely_contained = window_bounding_box.contains(&rectangle.bounding_box());
        if completely_contained && !intersects_existing {
            simulation.objects.push(rectangle);
        }
    }

    for event in window {
        if let Some(args) = event.update_args() {
            simulation.update(&args);
        }
        if let Some(args) = event.render_args() {
            simulation.render(&args);
        }
    }
}
