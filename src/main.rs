mod bounding_box;
mod intersects;
mod rigid_body;

use bounding_box::BoundingBox;
use intersects::{ConvexPolygon, *};
use rigid_body::*;

use glam::*;
use opengl_graphics::{GlGraphics, GlyphCache, OpenGL};
use piston_window::*;
use rand::{thread_rng, Rng};

use std::cmp::Ordering;
use std::collections::HashSet;
use std::f64::consts::TAU;

struct Simulation {
    objects: Vec<RigidBody>,

    gl: GlGraphics,
}

impl Simulation {
    fn new(width: f64, height: f64, gl: GlGraphics) -> Self {
        let shape = RectangleShape::new(width, height, f64::INFINITY, [0.5, 0.5, 0.5, 1.0]);
        let top = RigidBody::new(shape, dvec2(width / 2.0, -height / 2.0), 0.0, true);
        let bottom = RigidBody::new(shape, dvec2(width / 2.0, height * 1.5), 0.0, true);
        let left = RigidBody::new(shape, dvec2(-width / 2.0, height / 2.0), 0.0, true);
        let right = RigidBody::new(shape, dvec2(width * 1.5, height / 2.0), 0.0, true);
        let objects = vec![top, bottom, left, right];
        Simulation { objects, gl }
    }

    fn energy(&self) -> f64 {
        self.objects
            .iter()
            .filter(|object| !object.fixed)
            .map(|object| {
                let height = 768.0 - object.position.y;
                let gravity = 500.0;
                let potential_energy = height * gravity / object.shape.inv_mass();
                let linear_energy =
                    0.5 * object.velocity.length_squared() / object.shape.inv_mass();
                let angular_energy = 0.5 * object.angular_velocity * object.angular_velocity
                    / object.shape.inv_moment_of_inertia();
                potential_energy + linear_energy + angular_energy
            })
            .sum()
    }

    fn compute_forces(&mut self, _dt: f64) {
        self.objects.iter_mut().for_each(RigidBody::clear_forces);

        // Apply gravity
        self.objects
            .iter_mut()
            .for_each(|object| object.force += 500.0 / object.shape.inv_mass() * DVec2::Y);
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

        let corners: Vec<ConvexPolygon> = self.objects.iter().map(RigidBody::corners).collect();
        let bounding_boxes: Vec<BoundingBox> =
            corners.iter().map(Intersectable::bounding_box).collect();

        let mut x_coordinates: Vec<(f64, bool, usize)> = bounding_boxes
            .iter()
            .enumerate()
            .flat_map(|(idx, bounding_box)| {
                let start_x = bounding_box.min.x;
                let end_x = bounding_box.max.x;
                [(start_x, false, idx), (end_x, true, idx)]
            })
            .collect();

        x_coordinates.sort_unstable_by(|(x1, is_end1, idx1), (x2, is_end2, idx2)| -> Ordering {
            match x1.partial_cmp(x2).unwrap() {
                Ordering::Equal => Ord::cmp(&(is_end1, idx1), &(is_end2, idx2)),
                Ordering::Greater => Ordering::Greater,
                Ordering::Less => Ordering::Less,
            }
        });

        let mut active_indices: HashSet<usize> = HashSet::new();
        for (_x, is_end, idx) in x_coordinates {
            if is_end {
                active_indices.remove(&idx);
                continue;
            }

            for other_idx in &active_indices {
                if !bounding_boxes[idx].intersects(&bounding_boxes[*other_idx]) {
                    continue;
                }
                if let Some(direction) =
                    intersects::intersection_direction(&corners[idx], &corners[*other_idx])
                {
                    if direction.length() > 0.0001 {
                        result.push((idx, *other_idx, direction));
                    }
                }
            }
            active_indices.insert(idx);
        }
        result
    }

    fn resolve_collisions(&mut self) {
        for (i, j, direction) in self.colliding_objects() {
            let object1 = &self.objects[i];
            let object2 = &self.objects[j];

            let inv_mass1 = object1.shape.inv_mass();
            let inv_mass2 = object2.shape.inv_mass();
            let velocity1 = self.objects[i].velocity;
            let velocity2 = self.objects[j].velocity;
            let inv_moment_of_inertia1 = object1.shape.inv_moment_of_inertia();
            let inv_moment_of_inertia2 = object2.shape.inv_moment_of_inertia();

            let object1_move_factor =
                1.0 / (1.0 + object2.shape.inv_mass() / object1.shape.inv_mass());
            let object2_move_factor = 1.0 - object1_move_factor;

            self.objects[i].position -= direction * object1_move_factor;
            self.objects[j].position += direction * object2_move_factor;

            let n = -direction;
            let contact_point =
                intersects::contact_point(&self.objects[i].corners(), &self.objects[j].corners());

            let rap = contact_point - self.objects[i].center();
            let rap_perp = rap.rotate(dvec2(0.0, 1.0));
            let rap_perp_dot_n = rap_perp.dot(n);

            let rbp = contact_point - self.objects[j].center();
            let rbp_perp = rbp.rotate(dvec2(0.0, 1.0));
            let rbp_perp_dot_n = rbp_perp.dot(n);
            let epsilon = 1.0;

            let angular_contribution1 = rap_perp_dot_n * rap_perp_dot_n * inv_moment_of_inertia1;
            let angular_contribution2 = rbp_perp_dot_n * rbp_perp_dot_n * inv_moment_of_inertia2;

            let num = -(1.0 + epsilon) * (velocity2 - velocity1).dot(n);
            let denom = n.length_squared() * (inv_mass1 + inv_mass2)
                + angular_contribution1
                + angular_contribution2;

            let impulse = num / denom;
            if !self.objects[i].fixed {
                self.objects[i].velocity -= impulse * inv_mass1 * n;
                self.objects[i].angular_velocity -=
                    rap_perp.dot(n) * impulse * self.objects[i].shape.inv_moment_of_inertia();
            }

            if !self.objects[j].fixed {
                self.objects[j].velocity += impulse * inv_mass2 * n;
                self.objects[j].angular_velocity +=
                    rbp_perp.dot(n) * impulse * self.objects[j].shape.inv_moment_of_inertia();
            }
        }
    }

    fn step(&mut self, dt: f64) {
        self.compute_forces(dt);

        for object in &mut self.objects {
            if object.fixed {
                continue;
            }

            object.velocity += object.force * object.shape.inv_mass() * dt;
            object.position += object.velocity * dt;

            object.angular_velocity += object.torque * object.shape.inv_moment_of_inertia() * dt;
            object.angle += object.angular_velocity * dt;
        }
        self.resolve_collisions();
    }

    fn render(&mut self, args: &RenderArgs) {
        let mut glyphs = GlyphCache::new("assets/FiraSans-Regular.ttf", (), TextureSettings::new())
            .expect("Could not load font");
        let energy = self.energy();

        self.gl.draw(args.viewport(), |context, graphics| {
            clear([1.0; 4], graphics);

            for (idx, object) in self.objects.iter().enumerate() {
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

                let transform = context.transform.trans(x - 5.0, y + 5.0);

                text(
                    [0.0, 0.0, 0.0, 1.0],
                    20,
                    format!("{idx}").as_str(),
                    &mut glyphs,
                    transform,
                    graphics,
                )
                .expect("Could not print");
            }

            let transform = context.transform.trans(5.0, 25.0);
            text(
                [0.0, 0.0, 0.0, 1.0],
                20,
                format!("Energy: {energy:13.2}").as_str(),
                &mut glyphs,
                transform,
                graphics,
            )
            .expect("Could not print");
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

    let num_boxes = 1000;
    let mut num_failures = 0;
    while simulation.objects.len() < 4 + num_boxes && num_failures < 100 {
        let position = dvec2(
            rng.gen_range(0.0..width as f64),
            rng.gen_range(0.0..height as f64),
        );

        let colour: [f32; 4] = [
            rng.gen_range(0.5..0.9),
            rng.gen_range(0.5..0.9),
            rng.gen_range(0.5..0.9),
            1.0,
        ];
        let width: f64 = rng.gen_range(50.0..100.0);
        let height: f64 = rng.gen_range(50.0..100.0);
        let angle = rng.gen_range(0.0..TAU);
        let shape = RectangleShape::new(width, height, 1.0, colour);
        let rectangle = RigidBody::new(shape, position, angle, false);

        let intersects_existing = simulation
            .objects
            .iter()
            .any(|object| rectangle.intersects(object));
        let completely_contained =
            window_bounding_box.contains(&rectangle.corners().bounding_box());
        if completely_contained && !intersects_existing {
            simulation.objects.push(rectangle);
            num_failures = 0;
        } else {
            num_failures += 1;
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
