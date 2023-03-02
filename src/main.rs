extern crate find_folder;
extern crate opengl_graphics;
extern crate piston_window;

use glam::*;
use opengl_graphics::{GlGraphics, OpenGL};
use piston_window::*;
use rand::{thread_rng, Rng};

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
}

struct RigidBody {
    mass: f64,
    moment_of_inertia: f64,

    position: DVec2,
    velocity: DVec2,
    force: DVec2,

    angle: f64,
    angular_velocity: f64,
    torque: f64,

    shape: RectangleShape,
}

impl RigidBody {
    fn new(shape: RectangleShape, position: DVec2, angle: f64) -> RigidBody {
        let mass = shape.width * shape.height * shape.density;
        let moment_of_inertia =
            mass * (shape.width * shape.width + shape.height * shape.height) / 12.0;
        RigidBody {
            mass,
            moment_of_inertia,
            position,
            velocity: dvec2(0.0, 0.0),
            force: dvec2(0.0, 0.0),
            angle,
            angular_velocity: 0.0,
            torque: 0.0,
            shape,
        }
    }
}

struct Simulation {
    objects: Vec<RigidBody>,

    gl: GlGraphics,
}

impl Simulation {
    fn compute_forces(&mut self, dt: f64) {
        for object in &mut self.objects {
            object.force = dvec2(0.0, 0.0);
            object.force += dvec2(0.0, 50.0 * object.mass);

            object.torque = 0.0;
            object.torque += object.moment_of_inertia;
        }
    }

    fn update(&mut self, args: &UpdateArgs) {
        let dt = args.dt;
        self.compute_forces(dt);
        for object in &mut self.objects {
            object.velocity += object.force / object.mass * dt;
            object.position += object.velocity * dt;

            object.angular_velocity += object.torque / object.moment_of_inertia * dt;
            object.angle += object.angular_velocity * dt;
        }
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
                    .rot_deg(object.angle)
                    .scale(width, height);
                rectangle(object.shape.colour, shape, transform, graphics);
            }
        });
    }
}

fn main() {
    let opengl = OpenGL::V3_2;
    let window: PistonWindow = WindowSettings::new("2D Rigid Body Simulation", [1024, 768])
        .exit_on_esc(true)
        .graphics_api(opengl)
        .build()
        .unwrap();

    let mut simulation = Simulation {
        gl: GlGraphics::new(opengl),
        objects: Vec::new(),
    };

    let mut rng = thread_rng();
    for _ in 0..12 {
        let position = dvec2(rng.gen_range(0.0..1024.0), rng.gen_range(0.0..768.0));
        let angle = rng.gen_range(0.0..360.0);
        let colour: [f32; 4] = [
            rng.gen_range(0.5..1.0),
            rng.gen_range(0.5..1.0),
            rng.gen_range(0.5..1.0),
            1.0,
        ];
        let shape = RectangleShape::new(60.0, 60.0, 1.0, colour);
        let rectangle = RigidBody::new(shape, position, angle);
        simulation.objects.push(rectangle);
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
