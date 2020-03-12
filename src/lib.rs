mod utils;

use euclid::default::*;
use std::mem::swap;
use wasm_bindgen::prelude::*;

// When the `wee_alloc` feature is enabled, use `wee_alloc` as the global
// allocator.
#[cfg(feature = "wee_alloc")]
#[global_allocator]
static ALLOC: wee_alloc::WeeAlloc = wee_alloc::WeeAlloc::INIT;

fn init() {
    // console_log::init_with_level(log::Level::Debug);
    utils::set_panic_hook();
}

const BUCKET_BOX_SIZE: u32 = 64;

#[wasm_bindgen]
pub struct App {
    h: f64,
    width: f64,
    height: f64,
    wall: Wall,
    state: Vec<BallState>,
    params: Vec<BallParams>,
    slope_cur: Vec<BallSlope>,
    slope_next: Vec<BallSlope>,
    slope_sum: Vec<BallSlope>,
    // bucket: Vec<Vec<usize>>,
    // need_update_bucket
    env: Env,
    frame: usize,
    time: f64,
}

#[wasm_bindgen]
impl App {
    #[wasm_bindgen(constructor)]
    pub fn new(h: f64, width: f64, height: f64) -> Self {
        init();

        Self {
            h,
            width,
            height,
            wall: Wall {
                left: 0.0,
                right: width,
                top: 0.0,
                bottom: height,
            },
            state: Vec::new(),
            params: Vec::new(),
            slope_cur: Vec::new(),
            slope_next: Vec::new(),
            slope_sum: Vec::new(),
            // bucket: std::iter::repeat_with(|| Vec::new()).take(BUCKET_BOX_SIZE),
            env: Env {
                gravity: Vector2D::new(0., 50.0),
                drag: 0.0,
            },
            frame: 0,
            time: 0.,
        }
    }

    pub fn balls_len(&self) -> usize {
        self.state.len()
    }

    pub fn state_ptr(&self) -> *const BallState {
        self.state.as_ptr()
    }

    pub fn params_ptr(&self) -> *const BallParams {
        self.params.as_ptr()
    }

    pub fn add_ball(&mut self, x: f64, y: f64, vx: f64, vy: f64, rad: f64, mass: f64, k: f64) {
        self.state.push(BallState {
            pos: Vector2D::new(x, y),
            vel: Vector2D::new(vx, vy),
        });
        self.slope_cur.push(BallSlope::default());
        self.slope_next.push(BallSlope::default());
        self.slope_sum.push(BallSlope::default());
        self.params.push(BallParams { rad, mass, k });
    }

    pub fn set_gravity(&mut self, gx: f64, gy: f64) {
        self.env.gravity = Vector2D::new(gx, gy);
    }

    pub fn set_drag(&mut self, drag: f64) {
        self.env.drag = drag;
    }

    pub fn set_left_wall(&mut self, x: f64) {
        self.wall.left = x;
    }
    pub fn set_right_wall(&mut self, x: f64) {
        self.wall.right = x;
    }
    pub fn set_top_wall(&mut self, y: f64) {
        self.wall.top = y;
    }
    pub fn set_bottom_wall(&mut self, y: f64) {
        self.wall.bottom = y;
    }

    pub fn tick(&mut self) {
        self.frame += 1;
        self.time = self.frame as f64 * self.h;

        for cur in &mut self.slope_cur {
            *cur = BallSlope::default();
        }
        for sum in &mut self.slope_sum {
            *sum = BallSlope::default();
        }
        for i in 0..4 {
            self.update_slope(self.h * [0., 0.5, 0.5, 1.0][i]);
            swap(&mut self.slope_cur, &mut self.slope_next);
            let c = [1., 2., 2., 1.][i] / 6.;
            for (cur, sum) in self.slope_cur.iter().zip(self.slope_sum.iter_mut()) {
                sum.vel += cur.vel * c;
                sum.acc += cur.acc * c;
            }
        }
        for (state, slope) in self.state.iter_mut().zip(self.slope_sum.iter()) {
            state.pos += slope.vel * self.h;
            state.vel += slope.acc * self.h;
        }
    }

    fn update_slope(&mut self, h: f64) {
        let len = self.balls_len();
        for i in 0..len {
            if let (Some(state), Some(params), Some(cur), Some(next)) = (
                self.state.get(i),
                self.params.get(i),
                self.slope_cur.get(i),
                self.slope_next.get_mut(i),
            ) {
                let pos = state.pos + cur.vel * h;
                let vel = state.vel + cur.acc * h;

                next.vel = vel;

                // wall
                let mut f = Vector2D::zero();
                if pos.x - params.rad <= self.wall.left {
                    let d = pos.x - params.rad - self.wall.left;
                    f += Vector2D::new(-params.k * d, 0.);
                }
                if pos.x + params.rad >= self.wall.right {
                    let d = pos.x + params.rad - self.wall.right;
                    f += Vector2D::new(-params.k * d, 0.);
                }
                if pos.y - params.rad <= self.wall.top {
                    let d = pos.y - params.rad - self.wall.top;
                    f += Vector2D::new(0., -params.k * d);
                }
                if pos.y + params.rad >= self.wall.bottom {
                    let d = pos.y + params.rad - self.wall.bottom;
                    f += Vector2D::new(0., -params.k * d);
                }

                // ball
                for j in (0..len).filter(|&j| j != i) {
                    if let (Some(state_o), Some(params_o), Some(cur_o)) = (self.state.get(j), self.params.get(j), self.slope_cur.get(j)) {
                        let pos_o = state_o.pos + cur_o.vel * h;
                        let p = pos_o - pos;
                        let r = params.rad + params_o.rad;
                        if p.square_length() <= r * r {
                            let d = p - p.normalize() * r;
                            f += d * params.k / 2.0 /* fix later!!!!!!!!!!!!! */; 
                        }
                    }
                }

                f -= vel * self.env.drag;
                next.acc = f / params.mass + self.env.gravity;
            }
        }
    }

    pub fn kinetic_energy_sum(&self) -> f64 {
        let mut k = 0.0;
        for i in 0..self.balls_len() {
            if let (Some(state), Some(params)) = (self.state.get(i), self.params.get(i)) {
                k += params.mass * state.vel.square_length();
            }
        }
        k / 2.0
    }

    pub fn gravity_potential_energy_sum(&self) -> f64 {
        // wip
        let mut e = 0.0;
        for i in 0..self.balls_len() {
            if let (Some(state), Some(params)) = (self.state.get(i), self.params.get(i)) {
                e += params.mass * self.env.gravity.y * (self.height - state.pos.y);
            }
        }
        e
    }

    pub fn wall(&self) -> Wall {
        self.wall
    }
}

#[repr(C)]
#[derive(Default)]
pub struct BallState {
    pos: Vector2D<f64>,
    vel: Vector2D<f64>,
}

#[derive(Default)]
struct BallSlope {
    vel: Vector2D<f64>,
    acc: Vector2D<f64>,
}

#[repr(C)]
pub struct BallParams {
    rad: f64,
    mass: f64,
    k: f64,
}

#[wasm_bindgen]
#[derive(Clone, Copy)]
pub struct Wall {
    pub left: f64,
    pub right: f64,
    pub top: f64,
    pub bottom: f64,
}

struct Env {
    gravity: Vector2D<f64>,
    drag: f64,
}
