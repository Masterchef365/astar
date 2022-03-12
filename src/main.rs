use anyhow::Result;
use astar::djikstra::djikstra;
use idek::prelude::*;
use rand::prelude::*;
use std::fs::File;
use std::io::BufWriter;
use std::path::PathBuf;
use structopt::StructOpt;

type Coord = (isize, isize, isize);

#[derive(StructOpt, Debug)]
struct Opt {
    #[structopt(short, long, default_value = "100")]
    width: usize,

    #[structopt(short, long, default_value = "100")]
    height: usize,

    #[structopt(short, long, default_value = "100")]
    length: usize,

    /// Number of paths
    #[structopt(short, long, default_value = "400")]
    n_paths: usize,

    /// Maximum number of iterations
    #[structopt(short = "i", long, default_value = "8000")]
    max_iters: usize,

    /// Random seed
    #[structopt(short, long)]
    seed: Option<u64>,

    // /// Lines avoid one another in this radius
    // #[structopt(short = "r", long, default_value = "10")]
    // avoid_radius: isize,
    // /// Minimum path start/end distance
    // #[structopt(short = "d", long, default_value = "5000")]
    // min_path_dist: isize,
}

fn main() -> Result<()> {
    let mut args = Opt::from_args();
    args.height = args.width;
    args.length = args.width;

    let mut obstacles = vec![false; args.width * args.height * args.length];

    let seed = args.seed.unwrap_or_else(|| rand::thread_rng().gen());
    let mut rng = rand::rngs::SmallRng::seed_from_u64(seed);

    // Create path goals (beginnings and ends)
    let goals: Vec<(Coord, Coord)> = (0..args.n_paths)
        .map(|_| {
            (
                random_pos(&mut rng, args.width, args.height, args.length),
                random_pos(&mut rng, args.width, args.height, args.length),
            )
        })
        //.filter(|(begin, end)| dist_sq(*begin, *end) >= args.min_path_dist)
        .take(args.n_paths)
        .collect();

    let mut gb = GraphicsBuilder::new();

    for (goal_idx, (begin, end)) in goals.iter().enumerate() {
        if goal_idx % 1_000 == 0 {
            dbg!(goal_idx);
        }

        let path_color = index_color(goal_idx).map(|v| v as f32 / 256.0);

        // Neighbors
        let neighbors = |coord| {
            neighbors(coord).into_iter().filter(|&neigh| {
                match bounds(neigh, args.width, args.height, args.length) {
                    Some(idx) => !obstacles[idx],
                    None => false,
                }
            })
        };

        let cost = |a, b| 1 + heuristic(b, a, *end);

        // Calculate path
        if let Some(path) = djikstra(cost, neighbors, *begin, *end, args.max_iters) {
            for &coord in &path {
                if let Some(idx) = bounds(coord, args.width, args.height, args.length) {
                    obstacles[idx] = true;
                }
                //output_image[idx * 3..][..3].copy_from_slice(&path_color);
                //for (x, y) in circle((x, y), args.avoid_radius) {
                /*if let Some(idx) = bounds(coord, args.width, args.height, args.length) {
                    obstacles[idx] = true;
                }*/
                //}
            }

            let maxdim = args.width.max(args.height.max(args.length));
            for pair in path.windows(2) {
                let vert = |(x, y, z): Coord| {
                    Vertex::new(
                        [x, y, z]
                            .map(|v| v as f32 / maxdim as f32)
                            .map(|v| v * 2. - 1.),
                        path_color,
                    )
                };
                let begin = gb.push_vertex(vert(pair[0]));
                let end = gb.push_vertex(vert(pair[1]));
                gb.push_indices(&[begin, end]);
            }
        }
    }

    dbg!(gb.vertices.len());
    dbg!(gb.indices.len());

    launch::<_, Visualizer>(Settings::default().args(gb))
}

struct Visualizer {
    verts: VertexBuffer,
    indices: IndexBuffer,
    shader: Shader,
    camera: MultiPlatformCamera,
}

impl App<GraphicsBuilder> for Visualizer {
    fn init(ctx: &mut Context, platform: &mut Platform, gb: GraphicsBuilder) -> Result<Self> {
        let verts = ctx.vertices(&gb.vertices, false)?;
        let indices = ctx.indices(&gb.indices, false)?;
        let shader = ctx.shader(
            DEFAULT_VERTEX_SHADER,
            DEFAULT_FRAGMENT_SHADER,
            Primitive::Lines,
        )?;
        Ok(Self {
            verts,
            indices,
            shader,
            camera: MultiPlatformCamera::new(platform),
        })
    }

    fn frame(&mut self, _ctx: &mut Context, _: &mut Platform) -> Result<Vec<DrawCmd>> {
        Ok(vec![DrawCmd::new(self.verts)
            .indices(self.indices)
            .shader(self.shader)])
    }

    fn event(
        &mut self,
        ctx: &mut Context,
        platform: &mut Platform,
        mut event: Event,
    ) -> Result<()> {
        if self.camera.handle_event(&mut event) {
            ctx.set_camera_prefix(self.camera.get_prefix())
        }
        idek::close_when_asked(platform, &event);
        Ok(())
    }
}

/// Generate a random coordinate
fn random_pos(mut rng: impl Rng, width: usize, height: usize, length: usize) -> Coord {
    (
        rng.gen_range(0..width as isize),
        rng.gen_range(0..height as isize),
        rng.gen_range(0..length as isize),
    )
}

/// If the given coordinate is in bounds, return it's index within an image with the given
/// dimensions
fn bounds((x, y, z): Coord, width: usize, height: usize, length: usize) -> Option<usize> {
    let check = |v: isize, bound: usize| v >= 0 && v < bound as isize;
    (check(x, width) && check(y, height) && check(z, length))
        .then(|| x as usize + y as usize * width + z as usize * width * length)
}

fn heuristic(a: Coord, b: Coord, goal: Coord) -> isize {
    dist_sq(a, goal) - dist_sq(b, goal)
}

/// Squared distance
fn dist_sq((x, y, z): Coord, (goal_x, goal_y, goal_z): Coord) -> isize {
    let dx = x - goal_x;
    let dy = y - goal_y;
    let dz = z - goal_z;

    dx * dx + dy * dy + dz * dz
}

/// Up, down, left, right
fn neighbors((x, y, z): Coord) -> [Coord; 6] {
    [
        (x - 1, y, z),
        (x + 1, y, z),
        (x, y - 1, z),
        (x, y + 1, z),
        (x, y, z - 1),
        (x, y, z + 1),
    ]
}

/// Filled circle with the given center and radius
/*
   fn circle((x, y): Coord, r: isize) -> impl Iterator<Item = Coord> {
// TODO: Make this faster!
(-r..=r)
.map(move |dx| (-r..=r).map(move |dy| (dx, dy)))
.flatten()
.filter(move |(dx, dy)| dx * dx + dy * dy < r * r)
.map(move |(dx, dy)| (x + dx, y + dy))
}
*/
// TODO: Make this a parameter!!
/// Color LUT
fn index_color(idx: usize) -> [u8; 3] {
    let lut: [[i32; 3]; 3] = [[0xFF, 0xEC, 0x04], [0x38, 0xC6, 0xDB], [0xDB, 0x38, 0x83]];
    let base = lut[idx % lut.len()];
    let mut rng = rand::rngs::SmallRng::seed_from_u64(idx as _);
    let diff: i32 = 30;
    let mut component = |idx: usize| (base[idx] + rng.gen_range(-diff..=diff)).clamp(0, 255) as u8;
    [component(0), component(1), component(2)]
}

#[derive(Default, Clone, Debug)]
pub struct GraphicsBuilder {
    pub vertices: Vec<Vertex>,
    pub indices: Vec<u32>,
}

impl GraphicsBuilder {
    pub fn new() -> Self {
        Self::default()
    }

    /// Push a Vertex and return it's index
    pub fn push_vertex(&mut self, v: Vertex) -> u32 {
        let idx: u32 = self
            .vertices
            .len()
            .try_into()
            .expect("Vertex limit exceeded");
        self.vertices.push(v);
        idx
    }

    /// Push an index
    pub fn push_indices(&mut self, idx: &[u32]) {
        self.indices.extend_from_slice(idx);
    }

    /// Erase all content
    pub fn clear(&mut self) {
        self.indices.clear();
        self.vertices.clear();
    }
}
