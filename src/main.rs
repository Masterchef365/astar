use astar::djikstra::djikstra;
use rand::prelude::*;
use std::fs::File;
use std::io::BufWriter;
use std::str::FromStr;
use structopt::StructOpt;
use std::path::PathBuf;
use anyhow::{Result, ensure};

type Coord = (isize, isize);

#[derive(StructOpt, Debug)]
struct Opt {
    #[structopt(short, long, default_value = "1920")]
    width: usize,

    #[structopt(short, long, default_value = "1080")]
    height: usize,

    /// Input RGB PNG image, non-#000000 pixels will be considered obstacles.
    /// Overrides width and height parameters.
    #[structopt(short = "b", long)]
    obstacles: Option<PathBuf>,

    /// Output image path
    #[structopt(short = "o", long, default_value = "out.png")]
    out_path: PathBuf,

    /// Number of paths
    #[structopt(short = "n", long, default_value = "40000")]
    n_paths: usize,

    /// Maximum number of iterations
    #[structopt(short = "i", long, default_value = "8000")]
    max_iters: usize,

    /// Random seed
    #[structopt(short = "s", long)]
    seed: Option<u64>,

    /// Lines avoid one another in this radius
    #[structopt(short = "r", long, default_value = "11")]
    avoid_radius: isize,

    /// Minimum path start/end distance
    #[structopt(short = "d", long, default_value = "5000")]
    min_path_dist: isize,
    
    /// Mirror over X before output
    #[structopt(long, short = "x")]
    mirror_x: bool,

    /// Mirror over Y before output
    #[structopt(long, short = "y")]
    mirror_y: bool,

    // /// Palette
    // #[structopt(long, short = "p")]
    // palette: Vec<HexColor>,
}

/*
struct HexColor([u8; 3]);

impl FromStr for HexColor {

}
*/

fn main() -> Result<()> {
    let args = Opt::from_args();

    // Decode obstacles, if any
    let (width, height, mut obstacles) = match args.obstacles {
        Some(obstacle_path) => {
            let decoder = png::Decoder::new(File::open(obstacle_path)?);
            let mut reader = decoder.read_info()?;
            let mut buf = vec![0; reader.output_buffer_size()];
            let info = reader.next_frame(&mut buf)?;
            assert!(info.color_type == png::ColorType::Rgb);
            assert!(info.bit_depth == png::BitDepth::Eight);
            let input_bytes = &buf[..info.buffer_size()];

            let width = info.width as usize;
            let height = info.height as usize;
            assert_eq!(input_bytes.len(), width * height * 3);
            let mut obstacles = vec![false; width * height];
            obstacles.iter_mut().zip(input_bytes.chunks_exact(3)).for_each(|(ob, inp)| *ob = inp != &[0; 3]);

            (width, height, obstacles)
        },
        None => (args.width, args.height, vec![false; args.width * args.height]),
    };

    let seed = args.seed.unwrap_or_else(|| rand::thread_rng().gen());
    dbg!(seed);
    let mut rng = rand::rngs::SmallRng::seed_from_u64(seed);

    // Create path goals (beginnings and ends)
    let goals: Vec<(Coord, Coord)> = (0..args.n_paths)
        .map(|_| {
            (
                random_pos(&mut rng, width, height),
                random_pos(&mut rng, width, height),
            )
        })
        .filter(|(begin, end)| dist_sq(*begin, *end) >= args.min_path_dist)
        .take(args.n_paths)
        .collect();

    // Write to image
    let mut output_image = vec![0u8; width * height * 3];

    for (goal_idx, (begin, end)) in goals.iter().enumerate() {
        if goal_idx % 1_000 == 0 {
            dbg!(goal_idx);
        }

        let path_color = index_color(goal_idx);

        // Neighbors
        let neighbors = |(x, y)| {
            four_directions((x, y))
                .into_iter()
                .filter(|&(x, y)| match bounds(x, y, width, height) {
                    Some(idx) => !obstacles[idx],
                    None => false,
                })
        };

        let cost = |a, b| 1 + heuristic(b, a, *end);

        // Calculate path
        if let Some(path) = djikstra(cost, neighbors, *begin, *end, args.max_iters) {
            for (x, y) in path {
                if let Some(idx) = bounds(x, y, width, height) {
                    obstacles[idx] = true;
                    output_image[idx * 3..][..3].copy_from_slice(&path_color);
                    for (x, y) in circle((x, y), args.avoid_radius) {
                        if let Some(idx) = bounds(x, y, width, height) {
                            obstacles[idx] = true;
                        }
                    }
                }
            }
        }
    }

    let mut width = width;
    let mut height = height;
    
    if args.mirror_x {
        output_image = mirror_x(&output_image, width);
        height *= 2;
    }
 
    if args.mirror_y {
        output_image = mirror_y(&output_image, width);
        width *= 2;
    }

    // For reading and opening files
    let file = File::create(args.out_path)?;
    let ref mut w = BufWriter::new(file);

    let mut encoder = png::Encoder::new(w, width as _, height as _); // Width is 2 pixels and height is 1.
    encoder.set_color(png::ColorType::Rgb);
    encoder.set_depth(png::BitDepth::Eight);
    let mut writer = encoder.write_header()?;

    writer.write_image_data(&output_image)?; // Save
    Ok(())
}

/// Generate a random coordinate
fn random_pos(mut rng: impl Rng, width: usize, height: usize) -> Coord {
    (
        rng.gen_range(0..width as isize),
        rng.gen_range(0..height as isize),
    )
}

/// If the given coordinate is in bounds, return it's index within an image with the given
/// dimensions
fn bounds(x: isize, y: isize, width: usize, height: usize) -> Option<usize> {
    (x >= 0 && y >= 0 && x < width as isize && y < height as isize)
        .then(|| x as usize + y as usize * width)
}

fn heuristic(a: Coord, b: Coord, goal: Coord) -> isize {
    dist_sq(a, goal) - dist_sq(b, goal)
}

/// Squared distance
fn dist_sq((x, y): Coord, (goal_x, goal_y): Coord) -> isize {
    let dx = x - goal_x;
    let dy = y - goal_y;

    dx * dx + dy * dy
}

/// Up, down, left, right
fn four_directions((x, y): Coord) -> [Coord; 4] {
    [(x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1)]
}

/// Filled circle with the given center and radius 
fn circle((x, y): Coord, r: isize) -> impl Iterator<Item = Coord> {
    // TODO: Make this faster!
    (-r..=r)
        .map(move |dx| (-r..=r).map(move |dy| (dx, dy)))
        .flatten()
        .filter(move |(dx, dy)| dx * dx + dy * dy < r * r)
        .map(move |(dx, dy)| (x + dx, y + dy))
}

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

fn mirror_y(img: &[u8], w: usize) -> Vec<u8> {
    let mut output = Vec::with_capacity(img.len() * 2);
    for row in img.chunks_exact(w*3) {
        output.extend_from_slice(row);
        output.extend(row.iter().rev().copied());
    }
    output
}

fn mirror_x(img: &[u8], w: usize) -> Vec<u8> {
    let mut output = img.to_vec();
    output.reserve(img.len());
    for row in img.chunks_exact(w*3).rev() {
        output.extend_from_slice(row);
    }
    output
}
