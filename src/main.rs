use astar::djikstra::djikstra;
use rand::prelude::*;
use std::fs::File;
use std::io::BufWriter;

type Coord = (isize, isize);

fn main() {
    let width = 150;
    let height = 150;
    let mut obstacles = vec![false; width * height];
    let image_path = "out.png";

    let n_paths = 355;

    let mut rng = rand::thread_rng();

    fn random_pos(mut rng: impl Rng, width: usize, height: usize) -> Coord {
        (
            rng.gen_range(0..width as isize),
            rng.gen_range(0..height as isize),
        )
    }

    let mut goals: Vec<Option<(Coord, Coord)>> = (0..n_paths)
        .map(|_| {
            Some((
                random_pos(&mut rng, width, height),
                random_pos(&mut rng, width, height),
            ))
        })
        .collect();

    // Write to image
    let mut output_image = vec![0u8; width * height * 3];

    loop {
        for (goal_idx, goal) in goals.iter_mut().enumerate() {
            let (begin, end) = match goal.as_mut() {
                Some(g) => g,
                None => continue,
            };

            if *begin == *end {
                *goal = None;
                continue;
            }

            obstacles[bounds(begin.0, begin.1, width, height).unwrap()] = true;

            let path_color = index_color(goal_idx);

            // Neighbors
            let neighbors = |(x, y)| {
                four_directions((x, y))
                    //eight_directions((x, y))
                    .into_iter()
                    .filter(|&(x, y)| match bounds(x, y, width, height) {
                        Some(idx) => !obstacles[idx],
                        None => false,
                    })
            };

            let cost = |a, b| 1 + heuristic(b, a, *end);

            // Calculate path
            if let Some(path) = djikstra(cost, neighbors, *begin, *end) {
                let len = path.len();
                let next_step = path[len - 2];
                /*
                   output_image
                   .chunks_exact_mut(3)
                   .zip(obstacles)
                   .filter(|(_, b)| *b)
                   .for_each(|(u, _)| u[1] = 0xff);
                   */

                let (x, y) = next_step;
                //for (x, y) in path {
                if let Some(idx) = bounds(x, y, width, height) {
                    output_image[idx * 3..][..3].copy_from_slice(&path_color);
                }
                *begin = next_step;
                //}
            } else {
                *goal = None;
            }
        }

        if goals.iter().all(|g| g.is_none()) {
            break;
        }

        dbg!(goals.iter().filter(|g| g.is_some()).count());
        //dbg!(&goals);
    }

    // For reading and opening files
    let file = File::create(image_path).unwrap();
    let ref mut w = BufWriter::new(file);

    let mut encoder = png::Encoder::new(w, width as _, height as _); // Width is 2 pixels and height is 1.
    encoder.set_color(png::ColorType::Rgb);
    encoder.set_depth(png::BitDepth::Eight);
    let mut writer = encoder.write_header().unwrap();

    writer.write_image_data(&output_image).unwrap(); // Save
}

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

/// Adjacent and diagonally adjacent pixels
fn eight_directions((x, y): Coord) -> [Coord; 8] {
    [
        (x - 1, y),
        (x + 1, y),
        (x + 1, y - 1),
        (x, y - 1),
        (x - 1, y - 1),
        (x + 1, y + 1),
        (x, y + 1),
        (x - 1, y + 1),
    ]
}

fn index_color(idx: usize) -> [u8; 3] {
    let lut: [[i32; 3]; 3] = [[0xFF, 0xEC, 0x04], [0x38, 0xC6, 0xDB], [0xDB, 0x38, 0x83]];
    let base = lut[idx % lut.len()];
    let mut rng = rand::rngs::SmallRng::seed_from_u64(idx as _);
    let diff: i32 = 50;
    let mut component = |idx: usize| (base[idx] + rng.gen_range(-diff..=diff)).clamp(0, 255) as u8;
    [
        component(0),
        component(1),
        component(2),
    ]
}
