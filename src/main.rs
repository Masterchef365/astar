use astar::djikstra::djikstra;
use rand::prelude::*;
use std::fs::File;
use std::io::BufWriter;

type Coord = (isize, isize);

fn main() {
    let input_path = std::env::args().skip(1).next().unwrap();
    // The decoder is a build for reader and can be used to set various decoding options
    // via `Transformations`. The default output transformation is `Transformations::IDENTITY`.
    let decoder = png::Decoder::new(File::open(input_path).unwrap());
    let mut reader = decoder.read_info().unwrap();
    // Allocate the output buffer.
    let mut buf = vec![0; reader.output_buffer_size()];
    // Read the next frame. An APNG might contain multiple frames.
    let info = reader.next_frame(&mut buf).unwrap();
    assert!(info.color_type == png::ColorType::Rgb);
    assert!(info.bit_depth == png::BitDepth::Eight);
    // Grab the bytes of the image.
    let input_bytes = &buf[..info.buffer_size()];


    let width = info.width as usize;
    let height = info.height as usize;
    assert_eq!(input_bytes.len(), width * height * 3);
    let mut obstacles = vec![false; width * height];
    obstacles.iter_mut().zip(input_bytes.chunks_exact(3)).for_each(|(ob, inp)| *ob = inp == &[0; 3]);

    let output_path = "out.png";
    let max_iters = 4999;
    let seed = 4328070342;

    let min_path_len = (width + height) * 2;
    let n_paths = 400_000;

    let mut rng = rand::rngs::SmallRng::seed_from_u64(seed);

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

    for (goal_idx, goal) in goals.iter_mut().enumerate() {
        if goal_idx % 1_000 == 0 {
            dbg!(goal_idx);
        }

        let (begin, end) = match goal.as_mut() {
            Some(g) => g,
            None => continue,
        };

        if *begin == *end || dist_sq(*begin, *end) < min_path_len as isize {
            *goal = None;
            continue;
        }

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
        if let Some(path) = djikstra(cost, neighbors, *begin, *end, max_iters) {
            for (x, y) in path {
                if let Some(idx) = bounds(x, y, width, height) {
                    obstacles[idx] = true;
                    output_image[idx * 3..][..3].copy_from_slice(&path_color);
                }
            }
        } else {
            *goal = None;
        }
    }

    // For reading and opening files
    let file = File::create(output_path).unwrap();
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
    let diff: i32 = 30;
    let mut component = |idx: usize| (base[idx] + rng.gen_range(-diff..=diff)).clamp(0, 255) as u8;
    [component(0), component(1), component(2)]
}
