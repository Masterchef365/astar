use astar::djikstra::djikstra;
use std::fs::File;
use std::io::BufWriter;

type Coord = (isize, isize);

fn main() {
    let width = 500;
    let height = 500;
    let mut obstacles = vec![false; width * height];
    let image_path = "out.png";

    for y in 10..=90 {
        obstacles[bounds(40, y, width, height).unwrap()] = true;
    }

    for y in 100..=300 {
        obstacles[bounds(100, y, width, height).unwrap()] = true;
    }

    //let begin = (width as isize / 3, height as isize / 3);
    let begin = (10, 10);
    let end = (2 * width as isize / 3, 2 * height as isize / 3);

    /*
    let eight_directions = |(x, y): Coord| {
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
    };
    */

    let four_directions = |(x, y): Coord| [(x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1)];

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

    let cost = |a, b| 1 + heuristic(b, a, end);

    // Calculate path
    let path = djikstra(cost, neighbors, begin, end);
    let path = path.expect("No path!");

    // Write image
    let mut output_image = vec![0u8; width * height * 3];

    output_image
        .chunks_exact_mut(3)
        .zip(obstacles)
        .filter(|(_, b)| *b)
        .for_each(|(u, _)| u[1] = 0xff);

    for (x, y) in path {
        if let Some(idx) = bounds(x, y, width, height) {
            output_image[idx * 3 + 0] = 0xff;
            //output_image[idx * 3 + 1] = 0xff;
            //output_image[idx * 3 + 2] = 0xff;
        }
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
