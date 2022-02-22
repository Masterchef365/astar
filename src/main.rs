use astar::djikstra::djikstra;
use std::fs::File;
use std::io::BufWriter;

fn main() {
    let width = 500;
    let height = 500;
    let obstacles = vec![false; width * height];
    let image_path = "out.png";

    //let begin = (width as isize / 3, height as isize / 3);
    let begin = (10, 10);
    let end = (2 * width as isize / 3, 2 * height as isize / 3);

    // Neighbors
    let neighbors = |(x, y)| {
        //[(x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1)]
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
    for (x, y) in path {
        if let Some(idx) = bounds(x, y, width, height) {
            output_image[idx * 3 + 0] = 0xff;
            output_image[idx * 3 + 1] = 0xff;
            output_image[idx * 3 + 2] = 0xff;
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

fn heuristic(a: (isize, isize), b: (isize, isize), goal: (isize, isize)) -> isize {
    dist_sq(a, goal) - dist_sq(b, goal)
}

/// Squared distance
fn dist_sq((x, y): (isize, isize), (goal_x, goal_y): (isize, isize)) -> isize {
    let dx = x - goal_x;
    let dy = y - goal_y;

    dx * dx + dy * dy
}
