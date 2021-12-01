use std::ops::Add;
use std::collections::{HashMap, BinaryHeap, HashSet};
use std::cmp::Ordering;
use std::hash::Hash;

#[derive(Clone, Copy, PartialEq, Eq)]
struct DjikstraNode<P: Ord, V: Eq> {
    priority: P,
    vertex: V,
}

pub fn djikstra<V, W, I>(
    cost: impl Fn(V, V) -> W,
    adjacent: impl Fn(V) -> I,
    start: V,
    end: V,
) -> Option<Vec<V>>
where
    W: Copy + Ord + Add<Output = W> + Default,
    I: Iterator<Item = V>,
    V: Copy + Eq + Hash,
{
    let mut prev: HashMap<V, V> = HashMap::new();
    let mut queue: BinaryHeap<DjikstraNode<W, V>> = BinaryHeap::new();
    let mut dist: HashMap<V, W> = HashMap::new();

    queue.push(DjikstraNode::new(W::default(), start));

    while let Some(node) = queue.pop() {
        // Skip invalidated nodes
        if let Some(&min) = dist.get(&node.vertex) {
            if node.priority > min {
                continue;
            }
        }

        // Return the path if we've finished
        if node.vertex == end {
            let mut current = end;
            let mut path = vec![current];
            while let Some(&next) = prev.get(&current) {
                path.push(next);
                current = next;
                if current == start {
                    break;
                }
            }
            return Some(path);
        }

        // Explore neighbors
        for neighbor in adjacent(node.vertex) {
            let alt = node.priority + cost(node.vertex, neighbor);
            if !dist.contains_key(&neighbor) || alt < dist[&neighbor] {
                prev.insert(neighbor, node.vertex);
                dist.insert(neighbor, alt);
                queue.push(DjikstraNode::new(alt, neighbor));
            }
        }
    }

    // No path
    None
}

impl<P: Ord, V: Eq> DjikstraNode<P, V> {
    pub fn new(priority: P, vertex: V) -> Self {
        Self { priority, vertex }
    }
}

impl<P: Ord, V: Eq> Ord for DjikstraNode<P, V> {
    fn cmp(&self, other: &Self) -> Ordering {
        self.priority.cmp(&other.priority).reverse()
    }
}

impl<P: Ord, V: Eq> PartialOrd for DjikstraNode<P, V> {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(&other))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_astar_example_1() {
    }


}
