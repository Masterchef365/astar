//! Implementation of Djikstra's algorithm
use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap};
use std::hash::Hash;
use std::ops::Add;

#[derive(Clone, Copy, PartialEq, Eq)]
struct DjikstraNode<P: Ord, V: Eq> {
    priority: P,
    vertex: V,
}

/// Djikstra's algorithm
/// cost: Callback to compute the cost (W) between two vertices (V)
/// adjacent: Callback to compute the adjacent nodes of a given vertex (V). Expects an iterable (I)
/// start: Initial vertex
/// end: Goal vertex
/// If there is a path, the vertices along it are returned
pub fn djikstra<V, W, I>(
    cost: impl Fn(V, V) -> W,
    adjacent: impl Fn(V) -> I,
    start: V,
    end: V,
) -> Option<Vec<V>>
where
    W: Copy + Ord + Add<Output = W> + Default,
    I: IntoIterator<Item = V>,
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

pub type AdjMap<V, W> = HashMap<V, HashMap<V, W>>;

/// Make an adjacency map out of a set of edges
pub fn make_adj_map<V: Copy + Hash + Eq, W: Copy>(graph: &[(V, V, W)]) -> AdjMap<V, W> {
    let mut adj: AdjMap<V, W> = HashMap::new();
    for &(a, b, weight) in graph {
        adj.entry(a).or_default().insert(b, weight);
        adj.entry(b).or_default().insert(a, weight);
    }
    adj
}

/// Run djikstra's algorithm on an adjacency map
pub fn djikstra_adj_map<V, W>(adj: &AdjMap<V, W>, start: V, end: V) -> Option<Vec<V>>
where
    W: Copy + Ord + Add<Output = W> + Default,
    V: Copy + Eq + Hash,
{
    let cost = |a, b| adj[&a][&b];
    let adjacent = |a| adj[&a].keys().copied();

    djikstra(cost, adjacent, start, end)
}

// TODO: More tests...
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_djikstra_example_1() {
        test_case(
            &[
                ('a', 'b', 7),
                ('a', 'c', 9),
                ('a', 'f', 14),
                ('b', 'c', 10),
                ('b', 'd', 15),
                ('c', 'd', 11),
                ('c', 'f', 2),
                ('d', 'e', 6),
                ('e', 'f', 9),
            ],
            'a',
            'e',
            Some(vec!['e', 'f', 'c', 'a']),
        );
    }

    #[track_caller]
    fn test_case(
        graph: &[(char, char, u32)],
        start: char,
        end: char,
        expect_path: Option<Vec<char>>,
    ) {
        let adj = make_adj_map(graph);
        let path = djikstra_adj_map(&adj, start, end);
        assert_eq!(path, expect_path);
    }
}

