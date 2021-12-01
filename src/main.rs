use std::collections::HashMap;
use std::hash::Hash;
use astar::djikstra;
use std::ops::Add;

type AdjMap<V, W> = HashMap<V, HashMap<V, W>>;

fn make_adj_map<V: Copy + Hash + Eq, W: Copy>(graph: &[(V, V, W)]) -> AdjMap<V, W> {
    let mut adj: AdjMap<V, W> = HashMap::new();
    for &(a, b, weight) in graph {
        adj.entry(a).or_default().insert(b, weight);
        adj.entry(b).or_default().insert(a, weight);
    }
    adj
}

fn djikstra_adj_map<V, W>(adj: &AdjMap<V, W>, start: V, end: V) -> Option<Vec<V>> where 
    W: Copy + Ord + Add<Output = W> + Default,
    V: Copy + Eq + Hash,
{
    let cost = |a, b| adj[&a][&b];
    let adjacent = |a| adj[&a].keys().copied();

    djikstra(
        cost,
        adjacent,
        start,
        end,
    )
    
}

fn main() {
    let graph = [
        ('a','b',7) ,('a','c',9) ,('a','f',14),
        ('b','c',10),('b','d',15),('c','d',11),
        ('c','f',2) ,('d','e',6) ,('e','f',9) ,
    ];
    let adj = make_adj_map(&graph);
    dbg!(djikstra_adj_map(&adj, 'a', 'e'));
}
