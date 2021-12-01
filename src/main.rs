use std::collections::HashMap;
use astar::djikstra;

fn main() {
    let graph = [
        ('a','b',7) ,('a','c',9) ,('a','f',14),
        ('b','c',10),('b','d',15),('c','d',11),
        ('c','f',2) ,('d','e',6) ,('e','f',9) ,
    ];

    let mut adj: HashMap<char, HashMap<char, u32>> = HashMap::new();
    for (a, b, weight) in graph {
        adj.entry(a).or_default().insert(b, weight);
        adj.entry(b).or_default().insert(a, weight);
    }

    let cost = |a, b| adj[&a][&b];
    let adjacent = |a| adj[&a].keys().copied();

    let path = djikstra(
        cost,
        adjacent,
        'a',
        'e',
    );

    dbg!(path);
}
