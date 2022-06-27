use std::collections::hash_map::Entry;
use std::f32::consts::PI;
use std::iter::once;
use std::ops::Sub;

use glam::Vec2;
use itertools::Itertools;
use petgraph::Direction::Outgoing;
use serde::{Deserialize, Serialize};
use spade::{DelaunayTriangulation, HasPosition, Triangulation};

// fx hash for stable hashing
type HashMap<K, V> = rustc_hash::FxHashMap<K, V>;
type HashSet<K> = rustc_hash::FxHashSet<K>;

pub type Graph = petgraph::stable_graph::StableDiGraph<Vec2, EdgeKind>;
pub type LocalDelaunayGraph = petgraph::graph::UnGraph<NodeIndex, (), u16>;
pub type NodeIndex = petgraph::graph::NodeIndex<u32>;
pub type LDNodeIndex = petgraph::graph::NodeIndex<u16>;

type EdgeSet = HashMap<(NodeIndex, NodeIndex), EdgeKind>;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum EdgeKind {
    Radial,
    Circular,
    Temp,
}
/*
First up, the problem
then show a quick sample run where the algo constructs a graph from a random one
then show a quick sample of a placement/topo change

then we should show the algo in proper

- whats a delaunay graph
        - a few properties of it maybe
how does it work in more detail


*/

/// Advance the graph by one delaunay update step.
pub fn delaunay_update(g: &mut Graph) {
    let mut edges = HashMap::<_, EdgeKind>::default();
    g.node_indices()
        .flat_map(|u| select_edges(g, u))
        // this is the union operation, but since we differentiate the different edge kinds
        // for visualization purposes we deduplicate with preference
        .for_each(|(edge, ty)| match edges.entry(edge) {
            Entry::Occupied(mut slot) => match (slot.get_mut(), ty) {
                // never overwrite radial edges, never overwrite with temporary edges
                (_, EdgeKind::Temp) | (EdgeKind::Radial, _) => {}
                (kind, new) => *kind = new,
            },
            Entry::Vacant(slot) => drop(slot.insert(ty)),
        });

    g.clear_edges();
    g.extend_with_edges(edges.into_iter().map(|((s, d), v)| (s, d, v)));
}

/// Select the update edges for given node in the graph.
fn select_edges(g: &Graph, active: NodeIndex) -> EdgeSet {
    // Generate the local delaunay graph of the node.
    let (ldg, active) = local_delaunay(g, active);

    // Generate the stable edges.
    let mut edges = stable_edges(g, &ldg, active);
    // Generate the temporal edges and merge the edge sets.
    for (edge, ty) in temp_edges(g, &ldg, active) {
        if !edges.contains_key(&edge) {
            edges.insert(edge, ty);
        }
    }
    edges
}

/// Select the stable edges.
fn stable_edges(g: &Graph, local_delaunay: &LocalDelaunayGraph, active: LDNodeIndex) -> EdgeSet {
    let make_undirected = |(v, w)| [(v, w), (w, v)];
    // Select edges from u to its neighbors in the local delaunay graph.
    // { {u, v} : v ∈ NGL(G,u)(u) }
    let u_to_neighbors = local_delaunay
        .neighbors_directed(active, Outgoing)
        .zip(once(active).cycle())
        .flat_map(make_undirected);

    let active_pos = g[local_delaunay[active]];
    // Select circular edges between u's neighbors
    // {{v, w} : v, w ∈ NGL(G,u)(u) ∧ ∄x ∈ NGL(G,u)(u) : x ∈ ∠vuw}
    // For x ∈ ∠vuw we make use of another trick in the geometric algebra toolbox, the sweepline
    // instead of checking the property as defined by the paper we instead radially sort all the
    // neighbor nodes around the active node and then do a sweep around connecting every adjacent pair
    // that span an angle less than 180 deg/pi radians
    // This gives us the same result with an easier implementation (for me at least, I couldn't
    // out the math) and in theory a better runtime complexity (log n) instead of n^2
    let node_to_polar_coords = |v: LDNodeIndex| {
        let v = local_delaunay[v];
        let v_p = g[v] - active_pos;
        // We add PI so that this spans from 0 to 2PI, removing the negative numbers
        // this makes it easier to check if the angle is less than PI radians in the filter below
        (v, v_p.y.atan2(v_p.x) + PI)
    };
    let circular_neighbors: Vec<_> = local_delaunay
        .neighbors_directed(active, Outgoing)
        .map(node_to_polar_coords)
        .sorted_by(|(_, a), (_, b)| a.total_cmp(b))
        .collect();
    let circular_neighbors = match circular_neighbors.as_slice() {
        [] | [_] => None,
        circular_neighbors => {
            Some(
                circular_neighbors
                    .iter()
                    .copied()
                    .circular_tuple_windows::<(_, _)>()
                    .filter(|((_, a), (_, b))| {
                        let d = b.sub(a);
                        // if d is negative, then there was a wraparound, so a was in the first quadrant while b was in the fourth
                        //  in that case, only if d is less than -PI radians we have an angle of PI, a greater angle would cause
                        //  the wraparound to land between 0 and -PI
                        // if d is between 0 and PI the distance is less than PI with the two angles being in neighboring quadrants
                        d < -PI || (0.0 < d && d < PI)
                    })
                    .flat_map(|((v, _), (w, _))| [(v, w), (w, v)]),
            )
        }
    };

    let mut edges = HashMap::default();

    // Map the local indices to the global graph and merges the edges

    let map_graph_index_pair = |(v, w)| (local_delaunay[v], local_delaunay[w]);

    edges.extend(
        u_to_neighbors
            .map(map_graph_index_pair)
            .map(|it| (it, EdgeKind::Radial)),
    );
    if let Some(circular_neighbors) = circular_neighbors {
        edges.extend(
            circular_neighbors
                // circular_between_neighbors
                //     .map(map_graph_index_pair)
                .map(|it| (it, EdgeKind::Circular)),
        );
    }

    edges
}

/// Select the temporal edges.
fn temp_edges(g: &Graph, local_delaunay: &LocalDelaunayGraph, active: LDNodeIndex) -> EdgeSet {
    // {(v, w) : v ∈ NGL (G,u)(u), w ∈ NG(u) \ NGL (G,u)(u) ∧ ∀x ∈ NGL(G,u)(u) : ‖x − w‖ ≥ ‖v − w‖}

    // v ∈ NGL (G,u)(u)
    let vs = local_delaunay
        .neighbors_directed(active, Outgoing)
        .map(|v| local_delaunay[v]);
    // w ∈ NG(u) \ NGL (G,u)(u)
    let ws = {
        let mut ws: HashSet<_> = g
            .neighbors_directed(local_delaunay[active], Outgoing)
            .collect();
        // the paper specifically removes the the local neighborhood with active, but active should never be in here
        assert!(!ws.remove(&local_delaunay[active]));
        vs.clone().for_each(|v| drop(ws.remove(&v)));
        ws
    };

    vs.clone()
        .flat_map(|v| ws.iter().map(move |&w| (v, w)))
        .filter(|&(v, w)| {
            let (v, w) = (g[v], g[w]);

            // ‖v − w‖
            let vw_length = (v - w).length();

            // we reuse vs here, as it seeds from the same set as x
            // ∀x ∈ NGL(G,u)(u) : ‖x − w‖ ≥ ‖v − w‖
            vs.clone().all(|x| {
                let x = g[x];
                // ‖x − w‖ ≥ ‖v − w‖
                (x - w).length() >= vw_length
            })
        })
        .map(|it| (it, EdgeKind::Temp))
        .collect()
}

/// Construct a local delaunay graph for the active node from its neighborhood
fn local_delaunay(g: &Graph, active: NodeIndex) -> (LocalDelaunayGraph, LDNodeIndex) {
    let mut graph = LocalDelaunayGraph::default();
    let ldg_active = graph.add_node(active);

    let nodes = g
        .neighbors_directed(active, Outgoing)
        .map(|id| (g[id], graph.add_node(id)))
        .chain(Some((g[active], ldg_active)));

    {
        struct DPoint(spade::Point2<f32>, LDNodeIndex);

        impl HasPosition for DPoint {
            type Scalar = f32;

            fn position(&self) -> spade::Point2<Self::Scalar> {
                self.0
            }
        }

        let mut triangulation = DelaunayTriangulation::<DPoint>::new();

        nodes.for_each(|(pos, idx)| {
            triangulation
                .insert(DPoint(spade::Point2::new(pos[0], pos[1]), idx))
                .unwrap();
        });

        graph.extend_with_edges(triangulation.undirected_edges().map(|it| {
            let directed = it.as_directed();
            (directed.from().data().1, directed.to().data().1)
        }));
    }

    (graph, ldg_active)
}
