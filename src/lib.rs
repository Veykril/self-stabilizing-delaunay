#![feature(total_cmp)]
use std::collections::hash_map::Entry;
use std::f32::consts::PI;
use std::iter::once;
use std::ops::Sub;

use glam::Vec2;
use itertools::Itertools;
use petgraph::Direction::Outgoing;
use spade::{DelaunayTriangulation, HasPosition, Triangulation};

// fx hash for stable hashing
type HashMap<K, V> = rustc_hash::FxHashMap<K, V>;
type HashSet<K> = rustc_hash::FxHashSet<K>;

pub type Graph = petgraph::stable_graph::StableDiGraph<Vec2, EdgeKind>;
pub type LocalDelaunayGraph = petgraph::graph::UnGraph<NodeIndex, (), u16>;
pub type NodeIndex = petgraph::graph::NodeIndex<u32>;
pub type LDNodeIndex = petgraph::graph::NodeIndex<u16>;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum EdgeKind {
    Stable1,
    Stable2,
    Temp,
}

/// Advance the graph by one delaunay update step.
pub fn delaunay_update(g: &mut Graph) {
    let mut edges = HashMap::<_, EdgeKind>::default();
    g.node_indices()
        .flat_map(|u| select_edges(g, u))
        .for_each(|(edge, ty)| match edges.entry(edge) {
            Entry::Occupied(mut slot) => match (slot.get_mut(), ty) {
                (_, EdgeKind::Temp) | (EdgeKind::Stable1, _) => {}
                (kind, new) => *kind = new,
            },
            Entry::Vacant(slot) => drop(slot.insert(ty)),
        });

    g.clear_edges();
    g.extend_with_edges(edges.into_iter().map(|((s, d), v)| (s, d, v)));
}

/// Select the update edges for given node in the graph.
fn select_edges(g: &Graph, u: NodeIndex) -> HashMap<(NodeIndex, NodeIndex), EdgeKind> {
    // Generate the local delaunay graph of the node.
    let (ldg, u_in_ldg) = local_delaunay(g, u);

    // Generate the stable edges.
    let mut edges = stable_edges(g, &ldg, u_in_ldg);
    // Generate the temporal edges and merge the edge sets.
    for (edge, ty) in temp_edges(g, &ldg, u_in_ldg) {
        if !edges.contains_key(&edge) {
            edges.insert(edge, ty);
        }
    }
    edges
}

/// Select the stable edges.
fn stable_edges(
    g: &Graph,
    local_delaunay: &LocalDelaunayGraph,
    u: LDNodeIndex,
) -> HashMap<(NodeIndex, NodeIndex), EdgeKind> {
    let make_undirected = |(v, w)| [(v, w), (w, v)];
    // Select edges from u to its neighbors in the local delaunay graph.
    // { {u, v} : v ∈ NGL(G,u)(u) }
    let u_to_neighbors = local_delaunay
        .neighbors_directed(u, Outgoing)
        .zip(once(u).cycle())
        .flat_map(make_undirected);

    let u_p = g[local_delaunay[u]];
    // Select circular edges between u's neighbors
    let circular_neighbors: Vec<_> = local_delaunay
        .neighbors_directed(u, Outgoing)
        .map(|v| {
            let v = local_delaunay[v];
            let v_p = g[v] - u_p;
            (v, v_p.y.atan2(v_p.x) + PI)
        })
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
                        // if d is between -2PI and PI the distance is less than PI but we had a wrap around
                        // (that is one angle is in the first quadrant while the second is in the fourth)
                        // if d is between 0 and PI the distance is less than PI with the two angles being in neighboring quadrants
                        d < -PI || (0.0 < d && d < PI)
                    })
                    .flat_map(|((v, _), (w, _))| [(v, w), (w, v)]),
            )
        }
    };

    let mut edges = HashMap::default();

    // Make the edges undirected and map the indices into g accordingly.

    let map_graph_index_pair = |(v, w)| (local_delaunay[v], local_delaunay[w]);

    edges.extend(
        u_to_neighbors
            .map(map_graph_index_pair)
            .map(|it| (it, EdgeKind::Stable1)),
    );
    if let Some(circular_neighbors) = circular_neighbors {
        edges.extend(
            circular_neighbors
                // circular_between_neighbors
                //     .map(map_graph_index_pair)
                .map(|it| (it, EdgeKind::Stable2)),
        );
    }

    edges
}

/// Select the temporal edges.
fn temp_edges(
    g: &Graph,
    local_delaunay: &LocalDelaunayGraph,
    u: LDNodeIndex,
) -> HashMap<(NodeIndex, NodeIndex), EdgeKind> {
    let vs = local_delaunay
        .neighbors_directed(u, Outgoing)
        .map(|v| local_delaunay[v]);
    let ws = {
        let mut ws: HashSet<_> = g.neighbors_directed(local_delaunay[u], Outgoing).collect();
        ws.remove(&local_delaunay[u]);
        vs.clone().for_each(|v| drop(ws.remove(&v)));
        ws
    };

    vs.clone()
        .flat_map(|v| ws.iter().map(move |&w| (v, w)))
        .filter(|&(v, w)| {
            let v = g[v];
            let w = g[w];

            let vw_length = (v - w).length();

            vs.clone().all(|x| {
                let x = g[x];
                (x - w).length() >= vw_length
            })
        })
        .map(|it| (it, EdgeKind::Temp))
        .collect()
}

fn local_delaunay(g: &Graph, u: NodeIndex) -> (LocalDelaunayGraph, LDNodeIndex) {
    let mut graph = LocalDelaunayGraph::default();
    let ldg_u = graph.add_node(u);

    let nodes: Vec<_> = g
        .neighbors_directed(u, Outgoing)
        .map(|id| (g[id], graph.add_node(id)))
        .chain(Some((g[u], ldg_u)))
        .collect();
    {
        struct DPoint(spade::Point2<f32>, LDNodeIndex);

        impl HasPosition for DPoint {
            type Scalar = f32;

            fn position(&self) -> spade::Point2<Self::Scalar> {
                self.0
            }
        }

        let mut triangulation = DelaunayTriangulation::<DPoint>::new();

        nodes.iter().for_each(|&(pos, idx)| {
            triangulation
                .insert(DPoint(spade::Point2::new(pos[0], pos[1]), idx))
                .unwrap();
        });

        graph.extend_with_edges(triangulation.undirected_edges().map(|it| {
            (
                it.as_directed().from().data().1,
                it.as_directed().to().data().1,
            )
        }));
    }

    (graph, ldg_u)
}
