#![feature(total_cmp)]
use std::collections::hash_map::Entry;
use std::iter::once;
use std::ops::Sub;

use itertools::Itertools;
use nannou::color::Alpha;
use nannou::prelude::*;
use nannou::rand::prelude::{IteratorRandom, SliceRandom};
use nannou::rand::thread_rng;
use petgraph::data::Build;
use petgraph::graph::Node;
use petgraph::visit::{EdgeRef, IntoEdgeReferences, IntoNodeReferences};
use petgraph::{Directed, Direction, Undirected};
use spade::{DelaunayTriangulation, HasPosition, Triangulation};
use Direction::Outgoing;

fn main() {
    nannou::app(model)
        .update(update)
        .simple_window(view)
        .event(event)
        .run();
}

type HashMap<K, V> = rustc_hash::FxHashMap<K, V>;
type HashSet<K> = rustc_hash::FxHashSet<K>;

type Graph = petgraph::stable_graph::StableGraph<Point2, EdgeKind, Directed>;
type LocalDelaunayGraph = petgraph::stable_graph::StableGraph<NodeIndex, (), Undirected, u16>;
type NodeIndex = petgraph::graph::NodeIndex<u32>;
type LDNodeIndex = petgraph::graph::NodeIndex<u16>;

struct Model {
    graph: Graph,
}

fn model(app: &App) -> Model {
    let mut graph = Graph::new();
    // TODO: doesn't check for degeneracy
    let nodes: Vec<_> = (0..50)
        .map(|_| graph.add_node(Point2::new(random::<f32>() - 0.5, random::<f32>() - 0.5)))
        .collect();

    {
        assert!(nodes.iter().duplicates().next().is_none());
    }

    let edges: Vec<_> = nodes
        .into_iter()
        .permutations(2)
        .flat_map(|it| {
            [
                (it[0], it[1], EdgeKind::Temp),
                (it[1], it[0], EdgeKind::Temp),
            ]
        })
        .collect();

    let edges_to_retain = edges.len() / (10 / 2);
    let edges = edges.choose_multiple(&mut thread_rng(), edges_to_retain);
    graph.extend_with_edges(edges);

    if let components @ [_, _, ..] = petgraph::algo::tarjan_scc(&graph).as_slice() {
        println!("Unconnected components! Fixing up...");
        components.windows(2).for_each(|window| {
            let a = &window[0];
            let b = &window[1];
            let &a = a.choose(&mut thread_rng()).unwrap();
            let &b = b.choose(&mut thread_rng()).unwrap();
            graph.add_edge(a, b, EdgeKind::Temp);
            graph.add_edge(b, a, EdgeKind::Temp);
        });
    }

    // validate

    dbg!(graph.node_count());
    dbg!(graph.edge_count());
    Model { graph }
}

fn update(_app: &App, _model: &mut Model, _update: Update) {}

fn event(app: &App, _model: &mut Model, event: Event) {
    match event {
        Event::WindowEvent {
            simple: Some(WindowEvent::KeyReleased(Key::R)),
            ..
        } => *_model = model(app),
        Event::WindowEvent {
            simple: Some(WindowEvent::KeyPressed(Key::U)),
            ..
        } => delaunay_update(&mut _model.graph),
        Event::WindowEvent {
            simple: Some(WindowEvent::MousePressed(MouseButton::Left)),
            ..
        } => {
            let rect = app.main_window().rect();
            let connection = _model.graph.node_indices().choose(&mut thread_rng());
            let idx = _model
                .graph
                .add_node(Vec2::new(app.mouse.x, app.mouse.y) / Vec2::new(rect.w(), rect.h()));
            let connection = match connection {
                Some(it) => it,
                None => return,
            };
            _model.graph.add_edge(idx, connection, EdgeKind::Temp);
            _model.graph.add_edge(connection, idx, EdgeKind::Temp);
        }
        Event::WindowEvent {
            simple: Some(WindowEvent::MousePressed(MouseButton::Right)),
            ..
        } => {
            let rect = app.main_window().rect();
            let cursor = Vec2::new(app.mouse.x, app.mouse.y) / Vec2::new(rect.w(), rect.h());
            if let Some((node, _)) =
                _model
                    .graph
                    .node_references()
                    .min_by(|&(_, &pos), &(_, &pos2)| {
                        (cursor - pos)
                            .length_squared()
                            .total_cmp(&(cursor - pos2).length_squared())
                    })
            {
                _model.graph.remove_node(node);
            }
        }
        _ => (),
    }
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

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
enum EdgeKind {
    Stable1,
    Stable2,
    Temp,
}

/// Advance the graph by one delaunay update step.
fn delaunay_update(g: &mut Graph) {
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
    let circular_neighbors = local_delaunay
        .neighbors_directed(u, Outgoing)
        .map(|v| {
            let v = local_delaunay[v];
            let v_p = g[v] - u_p;
            (v, v_p.y.atan2(v_p.x) + PI)
        })
        .sorted_by(|(_, a), (_, b)| a.total_cmp(b))
        .circular_tuple_windows::<(_, _)>()
        .filter(|((a, _), (b, _))| a != b) // circular of 1 will add an (a, a) pair which creates a circular edge!
        // (a < b)
        .filter(|((av, a), (bv, b))| {
            let d = b.sub(a);
            d < -PI || (0.0 < d && d < PI)
        })
        .flat_map(|((v, _), (w, _))| [(v, w), (w, v)]);
    // let circular_between_neighbors = neighbors_directed
    //     .tuple_combinations::<(_, _)>()
    //     .filter(|&(v, w)| {
    //         // ∠vuw
    //         let v = g[local_delaunay[v]];
    //         let w = g[local_delaunay[w]];
    //         let vu = v - g[local_delaunay[u]];
    //         let wu = w - g[local_delaunay[u]];
    //         let cross2d = |a: Vec2, b: Vec2| a[0] * b[1] - a[1] * b[0];

    //         !local_delaunay.neighbors_directed(u, Outgoing).any(|x| {
    //             let x = g[local_delaunay[x]] - g[local_delaunay[u]];
    //             cross2d(vu, x) > 0.0 && cross2d(x, wu) > 0.0
    //                 || cross2d(vu, x) < 0.0 && cross2d(x, wu) < 0.0
    //         })
    //     })
    //     .flat_map(make_undirected);

    let mut edges = HashMap::default();

    // Make the edges undirected and map the indices into g accordingly.

    let map_graph_index_pair = |(v, w)| (local_delaunay[v], local_delaunay[w]);

    edges.extend(
        u_to_neighbors
            .map(map_graph_index_pair)
            .map(|it| (it, EdgeKind::Stable1)),
    );
    edges.extend(
        circular_neighbors
            // circular_between_neighbors
            //     .map(map_graph_index_pair)
            .map(|it| (it, EdgeKind::Stable2)),
    );

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

fn view(app: &App, model: &Model, frame: Frame) {
    let temp_color: Srgb = Srgb::new(1.0, 0.0, 0.0);
    let stable1_color: Srgb = Srgb::new(1.0, 1.0, 0.0);
    let stable2_color: Srgb = Srgb::new(0.0, 0.0, 1.0);

    let draw = app.draw();
    let window = app.main_window();
    let win = window.rect();
    draw.background().rgb(0.11, 0.12, 0.13);

    // 100-step and 10-step grids.
    draw_grid(&draw, &win, 100.0, 1.0);
    draw_grid(&draw, &win, 25.0, 0.5);

    render_graph(
        &draw,
        &win,
        &model.graph,
        temp_color,
        stable1_color,
        stable2_color,
    );

    // let mut x = Graph::new();
    // let delaun = local_delaunay(&model.graph, NodeIndex::new(0));
    // let xa: Vec<_> = delaun
    //     .node_references()
    //     .map(|n| (n.0, x.add_node(model.graph[*n.1])))
    //     .collect();
    // delaun.edge_references().for_each(|e| {
    //     x.add_edge(
    //         xa.iter().find(|&&(it, _)| it == e.source()).unwrap().1,
    //         xa.iter().find(|&&(it, _)| it == e.target()).unwrap().1,
    //         EdgeKind::Temp,
    //     );
    // });
    // render_graph(
    //     &draw,
    //     &(win.pad_top(-100.0)),
    //     &x,
    //     Srgb::new(1.0, 0.0, 1.0),
    //     Srgb::new(1.0, 0.0, 1.0),
    //     Srgb::new(1.0, 0.0, 1.0),
    // );

    // let first = model.graph[NodeIndex::new(0)];
    // draw.ellipse()
    //     .x_y((first.x - 0.5) * win.w(), (first.y - 0.5) * win.h())
    //     .w_h(10.0, 10.0)
    //     .rgba(0.0, 0.0, 0.0, 1.0);
    // model
    //     .graph
    //     .neighbors_directed(NodeIndex::new(0), Outgoing)
    //     .for_each(|id| {
    //         let node = model.graph[id];
    //         draw.ellipse()
    //             .x_y((node.x - 0.5) * win.w(), (node.y - 0.5) * win.h())
    //             .w_h(10.0, 10.0)
    //             .rgba(0.0, 0.0, 1.0, 1.0);
    //     });

    // Crosshair.
    let crosshair_color = gray(0.5);

    // Crosshair text.
    draw_crosshair(win, &draw, crosshair_color);

    // Ellipse at mouse.
    draw.ellipse().wh([5.0; 2].into()).xy(app.mouse.position());

    // Mouse position text.
    let mouse = app.mouse.position();
    let pos = format!("[{:.1}, {:.1}]", mouse.x, mouse.y);
    draw.text(&pos)
        .xy(mouse + vec2(0.0, 20.0))
        .font_size(14)
        .color(WHITE);

    draw.to_frame(app, &frame).unwrap();
}

fn render_graph(draw: &Draw, rect: &Rect, graph: &Graph, temp: Srgb, stable: Srgb, stable2: Srgb) {
    graph.edge_references().for_each(|edge| {
        let source = edge.source();
        let target = edge.target();
        let source = graph[source] * Vec2::new(rect.w(), rect.h());
        let target = graph[target] * Vec2::new(rect.w(), rect.h());
        draw.arrow()
            .stroke_weight(2.0)
            .start_cap_butt()
            .start(source)
            .end(target)
            .head_length(20.0)
            .head_width(4.0)
            .color(Alpha {
                alpha: 0.5f32,
                color: match edge.weight() {
                    EdgeKind::Stable1 => stable,
                    EdgeKind::Stable2 => stable2,
                    EdgeKind::Temp => temp,
                },
            });
    });
    graph.node_indices().for_each(|node| {
        let pos = graph[node];
        let pos = Vec2::new(pos.x * rect.w(), pos.y * rect.h());
        draw.ellipse().xy(pos).w_h(10.0, 10.0).color(stable);
        draw.text(&format!("{}", node.index()))
            .xy(pos + Vec2::new(0.0, 20.0))
            .w_h(10.0, 10.0);
    });
}

fn draw_crosshair(win: Rect, draw: &Draw, crosshair_color: Rgb) {
    let ends = [
        win.mid_top(),
        win.mid_right(),
        win.mid_bottom(),
        win.mid_left(),
    ];
    for &end in &ends {
        draw.arrow()
            .start_cap_round()
            .head_length(16.0)
            .head_width(8.0)
            .color(crosshair_color)
            .end(end);
    }

    let top = format!("{:.1}", win.top());
    let bottom = format!("{:.1}", win.bottom());
    let left = format!("{:.1}", win.left());
    let right = format!("{:.1}", win.right());
    let x_off = 30.0;
    let y_off = 20.0;
    draw.text("0.0")
        .x_y(15.0, 15.0)
        .color(crosshair_color)
        .font_size(14);
    draw.text(&top)
        .h(win.h())
        .font_size(14)
        .align_text_top()
        .color(crosshair_color)
        .x(x_off);
    draw.text(&bottom)
        .h(win.h())
        .font_size(14)
        .align_text_bottom()
        .color(crosshair_color)
        .x(x_off);
    draw.text(&left)
        .w(win.w())
        .font_size(14)
        .left_justify()
        .color(crosshair_color)
        .y(y_off);
    draw.text(&right)
        .w(win.w())
        .font_size(14)
        .right_justify()
        .color(crosshair_color)
        .y(y_off);
}

fn draw_grid(draw: &Draw, win: &Rect, step: f32, weight: f32) {
    let step_by = || (0..).map(|i| i as f32 * step);
    let r_iter = step_by().take_while(|&f| f < win.right());
    let l_iter = step_by().map(|f| -f).take_while(|&f| f > win.left());
    let x_iter = r_iter.chain(l_iter);
    for x in x_iter {
        draw.line()
            .weight(weight)
            .points(pt2(x, win.bottom()), pt2(x, win.top()));
    }
    let t_iter = step_by().take_while(|&f| f < win.top());
    let b_iter = step_by().map(|f| -f).take_while(|&f| f > win.bottom());
    let y_iter = t_iter.chain(b_iter);
    for y in y_iter {
        draw.line()
            .weight(weight)
            .points(pt2(win.left(), y), pt2(win.right(), y));
    }
}
