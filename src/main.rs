use std::path::Path;
use std::time::Instant;

use itertools::Itertools;
use nannou::color::Alpha;
use nannou::event::Key;
use nannou::prelude::{
    gray, pt2, random, vec2, App, Draw, Frame, MouseButton, Point2, Rect, Rgb, Srgb, Update, Vec2,
    WindowEvent,
};
use nannou::rand::prelude::{IteratorRandom, SliceRandom};
use nannou::rand::thread_rng;
use nannou_egui::egui::{DragValue, Slider};
use nannou_egui::{egui, Egui};
use petgraph::visit::{EdgeRef, IntoEdgeReferences, IntoNodeReferences};

use delaunay_stabilization::*;

fn main() {
    nannou::app(|app| {
        let window_id = app
            .new_window()
            .title("Nannou + Egui")
            .raw_event(|_, model: &mut Model, event| model.egui.handle_raw_event(event))
            .event(event)
            .view(view)
            .power_preference(nannou::wgpu::PowerPreference::LowPower)
            .build()
            .unwrap();

        let window = app.window(window_id).unwrap();

        let path = "./graphs/default.json".into();
        let mut graph = Default::default();

        deserialize_graph(&mut graph, &path);

        Model {
            egui: Egui::from_window(&window),
            graph,
            steps: 0,
            last_check: Some(Instant::now()),
            is_connected: true,
            update_speed: 500,
            count_nodes: 50,
            is_stable: false,
            path,
            preferred_edge: None,
            temp_color: [255, 0, 0],
            radial_color: [255, 255, 0],
            circular_color: [0, 0, 255],
        }
    })
    .update(update)
    .run();
}

struct Model {
    egui: Egui,
    graph: Graph,
    steps: usize,
    last_check: Option<Instant>,
    update_speed: u32,
    is_connected: bool,
    count_nodes: usize,
    is_stable: bool,
    path: String,
    preferred_edge: Option<u32>,
    temp_color: [u8; 3],
    radial_color: [u8; 3],
    circular_color: [u8; 3],
}

fn deserialize_graph(graph: &mut Graph, path: impl AsRef<Path>) {
    let path = path.as_ref();
    match std::fs::OpenOptions::new().read(true).open(path) {
        Ok(r) => match serde_json::from_reader(r) {
            Err(e) => {
                eprintln!("Failed to deserialize graph {e}");
            }
            Ok(g) => *graph = g,
        },
        Err(e) => {
            eprintln!("Failed to open file {}: {e}", path.display());
        }
    }
}
fn serialize_graph(graph: &Graph, path: impl AsRef<Path>) {
    let path = path.as_ref();
    match std::fs::OpenOptions::new()
        .write(true)
        .create(true)
        .truncate(true)
        .open(&path)
    {
        Ok(w) => {
            if let Err(e) = serde_json::to_writer(w, graph) {
                eprintln!("Failed to serialize graph {e}");
            }
        }
        Err(e) => {
            eprintln!("Failed to open file {}: {e}", path.display());
        }
    }
}

fn update(
    _app: &App,
    Model {
        egui,
        graph,
        steps,
        last_check,
        is_connected,
        update_speed,
        count_nodes,
        is_stable,
        path,
        preferred_edge,
        temp_color,
        radial_color,
        circular_color,
    }: &mut Model,
    update: Update,
) {
    egui.set_elapsed_time(update.since_start);
    let ctx = egui.begin_frame();
    egui::Window::new("tools")
        .default_size(egui::vec2(0.0, 200.0))
        .show(&ctx, |ui| {
            ui.add(Slider::new(count_nodes, 1..=1000).prefix("Random graph node count: "));
            ui.add_space(10.0);
            ui.add(
                Slider::new(update_speed, 1..=1000)
                    .prefix("Update delay: ")
                    .suffix(" ms"),
            );
            let mut checked = last_check.is_some();
            checked ^= ui.selectable_label(checked, "auto-update").clicked();
            match last_check {
                Some(_) if !checked => *last_check = None,
                None if checked => *last_check = Some(Instant::now()),
                _ => (),
            }

            let mut value = preferred_edge.map_or(-1, |it| it as i32);
            ui.add(
                DragValue::new(&mut value)
                    .prefix("Preferred edge partner: ")
                    .clamp_range(-1i32..=i32::MAX),
            );
            match value {
                i32::MIN..=-1 => *preferred_edge = None,
                val => *preferred_edge = Some(val as u32),
            }
            ui.text_edit_singleline(path);
            ui.vertical_centered_justified(|ui| {
                if ui.button("Load").clicked() {
                    deserialize_graph(graph, &path);
                }
                if ui.button("Serialize").clicked() {
                    serialize_graph(graph, &path);
                }
            });
        });
    egui::Window::new("info-panel")
        .default_size(egui::vec2(0.0, 200.0))
        .show(&ctx, |ui| {
            // ui.label("Left click to add node");
            // ui.label("Right click to remove node");
            // ui.label("Press R to regenerate a random graph");
            // ui.label("Press U to trigger a step manually");
            // ui.add_space(10.0);
            ui.label(format!(
                "Current step: {}{}",
                steps,
                if *is_stable { " (stable)" } else { "" }
            ));
            ui.label(format!("Num nodes: {}", graph.node_count()));
            ui.label(format!("Num edges: {}", graph.edge_count()));
            ui.horizontal(|ui| {
                ui.label("Temp Edges: ");
                ui.color_edit_button_srgb(temp_color);
            });
            ui.horizontal(|ui| {
                ui.label("Radial Edges: ");
                ui.color_edit_button_srgb(radial_color);
            });
            ui.horizontal(|ui| {
                ui.label("Circular Edges: ");
                ui.color_edit_button_srgb(circular_color);
            });
            if !*is_connected {
                ui.label("UNCONNECTED TOPOLOGY!");
                ui.label("Press C to connect via random edge");
            }
        });
    match last_check {
        Some(last_check) if last_check.elapsed().as_millis() >= *update_speed as _ => {
            *last_check = Instant::now();
        }
        _ => return,
    }

    let old = graph.clone();
    let old = old
        .edge_references()
        .sorted_by_key(|a| (a.source(), a.target()));
    delaunay_update(graph);
    let new = graph
        .edge_references()
        .sorted_by_key(|a| (a.source(), a.target()));
    if !new.eq(old) {
        *steps += 1;
    }
}

fn event(app: &App, model: &mut Model, event: WindowEvent) {
    if model.egui.ctx().wants_keyboard_input() {
        return;
    }
    match event {
        WindowEvent::KeyReleased(Key::R) => {
            (model.graph, model.is_connected) = create_graph(model.count_nodes);
            model.steps = 0
        }
        WindowEvent::KeyPressed(Key::U) => {
            delaunay_update(&mut model.graph);
            model.steps += 1
        }
        WindowEvent::KeyPressed(Key::C) => {
            for window in petgraph::algo::tarjan_scc(&model.graph).windows(2) {
                let a = &window[0];
                let b = &window[1];
                let &a = a.choose(&mut thread_rng()).unwrap();
                let &b = b.choose(&mut thread_rng()).unwrap();
                model.graph.add_edge(a, b, EdgeKind::Temp);
            }
        }
        WindowEvent::MousePressed(MouseButton::Left) if !model.egui.ctx().wants_pointer_input() => {
            let rect = app.main_window().rect();
            let connection = match model.preferred_edge {
                Some(it) => {
                    Some(NodeIndex::new(it as usize)).filter(|&it| model.graph.contains_node(it))
                }
                None => model.graph.node_indices().choose(&mut thread_rng()),
            };
            let idx = model
                .graph
                .add_node(Vec2::new(app.mouse.x, app.mouse.y) / Vec2::new(rect.w(), rect.h()));
            let connection = match connection {
                Some(it) => it,
                None => return,
            };
            model.graph.add_edge(idx, connection, EdgeKind::Temp);
            model.graph.add_edge(connection, idx, EdgeKind::Temp);
            model.steps = 0;
            if let Some(last_check) = &mut model.last_check {
                *last_check = Instant::now();
            }
        }
        WindowEvent::MousePressed(MouseButton::Right)
            if !model.egui.ctx().wants_pointer_input() =>
        {
            let rect = app.main_window().rect();
            let cursor = Vec2::new(app.mouse.x, app.mouse.y) / Vec2::new(rect.w(), rect.h());
            if let Some((node, _)) =
                model
                    .graph
                    .node_references()
                    .min_by(|&(_, &pos), &(_, &pos2)| {
                        (cursor - pos)
                            .length_squared()
                            .total_cmp(&(cursor - pos2).length_squared())
                    })
            {
                model.graph.remove_node(node);
                model.steps = 0;
                if let Some(last_check) = &mut model.last_check {
                    *last_check = Instant::now();
                }
                model.is_connected = !matches!(
                    petgraph::algo::tarjan_scc(&model.graph).as_slice(),
                    [_, _, ..]
                );
            }
        }
        _ => (),
    }
}

fn create_graph(num: usize) -> (Graph, bool) {
    let mut graph = Graph::new();
    // TODO: doesn't check for degeneracy!
    let nodes: Vec<_> = (0..num)
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

    let is_connected = !matches!(petgraph::algo::tarjan_scc(&graph).as_slice(), [_, _, ..]);
    (graph, is_connected)
}

fn view(
    app: &App,
    model @ Model {
        temp_color,
        radial_color,
        circular_color,
        ..
    }: &Model,
    frame: Frame,
) {
    let temp_color = Srgb::new(temp_color[0], temp_color[1], temp_color[2]);
    let radial_color = Srgb::new(radial_color[0], radial_color[1], radial_color[2]);
    let circular_color = Srgb::new(circular_color[0], circular_color[1], circular_color[2]);

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
        radial_color,
        circular_color,
    );

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
        .color(Rgb::new(0.0, 0.0, 0.0));

    draw.to_frame(app, &frame).unwrap();

    let _ = model.egui.draw_to_frame(&frame);
}

fn render_graph(
    draw: &Draw,
    rect: &Rect,
    graph: &Graph,
    temp: Srgb<u8>,
    radial: Srgb<u8>,
    circular: Srgb<u8>,
) {
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
                    EdgeKind::Radial => radial,
                    EdgeKind::Circular => circular,
                    EdgeKind::Temp => temp,
                },
            });
    });
    graph.node_indices().for_each(|node| {
        let pos = graph[node];
        let pos = Vec2::new(pos.x * rect.w(), pos.y * rect.h());
        draw.ellipse().xy(pos).w_h(10.0, 10.0).color(radial);
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
