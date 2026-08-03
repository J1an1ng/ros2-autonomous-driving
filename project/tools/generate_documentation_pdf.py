#!/usr/bin/env python3
"""Generate Team 20's standalone assignment documentation PDF."""

from __future__ import annotations

import json
from pathlib import Path
import re
import textwrap
import xml.etree.ElementTree as ET

import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.patches import FancyBboxPatch


PROJECT_DIR = Path(__file__).resolve().parents[1]
REPOSITORY_DIR = PROJECT_DIR.parent
OUTPUT_PATH = REPOSITORY_DIR / 'DocumentationTeam20.pdf'
METRICS_PATH = PROJECT_DIR / 'mission_results' / 'metrics.json'
MISSION_RESULTS_DIR = PROJECT_DIR / 'mission_results'
GRAPH_SVG_URL = (
    'https://gitlab.lrz.de/00000000014c3605/'
    'i2ros-project-ss26-team20/-/blob/main/project/docs/ros_graph.svg'
)

PORTRAIT = (8.27, 11.69)
LANDSCAPE = (11.69, 8.27)
NAVY = '#17365d'
BLUE = '#2f75b5'
LIGHT_BLUE = '#d9eaf7'
GREY = '#5b6573'
LIGHT_GREY = '#f1f4f7'
GREEN = '#2e7d32'
ORANGE = '#d97706'


def new_page(size=PORTRAIT):
    """Create a clean page canvas."""
    figure = plt.figure(figsize=size, facecolor='white')
    axis = figure.add_axes([0, 0, 1, 1])
    axis.set_xlim(0, 1)
    axis.set_ylim(0, 1)
    axis.axis('off')
    return figure, axis


def page_chrome(axis, page_number, landscape=False):
    """Add the common header rule and footer."""
    axis.plot([0.055, 0.945], [0.955, 0.955], color=BLUE, linewidth=1.2)
    axis.text(
        0.055,
        0.968,
        'INTRODUCTION TO ROS  |  AUTONOMOUS DRIVING',
        color=GREY,
        fontsize=7.2,
        fontweight='bold',
        va='bottom',
    )
    axis.plot([0.055, 0.945], [0.045, 0.045], color='#b8c2cc', linewidth=0.6)
    axis.text(0.055, 0.025, 'Team 20  |  DocumentationTeam20', color=GREY, fontsize=7)
    axis.text(0.945, 0.025, str(page_number), color=GREY, fontsize=7, ha='right')


def title(axis, text, subtitle=None, y=0.92):
    """Draw a page title and optional subtitle."""
    axis.text(0.06, y, text, color=NAVY, fontsize=18, fontweight='bold', va='top')
    if subtitle:
        axis.text(0.06, y - 0.04, subtitle, color=GREY, fontsize=8.5, va='top')


def wrapped_text(axis, text, x, y, width=95, fontsize=8.4, line_height=0.018,
                 color='#20262e', weight='normal'):
    """Draw wrapped text and return the next vertical position."""
    lines = textwrap.wrap(text, width=width, break_long_words=False)
    axis.text(
        x,
        y,
        '\n'.join(lines),
        fontsize=fontsize,
        color=color,
        va='top',
        fontweight=weight,
        linespacing=1.25,
    )
    return y - line_height * max(1, len(lines))


def bullet_list(axis, items, x, y, width=95, fontsize=8.1, line_height=0.0175,
                gap=0.009):
    """Draw a compact bulleted list and return the next vertical position."""
    for item in items:
        lines = textwrap.wrap(item, width=width, break_long_words=False)
        axis.text(x, y, '\u2022', color=BLUE, fontsize=fontsize + 1, va='top')
        axis.text(
            x + 0.018,
            y,
            '\n'.join(lines),
            fontsize=fontsize,
            color='#20262e',
            va='top',
            linespacing=1.22,
        )
        y -= line_height * max(1, len(lines)) + gap
    return y


def add_table(axis, rows, columns, bbox, widths, fontsize=7.0,
              header_color=NAVY):
    """Draw a table with measured row heights so text cannot overlap rows."""
    x, y, width, height = bbox
    line_step = 0.0102 * fontsize / 7.0
    padding = 0.0055

    def wrap_cell(value, column_width):
        max_chars = max(10, int(118 * column_width * 7.0 / fontsize))
        wrapped = []
        for existing_line in str(value).splitlines() or ['']:
            wrapped.extend(
                textwrap.wrap(
                    existing_line,
                    width=max_chars,
                    break_long_words=False,
                    break_on_hyphens=False,
                ) or ['']
            )
        return '\n'.join(wrapped)

    wrapped_columns = [
        wrap_cell(value, widths[index]) for index, value in enumerate(columns)
    ]
    wrapped_rows = [
        [wrap_cell(value, widths[index]) for index, value in enumerate(row)]
        for row in rows
    ]
    header_lines = max(value.count('\n') + 1 for value in wrapped_columns)
    header_height = max(0.027, header_lines * line_step + 2 * padding)
    row_heights = []
    for row in wrapped_rows:
        line_count = max(value.count('\n') + 1 for value in row)
        row_heights.append(max(0.028, line_count * line_step + 2 * padding))

    natural_height = header_height + sum(row_heights)
    if natural_height > height:
        raise ValueError(
            f'Table requires {natural_height:.3f} page units, only {height:.3f} available'
        )
    extra_per_row = (height - natural_height) / (len(row_heights) + 1)
    header_height += extra_per_row
    row_heights = [value + extra_per_row for value in row_heights]

    column_lefts = [x]
    for column_width in widths[:-1]:
        column_lefts.append(column_lefts[-1] + width * column_width)
    column_widths = [width * value for value in widths]

    def draw_row(values, top, row_height, facecolor, text_color, bold=False):
        for index, value in enumerate(values):
            left = column_lefts[index]
            cell_width = column_widths[index]
            axis.add_patch(
                plt.Rectangle(
                    (left, top - row_height),
                    cell_width,
                    row_height,
                    facecolor=facecolor,
                    edgecolor='#c6ced7',
                    linewidth=0.45,
                    zorder=1,
                )
            )
            axis.text(
                left + padding,
                top - row_height / 2,
                value,
                fontsize=fontsize,
                color=text_color,
                va='center',
                ha='left',
                fontweight='bold' if bold else 'normal',
                linespacing=1.15,
                zorder=2,
            )

    top = y + height
    draw_row(wrapped_columns, top, header_height, header_color, 'white', bold=True)
    top -= header_height
    for index, (row, row_height) in enumerate(zip(wrapped_rows, row_heights), start=1):
        draw_row(
            row,
            top,
            row_height,
            'white' if index % 2 else LIGHT_GREY,
            '#20262e',
        )
        top -= row_height


def draw_simple_svg(axis, path, font_scale=0.55):
    """Redraw the project's simple SVG evidence as native PDF vectors."""
    root = ET.parse(path).getroot()
    width = float(root.attrib['width'])
    height = float(root.attrib['height'])
    axis.set_xlim(0, width)
    axis.set_ylim(height, 0)
    axis.set_aspect('equal', adjustable='box')
    axis.axis('off')

    def tag_name(element):
        return element.tag.rsplit('}', 1)[-1]

    def number(value, reference=None, default=0.0):
        if value is None:
            return default
        if value.endswith('%') and reference is not None:
            return reference * float(value[:-1]) / 100.0
        return float(value)

    for element in root:
        tag = tag_name(element)
        attributes = element.attrib
        stroke = attributes.get('stroke', 'none')
        fill = attributes.get('fill', 'black')
        linewidth = number(attributes.get('stroke-width'), default=1.0)

        if tag == 'rect':
            x = number(attributes.get('x'), default=0.0)
            y = number(attributes.get('y'), default=0.0)
            rect_width = number(attributes.get('width'), width)
            rect_height = number(attributes.get('height'), height)
            axis.add_patch(
                plt.Rectangle(
                    (x, y),
                    rect_width,
                    rect_height,
                    facecolor='none' if fill == 'none' else fill,
                    edgecolor='none' if stroke == 'none' else stroke,
                    linewidth=linewidth,
                )
            )
        elif tag == 'line':
            axis.plot(
                [number(attributes.get('x1')), number(attributes.get('x2'))],
                [number(attributes.get('y1')), number(attributes.get('y2'))],
                color=stroke,
                linewidth=linewidth,
            )
        elif tag == 'polyline':
            values = [
                float(value)
                for value in re.split(r'[,\s]+', attributes['points'].strip())
                if value
            ]
            axis.plot(
                values[::2],
                values[1::2],
                color=stroke,
                linewidth=linewidth,
            )
        elif tag == 'text':
            anchors = {'start': 'left', 'middle': 'center', 'end': 'right'}
            axis.text(
                number(attributes.get('x')),
                number(attributes.get('y')),
                ''.join(element.itertext()),
                color=fill,
                fontsize=number(attributes.get('font-size'), default=12.0) * font_scale,
                ha=anchors.get(attributes.get('text-anchor', 'start'), 'left'),
                va='baseline',
            )


def page_one(pdf):
    """Render overview, architecture, and design page."""
    figure, axis = new_page()
    page_chrome(axis, 1)
    axis.text(
        0.06,
        0.91,
        'Autonomous Driving Project',
        color=NAVY,
        fontsize=24,
        fontweight='bold',
        va='top',
    )
    axis.text(0.06, 0.865, 'Standalone Documentation — Team 20', color=BLUE,
              fontsize=14, fontweight='bold', va='top')
    axis.text(
        0.06,
        0.825,
        'Lei Fang (03805710)  |  Yida Lyu (03816236)  |  '
        'Tao Zhang (03822999)  |  Jianing Huang (03814771)',
        color=GREY,
        fontsize=7.8,
        va='top',
    )
    axis.text(0.06, 0.802, 'ROS 2 Jazzy · Ubuntu 24.04 · Unity · 03 August 2026',
              color=GREY, fontsize=7.8, va='top')

    axis.add_patch(plt.Rectangle((0.06, 0.675), 0.88, 0.095,
                                 facecolor=LIGHT_BLUE, edgecolor='none'))
    wrapped_text(
        axis,
        'Team 20 implemented a complete ROS 2 autonomous-driving pipeline for the '
        'supplied Unity urban course. The vehicle follows a validated 64-waypoint route, '
        'detects vehicles and traffic lights without the semantic camera, generates a '
        'dynamically feasible local trajectory, applies traceable safety decisions, and '
        'controls steering, throttle, and braking. One full_system.launch.py entry point '
        'starts the production graph; controller_node is the sole /car_command publisher.',
        0.08,
        0.752,
        width=112,
        fontsize=8.7,
        line_height=0.018,
    )

    axis.text(0.06, 0.635, 'Architecture and authoritative data flow', color=NAVY,
              fontsize=11, fontweight='bold', va='top')
    flow = (
        'Unity RGB/depth/TF ──> perception ──> observations ───────────────┐\n'
        'Unity pose ──> route_manager ──> trajectory_generator ──> trajectory ─┤\n'
        '                                  └─> planning speed ─────────────┤\n'
        '                                                                 v\n'
        '                                                        decision_making\n'
        '                                                                 │ final speed/stop\n'
        'Unity velocity/TF ──> controller <──── timed trajectory ──────────┘\n'
        '                             └─> /car_command ──> Unity'
    )
    axis.add_patch(plt.Rectangle((0.06, 0.445), 0.88, 0.165,
                                 facecolor='#f8fafc', edgecolor='#c9d3dd', linewidth=0.7))
    axis.text(0.08, 0.59, flow, family='DejaVu Sans Mono', fontsize=7.4,
              color='#263238', va='top', linespacing=1.25)

    axis.text(0.06, 0.415, 'Main design choices', color=NAVY, fontsize=11,
              fontweight='bold', va='top')
    rows = [
        ['Route', 'Fixed 64-waypoint YAML',
         'Reproducible and inspectable on the fixed course; no global replanning.'],
        ['Trajectory', 'Projection + Hermite/Bézier + dynamic envelope',
         'Smooth path and per-point feasible speed/time under steering and acceleration limits.'],
        ['Perception', 'YOLOv8n ONNX on RGB + depth/point cloud',
         'Avoids semantic-camera shortcuts; route/HSV/temporal logic handles small lights.'],
        ['Decision', 'Explicit safety-priority state machine',
         'Traceable traffic-light, following, merge, emergency, and stale-input behavior.'],
        ['Control', 'Pure Pursuit + PID + safe-stop gates',
         'Transparent closed-loop control; stale or infeasible input produces braking.'],
    ]
    add_table(axis, rows, ['Area', 'Choice', 'Rationale / trade-off'],
              [0.06, 0.195, 0.88, 0.19], [0.12, 0.34, 0.54], fontsize=6.9)

    axis.text(0.06, 0.178, 'Implemented requirement coverage', color=NAVY, fontsize=10.5,
              fontweight='bold', va='top')
    bullet_list(
        axis,
        [
            'Route completion; traffic-light response at waypoints 25/32/52/59; adaptive '
            'vehicle following and safe merge/emergency handling.',
            'Curvature-, lateral-acceleration-, acceleration-, and braking-constrained '
            'timed trajectory with a Team 20 ROS message.',
            'Production perception uses regular RGB, depth, calibration, and TF; '
            'segCamera.enabled is false.',
        ],
        0.065,
        0.151,
        width=118,
        fontsize=7.5,
        line_height=0.015,
        gap=0.004,
    )
    pdf.savefig(figure)
    plt.close(figure)


def page_two(pdf):
    """Render package and node catalogue."""
    figure, axis = new_page()
    page_chrome(axis, 2)
    title(axis, 'ROS packages and nodes',
          'Every workspace package and installed ROS node, including offline/non-production tools')

    def wrap(value, width):
        return '\n'.join(textwrap.wrap(value, width=width, break_long_words=False))

    rows_raw = [
        ('simulation',
         'JSON_param_reader; Unity_ROS_message_Rx; ROS_Unity_command_Tx; 5 static TF nodes',
         'Course-supplied/reused Unity configuration, TCP sensors, UDP commands, and sensor TF.'),
        ('perception', 'yolo_rgbd_detector_node',
         'Tao: regular-RGB YOLO ONNX, RGB-D vehicle range, route-gated traffic-light observations.'),
        ('perception', 'occupancy_map_node',
         'Tao: filtered point clouds plus projected and extended occupancy maps.'),
        ('external octomap_server', 'octomap_server',
         'Reused OctoMap node producing 3-D octomap and projected-map topics.'),
        ('route_manager', 'route_manager_node; waypoint_recorder (offline tool)',
         'Lei: route validation/progress and local context; the offline recorder captures TF poses into route YAML.'),
        ('autonomy_interfaces', 'no runtime node',
         'Jianing: Team 20 TrajectoryPoint and PlannedTrajectory message definitions.'),
        ('trajectory_generator', 'trajectory_generator_node',
         'Jianing: projection, smoothing/resampling, curvature/dynamic checks, timed trajectory.'),
        ('decision_making', 'obstacle_monitor_node',
         'Yida: planning-corridor point cloud plus stable route-gated RGB-D vehicle range fusion.'),
        ('decision_making', 'traffic_light_state_machine_node',
         'Yida: safety-priority traffic-light/following/merge/emergency state machine and final intent.'),
        ('controller', 'controller_node',
         'Yida: input validation, Pure Pursuit, PID, bounded commands, safe stop, sole /car_command.'),
        ('system_bringup', 'launch; live_visualization; 2 RViz nodes',
         'Shared integration: one production launch. Visualization is diagnostic only.'),
        ('dummy_controller', 'dummy_controller_node (not production)',
         'Course-supplied example. It must never run with Team 20 controller_node.'),
    ]
    rows = [[wrap(a, 24), wrap(b, 34), wrap(c, 64)] for a, b, c in rows_raw]
    add_table(
        axis,
        rows,
        ['Package', 'Node(s)', 'Functionality / ownership'],
        [0.055, 0.205, 0.89, 0.60],
        [0.19, 0.29, 0.52],
        fontsize=6.4,
    )

    axis.add_patch(plt.Rectangle((0.055, 0.075), 0.89, 0.105,
                                 facecolor=LIGHT_BLUE, edgecolor='none'))
    axis.text(0.07, 0.166, 'Interface contract', color=NAVY, fontsize=9.5,
              fontweight='bold', va='top')
    wrapped_text(
        axis,
        'The custom /planning/trajectory contains ordered pose, signed curvature, target '
        'speed, and relative time. /planning/path remains the geometric interface for RViz '
        'and obstacle corridors. /planning/target_speed is a proposal: decision_making '
        'publishes the final /decision/target_speed and stop request used by the controller. '
        'Team 20 authored TrajectoryPoint.msg and PlannedTrajectory.msg, satisfying the '
        'required own-message element.',
        0.07,
        0.143,
        width=118,
        fontsize=7.5,
        line_height=0.015,
    )
    pdf.savefig(figure)
    plt.close(figure)


def page_modules(pdf):
    """Render a module-by-module explanation with the governing equations."""
    figure, axis = new_page()
    page_chrome(axis, 3)
    title(
        axis,
        'Functional modules and core equations',
        'How sensing is converted into one safe, dynamically feasible vehicle command',
    )

    def card(x, y, heading, owner, body, footer_label, footer, color,
             equation=False):
        width = 0.42
        height = 0.225
        axis.add_patch(
            FancyBboxPatch(
                (x, y),
                width,
                height,
                boxstyle='round,pad=0.012,rounding_size=0.012',
                facecolor='#fbfcfe',
                edgecolor=color,
                linewidth=1.1,
            )
        )
        axis.add_patch(
            plt.Rectangle((x, y + height - 0.047), width, 0.047,
                          facecolor=color, edgecolor='none')
        )
        axis.text(x + 0.014, y + height - 0.015, heading, color='white',
                  fontsize=9.2, fontweight='bold', va='top')
        axis.text(x + width - 0.014, y + height - 0.016, owner, color='white',
                  fontsize=6.8, ha='right', va='top')
        wrapped_text(axis, body, x + 0.016, y + height - 0.065, width=54,
                     fontsize=7.3, line_height=0.015)
        axis.text(x + 0.016, y + 0.052, footer_label.upper(), color=GREY,
                  fontsize=6.0, fontweight='bold', va='bottom')
        axis.text(
            x + (width / 2 if equation else 0.016),
            y + 0.023,
            footer,
            color=NAVY,
            fontsize=8.2 if equation else 7.0,
            ha='center' if equation else 'left',
            va='bottom',
        )

    card(
        0.055, 0.62,
        'Perception and mapping', 'Tao Zhang',
        'The regular RGB stream feeds YOLOv8n; aligned depth and calibration turn '
        'detections into metric vehicle ranges. Route-gated image evidence and temporal '
        'filtering classify small traffic lights. Point clouds produce local occupancy maps.',
        'RGB-D back-projection',
        r'$X=(u-c_x)Z/f_x,\quad Y=(v-c_y)Z/f_y$',
        '#2e7d32',
        equation=True,
    )
    card(
        0.525, 0.62,
        'Route management', 'Lei Fang',
        'A validated 64-waypoint YAML route supplies progress, local path, target point, '
        'speed limit, and intersection semantics. Ego pose is projected onto the route so '
        'downstream modules share one longitudinal reference.',
        'Key outputs',
        'Waypoint progress · local path · route context · speed limit',
        '#d97706',
    )
    card(
        0.055, 0.35,
        'Trajectory generation', 'Jianing Huang',
        'The local route is smoothed and resampled, then annotated with curvature, speed, '
        'and relative time. Forward/backward passes enforce steering, lateral acceleration, '
        'acceleration, and braking bounds before publication.',
        'Dynamic speed envelope',
        r'$v_{cap}=\min(v_{route},\sqrt{a_{lat,max}/|\kappa|})$',
        '#2f75b5',
        equation=True,
    )
    card(
        0.525, 0.35,
        'Decision making', 'Yida Lyu',
        'A safety-priority state machine combines traffic-light state, obstacle range, merge '
        'context, and stale-input checks. It can follow a lead vehicle, stop before a '
        'controlled waypoint, or request an emergency stop.',
        'Key outputs',
        'Final target speed · stop request · named behaviour state',
        '#7b1fa2',
    )
    card(
        0.055, 0.08,
        'Vehicle control', 'Yida Lyu',
        'Pure Pursuit computes steering from a look-ahead target; PID regulates longitudinal '
        'speed. Bounds, watchdogs, and feasibility gates convert the final target into the '
        'only production /car_command stream.',
        'Pure Pursuit steering',
        r'$\delta=\tan^{-1}(2L\sin\alpha/l_d)$',
        '#7b1fa2',
        equation=True,
    )
    card(
        0.525, 0.08,
        'System bringup and safety', 'Shared integration',
        'One launch file starts simulation, TF, perception, route, trajectory, decision, '
        'controller, and optional visualization. Motion is disabled by default; stale or '
        'invalid inputs lead to braking instead of an unchecked command.',
        'Operational boundary',
        'Single launch entry point · motion off by default · fail-safe braking',
        '#546e7a',
    )
    pdf.savefig(figure)
    plt.close(figure)


def page_three(pdf):
    """Render a readable vector overview and link to the full SVG graph."""
    figure, axis = new_page(LANDSCAPE)
    page_chrome(axis, 4, landscape=True)
    title(
        axis,
        'Runtime ROS 2 graph and contribution ownership',
        'Readable vector overview; the complete topic-level graph is linked below',
        y=0.925,
    )

    def module_box(x, y, width, height, heading, nodes, facecolor, edgecolor):
        patch = FancyBboxPatch(
            (x, y),
            width,
            height,
            boxstyle='round,pad=0.008,rounding_size=0.012',
            facecolor=facecolor,
            edgecolor=edgecolor,
            linewidth=1.4,
            zorder=3,
        )
        axis.add_patch(patch)
        axis.text(
            x + width / 2,
            y + height - 0.025,
            heading,
            ha='center',
            va='top',
            fontsize=8.5,
            fontweight='bold',
            color=NAVY,
            zorder=4,
        )
        axis.text(
            x + width / 2,
            y + height / 2 - 0.014,
            nodes,
            ha='center',
            va='center',
            fontsize=7.1,
            color='#20262e',
            linespacing=1.22,
            zorder=4,
        )

    def arrow(start, end, label, bend=0.0, color='#546e7a'):
        connection = f'arc3,rad={bend}'
        axis.annotate(
            '',
            xy=end,
            xytext=start,
            arrowprops={
                'arrowstyle': '-|>',
                'color': color,
                'linewidth': 1.15,
                'connectionstyle': connection,
                'shrinkA': 2,
                'shrinkB': 2,
            },
            zorder=2,
        )
        midpoint_x = (start[0] + end[0]) / 2
        midpoint_y = (start[1] + end[1]) / 2 + (0.035 if bend >= 0 else -0.035)
        axis.text(
            midpoint_x,
            midpoint_y,
            label,
            fontsize=6.2,
            color=color,
            ha='center',
            va='center',
            bbox={'facecolor': 'white', 'edgecolor': 'none', 'pad': 1.2, 'alpha': 0.9},
            zorder=5,
        )

    module_box(
        0.055, 0.48, 0.145, 0.20,
        'simulation\ncourse supplied',
        'Unity_ROS_message_Rx\nROS_Unity_command_Tx\nstatic TF publishers',
        '#f2f2f2', '#8c8c8c',
    )
    module_box(
        0.255, 0.69, 0.17, 0.16,
        'perception — Tao Zhang',
        'yolo_rgbd_detector_node\noccupancy_map_node\noctomap_server',
        '#e8f5e9', '#43a047',
    )
    module_box(
        0.255, 0.30, 0.17, 0.14,
        'route_manager — Lei Fang',
        'route_manager_node',
        '#fff3e0', '#fb8c00',
    )
    module_box(
        0.49, 0.30, 0.18, 0.14,
        'trajectory — Jianing Huang',
        'trajectory_generator_node\nautonomy_interfaces',
        '#e3f2fd', '#1e88e5',
    )
    module_box(
        0.51, 0.67, 0.19, 0.18,
        'decision_making — Yida Lyu',
        'obstacle_monitor_node\ntraffic_light_state_machine_node',
        '#f3e5f5', '#7b1fa2',
    )
    module_box(
        0.79, 0.52, 0.155, 0.18,
        'controller — Yida Lyu',
        'controller_node\nsole /car_command publisher',
        '#f3e5f5', '#7b1fa2',
    )

    arrow((0.20, 0.63), (0.255, 0.75), 'RGB + depth + TF')
    arrow((0.20, 0.52), (0.255, 0.37), 'vehicle pose / TF')
    arrow((0.425, 0.37), (0.49, 0.37), 'local route + limits')
    arrow((0.425, 0.77), (0.51, 0.77), 'light / vehicle observations')
    arrow((0.58, 0.44), (0.58, 0.67), 'planning speed + path')
    arrow((0.67, 0.39), (0.81, 0.52), 'timed trajectory', bend=-0.08, color='#1565c0')
    arrow((0.70, 0.75), (0.79, 0.64), 'final speed + stop')
    arrow((0.79, 0.56), (0.20, 0.49), '/car_command', bend=-0.34, color='#7b1fa2')
    arrow((0.20, 0.58), (0.79, 0.59), 'vehicle twist / TF', bend=-0.18)

    integration = FancyBboxPatch(
        (0.245, 0.12),
        0.52,
        0.085,
        boxstyle='round,pad=0.008,rounding_size=0.012',
        facecolor='#e0f7fa',
        edgecolor='#00838f',
        linewidth=1.3,
    )
    axis.add_patch(integration)
    axis.text(
        0.505,
        0.162,
        'system_bringup/full_system.launch.py — shared integration\n'
        'starts all production modules and optional visualization',
        ha='center',
        va='center',
        fontsize=7.5,
        color=NAVY,
        fontweight='bold',
    )

    axis.text(
        0.5,
        0.078,
        'Open the complete full-resolution ros_graph.svg (all nodes and topics)',
        color='#0b57d0',
        fontsize=9.0,
        fontweight='bold',
        ha='center',
        va='center',
        url=GRAPH_SVG_URL,
    )
    axis.text(
        0.5,
        0.058,
        'Repository path: project/docs/ros_graph.svg',
        color=GREY,
        fontsize=7.2,
        ha='center',
        va='center',
    )
    pdf.savefig(figure)
    plt.close(figure)


def page_four(pdf, metrics):
    """Render validation figures and limitations."""
    figure, axis = new_page()
    page_chrome(axis, 5)
    title(axis, 'Recorded results and known limitations',
          'Retained production run: 01 August 2026 · one reproducible evidence example')

    axis.add_patch(plt.Rectangle((0.055, 0.785), 0.89, 0.085,
                                 facecolor=LIGHT_BLUE, edgecolor='none'))
    summary = (
        f"Waypoint 63 reached · mission time {metrics['mission_time_s']:.3f} s · "
        f"{metrics['trajectory_messages']:,} trajectories / "
        f"{metrics['trajectory_points']:,} points · trajectory audit passed · "
        f"tracking RMSE {metrics['tracking_rmse_m']:.3f} m · no runtime process death"
    )
    wrapped_text(axis, summary, 0.075, 0.842, width=90, fontsize=9.0,
                 line_height=0.02, weight='bold', color=NAVY)

    stop_ax = figure.add_axes([0.08, 0.545, 0.37, 0.20])
    stops = metrics['stop_events']
    stop_ax.bar([str(item['waypoint']) for item in stops],
                [item['distance'] for item in stops], color=BLUE)
    stop_ax.set_title('Recorded traffic-light stop distance', fontsize=8.5,
                      fontweight='bold', color=NAVY)
    stop_ax.set_xlabel('controlled waypoint', fontsize=7)
    stop_ax.set_ylabel('distance (m)', fontsize=7)
    stop_ax.tick_params(labelsize=7)
    stop_ax.grid(axis='y', alpha=0.2)

    dynamics_ax = figure.add_axes([0.56, 0.545, 0.36, 0.20])
    measured = [
        metrics['max_abs_curvature_per_m'],
        metrics['max_lateral_acceleration_mps2'],
        metrics['max_longitudinal_acceleration_mps2'],
        metrics['max_longitudinal_deceleration_mps2'],
    ]
    allowed = [
        metrics['configured_max_curvature_per_m'],
        metrics['configured_limits_mps2']['lateral'],
        metrics['configured_limits_mps2']['longitudinal_acceleration'] + 0.05,
        metrics['configured_limits_mps2']['longitudinal_deceleration'] + 0.05,
    ]
    ratios = [100 * value / limit for value, limit in zip(measured, allowed)]
    dynamics_ax.bar(['curvature', 'lateral a', 'accel.', 'decel.'], ratios,
                    color=[GREEN, GREEN, ORANGE, GREEN])
    dynamics_ax.axhline(100, color='#b91c1c', linewidth=1, linestyle='--')
    dynamics_ax.set_ylim(0, 108)
    dynamics_ax.set_title('Constraint use (incl. audit tolerance)', fontsize=8.5,
                          fontweight='bold', color=NAVY)
    dynamics_ax.set_ylabel('% of allowed value', fontsize=7)
    dynamics_ax.tick_params(labelsize=6.8)
    dynamics_ax.grid(axis='y', alpha=0.2)

    availability_ax = figure.add_axes([0.08, 0.31, 0.37, 0.19])
    availability_values = [
        100 * metrics['runtime_empty_trajectory_fraction'],
        metrics['maximum_continuous_empty_trajectory_s'],
        metrics['maximum_runtime_trajectory_message_gap_s'],
    ]
    availability_limits = [
        100 * metrics['maximum_runtime_empty_trajectory_fraction'],
        metrics['maximum_allowed_continuous_empty_trajectory_s'],
        metrics['maximum_allowed_runtime_trajectory_message_gap_s'],
    ]
    availability_ratios = [100 * value / limit for value, limit in
                           zip(availability_values, availability_limits)]
    availability_ax.bar(['empty fraction', 'empty interval', 'message gap'],
                        availability_ratios, color=GREEN)
    availability_ax.axhline(100, color='#b91c1c', linewidth=1, linestyle='--')
    availability_ax.set_title('Trajectory availability vs acceptance gate', fontsize=8.5,
                              fontweight='bold', color=NAVY)
    availability_ax.set_ylabel('% of maximum', fontsize=7)
    availability_ax.tick_params(labelsize=6.8)
    availability_ax.grid(axis='y', alpha=0.2)

    lights_ax = figure.add_axes([0.56, 0.31, 0.36, 0.19])
    light_counts = metrics['traffic_light_state_counts']
    names = ['unknown', 'red', 'yellow', 'green']
    colors = ['#9ca3af', '#c62828', '#f9a825', '#2e7d32']
    lights_ax.bar(names, [light_counts[name] for name in names], color=colors)
    lights_ax.set_title('Traffic-light observations', fontsize=8.5,
                        fontweight='bold', color=NAVY)
    lights_ax.set_ylabel('messages', fontsize=7)
    lights_ax.tick_params(labelsize=6.8)
    lights_ax.grid(axis='y', alpha=0.2)

    axis.text(0.06, 0.275, 'Known limitations / incomplete extensions', color=NAVY,
              fontsize=10.5, fontweight='bold', va='top')
    bullet_list(
        axis,
        [
            'The bundled detector is COCO-pretrained YOLOv8n, not a Team 20 Unity-fine-tuned '
            'model. Small lights need route gates, per-intersection ROIs, HSV/layout evidence, '
            'and temporal filtering.',
            'A changed map, route, camera resolution, or field of view requires recalibration '
            'and full validation; the fixed YAML route does not globally replan.',
            'The trajectory envelope omits jerk, tyre-force coupling, steering rate, drivetrain '
            'lag, and free-space obstacle replanning. Obstacle risk is handled longitudinally.',
            'Missing/stale sensor data deliberately brakes; brief TCP/depth interruptions may '
            'therefore cause conservative transient stops.',
            'Large raw bags are retained through the documented Git LFS reproduction record; '
            'a separate no-event run would strengthen comparative evidence.',
        ],
        0.065,
        0.245,
        width=117,
        fontsize=7.4,
        line_height=0.0145,
        gap=0.004,
    )
    pdf.savefig(figure)
    plt.close(figure)


def page_mission_figures(pdf, metrics):
    """Render four retained mission-result SVGs as legible PDF vectors."""
    figure, axis = new_page(LANDSCAPE)
    page_chrome(axis, 6, landscape=True)
    title(
        axis,
        'Mission-result evidence',
        'Retained full-system run; source SVGs are redrawn as native vectors for lossless zoom',
        y=0.925,
    )

    panels = [
        ([0.055, 0.49, 0.42, 0.37], 'route_trace.svg'),
        ([0.525, 0.56, 0.42, 0.30], 'speed_profile.svg'),
        ([0.525, 0.245, 0.42, 0.27], 'tracking_error.svg'),
        ([0.055, 0.16, 0.42, 0.27], 'traffic_light_state.svg'),
    ]
    for bounds, filename in panels:
        evidence_axis = figure.add_axes(bounds)
        draw_simple_svg(evidence_axis, MISSION_RESULTS_DIR / filename)

    axis.add_patch(
        FancyBboxPatch(
            (0.505, 0.075),
            0.44,
            0.115,
            boxstyle='round,pad=0.009,rounding_size=0.01',
            facecolor=LIGHT_BLUE,
            edgecolor='#8bb9d9',
            linewidth=0.7,
        )
    )
    analysis = (
        f"Analysis — the vehicle reached waypoint 63 in {metrics['mission_time_s']:.1f} s. "
        f"The speed trace reflects planned slowdowns and stops rather than a constant-speed "
        f"replay. Tracking RMSE was {metrics['tracking_rmse_m']:.3f} m; the bounded error "
        f"supports closed-loop path following. Traffic-light transitions contain all four "
        f"states and the recorded red-light stops occurred before their controlled waypoints."
    )
    wrapped_text(axis, analysis, 0.522, 0.172, width=78, fontsize=7.2,
                 line_height=0.015, color=NAVY)
    axis.add_patch(
        FancyBboxPatch(
            (0.055, 0.075),
            0.42,
            0.09,
            boxstyle='round,pad=0.008,rounding_size=0.008',
            facecolor=LIGHT_GREY,
            edgecolor='#c6ced7',
            linewidth=0.6,
        )
    )
    wrapped_text(
        axis,
        'Source SVGs: route_trace · speed_profile · tracking_error · '
        'traffic_light_state',
        0.07,
        0.145,
        width=66,
        fontsize=6.8,
        line_height=0.014,
        color=GREY,
        weight='bold',
    )
    wrapped_text(
        axis,
        'Directory: project/mission_results/ · Numerical evidence: metrics.json and '
        'validation_provenance.json',
        0.07,
        0.105,
        width=66,
        fontsize=6.6,
        line_height=0.013,
        color=GREY,
    )
    pdf.savefig(figure)
    plt.close(figure)


def page_safety_figures(pdf, metrics):
    """Render obstacle and actuator evidence as native PDF vectors."""
    figure, axis = new_page(LANDSCAPE)
    page_chrome(axis, 7, landscape=True)
    title(
        axis,
        'Obstacle and actuator safety evidence',
        'Full-run signals retained alongside the route, speed, tracking, and traffic-light plots',
        y=0.925,
    )

    obstacle_axis = figure.add_axes([0.07, 0.56, 0.86, 0.245])
    draw_simple_svg(obstacle_axis, MISSION_RESULTS_DIR / 'obstacle_distance.svg')
    command_axis = figure.add_axes([0.07, 0.275, 0.86, 0.245])
    draw_simple_svg(command_axis, MISSION_RESULTS_DIR / 'command_outputs.svg')

    axis.add_patch(
        FancyBboxPatch(
            (0.07, 0.095),
            0.86,
            0.125,
            boxstyle='round,pad=0.009,rounding_size=0.01',
            facecolor=LIGHT_BLUE,
            edgecolor='#8bb9d9',
            linewidth=0.7,
        )
    )
    event2_distance = min(
        metrics['event2_min_front_m'], metrics['event2_min_merge_m']
    )
    analysis = (
        f"Analysis — finite ranges indicate intervals in which the obstacle monitor had "
        f"a relevant front or merge target; the Event2 minimum was {event2_distance:.3f} m. "
        f"The vehicle completed the route without collision or emergency-state entry. "
        f"The controller trace shows bounded steering and alternating acceleration/braking "
        f"phases. No retained sample commanded throttle and brake simultaneously, which "
        f"confirms the mutual-exclusion gate at the final command boundary."
    )
    wrapped_text(axis, analysis, 0.09, 0.196, width=150, fontsize=7.7,
                 line_height=0.016, color=NAVY)
    pdf.savefig(figure)
    plt.close(figure)


def page_five(pdf):
    """Render contributions, reuse boundary, and bibliography."""
    figure, axis = new_page()
    page_chrome(axis, 8)
    title(axis, 'Team contribution, reused code, and bibliography',
          'Ownership describes project implementation, not invention of public algorithms')

    contributions = [
        ['Lei Fang\n03805710', 'route_manager; integration',
         'Route YAML/model, validation, waypoint progress, local route/context, speed limits, tests.'],
        ['Yida Lyu\n03816236', 'decision_making; controller; integration',
         'Obstacle fusion, behavior state machine, Pure Pursuit/PID, safety and command interfaces.'],
        ['Tao Zhang\n03822999', 'perception; integration',
         'YOLO RGB-D ROS integration, light processing, vehicle range, point clouds/maps.'],
        ['Jianing Huang\n03814771', 'trajectory_generator; autonomy_interfaces; integration; documentation',
         'Projection, smoothing/resampling, constrained time profile, custom messages, documentation.'],
    ]
    add_table(axis, contributions, ['Member', 'Primary responsibility', 'Traceable work'],
              [0.055, 0.63, 0.89, 0.23], [0.18, 0.31, 0.51], fontsize=6.8)
    wrapped_text(
        axis,
        'All four members contributed to interface/configuration alignment, system_bringup, '
        'end-to-end tests, and dynamic validation at their module boundaries. No finer '
        'individual split of shared integration work is claimed.',
        0.06,
        0.605,
        width=120,
        fontsize=7.5,
        line_height=0.015,
    )

    axis.text(0.06, 0.545, 'External code and reuse boundary', color=NAVY, fontsize=10.5,
              fontweight='bold', va='top')
    bullet_list(
        axis,
        [
            'Course-supplied: Unity simulator, original simulation/ROS bridge, and '
            'dummy_controller. These are retained/reused and not claimed as Team 20 code.',
            'Third-party: YOLOv8n/COCO weights and Ultralytics tooling, ONNX Runtime, OpenCV, '
            'ROS 2/rclcpp/rclpy, TF2, OctoMap, Eigen, NumPy, PyYAML, nlohmann/json, and the '
            'Unity runtime. Their licences remain applicable.',
            'Public methods: Pure Pursuit, PID, Hermite/Bézier interpolation, constant-'
            'acceleration reachability, and finite-state machines. Team 20 owns its project '
            'implementations, ROS integration, tests, and tuning, not the underlying methods.',
            'Model record: yolov8n.onnx, 640×640, 80 COCO classes, SHA-256 '
            '8707bb120b03a6bf8a199e7c35c6d47ad5f83de4fdd233110db270f7d355a3be.',
        ],
        0.065,
        0.515,
        width=117,
        fontsize=7.3,
        line_height=0.014,
        gap=0.004,
    )

    axis.text(0.06, 0.315, 'Bibliography', color=NAVY, fontsize=10.5,
              fontweight='bold', va='top')
    references = [
        '1. Technical University of Munich, Introduction to ROS: Autonomous Driving '
        'Project Assignment, AutonomousDriving.pdf, 2026.',
        '2. R. C. Coulter, Implementation of the Pure Pursuit Path Tracking Algorithm, '
        'CMU-RI-TR-92-01, Carnegie Mellon University, 1992.',
        '3. G. Farin, Curves and Surfaces for CAGD: A Practical Guide, 5th ed., Morgan '
        'Kaufmann, 2002.',
        '4. K. J. Åström and T. Hägglund, PID Controllers: Theory, Design, and Tuning, '
        '2nd ed., ISA, 1995.',
        '5. Open Robotics, ROS 2 Jazzy documentation, https://docs.ros.org/en/jazzy/, '
        'accessed 2026-08-01.',
        '6. Ultralytics, Model Export with Ultralytics YOLO, '
        'https://docs.ultralytics.com/modes/export/, accessed 2026-08-01.',
        '7. Microsoft, ONNX Runtime C++ documentation, '
        'https://onnxruntime.ai/docs/get-started/with-cpp.html, accessed 2026-08-01.',
        '8. A. Hornung et al., “OctoMap: An Efficient Probabilistic 3D Mapping Framework '
        'Based on Octrees,” Autonomous Robots 34, 189–206, 2013.',
    ]
    y = 0.287
    for reference in references:
        y = wrapped_text(axis, reference, 0.065, y, width=122, fontsize=6.8,
                         line_height=0.0128)
        y -= 0.004

    axis.add_patch(plt.Rectangle((0.055, 0.052), 0.89, 0.046,
                                 facecolor=LIGHT_BLUE, edgecolor='none'))
    wrapped_text(
        axis,
        'Formal source: DocumentationTeam20.md · ROS graph: project/docs/ros_graph.svg · '
        'Validation data: project/mission_results/metrics.json · Reproduction record: '
        'project/Rosbag_reproduction/README.md',
        0.07,
        0.087,
        width=125,
        fontsize=7.0,
        line_height=0.014,
        color=NAVY,
        weight='bold',
    )
    pdf.savefig(figure)
    plt.close(figure)


def main():
    """Generate the eight-page PDF."""
    metrics = json.loads(METRICS_PATH.read_text(encoding='utf-8'))
    metadata = {
        'Title': 'DocumentationTeam20 — Autonomous Driving Project',
        'Author': 'Team 20: Lei Fang, Yida Lyu, Tao Zhang, Jianing Huang',
        'Subject': 'Introduction to ROS autonomous-driving project documentation',
        'Keywords': 'ROS 2, autonomous driving, Team 20, Unity, YOLO, trajectory',
    }
    with PdfPages(OUTPUT_PATH, metadata=metadata) as pdf:
        page_one(pdf)
        page_two(pdf)
        page_modules(pdf)
        page_three(pdf)
        page_four(pdf, metrics)
        page_mission_figures(pdf, metrics)
        page_safety_figures(pdf, metrics)
        page_five(pdf)
    print(f'Generated {OUTPUT_PATH}')


if __name__ == '__main__':
    main()
