#!/usr/bin/env python3
"""Generate the Gazebo world with a painted figure-eight track.

The track is generated from the same parametric curve used by the
trajectory controller.  The raw Gerono lemniscate

    x_raw(t) = A cos(t)
    y_raw(t) = B sin(t) cos(t)

starts at (A, 0) with tangent +Y.  It is rotated -90 deg and translated so
that the path starts at (0, 0) with tangent +X, matching the robot spawn pose
and the odometry frame used by the controller.
"""

import math
from pathlib import Path

import numpy as np

A = 1.8
B = 1.44
N = 250
HALF_WIDTH = 0.008
HEIGHT = 0.002


def figure_eight(n=N):
    t = np.linspace(0.0, 2.0 * math.pi, n, endpoint=False)
    x_raw = A * np.cos(t)
    y_raw = B * np.sin(t) * np.cos(t)
    # Start at the self-intersection (t = pi/2), where curvature is zero.
    # Rotate and translate so the path starts at (0, 0) with tangent +X.
    t0 = 0.5 * math.pi
    x0 = A * math.cos(t0)
    y0 = B * math.sin(t0) * math.cos(t0)
    vx0 = -A * math.sin(t0)
    vy0 = B * (math.cos(t0) ** 2 - math.sin(t0) ** 2)
    theta0 = math.atan2(vy0, vx0)
    c, s = math.cos(-theta0), math.sin(-theta0)
    x = c * (x_raw - x0) - s * (y_raw - y0)
    y = s * (x_raw - x0) + c * (y_raw - y0)
    return np.column_stack([x, y])


def resample_closed(points, n=N):
    closed = np.vstack([points, points[0]])
    d = np.linalg.norm(np.diff(closed, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(d)])
    total = s[-1]
    si = np.linspace(0.0, total, n, endpoint=False)
    x = np.interp(si, s, closed[:, 0])
    y = np.interp(si, s, closed[:, 1])
    return np.column_stack([x, y])


def track_links(points):
    n = len(points)
    out = []
    for i in range(n):
        p0 = points[i]
        p1 = points[(i + 1) % n]
        mid = 0.5 * (p0 + p1)
        dx, dy = p1 - p0
        length = math.hypot(dx, dy)
        yaw = math.atan2(dy, dx)
        # overlap segments slightly to avoid gaps
        seg_len = length + 0.006
        out.append(
            f"""    <link name="seg_{i:04d}">
      <pose>{mid[0]:.6f} {mid[1]:.6f} {HEIGHT / 2:.6f} 0 0 {yaw:.6f}</pose>
      <visual name="visual">
        <geometry>
          <box>
            <size>{seg_len:.6f} {HALF_WIDTH * 2:.6f} {HEIGHT:.6f}</size>
          </box>
        </geometry>
        <material>
          <ambient>0.95 0.95 0.95 1</ambient>
          <diffuse>0.95 0.95 0.95 1</diffuse>
          <specular>0.1 0.1 0.1 1</specular>
        </material>
      </visual>
    </link>"""
        )
    return "\n".join(out)


def main():
    points = resample_closed(figure_eight())
    links = track_links(points)
    world = f"""<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="ackermann_track">
    <physics name="1ms" type="ignored">
      <max_step_size>0.002</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>

    <scene>
      <ambient>0.75 0.75 0.75 1</ambient>
      <background>0.80 0.85 0.90 1</background>
      <shadows>true</shadows>
    </scene>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.4 0.2 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>20 20</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>20 20</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.55 0.57 0.60 1</ambient>
            <diffuse>0.55 0.57 0.60 1</diffuse>
            <specular>0.2 0.2 0.2 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <model name="figure_eight_track">
      <static>true</static>
{links}
    </model>

    <!-- Top-down camera for trajectory recording (image streamed through the
         /top_camera/image_raw topic via ros_gz_image). -->
    <model name="top_camera">
      <static>true</static>
      <pose>0 0 3.00 0 0 0</pose>
      <link name="link">
        <sensor name="camera" type="camera">
          <pose>0 0 0 0 1.5708 0</pose>
          <camera>
            <horizontal_fov>1.2</horizontal_fov>
            <image>
              <width>1280</width>
              <height>720</height>
            </image>
            <clip>
              <near>0.05</near>
              <far>100.0</far>
            </clip>
          </camera>
          <always_on>1</always_on>
          <update_rate>15</update_rate>
          <topic>top_camera/image_raw</topic>
          <visualize>false</visualize>
        </sensor>
      </link>
    </model>
  </world>
</sdf>
"""
    out = Path(__file__).resolve().parents[1] / "ros2_ws" / "src" / "proyecto_rm_bringup" / "worlds" / "ackermann_track.sdf"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(world)
    print(f"Wrote {out} ({len(points)} segments, path length {np.sum(np.linalg.norm(np.diff(np.vstack([points, points[0]]), axis=0), axis=1)):.3f} m)")


if __name__ == "__main__":
    main()
