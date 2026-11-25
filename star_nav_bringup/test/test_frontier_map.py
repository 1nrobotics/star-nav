#!/usr/bin/env python3
"""Tests of scripts/frontier_map.py, without ROS:

    python3 -m unittest discover -s star_nav_bringup/test -p 'test_frontier_map.py'

A lidar is imitated in a flat world of walls: two rooms joined by a door.
"""

import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "scripts"))
import frontier_map  # noqa: E402

RES = 0.1
SENSOR_Z = 0.44


class World:
    """Two 4 x 4 m rooms side by side, a 1 m door in the wall between them."""

    def __init__(self, door=True):
        self.shape = (100, 60)  # 10 x 6 m, origin (-5, -3)
        self.origin = np.array([-5.0, -3.0])
        w = np.zeros(self.shape, bool)
        w[9:11, 9:51] = w[89:91, 9:51] = True      # x = -4, x = 4
        w[9:91, 9:11] = w[9:91, 49:51] = True      # y = -2, y = 2
        w[49:51, 9:51] = True                      # x = 0
        if door:
            w[49:51, 25:35] = False
        self.walls = w

    def scan(self, xy, rays=720, max_range=12.0):
        """Returns of a lidar at ``xy``: the floor every 0.25 m from 0.8 m on
        and two points on the wall each ray ends at."""
        pts = []
        for a in np.linspace(0.0, 2 * np.pi, rays, endpoint=False):
            d = np.array([np.cos(a), np.sin(a)])
            for r in np.arange(0.0, max_range, RES / 2):
                p = xy + r * d
                i, j = np.floor((p - self.origin) / RES).astype(int)
                if not (0 <= i < self.shape[0] and 0 <= j < self.shape[1]):
                    break
                if self.walls[i, j]:
                    pts += [(p[0], p[1], 0.1), (p[0], p[1], 0.4)]
                    break
                if r >= 0.8 and abs(r / 0.25 - round(r / 0.25)) < 1e-6:
                    pts.append((p[0], p[1], 0.0))
        return np.array(pts)


def explore(world, start, max_goals=40, **kwargs):
    """Walk from frontier to frontier until none is left."""
    m = frontier_map.FrontierMap(size=(12.0, 8.0), resolution=RES, **kwargs)
    xy = np.array(start, float)
    goals = []
    for _ in range(max_goals):
        m.insert((xy[0], xy[1], SENSOR_Z), world.scan(xy))
        m.mark_seen(xy, 0.5)
        clusters, dist = m.plan(xy)
        if not clusters:
            break
        goal = clusters[0].goal
        assert np.isfinite(dist[goal])
        if np.hypot(*(m.point(goal) - xy)) < 0.3:
            m.give_up(xy, 0.5)  # nothing more to see from here
        xy = m.point(goal)
        goals.append(xy)
    return m, goals


class FrontierMapTest(unittest.TestCase):

    def test_one_scan(self):
        world = World()
        m = frontier_map.FrontierMap(size=(12.0, 8.0), resolution=RES)
        m.insert((-2.0, 0.0, SENSOR_Z), world.scan(np.array([-2.0, 0.0])))
        m.mark_seen((-2.0, 0.0), 0.5)
        g = m.grid()
        # The room it stands in is known, the wall is an obstacle, the far
        # corner of the other room is not seen.
        self.assertEqual(g[m.cell((-3.0, 1.0))], frontier_map.FREE)
        self.assertEqual(g[m.cell((-2.0, 0.0))], frontier_map.FREE)
        self.assertEqual(g[m.cell((-2.0, 1.95))], frontier_map.OBSTACLE)
        self.assertEqual(g[m.cell((3.0, 1.5))], frontier_map.UNKNOWN)
        self.assertEqual(g[m.cell((-4.6, 0.0))], frontier_map.UNKNOWN)  # outside
        clusters, dist = m.plan((-2.0, 0.0))
        self.assertTrue(clusters)
        # Every frontier is where the robot can stand: 0.4 m from the walls.
        occ = g == frontier_map.OBSTACLE
        near = frontier_map.dilate(occ, 0.4 / RES)
        for c in clusters:
            for cell in c.cells:
                self.assertFalse(near[cell])
            self.assertAlmostEqual(c.cost, dist[c.goal], places=6)
        # Nearest first.
        self.assertEqual([c.cost for c in clusters], sorted(c.cost for c in clusters))

    def test_explores_both_rooms(self):
        world = World()
        m, goals = explore(world, (-2.0, 0.0))
        self.assertLess(len(goals), 40, "never ran out of frontiers")
        self.assertTrue(any(g[0] > 0.5 for g in goals), "never went through the door")
        self.assertGreater(m.coverage((-3.9, -1.9, -0.1, 1.9)), 0.99)
        self.assertGreater(m.coverage((0.1, -1.9, 3.9, 1.9)), 0.97)
        # Nothing outside the walls.
        self.assertLess(m.coverage((-5.0, -3.0, -4.3, 3.0)), 0.01)

    def test_closed_room_is_left_alone(self):
        world = World(door=False)
        m, goals = explore(world, (-2.0, 0.0))
        self.assertTrue(all(g[0] < 0.0 for g in goals))
        self.assertGreater(m.coverage((-3.9, -1.9, -0.1, 1.9)), 0.99)
        self.assertLess(m.coverage((0.2, -1.9, 3.9, 1.9)), 0.01)

    def test_bounds_and_giving_up(self):
        world = World()
        xy = np.array([-2.0, 0.0])
        m = frontier_map.FrontierMap(size=(12.0, 8.0), resolution=RES, bounds=(-4.0, -2.0, -0.5, 2.0))
        m.insert((xy[0], xy[1], SENSOR_Z), world.scan(xy))
        self.assertEqual(m.plan(xy)[0], [], "frontier outside the bounds")
        m = frontier_map.FrontierMap(size=(12.0, 8.0), resolution=RES)
        m.insert((xy[0], xy[1], SENSOR_Z), world.scan(xy))
        clusters, _ = m.plan(xy)
        for c in clusters:
            m.give_up(m.point(c.goal), 1.5)
        self.assertEqual(m.plan(xy)[0], [])

    def test_path_goes_through_the_door(self):
        world = World()
        m, _ = explore(world, (-2.0, 0.0))
        _, dist = m.plan((-3.0, 1.0))
        cells = m.path(dist, m.cell((3.0, 1.0)))
        self.assertEqual(cells[0], m.cell((-3.0, 1.0)))
        self.assertEqual(cells[-1], m.cell((3.0, 1.0)))
        xy = np.array([m.point(c) for c in cells])
        crossing = xy[np.argmin(np.abs(xy[:, 0]))]
        self.assertLess(abs(crossing[1]), 0.5)
        steps = np.hypot(*np.diff(xy, axis=0).T)
        self.assertLess(steps.max(), RES * 1.5)
        self.assertAlmostEqual(steps.sum(), dist[m.cell((3.0, 1.0))], places=6)

    def test_low_step_is_floor_and_box_is_not(self):
        m = frontier_map.FrontierMap(size=(6.0, 6.0), resolution=RES)
        pts = [(x, y, 0.0) for x in np.arange(-2, 2, 0.05) for y in np.arange(-2, 2, 0.05)]
        # A 7 cm step (walkable) and a 15 cm box (not).
        pts += [(1.0 + dx, dy, 0.07) for dx in np.arange(0, 0.5, 0.05) for dy in np.arange(-0.5, 0.5, 0.05)]
        pts += [(-1.0 + dx, dy, 0.15) for dx in np.arange(0, 0.3, 0.05) for dy in np.arange(0, 0.3, 0.05)]
        m.insert((0.0, 0.0, SENSOR_Z), np.array(pts))
        g = m.grid()
        self.assertEqual(g[m.cell((1.25, 0.0))], frontier_map.FREE)
        self.assertEqual(g[m.cell((-0.85, 0.15))], frontier_map.OBSTACLE)


if __name__ == "__main__":
    unittest.main()
