#!/usr/bin/env python

# Copyright 2020 RUVU Robotics B.V.

import networkx as nx
import argparse

from geometry_msgs.msg import Pose, Point

parser = argparse.ArgumentParser("Generate random pose graph")
parser.add_argument("output_file")
args = parser.parse_args()

G = nx.DiGraph()

G.add_node(1, pose=Pose())
G.add_node(2, pose=Pose(position=Point(x=2)))
G.add_node(3, pose=Pose(position=Point(x=2, y=2)))

G.add_edge(1, 2)
G.add_edge(2, 3)

nx.write_yaml(G, args.output_file)
