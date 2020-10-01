#!/usr/bin/env python

# Copyright 2020 RUVU Robotics B.V.

import networkx as nx
import matplotlib.pyplot as plt
import argparse

from ruvu_networkx import nx_draw

parser = argparse.ArgumentParser("Draw pose graph using matplotlib")
parser.add_argument("input_file")
args = parser.parse_args()

nx_draw.draw_pose_graph(nx.read_yaml(args.input_file))
plt.show()
