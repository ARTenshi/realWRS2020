#!/usr/bin/env python
# -*- coding: utf-8 -*

import math

LOCATION_ENTRANCE = 'entrance'
LOCATION_EXIT = 'exit'
LOCATION_CORRIDOR_ENTRANCE = 'corridor entrance'
LOCATION_CORRIDOR_EXIT ='corridor exit'
LOCATION_BED = 'bed'
LOCATION_SIDE_SHELF = 'side shelf'
LOCATION_TEEPEE = 'teepee'
LOCATION_DESK = 'desk'
LOCATION_BOOK_SHELF = 'book shelf'
LOCATION_LIVING_RACK = 'living rack'
LOCATION_SOFA = 'sofa'
LOCATION_LIVING_TABLE = 'living table'
LOCATION_LIVING_TABLE_SIDE ='living table side'
LOCATION_CUPBOARD = 'cupboard'
LOCATION_KITCHEN_TABLE = 'kitchen table'
LOCATION_KITCHEN_TABLE_SIDE = 'kitchen table side'
LOCATION_KITCHEN_RACK = 'kitchen rack'

LOCATIONS = {LOCATION_ENTRANCE: ((2.37, -0.56), 2.54),\
             LOCATION_EXIT: ((2.07, -8.62), -1.59),\
             #corridor
             LOCATION_CORRIDOR_ENTRANCE: ((2.05, -0.28), -1.64),\
             LOCATION_CORRIDOR_EXIT: ((1.97, -8.07), 1.56),\
             # bed room
             LOCATION_BED: ((4.43, -0.38), 1.57),\
             LOCATION_SIDE_SHELF: ((4.53, -1.69), -1.57),\
             #kids room
             LOCATION_TEEPEE: ((8.09, -1.16), 2.39),\
             LOCATION_DESK: ((9.68, -1.17), -0.02),\
             LOCATION_BOOK_SHELF: ((10.15, 0.101), 0.01),\
             #living room
             LOCATION_LIVING_RACK: ((4.51, -3.55), 1.52),\
             LOCATION_SOFA: ((4.94, -4.72), -0.02),\
             LOCATION_LIVING_TABLE: ((7.91, -4.82), 3.11),\
             LOCATION_LIVING_TABLE_SIDE: ((7.08, -6.06), 1.50),\
             #kitchen

             LOCATION_CUPBOARD: ((10.1, -9.95), -0.01),\
             LOCATION_KITCHEN_TABLE: ((5.69, -8.19), 0.03),\

             LOCATION_KITCHEN_TABLE_SIDE: ((6.64, -6.78), -1.61),\
             LOCATION_KITCHEN_RACK: ((4.171, -7.888), -3.11)
             }

GPSR_INITIAL_POSE = ((0., 0.), 0.)
#TODO
GPSR_COMMAND_POSE = ((4.50, -1.20), -0.01)
GPSR_FINAL_POSE = ((2.07, -11.62), -1.59)

SG_INITIAL_POSE = LOCATIONS['kitchen rack']

DOOR_POSES = [((-5.1, 4.26), -math.pi/2),
              ((-1.2, 0.), math.pi),
              ((-0.6, 6.6), -math.pi/2),
              ((-1.94, 12.), 0.)]

PLACEMENT_HEIGHTS = {
             LOCATION_ENTRANCE: [0.],\
             LOCATION_EXIT: [0.],\
             #corridor
             LOCATION_CORRIDOR_ENTRANCE: [0.],\
             LOCATION_CORRIDOR_EXIT: [0.],\
             # bed room
             LOCATION_BED: [0.25, 0.36],\
             LOCATION_SIDE_SHELF: [0.02, 0.31, 0.61, 0.90],\
             #kids room
             LOCATION_TEEPEE: [0.],\
             LOCATION_DESK: [0.70],\
             LOCATION_BOOK_SHELF: [0.02, 0.31, 0.61, 0.90],\
             #living room
             LOCATION_LIVING_RACK: [0.10, 0.38, 0.71, 1.06,  1.38, 1.70, 2.02],\
             LOCATION_SOFA: [0.20],\
             LOCATION_LIVING_TABLE: [0.46],\
             LOCATION_LIVING_TABLE_SIDE: [0.46],\
             #kitchen
             LOCATION_CUPBOARD: [0.15, 0.54, 0.92, 1.31],\
             LOCATION_KITCHEN_TABLE: [0.76],\
             LOCATION_KITCHEN_TABLE_SIDE: [0.76],\
             LOCATION_KITCHEN_RACK: [0.10, 0.38, 0.71, 1.06,  1.38, 1.70, 2.02]
             }

PLACEMENT_WIDTHS = {
             LOCATION_ENTRANCE: [0.],\
             LOCATION_EXIT: [0.],\
             #corridor
             LOCATION_CORRIDOR_ENTRANCE: [0.],\
             LOCATION_CORRIDOR_EXIT: [0.],\
             # bed room
             LOCATION_BED: [0.],\
             LOCATION_SIDE_SHELF: [0.42],\
             #kids room
             LOCATION_TEEPEE: [0.],\
             LOCATION_DESK: [1.0],\
             LOCATION_BOOK_SHELF: [0.42],\
             #living room
             LOCATION_LIVING_RACK: [0.76],\
             LOCATION_SOFA: [0.],\
             LOCATION_LIVING_TABLE: [1.10],\
             LOCATION_LIVING_TABLE_SIDE: [0.59],\
             #kitchen
             LOCATION_CUPBOARD: [0.55],\
             LOCATION_KITCHEN_TABLE: [1.60],\
             LOCATION_KITCHEN_TABLE_SIDE: [0.80],\
             LOCATION_KITCHEN_RACK: [0.76]
             }

PLACEMENT_DEPTHS = {
             LOCATION_ENTRANCE: [0.],\
             LOCATION_EXIT: [0.],\
             #corridor
             LOCATION_CORRIDOR_ENTRANCE: [0.],\
             LOCATION_CORRIDOR_EXIT: [0.],\
             # bed room
             LOCATION_BED: [0.],\
             LOCATION_SIDE_SHELF: [0.29],\
             #kids room
             LOCATION_TEEPEE: [0.],\
             LOCATION_DESK: [0.4, 0.6],\
             LOCATION_BOOK_SHELF: [0.29],\
             #living room
             LOCATION_LIVING_RACK: [0.28],\
             LOCATION_SOFA: [0.],\
             LOCATION_LIVING_TABLE: [0.59],\
             LOCATION_LIVING_TABLE_SIDE: [1.10],\
             #kitchen
             LOCATION_CUPBOARD: [0.30],\
             LOCATION_KITCHEN_TABLE: [0.80],\
             LOCATION_KITCHEN_TABLE_SIDE: [1.60],\
             #chair height:0.46
             LOCATION_KITCHEN_RACK: [0.28]
             }

