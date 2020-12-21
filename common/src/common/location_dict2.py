import math

LOCATIONS = {'desk': ((2.722, -0.311), -1.6), 'left planks': ((6.26, 5.15), -2.41), 'right rack': ((13.055, 2.137), -0.844), 'bedroom': ((4.13, -2.23), -2.38), 'sideboard': ((9.7, -0.31), -0.86), 'tv': ((10.85, 4.8), 1.66), 'balcony shelf': ((7.86, 7.88), 0.68), 'left rack': ((12.94, 2.51), -0.77), 'little desk': ((6.27, -3.14), -0.78), 'corridor': ((5.1, 1.7), 0), 'entrance': ((2.0, 0.0), 0), 'living room': ((10.85, 4.8), 1.66), 'cupboard': ((0.964, 0.292), -0.491), 'sofa': ((1.131, -1.475), -0.691), 'kitchen counter': ((6.25, 0.34), -2.39), 'fridge': ((7.05, -1.04), -2.27), 'bookcase': ((-6.528, 10.507), -2.921), 'coffee table': ((10.7, 4.78), 2.3), 'bistro table': ((8.09, 6.56), -2.97), 'right planks': ((5.99, 6.02), 3.08), 'kitchen': ((7.38, 1.49), 0), 'entrance shelf': ((2.3, 1.0), 0.0), 'teepee': ((-0.039, -0.088), -0.076), 'dining table': ((-0.125, 1.674), -3.126), 'kitchen table': ((-0.121, 1.671), -3.13), 'bed': ((-0.037, -0.087), -0.076), 'kitchen shelf': ((0.962, 0.291), -0.491), 'kitchen rack': ((8.25, 0.88), 0.82)}


GPSR_INITIAL_POSE = ((12.84, 2.54), -2.39)
GPSR_COMMAND_POSE = ((12.84, 2.54), -2.39)
GPSR_FINAL_POSE = ((12, 0.1),0)
EEGPSR_INITIAL_POSE = ((12.84, 2.54), -2.39)
EEGPSR_COMMAND_POSE = ((12.84,2.54), -2.39)
EEGPSR_FINAL_POSE = ((13.04, 0.1),0)
SG_INITIAL_POSE = ((7.37, 1.48), 0.95)

DOOR_POSES = [((-5.1, 4.26), -math.pi/2),
              ((-1.2, 0.), math.pi),
              ((-0.6, 6.6), -math.pi/2),
              ((-1.94, 12.), 0.)]

PLACEMENT_HEIGHTS = {
             'entrance shelf': 0.934,
             'sideboard': 0.903,
             'kitchen rack': 0.903,
             'left rack': 0.903,
             'tv': .0,
             'kitchen table': 0.903,
             'desk': 0.903,
             'kitchen counter': 0.903,
             'left planks': 0.903,
             'bed': .0,
             'right rack': 0.903,
             'teepee': .0,
             'bookcase': 0.903,
             'coffee table': 0.705,
             'bistro table': .705,
             'sofa':.0,
             'little desk': 0.705,
             'balcony shelf': 0.705,
             'right planks': 0.705,
             'fridge': 0.705,
             'kitchen shelf': 0.705,#[0.10, 0.51, 0.74, 1.06, 1.47]
             'kitchen': .0,
             'bedroom': .0,
             'living room': .0,
             'balcony shelf': .0,
             'entrance': .0,
             'corridor': .0
             }
