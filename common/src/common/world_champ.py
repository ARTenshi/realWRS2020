import math

#LOCATIONS = {'bar': (0, 0), 'bookcase': (0, 0)}
LOCATIONS = {'entrance shelf': ((2.3, 1.0), .0),
             'sideboard': ((9.70, -0.31), -0.86),
             'kitchen rack': ((8.25, 0.88), 0.82),
             'left rack': ((12.94, 2.51), -0.77),
             'tv': ((10.85, 4.80), 1.66),
             'kitchen table': ((6.77, 0.8), -0.83),
             'desk': ((11.65, 3.19), 0.75),
             'kitchen counter': ((6.25, 0.34), -2.39),
             'left planks': ((6.26, 5.15), -2.41),
             #bed 
             'bed': ((4.11, -2.17), -2.17),
             'right rack': ((13.055, 2.137), -0.844),
             'teepee': ((5.24, -3.42), -1.47),
             'bookcase': ((7.97, 3.44), -1.69),
             'coffee table': ((10.7, 4.78), 2.3),
             'bistro table': ((8.09, 6.56), -2.97),
             'sofa': ((10.29, 3.33), 2.37),
             'little desk': ((6.27, -3.14), -0.78),
             'balcony shelf': ((7.86, 7.88), 0.68),
             'right planks': ((5.99, 6.02), 3.08),
             'fridge': ((7.05, -1.04), -2.27),
             'kitchen shelf': ((7.35, 1.46), 0.95 ),
             
             'kitchen': ((7.38, 1.49), 0),
             'bedroom': ((4.13, -2.23), -2.38),
             'living room': ((10.85, 4.80), 1.66),
             'balcony shelf': ((7.86, 7.88), 0.68),
             'entrance': ((2.0, 0.0), 0),
             'corridor': ((5.1,1.7),0)}


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
