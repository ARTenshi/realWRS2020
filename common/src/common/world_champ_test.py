import math

#LOCATIONS = {'bar': (0, 0), 'bookcase': (0, 0)}
#(12.94, 2.51),-0.77)
#(11.65, 3.19), 0.75)
LOCATIONS = {'entrance shelf': ((12.94, 2.51),-0.77),
             'sideboard': ((12.94, 2.51),-0.77),
             'kitchen rack': ((12.94, 2.51),-0.77),
             'left rack': ((12.94, 2.51), -0.77),
             'tv': ((12.94, 2.51),-0.77),
             'kitchen table': ((12.94, 2.51),-0.77),
             'desk': ((12.94, 2.51),-0.77),
             'kitchen counter': ((12.94, 2.51),-0.77),
             'left planks': ((12.94, 2.51),-0.77),
             'bed': ((12.94, 2.51),-0.77),
             'right rack': ((12.94, 2.51),-0.77),
             'teepee': ((12.94, 2.51),-0.77),
             'bookcase': ((12.94, 2.51),-0.77),
             'coffee table': ((11.65, 3.19), 0.75),
             'bistro table': ((11.65, 3.19), 0.75),
             'sofa': ((11.65, 3.19), 0.75),
             'little desk': ((11.65, 3.19), 0.75),
             'balcony shelf': ((11.65, 3.19), 0.75),
             'right planks': ((11.65, 3.19), 0.75),
             'fridge': ((11.65, 3.19), 0.75),
             'kitchen shelf': ((11.65, 3.19), 0.75),
             'kitchen': ((11.65, 3.19), 0.75),
             'bedroom': ((11.65, 3.19), 0.75),
             'living room': ((11.65, 3.19), 0.75),
             'balcony shelf': ((11.65, 3.19), 0.75),
             'entrance': ((11.65, 3.19), 0.75),
             'corridor': ((11.65, 3.19), 0.75)}#vivePC

GPSR_INITIAL_POSE = ((11.9, 1.32), -0.79)
GPSR_COMMAND_POSE = ((11.9, 1.32), -0.79-math.pi)
GPSR_FINAL_POSE = ((0, 0),0)
EEGPSR_INITIAL_POSE = ((11.9, 1.32), -0.79)
EEGPSR_COMMAND_POSE = ((11.9, 1.32), -0.79)
EEGPSR_FINAL_POSE = ((0, 0),0)

DOOR_POSES = [((-5.1, 4.26), -math.pi/2),
              ((-1.2, 0.), math.pi),
              ((-0.6, 6.6), -math.pi/2),
              ((-1.94, 12.), 0.)]

#0.903
#0.705
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
             'bistro table': .0,
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
