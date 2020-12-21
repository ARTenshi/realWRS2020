import math

OBJECT_SNACK = 'snack'
OBJECT_FRUIT = 'fruit'
OBJECT_JUICE = 'juice'

OBJECTS = {OBJECT_SNACK: ((0.0, 0.0), 0.0),\
           OBJECT_JUICE:((0.0, 0.0), 0.0),\
           OBJECT_FRUIT: ((0.0, 0.0), 0.0)}


LOCATION_ENTRANCE = 'entrance'
LOCATION_EXIT = 'exit'

LOCATION_LOUNGE = 'lounge'
LOCATION_OFFICE = 'office'
LOCATION_BEDROOM = 'bedroom'
LOCATION_KITCHEN = 'kitchen'

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

LOCATION_CORRIDOR = 'corridor'
LOCATION_BEDSIDE = 'bedside'
LOCATION_LIVING_ROOM = 'living room'
LOCATION_LIVING_SHELF = 'living shelf'
LOCATION_TV_STAND = 'tv stand'
LOCATION_CENTER_TABLE = 'center table'
LOCATION_BAR = 'bar'
LOCATION_DRAWER = 'drawer'
LOCATION_KITCHENETTE = 'kitchenette'
LOCATION_SINK = 'sink'
LOCATION_DINING_TABLE = 'dining table'
LOCATION_FRIDGE = 'fridge'
LOCATION_CABINET = 'cabinet'
LOCATION_BATHROOM = 'bathroom'
LOCATION_BOOKCASE = 'bookcase'
LOCATION_COUNTER = 'counter'
LOCATION_SIDE_TABLE = 'side table'

#LOCATIONS = {'bar': (0, 0), 'bookcase': (0, 0)}
LOCATIONS = {LOCATION_LIVING_ROOM: ((0.0, 0.0), 0.0),\
             LOCATION_OFFICE: ((0.0, 0.0), 0.0),\
             LOCATION_BEDROOM: ((0.0, 0.0), 0.0),\
             LOCATION_KITCHEN :((0.0, 0.0), 0.0)}

SUB_LOCATIONS = {LOCATION_LIVING_TABLE_SIDE: LOCATION_LIVING_TABLE,\
                 LOCATION_KITCHEN_TABLE_SIDE: LOCATION_KITCHEN_TABLE}

GPSR_INITIAL_POSE = ((0., 0.),0.)
GPSR_COMMAND_POSE = ((-0.07, 1.12),1.48)
GPSR_FINAL_POSE = ((0., 0.),0.)
EEGPSR_INITIAL_POSE = ((-0.407, 5.831), 1.708)
EEGPSR_COMMAND_POSE = ((-0.407, 8.), 1.708)
EEGPSR_FINAL_POSE = ((0, 0),0)

#DOOR_POSES = [((-5.1, 4.26), -math.pi/2),
#              ((-1.2, 0.), math.pi),
#              ((-0.6, 6.6), -math.pi/2),
#              ((-1.94, 12.), 0.)]

DOOR_POSES = [((0.04, 6.05), 1.56)]


'''
PLACEMENT_HEIGHTS = {'bar': 0.01,
                     'bookcase': 0.01,
                     }
'''
PLACEMENT_HEIGHTS = {
             LOCATION_ENTRANCE: [0.],\
             LOCATION_EXIT: [0.],\
             #corridor
             LOCATION_CORRIDOR_ENTRANCE: [0.],\
             LOCATION_CORRIDOR_EXIT: [0.],\
             # bed room
             LOCATION_BED: [0.25, 0.36],\
             LOCATION_SIDE_SHELF: [0.50, 0.95],\
             LOCATION_SIDE_TABLE: [0.86],\
             #kids room
             LOCATION_TEEPEE: [0.9],\
             LOCATION_DESK: [0.90],\
             LOCATION_BOOK_SHELF: [0.02, 0.31, 0.61, 0.90],\
             #living room
             LOCATION_LIVING_RACK: [0.10, 0.38, 0.71, 1.06,  1.38, 1.70, 2.02],\
             LOCATION_SOFA: [0.20],\
             LOCATION_LIVING_TABLE: [0.8],\
             LOCATION_LIVING_TABLE_SIDE: [0.8],\
             #kitchen
             LOCATION_CUPBOARD: [0.15, 0.54, 0.92, 1.31],\
             LOCATION_KITCHEN_TABLE: [0.76],\
             LOCATION_KITCHEN_TABLE_SIDE: [0.76],\
             LOCATION_KITCHEN_RACK: [0.90],\

             LOCATION_CENTER_TABLE: [0.5],\
             LOCATION_DINING_TABLE: [0.5],\
             LOCATION_KITCHEN: [0.8],\
             LOCATION_BED: [0.5],\
             LOCATION_BEDSIDE: [0.5],\
             LOCATION_BAR: [0.5],\
             LOCATION_SINK: [0.5],\
             LOCATION_BOOKCASE: [0.5],\
             LOCATION_FRIDGE: [0.5],\
             LOCATION_COUNTER: [0.5],\
             LOCATION_DRAWER: [0.72],\
             LOCATION_CABINET: [0.89],\
             LOCATION_KITCHENETTE : [0.89]\
             }

PLACEMENT_WIDTHS = {
             LOCATION_ENTRANCE: [0.],\
             LOCATION_EXIT: [0.],\
             #corridor
             LOCATION_CORRIDOR_ENTRANCE: [0.],\
             LOCATION_CORRIDOR_EXIT: [0.],\
             # bed room
             LOCATION_BED: [0.],\
             LOCATION_SIDE_SHELF: [1.20],\
             LOCATION_SIDE_TABLE: [0.65],\
             #kids room
             LOCATION_TEEPEE: [0.],\
             LOCATION_DESK: [0.8],\
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
             LOCATION_KITCHEN_RACK: [0.76],\

             LOCATION_CENTER_TABLE: [0.5],\
             LOCATION_DINING_TABLE: [0.5],\
             LOCATION_KITCHEN: [0.8],\
             LOCATION_BED: [0.5],\
             LOCATION_BEDSIDE: [0.5],\
             LOCATION_BAR: [0.5],\
             LOCATION_SINK: [0.5],\
             LOCATION_BOOKCASE: [0.5],\
             LOCATION_FRIDGE: [0.5],\
             LOCATION_COUNTER: [0.5],\
             LOCATION_DRAWER: [0.74],\
             LOCATION_CABINET: [0.5],\
             LOCATION_KITCHENETTE : [0.89]\
             }

PLACEMENT_DEPTHS = {
             LOCATION_ENTRANCE: [0.],\
             LOCATION_EXIT: [0.],\
             #corridor
             LOCATION_CORRIDOR_ENTRANCE: [0.],\
             LOCATION_CORRIDOR_EXIT: [0.],\
             # bed room
             LOCATION_BED: [0.],\
             LOCATION_SIDE_SHELF: [0.42],\
             LOCATION_SIDE_TABLE: [0.15],\
             #kids room
             LOCATION_TEEPEE: [0.],\
             LOCATION_DESK: [0.7],\
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
             LOCATION_KITCHEN_RACK: [0.65],\

             LOCATION_CENTER_TABLE: [0.5],\
             LOCATION_DINING_TABLE: [0.5],\
             LOCATION_KITCHEN: [0.8],\
             LOCATION_BED: [0.5],\
             LOCATION_BEDSIDE: [0.5],\
             LOCATION_BAR: [0.5],\
             LOCATION_SINK: [0.5],\
             LOCATION_BOOKCASE: [0.5],\
             LOCATION_FRIDGE: [0.5],\
             LOCATION_COUNTER: [0.5],\
             LOCATION_DRAWER: [0.4],\
             LOCATION_CABINET: [0.5],\
             LOCATION_KITCHENETTE : [0.89]\
             }

# [x, y, z] = [height, width. depth]
OBJECT_SIZE_INDEX_HEIGHT = 0
OBJECT_SIZE_INDEX_WIDTH = 1
OBJECT_SIZE_INDEX_DEPTH = 2
DEFAULT_OBJECT_SIZES = [0.1, 0.1, 0.1]
OBJECTS_SIZES = { 'curry':[0.1, 0.1, 0.1],
                  'soup':[0.1, 0.1, 0.1],
                  'jam':[0.1, 0.1, 0.1],
                  'soymilk':[0.1, 0.1, 0.1],
                  'energy drink':[0.1, 0.1, 0.1],
                  'tea':[0.1, 0.1, 0.1],
                  'kiwi':[0.1, 0.1, 0.1],
                  'tangerine':[0.1, 0.1, 0.1],
                  'biscuit':[0.1, 0.1, 0.1],
                  'chewing gum':[0.1, 0.1, 0.1],

                  'apple':[0.1, 0.1, 0.1],
                  'cup star':[0.1, 0.1, 0.1],
                  }
