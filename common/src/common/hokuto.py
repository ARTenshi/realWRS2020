import math

#OBJECT_SODA = 'soda'
#OBJECT_SNACK = 'snack'
#OBJECT_TOY = 'toy'
#OBJECT_FRUIT = 'fruit'
#OBJECT_JUICE = 'juice'

OBJECT_GELATIN_BOX = 'gelatin box'
OBJECT_PUDDING_BOX = 'pudding box'
OBJECT_APPLE = 'apple'
OBJECT_SUGAR_BOX = 'sugar box'
OBJECT_PEAR = 'pear'
OBJECT_CHIPS_CAN = 'chips can'
OBJECT_MUSTARD_BOTTLE = 'mustard bottle'
OBJECT_LEMON = 'lemon'
OBJECT_TOMATO_SOUP_CAN = 'tomato soup can'
OBJECT_CRACKER_BOX = 'cracker box'
OBJECT_STRAWBERRY = 'strawberry'
OBJECT_ORANGE = 'orange'
OBJECT_TUNA_FISH_CAN = 'tuna fish can'
OBJECT_CHEF_CAN = 'chef can'
OBJECT_APRICOT = 'apricot'
OBJECT_BANANA = 'banana'
OBJECT_PEACH = 'peach'
OBJECT_POTTED_MEAT_CAN = "potted meat can"

OBJECTS = {OBJECT_GELATIN_BOX: ((0.0, 0.0), 0.0),\
           OBJECT_PUDDING_BOX:((0.0, 0.0), 0.0),\
           OBJECT_APPLE:((0.0, 0.0), 0.0),\
           OBJECT_SUGAR_BOX:((0.0, 0.0), 0.0),\
           OBJECT_PEAR:((0.0, 0.0), 0.0),\
           OBJECT_CHIPS_CAN:((0.0, 0.0), 0.0),\
           OBJECT_MUSTARD_BOTTLE:((0.0, 0.0), 0.0),\
           OBJECT_LEMON:((0.0, 0.0), 0.0),\
           OBJECT_TOMATO_SOUP_CAN:((0.0, 0.0), 0.0),\
           OBJECT_CRACKER_BOX:((0.0, 0.0), 0.0),\
           OBJECT_STRAWBERRY:((0.0, 0.0), 0.0),\
           OBJECT_ORANGE:((0.0, 0.0), 0.0),\
           OBJECT_TUNA_FISH_CAN:((0.0, 0.0), 0.0),\
           OBJECT_CHEF_CAN:((0.0, 0.0), 0.0),\
           OBJECT_APRICOT:((0.0, 0.0), 0.0),\
           OBJECT_BANANA:((0.0, 0.0), 0.0),\
           OBJECT_PEACH:((0.0, 0.0), 0.0),\
           OBJECT_POTTED_MEAT_CAN: ((0.0, 0.0), 0.0)}


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

LOCATION_CORRIDOR = 'corridor'
LOCATION_BEDROOM = 'bedroom'
LOCATION_BEDSIDE = 'bedside'
LOCATION_LIVING_ROOM = 'living room'
LOCATION_LIVING_SHELF = 'living shelf'
LOCATION_TV_STAND = 'tv stand'
LOCATION_CENTER_TABLE = 'center table'
LOCATION_BAR = 'bar'
LOCATION_OFFICE = 'office'
LOCATION_DRAWER = 'drawer'
LOCATION_KITCHENETTE = 'kitchenette'
LOCATION_KITCHEN = 'kitchen'
LOCATION_SINK = 'sink'
LOCATION_DINING_TABLE = 'dining table'
LOCATION_FRIDGE = 'fridge'
LOCATION_CABINET = 'cabinet'
LOCATION_BATHROOM = 'bathroom'
LOCATION_BOOKCASE = 'bookcase'
LOCATION_COUNTER = 'counter'
LOCATION_SIDE_TABLE = 'side table'

#LOCATIONS = {'bar': (0, 0), 'bookcase': (0, 0)}
LOCATIONS = {LOCATION_ENTRANCE: ((-0.407, 5.831), 1.708),\
             LOCATION_EXIT: ((2.07, -8.62), -1.59),\
             #corridor
             LOCATION_CORRIDOR_ENTRANCE: ((2.05, -0.28), -1.64),\
             LOCATION_CORRIDOR_EXIT: ((1.97, -8.07), 1.56),\
             # bed room
             LOCATION_BED: ((1.023, -0.842),-0.574),\
             LOCATION_SIDE_SHELF: ((3.125, -0.81), 0.016),\
             LOCATION_SIDE_TABLE: ((2.03, 0.216), 1.659),\
             #kids room
             LOCATION_TEEPEE: ((2.15, 5.66), 1.63),\
             LOCATION_DESK: ((0.74, 2.76),-0.034),\
             LOCATION_BOOK_SHELF: ((10.15, 0.101), 0.01),\
             #living room
             LOCATION_LIVING_RACK: ((4.51, -3.55), 1.52),\
             LOCATION_SOFA: ((4.94, -4.72), -0.02),\
             LOCATION_LIVING_TABLE: ((1.32, 1.06),-0.92),\
             LOCATION_LIVING_TABLE_SIDE: ((7.08, -6.06), 1.50),\
             #kitchen
             LOCATION_CUPBOARD: ((10.1, -9.95), -0.01),\
             LOCATION_KITCHEN_TABLE: ((0.31, 4.59), 0.06),\
             LOCATION_KITCHEN_TABLE_SIDE: ((1.38, 5.78), -1.46),\
             LOCATION_KITCHEN_RACK: ((0.714, 2.63), 0.028),\

             LOCATION_BEDROOM :((-1.48, -0.07),-0.02),\
             LOCATION_BEDSIDE :((-0.39, -0.91),-2.33),\

             LOCATION_LIVING_ROOM :((0.0, 0.0),0.0),\
             LOCATION_LIVING_SHELF:((1.1, 5.5),-math.pi/2),\
             LOCATION_TV_STAND    :((0.58, -0.56),-1.47),\
             LOCATION_CENTER_TABLE:((-0.19, 1.91),-3.13),\

             LOCATION_OFFICE :((1.77, 1.67),1.61),\
             LOCATION_DRAWER :((1.77, 2.71),2.61),\
             LOCATION_KITCHENETTE   :((1.01, 2.63),0.02),\

             LOCATION_KITCHEN      :((-0.05, 8.97), 1.708),\
             LOCATION_BAR          :((2.95, 1.93),1.03),\
             LOCATION_SINK         :((2.73, 1.32),-1.56),\
             LOCATION_BOOKCASE     :((4.22, -1.23),-1.49),\
             LOCATION_DINING_TABLE :((3.61, 1.51),0.81),\
             LOCATION_FRIDGE       :((-0.43, -0.98),-1.56),\
             LOCATION_COUNTER      :((1.76, -1.11),0.81),\
             #'kitchen table' :((0.45, 4.51),0.00),

             LOCATION_CORRIDOR :((-1.45, -0.04),-3.11),\
             LOCATION_CABINET  :((-2.11, -0.04),2.39),\


             LOCATION_BATHROOM :((3.61, 1.51),0.81)} #vivePC

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
