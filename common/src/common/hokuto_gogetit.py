import math

OBJECT_START = 'start'
OBJECT_STOP = 'stop'

OBJECT_RED = 'red'
OBJECT_BLUE = 'blue'
OBJECT_WHITE = 'white'
OBJECT_BLACK = 'black'
OBJECT_YELLOW = 'yellow'
OBJECT_ORANGE = 'orange'

OBJECT_BIG = 'big'
OBJECT_SMALL = 'small'
OBJECT_TOY = 'toy'
OBJECT_SOFT = 'soft'
OBJECT_HARD = 'hard'
OBJECT_LIGHT = 'light'
OBJECT_HEAVY = 'heavy'

OBJECT_SNACK = 'snack'
OBJECT_DRINK = 'drink'
OBJECT_FOOD = 'food'
OBJECT_TOOL = 'tool'
OBJECT_KITCHEN = 'kitchen'
OBJECT_SHAPE = 'shape'


OBJECTS = {OBJECT_START: ((0.0, 0.0), 0.0),\
           OBJECT_STOP: ((0.0, 0.0), 0.0),\
           OBJECT_RED: ((0.0, 0.0), 0.0),\
           OBJECT_BLUE:((0.0, 0.0), 0.0),\
           OBJECT_WHITE:((0.0, 0.0), 0.0),\
           OBJECT_BLACK:((0.0, 0.0), 0.0),\
           OBJECT_YELLOW:((0.0, 0.0), 0.0),\
           OBJECT_ORANGE:((0.0, 0.0), 0.0),\
           OBJECT_BIG:((0.0, 0.0), 0.0),\
           OBJECT_SMALL:((0.0, 0.0), 0.0),\
           OBJECT_SOFT:((0.0, 0.0), 0.0),\
           OBJECT_HARD:((0.0, 0.0), 0.0),\
           OBJECT_LIGHT:((0.0, 0.0), 0.0),\
           OBJECT_HEAVY: ((0.0, 0.0), 0.0),\
           OBJECT_SNACK:((0.0, 0.0), 0.0),\
           OBJECT_DRINK:((0.0, 0.0), 0.0),\
           OBJECT_FOOD:((0.0, 0.0), 0.0),\
           OBJECT_TOOL:((0.0, 0.0), 0.0),\
           OBJECT_KITCHEN:((0.0, 0.0), 0.0),\
           OBJECT_SHAPE:((0.0, 0.0), 0.0),\
           OBJECT_TOY:((0.0, 0.0), 0.0)}

LOCATION_START = 'start'
LOCATION_STOP = 'stop'

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
LOCATIONS = {LOCATION_START: ((0.0, 0.0), 0.0),\
             LOCATION_STOP: ((0.0, 0.0), 0.0),\
             LOCATION_ENTRANCE: ((0.0, 0.0), 0.0),\
             LOCATION_EXIT: ((0.0, 0.0), 0.0),\
             LOCATION_LOUNGE: ((0.0, 0.0), 0.0),\
             LOCATION_OFFICE: ((0.0, 0.0), 0.0),\
             LOCATION_BEDROOM: ((0.0, 0.0), 0.0),\
             LOCATION_KITCHEN: ((0.0, 0.0), 0.0),\
             LOCATION_CORRIDOR_ENTRANCE: ((0.0, 0.0), 0.0),\
             LOCATION_CORRIDOR_EXIT: ((0.0, 0.0), 0.0),\
             LOCATION_BED: ((0.0, 0.0), 0.0),\
             LOCATION_SIDE_SHELF: ((0.0, 0.0), 0.0),\
             LOCATION_TEEPEE: ((0.0, 0.0), 0.0),\
             LOCATION_DESK: ((0.0, 0.0), 0.0),\
             LOCATION_BOOK_SHELF: ((0.0, 0.0), 0.0),\
             LOCATION_LIVING_RACK: ((0.0, 0.0), 0.0),\
             LOCATION_SOFA: ((0.0, 0.0), 0.0),\
             LOCATION_LIVING_TABLE: ((0.0, 0.0), 0.0),\
             LOCATION_LIVING_TABLE_SIDE: ((0.0, 0.0), 0.0),\
             LOCATION_CUPBOARD: ((0.0, 0.0), 0.0),\
             LOCATION_KITCHEN_TABLE: ((0.0, 0.0), 0.0),\
             LOCATION_KITCHEN_TABLE_SIDE: ((0.0, 0.0), 0.0),\
             LOCATION_KITCHEN_RACK: ((0.0, 0.0), 0.0),\
             LOCATION_CORRIDOR: ((0.0, 0.0), 0.0),\
             LOCATION_BEDSIDE: ((0.0, 0.0), 0.0),\
             LOCATION_LIVING_ROOM: ((0.0, 0.0), 0.0),\
             LOCATION_LIVING_SHELF: ((0.0, 0.0), 0.0),\
             LOCATION_TV_STAND: ((0.0, 0.0), 0.0),\
             LOCATION_CENTER_TABLE: ((0.0, 0.0), 0.0),\
             LOCATION_BAR: ((0.0, 0.0), 0.0),\
             LOCATION_DRAWER: ((0.0, 0.0), 0.0),\
             LOCATION_KITCHENETTE: ((0.0, 0.0), 0.0),\
             LOCATION_SINK: ((0.0, 0.0), 0.0),\
             LOCATION_DINING_TABLE: ((0.0, 0.0), 0.0),\
             LOCATION_FRIDGE: ((0.0, 0.0), 0.0),\
             LOCATION_CABINET: ((0.0, 0.0), 0.0),\
             LOCATION_BATHROOM: ((0.0, 0.0), 0.0),\
             LOCATION_BOOKCASE: ((0.0, 0.0), 0.0),\
             LOCATION_COUNTER: ((0.0, 0.0), 0.0),\
             LOCATION_SIDE_TABLE: ((0.0, 0.0), 0.0)}
