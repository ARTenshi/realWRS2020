import math

####coriddor####
LOCATION_ENTRANCE = 'entrance'
LOCATION_CORRIDOR_ENTRANCE = 'corridor entrance'
LOCATION_CORRIDOR_EXIT ='corridor exit'
LOCATION_CORRIDOR = 'corridor'

####kitchen####
LOCATION_CUPBOARD = 'cupboard'
LOCATION_STORAGE_TABLE = 'storage table'
LOCATION_KITCHEN = 'kitchen'
LOCATION_SINK = 'sink'
LOCATION_COUNTER_FRONT = 'counter front'
LOCATION_COUNTER = 'counter'
LOCATION_DISHWASHER = 'dishwasher'

####bedroom####
LOCATION_BEDROOM = 'bedroom'
LOCATION_BED = 'bed'
LOCATION_DESK = 'desk'
LOCATION_SIDE_TABLE = 'side table'

####livingroom####
LOCATION_LIVING_ROOM = 'living room'
LOCATION_BOOKCASE = 'bookcase'
LOCATION_COUCH= 'couch'
LOCATION_COUCH_RIGHT= 'couch right'
LOCATION_END_TABLE = 'end table'

####dining room####
LOCATION_DINING_ROOM = 'dining room'
LOCATION_DINING_ROOM_SUB = 'dining room sub'
LOCATION_DINING_TABLE = 'dining table'

####exit####
LOCATION_EXIT = 'exit'




LOCATIONS = {####coriddor####
             LOCATION_ENTRANCE  :((0, 0),0),
             LOCATION_CORRIDOR_ENTRANCE :((2.24,-0.02),0),
             LOCATION_CORRIDOR_EXIT :((0, 0),0),
             LOCATION_CORRIDOR :((5.28, -0.16),-3.13),

             ####kitchen####
             LOCATION_CUPBOARD :((7.44, -1.78),0.02),
             LOCATION_STORAGE_TABLE :((7.29,-2.63),0.02),
             LOCATION_KITCHEN :((8.07, -3.66),2.28),
             LOCATION_SINK :((7.56, -3.96),-3.11),
             LOCATION_COUNTER_FRONT :((5.78, -2.80),-1.58),
             LOCATION_COUNTER :((5.91, -5.21),1.58),
             LOCATION_DISHWASHER :((5.78, -2.80),-1.58),
             
             ####bedroom####
             LOCATION_BEDROOM :((10.31, -3.00),1.79),
             LOCATION_BED :((10.31, -0.86),-0.02),
             LOCATION_DESK :((11.04, -1.65),-0.47),
             LOCATION_SIDE_TABLE :((10.37, -0.52),0.79),

             ####livingroom####
             LOCATION_LIVING_ROOM :((9.48, -4.20),-0.64),
             LOCATION_BOOKCASE :((9.98, -7.03),-1.55),
             LOCATION_COUCH :((11.42, -4.11),-0.97),
             LOCATION_COUCH_RIGHT :((11.06,-7.14),0.71),
             LOCATION_END_TABLE :((10.12, -5.67),0.00),

             ####dining room####
             LOCATION_DINING_ROOM :((8.39, -4.30),-2.17),
             LOCATION_DINING_ROOM_SUB :((9.26, -6.80),-3.12,),
             LOCATION_DINING_TABLE :((7.55, -4.99),-1.57),

             ####exit####
             LOCATION_EXIT :((11.51, -8.95),-1.57)}
                                                                              

SUB_LOCATIONS = {LOCATION_COUCH:LOCATION_COUCH_RIGHT,
                 LOCATION_COUNTER:LOCATION_COUNTER_FRONT,
                 LOCATION_DINING_ROOM:LOCATION_DINING_ROOM_SUB}

GPSR_INITIAL_POSE = ((0., 0.),0.)
GPSR_COMMAND_POSE = ((6.00, -1.78),-0.01,)
GPSR_FINAL_POSE = ((11.51, -8.95),-1.57,)
EEGPSR_INITIAL_POSE = ((0, 0), 0,)
EEGPSR_COMMAND_POSE = ((5.78, -2.80), -1.58)
EEGPSR_FINAL_POSE = ((11.51, -8.95),-1.57,)

DOOR_POSES = [((-5.1, 4.26), -math.pi/2),
              ((-1.2, 0.), math.pi),
              ((-0.6, 6.6), -math.pi/2),
              ((-1.94, 12.), 0.)]

'''
PLACEMENT_HEIGHTS = {'bar': 0.01,
                     'bookcase': 0.01,
                     }
'''
PLACEMENT_HEIGHTS = {####coriddor####
                     LOCATION_ENTRANCE : [0.],
                     LOCATION_CORRIDOR_ENTRANCE : [0.],
                     LOCATION_CORRIDOR_EXIT : [0.],
                     LOCATION_CORRIDOR : [0.],

                     ####kitchen####
                     LOCATION_CUPBOARD : [0.1,0.29,0.64,1.06,1.41],
                     LOCATION_STORAGE_TABLE : [0.45],
                     LOCATION_KITCHEN : [0.91],
                     LOCATION_SINK : [0.82],
                     LOCATION_COUNTER : [0.91],
                     LOCATION_DISHWASHER : [0.],

                     ####bedroom####
                     LOCATION_BEDROOM : [0.],
                     LOCATION_BED : [0.55],
                     LOCATION_DESK : [0.73],
                     LOCATION_SIDE_TABLE : [0.45],

                     ####livingroom####
                     LOCATION_LIVING_ROOM : [0.],
                     LOCATION_BOOKCASE : [0.07,0.40,0.67,0.94,1.34],
                     LOCATION_COUCH : [0.37],
                     LOCATION_END_TABLE : [0.45],
                     ####dining room####
                     LOCATION_DINING_ROOM : [0.],
                     LOCATION_DINING_TABLE : [0.74],

                     ####exit####
                     LOCATION_ENTRANCE : [0.],
                    }

PLACEMENT_WIDTHS = {####coriddor####
                    LOCATION_ENTRANCE : [0.],
                    LOCATION_CORRIDOR_ENTRANCE : [0.],
                    LOCATION_CORRIDOR_EXIT : [0.],
                    LOCATION_CORRIDOR : [0.],

                    ####kitchen####
                    LOCATION_CUPBOARD : [0.57],
                    LOCATION_STORAGE_TABLE : [0.55],
                    LOCATION_KITCHEN : [1.63],
                    LOCATION_SINK : [0.61],
                    LOCATION_COUNTER : [1.63],
                    LOCATION_DISHWASHER : [0.],

                    ####bedroom####
                    LOCATION_BEDROOM : [0.],
                    LOCATION_BED : [0.],
                    LOCATION_DESK : [0.76],
                    LOCATION_SIDE_TABLE : [0.55],

                    ####livingroom####
                    LOCATION_LIVING_ROOM : [0.],
                    LOCATION_BOOKCASE : [0.57],
                    LOCATION_COUCH : [0.],
                    LOCATION_END_TABLE : [1.10],
                    ####dining room####
                    LOCATION_DINING_ROOM : [0.],
                    LOCATION_DINING_TABLE : [1.0],

                    ####exit####
                    LOCATION_ENTRANCE : [0.],
                   }

PLACEMENT_DEPTHS = {####coriddor####
                    LOCATION_ENTRANCE : [0.],
                    LOCATION_CORRIDOR_ENTRANCE : [0.],
                    LOCATION_CORRIDOR_EXIT : [0.],
                    LOCATION_CORRIDOR : [0.],

                    ####kitchen####
                    LOCATION_CUPBOARD : [0.26],
                    LOCATION_STORAGE_TABLE : [0.55],
                    LOCATION_KITCHEN : [0.61],
                    LOCATION_SINK : [0.46],
                    LOCATION_COUNTER : [0.61],
                    LOCATION_DISHWASHER : [0.],

                    ####bedroom####
                    LOCATION_BEDROOM : [0.],
                    LOCATION_BED : [0.],
                    LOCATION_DESK : [0.6],
                    LOCATION_SIDE_TABLE : [0.55],

                    ####livingroom####
                    LOCATION_LIVING_ROOM : [0.],
                    LOCATION_BOOKCASE : [0.22],
                    LOCATION_COUCH : [0.],
                    LOCATION_END_TABLE : [0.55],
                    ####dining room####
                    LOCATION_DINING_ROOM : [0.],
                    LOCATION_DINING_TABLE : [0.6],

                    ####exit####
                    LOCATION_ENTRANCE : [0.],
                    }

# [x, y, z] = [height, width. depth]
OBJECT_SIZE_INDEX_HEIGHT = 0
OBJECT_SIZE_INDEX_WIDTH = 1
OBJECT_SIZE_INDEX_DEPTH = 2
DEFAULT_OBJECT_SIZES = [0.1, 0.1, 0.1]
OBJECTS_SIZES = { 'chocolate_drink':[0.18, 0.06, 0.06],
		  'coke':[0.12, 0.07, 0.07],
                  'grape_juice':[0.04, 0.05, 0.04],
                  'orange_juice':[0.04, 0.05, 0.04],
                  'sprite':[0.12, 0.07, 0.07],
                  'cereal':[0.12, 0.07, 0.04],
                  'noodles':[0.03, 0.15, 0.1],
                  'sausages':[0.06, 0.06, 0.06],
                  'apple':[0.08, 0.08, 0.08],
                  'orange':[0.08, 0.08, 0.08],
                  'paprika':[0.10, 0.08, 0.08],
		  'crackers':[0.10, 0.10, 0.03],
		  'potato_chips':[0.22, 0.14, 0.07],
		  'pringles':[0.23, 0.08, 0.08],
		  }

                  










