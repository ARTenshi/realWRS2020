CATEGORY_DRINKS = 'drinks'
CATEGORY_FRUITS = 'fruits'
CATEGORY_FOOD = 'food'
CATEGORY_SNACKS = 'snacks'
CLASS = {
    CATEGORY_DRINKS:['chocolate_drink','coke','grape_juice','orange_juice','sprite'],
    CATEGORY_FRUITS:['apple','orange','paprika'],
    CATEGORY_FOOD:['cereal','noodles','sausages'],
    CATEGORY_SNACKS:['crackers','potato_chips','pringles']
}

'''
IGNORE_CLASS = {
    'paprika':[81],
}
'''
# <category name="" defaultlocation="" room="">
THRESHOLD = {
    # name:drink
    # defaultlocation:counter
    # room:kitchen
    CATEGORY_DRINKS:[0.2, 0.2, 0.2, 0.2, 0.2],
    # name:fruit
    # defaultlocation:bookcase
    # room:living room
    CATEGORY_FRUITS:[0.2, 0.2, 0.2],
    # name:food
    # defaultlocation:cupboard
    # room:kitchen room
    CATEGORY_FOOD:[0.2, 0.2, 0.2],
    # name:snack
    # defaultlocation:bookcase
    # room:living room
    CATEGORY_SNACKS:[0.2, 0.2, 0.2]
}
