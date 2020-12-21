import os
import re
import xml.etree.ElementTree as ET
from IPython.utils._tokenize_py2 import group
from numpy import prod

import wildcard_parser

class GrammarParseResult:
    def __init__(self, rule, rule_command_tuples, fully_expanded_rule, rest):
        self.__rule = rule
        self.__rule_command_tuples = rule_command_tuples
        self.__fully_expanded_rule = fully_expanded_rule
        self.__rest = rest

    def get_rule(self):
        return self.__rule

    def get_rule_command_tuples(self):
        return self.__rule_command_tuples

    def get_fully_expanded_command(self):
        __fully_expanded_command = []
        __fully_expanded_rule_index = 0
        for rule, command in self.__rule_command_tuples:
            while __fully_expanded_rule_index < len(self.__fully_expanded_rule):
                if rule == self.__fully_expanded_rule[__fully_expanded_rule_index]:
                    __fully_expanded_command.append(command)
                    __fully_expanded_rule_index += 1
                    break
                else:
                    __fully_expanded_command.append(self.__fully_expanded_rule[__fully_expanded_rule_index])
                    __fully_expanded_rule_index += 1
        return ' '.join(__fully_expanded_command)

    def check_rule_command_tuples_inclusion(self, target_rule_command_tuples):
        __LENGTH_TARGET_RULE_COMMAND_TUPLES = len(target_rule_command_tuples)
        __index_target_rule_command_tuples = 0
        __inclusion = True
        for rule, command in self.__rule_command_tuples:
            __next_rule = False
            while(__index_target_rule_command_tuples < __LENGTH_TARGET_RULE_COMMAND_TUPLES):
                __target_rule, _ = target_rule_command_tuples[__index_target_rule_command_tuples]
                __index_target_rule_command_tuples += 1
                if rule==__target_rule:
                    __next_rule = True
                    break

            if __next_rule is True:
                continue
            else:
                __inclusion =  False
                break

        return __inclusion

    def get_rest(self):
        return self.__rest

class Grammar:
    CHAR_WILDCARD_OBFUSCATED = '?'
    WILDCARD_CATEGORY = 'category'
    WILDCARD_GESTURE = 'gesture'
    WILDCARD_NAME = 'name'
    WILDCARD_FEMALE = 'female' 
    WILDCARD_MALE = 'male'
    WILDCARD_BEACON = 'beacon'
    WILDCARD_PLACEMENT = 'placement'
    WILDCARD_LOCATION = 'location'
    WILDCARD_ROOM = 'room'
    WILDCARD_OBJECT = 'object'
    WILDCARD_AOBJECT = 'aobject'
    WILDCARD_KOBJECT = 'kobject'
    WILDCARD_SOBJECT = 'sobject'
    WILDCARD_QUESTION = 'question'
    WILDCARD_VOID = 'void'
    WILDCARD_PRON = 'pron'
    def __init__(self):
        self.rules = {'$question':['question'],
                      '$void':[''],
                      '$cmanwarn':[''], #Is this correct?
                      '$pron':['he', 'she', 'his', 'her', 'him', 'it', 'its'],
                      '$category_obfuscated':['object', 'objects'],
                      '$location_obfuscated':['room']}
        self.__new_rules = {}
        self.wildcards = ['$question', '$void',
                          '$category_obfuscated',
                          '$location_obfuscated']
        self.__excluded_names = ['$vbbtake', '$vbplace',
                                 '$vbbring', '$vbdeliver',
                                 '$vbtake', '$vbspeak',
                                 '$vbgopl', '$vbgor',
                                 '$vbfind', '$vbguide',
                                 '$vbfollow', '$vbfollow',
                                 '$whattosay', '$oprop',
                                 '$pgenders', '$pgenderp',
                                 '$pose']
        self.__wildcard_pron = ['I', 'you', 'he', 'she', 'it', 'we', 'they',\
                               'me', 'him', 'her', 'us', 'them']
        self.__wildcard_void = ''

        self.__object_info = None
        self.__name_info = None
        self.__question_info = None
        self.__gesture_info = None
        self.__location_info = None

        self.__wildcard_parser = wildcard_parser.WildcardParser()

    def __check_wildcard_info(self, name):
        if name is None:
            raise Exception('name is None.')
        if name == '':
            raise Exception('name == \'\'.')
        else:
            __obfuscated = False
            if name[-1] == self.CHAR_WILDCARD_OBFUSCATED:
                __obfuscated = True
                return __obfuscated, name[:-1]
            return __obfuscated, name


    def get_wildcard_info_location(self, name):
        instance = self.__location_info
        if instance is None:
            raise Exception('instance is None.')
        __obfuscated, __wildcard = self.__check_wildcard_info(name)
        if __wildcard == self.WILDCARD_BEACON:
            if __obfuscated == True:
                return instance.get_beacons_obfuscated()
            else:
                return instance.get_beacons()
        elif __wildcard == self.WILDCARD_PLACEMENT:
            if __obfuscated == True:
                return instance.get_placements_obfuscated()
            else:
                return instance.get_placements()
        elif  __wildcard == self.WILDCARD_LOCATION:
            if __obfuscated == True:
                return instance.get_locations_obfuscated()
            else:
                return instance.get_locations()
        elif __wildcard == self.WILDCARD_ROOM:
            if __obfuscated == True:
                return instance.get_rooms_obfuscated()
            else:
                return instance.get_rooms()
        else:
            print('self.WILDCARD_PLACEMENT: {}'.format(self.WILDCARD_PLACEMENT))
            print('__wildcard: {}'.format(__wildcard))
            raise Exception('invalid wildcard.')
    def get_wildcard_info_object(self, name):
        instance = self.__object_info
        if instance is None:
            raise Exception('instance is None.')
        __obfuscated, __wildcard = self.__check_wildcard_info(name)
        if __wildcard == self.WILDCARD_CATEGORY:
            if __obfuscated == True:
                return instance.get_categories_obfuscated()
            else:
                return instance.get_categories()
        elif __wildcard == self.WILDCARD_OBJECT:
            if __obfuscated == True:
                return instance.get_objects_obfuscated()
            else:
                return instance.get_objects()
        elif __wildcard == self.WILDCARD_AOBJECT:
            if __obfuscated == True:
                return instance.get_aobjects_obfuscated()
            else:
                return instance.get_aobjects()
        elif __wildcard == self.WILDCARD_KOBJECT:
            if __obfuscated == True:
                return instance.get_kobjects_obfuscated()
            else:
                return instance.get_kobjects()
        elif __wildcard == self.WILDCARD_SOBJECT:
            if __obfuscated == True:
                return instance.get_sobjects_obfuscated()
            else:
                return instance.get_sobjects()
        else:
            raise Exception('invalid wildcard.')
    def get_wildcard_info_name(self, name):
        instance = self.__name_info
        if instance is None:
            raise Exception('instance is None.')
        __obfuscated, __wildcard = self.__check_wildcard_info(name)
        if __wildcard == self.WILDCARD_NAME:
            if __obfuscated == True:
                return instance.get_names_obfuscated()
            else:
                return instance.get_names()
        elif __wildcard == self.WILDCARD_FEMALE:
            if __obfuscated == True:
                return instance.get_names_obfuscated()
            else:
                return instance.get_female_names()        
        elif  __wildcard == self.WILDCARD_MALE:
            if __obfuscated == True:
                return instance.get_names_obfuscated()
            else:
                return instance.get_male_names()
        else:
            raise Exception('invalid wildcard.')
    def get_wildcard_info_question(self, name):
        instance = self.__question_info
        if instance is None:
            raise Exception('instance is None.')
        __obfuscated, __wildcard = self.__check_wildcard_info(name)
        if __wildcard == self.WILDCARD_QUESTION:
            if __obfuscated == True:
                return instance.get_questions_obfuscated()
            else:
                return instance.get_questions()
        else:
            raise Exception('invalid wildcard.')
    def get_wildcard_info_gesture(self, name):
        instance = self.__gesture_info
        if instance is None:
            raise Exception('instance is None.')
        __obfuscated, __wildcard = self.__check_wildcard_info(name)
        #for __gesture in instance.get_gestures():
        #    print('__gesture:{}'.format(__gesture))
        if __wildcard == self.WILDCARD_GESTURE:
            if __obfuscated == True:
                raise Exception('__obfuscated must be False.')
            else:
                return instance.get_gestures()
        else:
            raise Exception('invalid wildcard.')
    def get_wildcard_info_void(self, name):
        __obfuscated, __wildcard = self.__check_wildcard_info(name)
        if __wildcard == self.WILDCARD_VOID:
            if __obfuscated == True:
                raise Exception('__obfuscated must be False.')
            else:
                return self.__wildcard_void
        else:
            raise Exception('invalid wildcard.')
    def get_wildcard_info_pron(self, name):
        __obfuscated, __wildcard = self.__check_wildcard_info(name)
        if __wildcard == self.WILDCARD_PRON:
            if __obfuscated == True:
                raise Exception('__obfuscated must be False.')
            else:
                return self.__wildcard_pron
        else:
            raise Exception('invalid wildcard.')

    def get_objects(self):
        return self.__object_info.get_objects()
    def get_object_room(self, object):
        return self.__object_info.room_of(object)
    def get_object_category(self, object):
        return self.__object_info.category_of(object)
    def get_placement_category(self, placement):
        return self.__object_info.get_room_category(self.__location_info.room_of(placement))
    def get_category_objects(self, category):
        __objects = self.get_objects()
        return filter((lambda obj:self.__object_info.category_of(obj)==category), __objects)
    def get_room_placements(self, room):
        __locations = self.__location_info.get_locations()
        return filter((lambda loc:self.__location_info.room_of(loc)==room), __locations)
    def get_rooms(self):
        return self.__location_info.get_rooms()
    def get_question_answer(self, question):
        return self.__question_info.get_answer(question)
    def get_questions(self):
        return self.__question_info.get_questions()
    def get_names(self):
        return self.__name_info.get_names()

    WILDCARD_INFO_DIC = {WILDCARD_CATEGORY:get_wildcard_info_object,\
                         WILDCARD_GESTURE:get_wildcard_info_gesture,\
                         WILDCARD_NAME:get_wildcard_info_name,\
                         WILDCARD_FEMALE:get_wildcard_info_name,\
                         WILDCARD_MALE:get_wildcard_info_name,\
                         WILDCARD_BEACON:get_wildcard_info_location,\
                         WILDCARD_PLACEMENT:get_wildcard_info_location,\
                         WILDCARD_LOCATION:get_wildcard_info_location,\
                         WILDCARD_ROOM:get_wildcard_info_location,\
                         WILDCARD_OBJECT:get_wildcard_info_object,\
                         WILDCARD_AOBJECT:get_wildcard_info_object,\
                         WILDCARD_KOBJECT:get_wildcard_info_object,\
                         WILDCARD_SOBJECT:get_wildcard_info_object,\
                         WILDCARD_QUESTION:get_wildcard_info_question,\
                         WILDCARD_VOID:get_wildcard_info_void,\
                         WILDCARD_PRON:get_wildcard_info_pron }

    def load_rules(self, path):
        pattern = re.compile('\s*(?P<name>\$[0-9A-Za-z_]+)\s*=\s*(?P<prod>.+)\s*$')
        wc_pattern = re.compile('(?P<name>[A-Za-z_]+)(?P<obfuscation>\?)?(\s+(?P<type>[A-Za-z_]+))?(\s(?P<id>[0-9]+))?(\s+meta\s*:(?P<meta>.+))?')

        rules = {}
        with open(path) as f:
            lines = f.readlines()

        # get rules from line
        __multiline_comment = False
        # The first line contains strange character
        for l in lines[1:]:
            l, __multiline_comment = self.__strip_comments(l, __multiline_comment)
            if l == '':
                continue
            else:
                l = l.lower().strip()
            m = pattern.match(l)
            if m:
                name = m.groupdict()['name']
                prod = m.groupdict()['prod']
                # split prod at '|'
                prods = self.__split_prods(prod, 0, False)['g0']
                if name in rules:
                    rules[name].extend(prods)
                else:
                    rules[name] = prods

        #print( "rules: {}".format(rules) )
        # expand rules
        items = rules.items()
        rules = {}
        while items:
            k1, v1 = items.pop()
            #print("k1:{}, v1:{}".format(k1, v1))
            for i, s in enumerate(v1):
                # expand groups (XXXX) if any
                #print("k1:{}, v1:{}, i:{}, s:{}".format(k1, v1, i, s))
                __prods = self.__split_prods(s, 0, True)
                if __prods:
                    if k1 not in rules.keys():
                        rules[k1] = []
                    for __prod in __prods['g0']:
                        #TODO
                        #delete comment below
                        #if __prod not in rules[k1]:
                        rules[k1].append(__prod)
            #print("rules[k1:{}]:{}".format(k1, rules[k1]))

        # set rules to self.__new_rules
        for k, v in rules.items():
            #print('self.__new_fules[{}]:{}'.format(k, v))
            if k in self.__new_rules:
                self.__new_rules[k].extend(v)
            else:
                self.__new_rules[k] = v
        #print( "self.__new_rules: {}".format(self.__new_rules) )

    def __strip_comments(self, line, multiline_comment):
        __result_line = line
        __result_multiline_comment = multiline_comment
        if __result_multiline_comment == True:
            c = line.find('*/')
            if c >= 0:
                __result_line = line[(c+2):]
                __result_multiline_comment = False
            else:
                __result_line = ''

        while( __result_multiline_comment == False ):
            # multi-line comment
            __index_multiline_comment = __result_line.find('/*')
            # single-line comment
            __index_singleline_comment = -1
            for x in ['//', '#', ';', '%']:
                c = __result_line.find(x)
                if c >= 0:
                    index_update = False
                    if __index_singleline_comment < 0:
                        index_update = True
                    else:
                        if c < __index_singleline_comment:
                            index_update = True        
                    if index_update == True:
                        __index_singleline_comment = c
            # check comment existence
            __boolean_multiline_comment = (__index_multiline_comment>=0)
            __boolean_singleline_comment = (__index_singleline_comment>=0)            
            if (__boolean_multiline_comment == True) and (__boolean_singleline_comment == True):
                if __index_multiline_comment < __index_singleline_comment:
                    __boolean_singleline_comment = False
                else:
                    __boolean_multiline_comment = False

            if __boolean_multiline_comment == True:
                c = __result_line[__index_multiline_comment+2:].find('*/')
                if c >= 0:
                    __result_line = __result_line[:__index_multiline_comment] + __result_line[(__index_multiline_comment+2+c+2):]
                else:
                    __result_line = __result_line[:__index_multiline_comment]
                    __result_multiline_comment = True

            if __boolean_singleline_comment == True:
                __result_line = __result_line[:__index_singleline_comment]
                break
            
            if (__boolean_multiline_comment == False) and (__boolean_singleline_comment == False):
                break

        return (__result_line, __result_multiline_comment)

    def _find_braces(self, string):
        braces = []
        begin = 0
        contents = ''
        br = 0
        for i, c in enumerate(string):
            if c == '{':
                br += 1
                if br == 1:
                    begin = i
                    continue
            if c == '}':
                if br == 1:
                    braces.append((contents, begin, i))
                    contents = ''
                br -= 1
            if br >= 1:
                contents += c
        return braces

    def __split_prods(self, string, group_index=0, expansion=False):
        #print("__split_prods(), string:{}, expansion:{}".format(string, expansion)) 
        def __get_key_string_group(key):
            return 'g%d' % key
        def __get_key_string_par_start(key):
            return 'b%d' % key
        def __get_key_string_par_end(key):
            return 'e%d' % key
        def __get_key_next_group(dic, key):
            __group_index = key + 1
            __group_key = __get_key_string_group(__group_index)
            while __group_key in dic:
                __group_index += 1
                __group_key = __get_key_string_group(__group_index)
            return (__group_index, __group_key)

        def __append(dic, key, val):
            key = __get_key_string_group(key)
            val = val.strip()
            if val == '':
                return
            if key in dic:
                dic[key].append(val)
            else:
                dic[key] = [val]

        prods = {}
        prod = ''
        par_index_tuples = []

        for i, c in enumerate(string):
            if c == '(':
                par_index_tuples.append((i, len(prod)))
            elif c == ')':
                __len_par_index_tuples = len(par_index_tuples)
                if (__len_par_index_tuples > 0):
                    __par_index_string, __par_index_prod = par_index_tuples.pop(-1)
                    if (expansion == True) and (__len_par_index_tuples == 1):
                        __next_group_index, __next_group_key = __get_key_next_group(prods, group_index)
                        #print("__next_group_index:{}, __next_group_key:{}, string:{}".format(__next_group_index, __next_group_key, string[__par_index_string+1:i]))
                        # pass XXXX inside (XXXX) to __split_prods()
                        __sub_prods = self.__split_prods(string[__par_index_string+1:i].strip(), __next_group_index, False)
                        if len(__sub_prods) == 0:
                            prod = prod[:__par_index_prod]
                            continue
                        else:
                            #print(__sub_prods)
                            prods[__get_key_string_par_start(__next_group_index)] = __par_index_string
                            prods[__get_key_string_par_end(__next_group_index)] = i 
                            for __sub_prod in __sub_prods[__next_group_key]:
                                __append(prods, __next_group_index, __sub_prod)
                                #if __next_group_index >= 2:
                                #    print(prod[:__par_index_prod]+__sub_prod)

                                # replace (XXXX)
                                __additional_prods = self.__split_prods( (prod[:__par_index_prod]+__sub_prod+string[i+1:]), group_index, True)
                                for __additional_prod in __additional_prods[__get_key_string_group(group_index)]:
                                    __append(prods, group_index, __additional_prod)
                            return prods

                else:
                    continue

            elif c == '|':
                if len(par_index_tuples) == 0:
                    __append(prods, group_index, prod)
                    prod = ''
                    continue
            else:
                pass
            prod += c
        if len(par_index_tuples) > 0:
            # delete '('
            __par_deletion_count = 0
            for _, __par_index_prod in par_index_tuples:
                __par_index_prod -= __par_deletion_count
                if __par_index_prod + 1 >= len(prod):
                    prod = prod[:__par_index_prod]
                else:
                    prod = prod[:__par_index_prod] + prod[__par_index_prod+1:]
                __par_deletion_count += 1
 
            __additional_prods = self.__split_prods( prod, group_index, expansion)
            for __additional_prod in __additional_prods[__get_key_string_group(group_index)]:
                __append(prods, group_index, __additional_prod)
            return prods

        if prod.strip() != '':
            __append(prods, group_index, prod)
            prods[__get_key_string_par_start(group_index)] = 0
            prods[__get_key_string_par_end(group_index)] = len(string)
        #print("group_index:{}".format(group_index))
        return prods


    # Objects.xml, Locations.xml, Names.xml, Gestures.xml
    def new_load_wildcards(self, resources_dir):
        self.__object_info = ObjectInfo(os.path.join(resources_dir, 'Objects.xml'))
        self.__name_info = NameInfo(os.path.join(resources_dir, 'Names.xml'))
        self.__question_info = QuestionInfo(os.path.join(resources_dir, 'Questions.xml'))
        self.__gesture_info = GestureInfo(os.path.join(resources_dir, 'Gestures.xml'))
        self.__location_info = LocationInfo(os.path.join(resources_dir, 'Locations.xml'))


    # Objects.xml, Locations.xml, Names.xml, Gestures.xml
    def load_wildcards(self, path):
        root = ET.parse(path).getroot()
        rules = {}
        def append(k, v):
            if k in rules:
                rules[k].append(v)
            else:
                self.wildcards.append(k)
                rules[k] = [v]
        for c in root:
            tag = '$'+c.tag
            if tag=='$name':
                name = c.text
                append(tag, name)
                if 'gender' in c.attrib and c.attrib['gender'].lower() == 'male':
                    append('$male', name)
                else:
                    append('$female', name)
                continue
            name = c.attrib['name']
            append(tag, name)
            if tag=='$category':
                append('$object_obfuscated', name)
                append('$kobject_obfuscated', name)
                append('$aobject_obfuscated', name)
            if tag=='$room':
                append('$location_obfuscated', name)
            for cc in c:
                tag = '$'+cc.tag
                name = cc.attrib['name']
                append(tag, name)
                if 'isPlacement' in cc.attrib and cc.attrib['isPlacement']:
                    append('$placement', name)
                if 'isBeacon' in cc.attrib and cc.attrib['isBeacon']:
                    append('$beacon', name)
                if tag == '$object':
                    if 'type' in cc.attrib and cc.attrib['type'] == 'alike':
                        append('$aobject', name)
                    else:
                        append('$kobject', name)
        self.rules.update(rules)

    def missing_rules(self):
        missing = []
        for v in self.rules.values():
            for s in v:
                for t in re.findall('\$[0-9A-Za-z_]+', s):
                    if t not in self.rules:
                        missing.append(t)
        return missing

    __NEWREGEX_TUPLE_INDEX_RULE = 0
    __NEWREGEX_TUPLE_INDEX_EXPANDED_RULE = 1
    __NEWREGEX_TUPLE_INDEX_FULLY_EXPANDED_RULE = 2    
    # expand $XXXX in rule
    def __expand_rule(self, rule, excluded_names):
        __result_rules = []
        __processed_rules = []
        ex = re.compile('\$[0-9A-Za-z_]+')
        
        __processed_rules.append(rule)
        while len(__processed_rules) > 0:
            __target_rule = __processed_rules.pop(0)
            __result_rules_addition = True
            while True:
                m = ex.search(__target_rule)
                if m is None:
                    break
                b, e = m.span()
                #check m.group() is in excluded_names
                __exclusion = False
                if m.group() in excluded_names:
                    __exclusion = True
                replacements = None
                if __exclusion == True:
                    #check rest
                    __rule_first_part = __target_rule[:e] + ' '
                    replacements = self.__expand_rule(__target_rule[e:], excluded_names)
                    for replacement in replacements:
                        __result_rules.append(__rule_first_part + replacement)
                    #print("__result_rules(__exclusion == True): {}".format(__result_rules))
                    __result_rules_addition = False
                    break
                else:
                    replacements = self.__new_regex(m.group(), self.__NEXT_GENERATION, excluded_names)
                    #print('rule:{}, replacements:{}'.format(rule, replacements))
                    if replacements:
                        __updated_target_rule = ''
                        for index, replacement in enumerate(replacements):
                            if index==0:
                                __updated_target_rule = __target_rule[:b].strip() + ' ' + replacement[self.__NEWREGEX_TUPLE_INDEX_EXPANDED_RULE]+ ' ' + __target_rule[e:].strip()
                            else:
                                # add the others to __processed_rules
                                __processed_rules.append(__target_rule[:b].strip() + ' ' + replacement[self.__NEWREGEX_TUPLE_INDEX_EXPANDED_RULE]+ ' ' + __target_rule[e:].strip())
                        __target_rule = __updated_target_rule
                    else:
                        __target_rule = __target_rule[:b].strip() + ' ' + __target_rule[e:].strip()
            if __result_rules_addition is True:
                __result_rules.append( __target_rule.strip() )
        #print("__result_rules: {}".format(__result_rules))
        return __result_rules

    def __combine_string_lists_with_space(self, first_list, second_list):
        if len(first_list)<=0:
            return second_list
        elif len(second_list)<=0:
            return first_list
        else:
            __result_list = []
            for first_list_element in first_list:
                for second_list_element in second_list:
                    __combined_string = first_list_element + ' ' + second_list_element
                    __result_list.append(__combined_string)
            return __result_list

    __NEXT_GENERATION = 1
    def __new_regex(self, name, generations=__NEXT_GENERATION, excluded_names=[]):
        # input value:
        #    name : rule $XXXX you want to know its subrules and their expansion results
        #    generations : how many generations from 'name'
        #    excluded_names : rules you don't want to expand
        # output value:
        #    list of tuples
        #        [(name's subrule, expansion result), (names's subrule, expansion result), ...]
        if name not in self.__new_rules.keys():
            return None
        # search target rules
        __target_rules = [name]
        if generations > 0:
            __generation_count = 0
            while __generation_count < generations:
                __temporary_rules = []
                for target_rule in __target_rules:
                    __next_generation_target_rules = []
                    for target_rule_word in target_rule.split():
                        target_rule_word = target_rule_word.lower().replace(', ', ' ').replace(',', '')
                        #print('__generation_count:{}, target_rule_word:{}'.format(__generation_count, target_rule_word))
                        if target_rule_word in self.__new_rules.keys():
                            __next_generation_target_rules = self.__combine_string_lists_with_space(__next_generation_target_rules, self.__new_rules[target_rule_word][:])
                        else:
                            #print('target_rule_word:{} not in self.__new_rules_keys()'.format(target_rule_word))
                            __next_generation_target_rules = self.__combine_string_lists_with_space(__next_generation_target_rules, [target_rule_word])
                    for next_generation_target_rule in __next_generation_target_rules:
                        next_generation_target_rule = next_generation_target_rule.lower().replace(', ', ' ').replace(',', ' ')
                        #print('next_generation_target_rule:{}'.format(next_generation_target_rule))
                    __temporary_rules.extend(__next_generation_target_rules)
                __target_rules = list(set(__temporary_rules))
                __generation_count +=1
        # expand target rules
        __result_rules = []
        for i, rule in enumerate(__target_rules):
            rule = rule.lower().replace(', ', ' ').replace(',', ' ')
            __expanded_rules = self.__expand_rule(rule, excluded_names)
            #print('name:{}, i:{}, rule:{}, __expanded_rules:{}'.format(name, i, rule, __expanded_rules))
            for expanded_rule in __expanded_rules:
                __result_rules.append((rule, expanded_rule))
                #print('name:{}, i:{}, rule:{}, expanded_rule:{}'.format(name, i, rule, expanded_rule))

        return __result_rules

    __RULE_COMMAND_TUPLE_INDEX_RULE = 0
    __RULE_COMMAND_TUPLE_INDEX_COMMAND = 1
    def get_parse_result_tuple_index_rule(self):
        return self.__NEWREGEX_TUPLE_INDEX_RULE
    def get_parse_result_tuple_index_rule_command_tuples(self):
        return self.__NEWREGEX_TUPLE_INDEX_EXPANDED_RULE
    def get_rule_command_tuple_index_rule(self):
        return self.__RULE_COMMAND_TUPLE_INDEX_RULE
    def get_rule_command_tuple_index_command(self):
        return self.__RULE_COMMAND_TUPLE_INDEX_COMMAND

    def new_parse_command(self, cmd, top='$Main', generations=__NEXT_GENERATION):
        #print("new_parse_command(cmd:{}, top:{}, generations:{}) start".format(cmd, top, generations))
        # input value:
        #     cmd : command you want to parse
        #     top : rule you want to know after you parsed 'cmd'
        #     keys : not used
        #     generations : from 'top', how maany next generations you want to know
        # return value:
        #     GrammarParseResult()
        if cmd is None:
            return None
        top = top.lower()
        cmd = cmd.replace(', ', ' ').replace(',', ' ').lower().strip()
        cmd = re.sub('(\s+)', ' ', cmd)
        #print("cmd after processing:{}".format(cmd))
        __top_rules = self.__new_regex(top, generations, self.__excluded_names)
        __top_rules_words = []
        __removed_words = ['a', 'the']
        for top_rule in __top_rules:
            #print ("top_rule (before):: rule:{}, expanded rule:{}".format(top_rule[self.__NEWREGEX_TUPLE_INDEX_RULE], top_rule[self.__NEWREGEX_TUPLE_INDEX_EXPANDED_RULE]))
            # make  __top_rule words list
            __top_rule_words = []
            __temp_top_rule_words = []
            __temp_top_rule_index = 0
            __temp_top_rule = re.sub('(\s+)', ' ', top_rule[self.__NEWREGEX_TUPLE_INDEX_EXPANDED_RULE])
            __temp_top_rule = __temp_top_rule.strip()
            #print("__temp_top_rule: {}".format(__temp_top_rule))
            for (content, b, e) in self._find_braces(__temp_top_rule):
                if b > 0:
                    # -1 means ' {'
                    for temp_top_rule_word in __temp_top_rule[__temp_top_rule_index:b-1].split(' '):
                        __temp_top_rule_words.append(temp_top_rule_word)
                __temp_top_rule_words.append(__temp_top_rule[b:e+1])
                __temp_top_rule_index = e+2 # +2 means '} '
            if __temp_top_rule_index < len(__temp_top_rule):
                for temp_top_rule_word in __temp_top_rule[__temp_top_rule_index:].split(' '):
                    __temp_top_rule_words.append(temp_top_rule_word)
                    #print("__temp_top_rule_words.append: {}".format(temp_top_rule_word))
            for top_rule_word in __temp_top_rule_words:
                __addition = True
                # remove designated words like 'a', 'the'
                for __removed_word in __removed_words:
                    if top_rule_word == __removed_word:
                        __addition = False
                        break
                if __addition == True:
                    __top_rule_words.append(top_rule_word)
            __top_rules_words.append((top_rule[self.__NEWREGEX_TUPLE_INDEX_RULE], __top_rule_words, __temp_top_rule_words))
            #print ("top_rule (after):: rule:{}, expanded rule:{}, fully expanded rule:{}".format(top_rule[self.__NEWREGEX_TUPLE_INDEX_RULE], __top_rule_words, __temp_top_rule_words))

        # check whether top_rule_words list matches cmd in order
        __parse_results = []
        for top_rule_words in __top_rules_words:
            #print("top_rule_words: {}".format(top_rule_words))
            __rule_command_tuples = []
            __result_rule_addition = True
            __target_string = cmd
            for top_rule_word in top_rule_words[self.__NEWREGEX_TUPLE_INDEX_EXPANDED_RULE]:
                #print("top_rule_word: {}".format(top_rule_word))
                name = ''
                if top_rule_word.startswith('$') == True:
                    __name_tuple_list = self.__new_regex(top_rule_word)
                    if __name_tuple_list is None:
                        raise Exception('__name_tuple_list is None.')
                    else:
                        #print('top_rule_word: {}, __name_tuple_list: {}'.format(top_rule_word, __name_tuple_list))
                        __name_list = []
                        for name_tuple in __name_tuple_list:
                            __name_list.append(name_tuple[self.__NEWREGEX_TUPLE_INDEX_EXPANDED_RULE])
                        name = '(' + '|'.join(__name_list) + ')'
                        #if top_rule_word=='$object':
                        #    print('top_rule_word: {}, name: {}'.format(top_rule_word, name))
                elif top_rule_word.startswith('{') == True:
                    # wildcard
                    __wildcard_parse_result = self.__wildcard_parser.parse(top_rule_word)
                    wc_name = __wildcard_parse_result.get_name()
                    if wc_name in self.WILDCARD_INFO_DIC.keys():
                        #print('top_rule_word: {}, wc_name: {}'.format(top_rule_word, wc_name))
                        __name_list = []
                        if wc_name == self.WILDCARD_QUESTION:
                            __name_list.append(wc_name)
                        else:
                            __wildcard_names = self.WILDCARD_INFO_DIC[wc_name](self, wc_name)
                            for wildcard_name in __wildcard_names:
                                __wildcard_name_words = []
                                __temp_wildcard_name_words = wildcard_name.split()
                                for temp_wildcard_name_word in __temp_wildcard_name_words:
                                    __addition = True
                                    for __removed_word in __removed_words:
                                        if temp_wildcard_name_word == __removed_word:
                                            __addition = False
                                            break
                                    if __addition is True:
                                        __wildcard_name_words.append(temp_wildcard_name_word)
                                if len(__wildcard_name_words) > 1:
                                    __name_list.append('(' + ' '.join(__wildcard_name_words) + ')')
                                elif len(__wildcard_name_words) == 1:
                                    __name_list.append(__wildcard_name_words[0])
                                else:
                                    pass
                        #print('wc_name:{}'.format(wc_name))
                        #for wc_actual_name in __name_list:
                        #    print('wc_actual_name:{}'.format(wc_actual_name))
                        #print('top_rule_word: {}, __name_list: {}'.format(top_rule_word, __name_list))
                        if len(__name_list) > 0:
                            name = '(' + '|'.join(__name_list) + ')'
                        #print('top_rule_word: {}, name: {}'.format(top_rule_word, name))
                    else:
                        #print('top_rule_word:{}, wc_name:{}'.format(top_rule_word, wc_name))
                        raise Exception('invalid wildcard.')
                else:
                    name = top_rule_word
                    #print('name:{}'.format(name))

                m = re.search(name, __target_string)
                if not m:
                    #print('search() failure:: name:{}, __target_string:{}'.format(name, __target_string))
                    __result_rule_addition = False
                    break
                else:
                    _, __e = m.span()
                    __target_string = __target_string[__e:]
                    __target_string = __target_string.strip()
                    __rule_command_tuples.append((top_rule_word, m.group()))
                    #print("__rule_command_tuples: {}".format(__rule_command_tuples))
            if __result_rule_addition == True:
                # currently, use the first matched top_rule_words
                __parse_results.append( GrammarParseResult(top_rule_words[self.__NEWREGEX_TUPLE_INDEX_RULE],\
                                                           __rule_command_tuples,\
                                                           top_rule_words[self.__NEWREGEX_TUPLE_INDEX_FULLY_EXPANDED_RULE],\
                                                           __target_string) )

                #print('top_rule:{}, fully expanded rule:{}'.format(top_rule_words[self.__NEWREGEX_TUPLE_INDEX_RULE],\
                #                                                   top_rule_words[self.__NEWREGEX_TUPLE_INDEX_FULLY_EXPANDED_RULE]))

        # if __parse_result is a part of parse_result, remove it from __parse_results
        __final_parse_results = []
        __length_parse_results = len(__parse_results)
        __index_parse_results = 0
        while __index_parse_results < __length_parse_results:
            __parse_result = __parse_results[__index_parse_results]
            __next_parse_result = False
            for index_parse_result, parse_result in enumerate(__parse_results):
                if index_parse_result != __index_parse_results:
                    if __parse_result.check_rule_command_tuples_inclusion(parse_result.get_rule_command_tuples()) is True:
                        __next_parse_result = True
                        break
            if __next_parse_result is False:
                __final_parse_results.append(__parse_result)
            __index_parse_results += 1

        # use final_parse_result which has the longest rest
        __final_parse_result = None
        for final_parse_result in __final_parse_results:
            if __final_parse_result is None:
                __final_parse_result = final_parse_result
            elif len(__final_parse_result.get_rest()) < len(final_parse_result.get_rest()):
                __final_parse_result = final_parse_result
            else:
                pass
        #print('len(__final_parse_results):{}'.format(len(__final_parse_results)))
        #print("new_parse_command(cmd:{}) end".format(cmd))
        return __final_parse_result


    def regex(self, name, name_dict={}):
        rules = self.__new_rules[name][:]
        ex = re.compile('\$[0-9A-Za-z_]+')
        for i, r in enumerate(rules):
            r = r.lower().replace(', ', ' ').replace(',', ' ')
            while True:
                m = ex.search(r)
                if m is None:
                    break
                b, e = m.span()
                replacement = self.regex(m.group(), name_dict)
                if replacement:
                    r = r[:b].strip() + ' (?:'+replacement+') '+ r[e:].strip()
                else:
                    r = r[:b].strip() + ' ' + r[e:].strip()
            rules[i] = r.strip()
        result = '|'.join(rules)
        if name in name_dict:
            ind = name_dict[name]
            name_dict[name] = ind+1
            result = '(?P<'+name[1:]+'_'+str(ind)+'>'+result+')'
        return result#.replace(' ', '(?:^|$|\s)+?')

    def parse_command(self, cmd, top='$Main', keys=None):
        cmd = cmd.replace(', ', ' ').replace(',', ' ').lower().strip()
        cmd = re.sub('(\s*)', ' ', cmd)
        _cmd_words_list = cmd.split(' ')
        if keys is None:
            if top=='$Main':
                keys = self.rules['$Main']
            else:
                keys = []
        _search_pattern = self.regex(top, {k:0 for k in keys})
        #print ("_search_pattern:{}".format(_search_pattern))
        m = re.search(self.regex(top, {k:0 for k in keys}), cmd)
        if not m:
            print ("m is None.")
            return None

        gd = m.groupdict()
        print gd
        vs = []
        for k,v in gd.items():
            if not v:
                continue
            k = '_'.join(k.split('_')[:-1])
            for m in re.finditer('(^|$|\s)('+self.regex('$'+k)+')(^|$|\s)', cmd):
                b, e = m.span()
                if all(map(lambda x: k!=x[2] or (b!=x[0] and e!=x[1]), vs)):
                    vs.append([b, e, k, cmd[b:e].strip()])
        vs.sort(lambda a,b: a[0]-b[0])
        #print vs

        return [(k, s) for _,_,k,s in vs]

    def to_jsgf(self, category, public_rules=['$Main']):
        __empty_rules = ['$polite', '$gdmeta', '$fllwdest', '$fllwhdst']


        rules_dic = {}
        for name, rules in self.__new_rules.items():
            __valid_rules = []
            # eliminate __empty_rules
            if name in __empty_rules:
                continue
            for rule in rules:
                #print ('rule:{}'.format(rule))
                for empty_rule in __empty_rules:
                    m = re.search('\\' + empty_rule, rule)
                    if m is not None:
                        rule = rule[:m.start()] + rule[m.end():]
                        #print ('name:{}, rule:{}'.format(name, rule))
                if rule.strip() != '':
                    __valid_rules.append(rule)

            # list to string
            __result_string = ''
            __joined_rules_string = ' | '.join(__valid_rules)
            if len(__valid_rules)>1:
                __joined_rules_string = '( '+ __joined_rules_string +' )'
            while True:
                __wildcard_search_result = self.__wildcard_parser.search(__joined_rules_string)
                if __wildcard_search_result.get_group() is None:
                    __result_string += __joined_rules_string
                    break
                else:
                    __result_string += __joined_rules_string[:__wildcard_search_result.get_start()]
                    # expand wildcard
                    __wildcard_parse_result = self.__wildcard_parser.parse(__wildcard_search_result.get_group())
                    wc_name = __wildcard_parse_result.get_name()
                    #print('wc_name:{}, __wildcard_search_result.group():{}'.format(wc_name, __wildcard_search_result.get_group() ))
                    if wc_name in self.WILDCARD_INFO_DIC.keys():
                        __name_list = []
                        if wc_name == self.WILDCARD_QUESTION:
                            __name_list.append(wc_name)
                        else:
                            __name_list = self.WILDCARD_INFO_DIC[wc_name](self, wc_name)

                        __length_name_list = len(__name_list)
                        if __length_name_list > 0:
                            if __length_name_list==1:
                                __result_string = __result_string + ' | '.join(__name_list)
                            else:
                                __result_string = __result_string + '( ' + ' | '.join(__name_list) + ' )'
                    else:
                        raise Exception('invalid wildcard name.')

                    
                    __joined_rules_string = __joined_rules_string[__wildcard_search_result.get_end():]


            rules_dic[name] =  __result_string
            #print('rules_dic= key:{}, value:{}'.format(name, __result_string))


        def replace(s, category):
            # replace $XXX to <XXX> 
            #  <XXX> is JSGF's rule.
            s = s.replace(', ', ' ').replace(',', ' ')
            ss = s.split()
            for i, sss in enumerate(ss):
                if sss[0] == '$':
                    ss[i] = '<category'+str(category)+sss[1:]+'>'
                else:
                    ss[i] = sss.upper()
            return ' '.join(ss)
        # build a grammar body
        #  build a JSGF's rule shown below/
        #   <rulename> = ruleExpansion ;
        #   public <rulename> = ruleExpansion ;
        jsgf = []
        for k, v in rules_dic.items():
            #print('rules_dic= k:{}, v:{}'.format(k, v))
            v = replace(v, category)
            #print('after map() v:{}'.format(v))
            v = v.strip()
            #print('after filter() v:{}'.format(v))
            if not v:
                continue
            if k in public_rules:
                jsgf.append('public ')
            jsgf.append('<category'+str(category)+k[1:].lower()+'> = ')
            jsgf.append(v)
            jsgf.append(';\n')
        #print('jsgf:{}'.format(''.join(jsgf)))
        return jsgf

# Objects.xml
class ObjectInfo:
    def __init__(self, path):
        self.objects = []
        self.categories = []
        self._category = {}
        self._location = {}
        self._room = {}
        self._difficulty = {}
        self._type = {}
        self._diff = {}

        root = ET.parse(path).getroot()
        if root.tag != 'categories':
            raise Exception('The root tag must be <categories>.')
        for c in root:
            if c.tag != 'category':
                continue
            c_name = c.attrib['name'].lower()
            if 'defaultlocation' in c.attrib:
                c_loc = c.attrib['defaultlocation'].lower()
            if 'defaultLocation' in c.attrib:
                c_loc = c.attrib['defaultLocation'].lower()
            c_room = c.attrib['room'].lower()
            self.categories.append(c_name)
            self._location[c_name] = c_loc
            self._room[c_name] = c_room
            for cc in c:
                if cc.tag != 'object':
                    continue
                cc_name = cc.attrib['name'].lower()
                cc_type = cc.attrib['type'].lower() if 'type' in cc.attrib else 'known'
                cc_diff = cc.attrib['difficulty'].lower()
                self.objects.append(cc_name)
                self._category[cc_name] = c_name
                self._type[cc_name] = cc_type
                self._diff[cc_name] = cc_diff

    def category_of(self, obj):
        return self._category[obj]
    def objects_in(self, cat):
        return filter(lambda obj: self._category[obj]==cat, self.objects)

    def location_of(self, obj_or_cat):
        if obj_or_cat in self.categories:
            cat = obj_or_cat
        else:
            cat = self._category[obj_or_cat]
        return self._location[cat]
    def room_of(self, obj_or_cat):
        if obj_or_cat in self.categories:
            cat = obj_or_cat
        else:
            cat = self._category[obj_or_cat]
        return self._room[cat]

    def is_known(self, obj):
        return self._type[obj] == 'known'
    def is_alike(self, obj):
        return self._type[obj] == 'alike'
    def is_special(self, obj):
        return self._type[obj] == 'special'
    def type_of(self, obj):
        return self._type[obj]

    def is_difficult(self, obj):
        return self._diff[obj] == 'high'
    def is_easy(self, obj):
        return self._diff[obj] == 'easy'
    def difficulty_of(self, obj):
        return self._diff[obj]

    def get_room_category(self, room):
        __categories = []
        for __category, __room in self._room.items():
            if __room == room:
                __categories.append(__category)
        return __categories
    def get_categories(self):
        return self.categories
    def get_categories_obfuscated(self):
        return 'objects'

    def get_objects(self):
        return self.objects
    def get_objects_obfuscated(self):
        __objects_obfuscated = set()
        for object in self.objects:
            __objects_obfuscated.add(self._category[object])
        return __objects_obfuscated

    def get_aobjects(self):
        return filter(lambda obj: self.is_alike(obj), self.objects)
    def get_aobjects_obfuscated(self):
        __aobjects_obfuscated = set()
        for aobject in self.aobjects():
            __aobjects_obfuscated.add(self._category[aobject])
        return __aobjects_obfuscated

    def get_kobjects(self):
        return filter(lambda obj: self.is_known(obj), self.objects)
    def get_kobjects_obfuscated(self):
        __kobjects_obfuscated = set()
        for kobject in self.kobjects():
            __kobjects_obfuscated.add(self._category[kobject])
        return __kobjects_obfuscated

    def get_sobjects(self):
        return filter(lambda obj: self.is_special(obj), self.objects)
    def get_sobjects_obfuscated(self):
        __sobjects_obfuscated = set()
        for sobject in self.sobjects():
            __sobjects_obfuscated.add(self._category[sobject])
        return __sobjects_obfuscated

# Names.xml
class NameInfo:
    def __init__(self, path):
        self.names = set()
        self.male_names = set()
        self.female_names = set()

        root = ET.parse(path).getroot()
        if root.tag != 'names':
            raise Exception('The root tag must be <names>.')
        for c in root:
            if c.tag != 'name':
                continue
            name = c.text.strip().lower()
            if 'gender' in c.attrib and c.attrib['gender'].lower()=='male':
                self.male_names.add(name)
            else:
                self.female_names.add(name)
        self.names = self.male_names.union(self.female_names)

    def get_names(self):
        return list(self.names)

    def get_names_obfuscated(self):
        return 'person'

    def get_male_names(self):
        return list(self.male_names)

    def get_female_names(self):
        return list(self.female_names)

# Locations.xml
class LocationInfo:
    def __init__(self, path):
        self.locations = set()
        self.rooms = set()
        self.beacons = set()
        self.placements = set()
        self._loc_to_room = {}

        root = ET.parse(path).getroot()
        if root.tag != 'rooms':
            raise Exception('The root tag must be <rooms>.')
        for c in root:
            if c.tag != 'room':
                continue
            room = c.attrib['name'].lower()
            self.rooms.add(room)
            for cc in c:
                if cc.tag != 'location':
                    continue
                loc = cc.attrib['name'].lower()
                self.locations.add(loc)
                self._loc_to_room[loc] = room
                if 'isBeacon' in cc.attrib and cc.attrib['isBeacon'].lower()=='true':
                    self.beacons.add(loc)
                if 'isPlacement' in cc.attrib and cc.attrib['isPlacement'].lower()=='true':
                    self.placements.add(loc)

    def locations_in(self, room):
        return filter(lambda x: self._loc_to_room[x]==room, self.locations)

    def beacons_in(self, room):
        return filter(lambda x: self._loc_to_room[x]==room, self.beacons)

    def placements_in(self, room):
        return filter(lambda x: self._loc_to_room[x]==room, self.placements)

    def room_of(self, loc):
        return self._loc_to_room[loc]

    def is_placement(self, loc):
        return loc in self.placements

    def is_beacon(self, loc):
        return loc in self.beacon

    def get_rooms(self):
        return list(self.rooms)

    def get_rooms_obfuscated(self):
        return 'apartment'

    def get_locations(self):
        return list(self.locations)

    def get_locations_obfuscated(self):
        return 'somewhere'

    def get_beacons(self):
        return list(self.beacons)

    def get_beacons_obfuscated(self):
        __beacons_obfuscated = []
        for beacon in self.beacons:
            __beacons_obfuscated.add(self._loc_to_room[beacon])
        return __beacons_obfuscated

    def get_placements(self):
        #print('paaaarint: placements')
        return list(self.placements)

    def get_placements_obfuscated(self):
        __placements_obfuscated = []
        for placement in self.placements:
            __placements_obfuscated.add(self._loc_to_room[placement])
        return __placements_obfuscated


# Questions.xml
class QuestionInfo:
    def __init__(self, path):
        self.questions = {}

        root = ET.parse(path).getroot()
        if root.tag != 'questions':
            raise Exception('The root tag must be <questions>.')
        for c in root:
            if c.tag != 'question':
                continue
            q = c.find('q')
            a = c.find('a')
            if q is None or a is None:
                raise Exception('<question> must have <q> and <a>.')
            qtext = q.text
            qtext = qtext.replace('colour', 'color')
            qtext = qtext.replace('?', '')
            self.questions[qtext.lower()] = a.text.lower()

    def get_questions(self):
        return self.questions.keys()
    def get_questions_obfuscated(self):
        return 'question'
    def get_answer(self, question):
        answer = ''
        if question in self.questions.keys():
            answer = self.questions[question]
        return answer

#Gestures.xml
class GestureInfo:
    def __init__(self, path):
        self.gestures = []
        self._difficulty = {}

        root = ET.parse(path).getroot()
        if root.tag != 'gestures':
            raise Exception('The root tag must be <gestures>')
        for c in root:
            if c.tag != 'gesture':
                continue
            name = c.attrib['name'].lower()
            self.gestures.append(name)
            if 'difficulty' in c.attrib:
                self._difficulty[name] = c.attrib['difficulty'].lower()

    def is_difficult(self, obj):
        return self._difficulty[obj] == 'high'
    def is_easy(self, obj):
        return self._difficulty[obj] == 'easy'
    def difficulty_of(self, obj):
        return self._difficulty[obj]
    def get_gestures(self):
        return self.gestures


if __name__=='__main__':
    g = Grammar()
    # This is a test code assuming you are in the Resource directory of GPSRCmdGen:
    # you must run this script in a proper directory or specify valid paths!
    #__resources_dir = '/home/np0020/Applications/GPSRCmdGen/GPSRCmdGen/Resources/'
    __resources_dir = os.path.expanduser('~') + '/Applications/GPSRCmdGen/GPSRCmdGen/Resources/'
    g.load_rules(__resources_dir+'Category1Grammar.txt')
    g.load_rules(__resources_dir+'CommonRules.txt')
    g.new_load_wildcards(__resources_dir)


    #cmd = 'get, yeah, spoon from the sofa and place it on drawer.'
    #cmd = None
    cmd = 'give me the mug from the bedside'
    result = g.new_parse_command(cmd)
    if result is None:
        print 'Could not parse the command.'
    else:
        pass

    '''
    speech = 'Go to the cabinet locate the noodles and put it on the center table'.lower()
    speech = "Find a person in the bathroom and tell your team's affiliation".lower()
    while True:
        print 'Enter the command.'
        cmd = raw_input()
        if not cmd:
            print 'Command was empty. Quitting.'
            break
        result = g.parse_command(cmd)
        if result is None:
            print 'Could not parse the command.'
        else:
            for i, d in enumerate(result):
                v = d.pop('verb')
                text = d.pop('text').strip()
                print i, v, d, '"'+text+'"'
    '''
