#!/usr/bin/env python
# -*- coding: utf-8 -*


import re

class WildcardSearchResult():
    def __init__(self, group, start, end):
        self.__group = group
        self.__start = start
        self.__end = end

    def get_group(self):
        return self.__group
    def get_start(self):
        return self.__start
    def get_end(self):
        return self.__end


class WildcardParseResult():
    def __init__(self, name, obfuscated, id, meta):
        self.__name = name
        self.__obfuscated = obfuscated
        self.__id = id
        self.__meta = meta
        
    def get_name(self):
        return self.__name
    def get_obfuscated(self):
        return self.__obfuscated
    def get_id(self):
        return self.__id
    def get_meta(self):
        return self.__meta


class WildcardParser:
    def __init__(self):
        pass

    def search(self, target_string):
        __brace_count = 0
        __start = None
        __end = None
        for index, char in enumerate(target_string):
            if char=='{':
                if __brace_count==0:
                    __start = index
                __brace_count += 1
            elif char=='}':
                __brace_count -= 1
                if __brace_count < 0:
                    return WildcardSearchResult(None, __start, __end)
                elif __brace_count == 0:
                    __end = index + 1
                    return WildcardSearchResult(target_string[__start:__end], __start, __end)
                else:
                    pass
            else:
                pass
        return WildcardSearchResult(None, __start, __end)


    def parse(self, rule_word):
        _ex = re.compile('{(?P<name>[0-9A-Za-z_]+)(?P<obfuscated>[\?]?)(\s+(?P<id>[0-9]+))?(\s+meta:\s*(?P<meta>.+))?}')
        _match_result = _ex.match(rule_word)
        _obfuscated = False
        if _match_result is None:
            return WildcardParseResult('', _obfuscated, '', '')
        else:
            '''
            print('_match_result:: name:{}, obfuscated:{}, id:{}, meta:{}'.format(_match_result.group('name'),\
                                                                                           _match_result.group('obfuscated'),\
                                                                                           _match_result.group('id'),\
                                                                                           _match_result.group('meta')))
            '''
            if _match_result.group('obfuscated') == '?':
                #print('rule_word:{}'.format(rule_word))
                _obfuscated = True
        return WildcardParseResult(_match_result.group('name'), _obfuscated, _match_result.group('id'), _match_result.group('meta'))


# Test code
if __name__=='__main__':    
    #__test_wildcard = '{void meta: The person being guided must deviate when indicated by the referee before reaching the {beacon 2}'
    __test_wildcard = '{void 3 meta:testtesttest }'
    __test_wildcard2 =  '{question}'

    #__test_string = 'aaaa {' + __test_wildcard + ' bbbb ' + __test_wildcard2
    #__test_string = '{test}'
    #__test_string = '{}'
    __test_string = '( {void meta: Follow {name 1} to the {room 2}} test test {question} )'

    __wildcard_parser = WildcardParser()
    __search_result = __wildcard_parser.search(__test_string)
    if __search_result.get_group() is not None:
        print('__search_result:{}, start_pos:{}, end_pos:{}'.format(__search_result.get_group(), __search_result.get_start(), __search_result.get_end()))

    __parse_result = __wildcard_parser.parse(__search_result.get_group())
    __name = __parse_result.get_name()
    __obfuscated = __parse_result.get_obfuscated()
    __id = __parse_result.get_id()
    __meta = __parse_result.get_meta()

    print('parse_result:: name:{}, obfuscated:{}, id:{}, meta:{}'.format(__name,\
                                                                         __obfuscated,\
                                                                         __id,\
                                                                         __meta))


