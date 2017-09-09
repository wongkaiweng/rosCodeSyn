import ast
import traceback
import codegen
import logging

import topics_in_file
import logging_config
test_logger = logging.getLogger("test_logger")

def dict_insert(cur, list, value):
    if len(list) == 1:
        if list[0] in cur.keys():
            cur[list[0]].append(value)
        else:
            cur[list[0]] = [value]
        return
    if not cur.has_key(list[0]):
        cur[list[0]] = {}
    dict_insert(cur[list[0]], list[1:], value)


def attribute_in_var_names_list(ast_attribute_obj, var_names_list):
    attributevisitor = topics_in_file.FuncCallVisitor()
    attributevisitor.visit(ast_attribute_obj)
    test_logger.log(2, "Attribute name: {0}".format(attributevisitor.name))
    # check if part of attribute is a msgType Object
    if any([x in attributevisitor.name for x in var_names_list]):
        test_logger.log(2, "Save Parameter: TRUE in var_names_list:{0}".format(var_names_list))
        attribute_name_list = attributevisitor.name.replace(x,'').split()
        return True, attribute_name_list
    else:
        test_logger.log(2, "Save Parameter: FALSE in var_names_list:{0}".format(var_names_list))
        return False, []


def get_parameters_in_file(fname, msgTypeList, msg_fields = {}):
    var_names_list = []

    try:
        with open(fname) as f:
            a = ast.parse(f.read())
        for node in ast.walk(a):
            if isinstance(node, ast.Assign):

                # FIXIT: skips Subscript objects, cannot use codegen
                #if isinstance(node.targets[0], ast.Subscript):
                #    continue
                test_logger.log(2, "------")
                test_logger.log(2, "node targets:{0}, node value: {1}".format(node.targets, node.value))
                test_logger.log(2, codegen.to_source(node))


                # check if it is initializing a msgType object
                if isinstance(node.value, ast.Call):
                    callvisitor = topics_in_file.FuncCallVisitor()
                    callvisitor.visit(node.value.func)
                    if callvisitor.name in msgTypeList:
                        # store name to look for in the tree
                        namevisitor = topics_in_file.FuncCallVisitor()
                        namevisitor.visit(node.targets[0])
                        var_names_list.append(namevisitor.name)

                    #TODO: some does assignemnt when initialing - do that too

                # assign target is an attribute
                elif isinstance(node.targets[0], ast.Attribute):
                    # check if first part of attribute is a msgType Object
                    result, attribute_list = attribute_in_var_names_list(node.targets[0], var_names_list)
                    if result:
                        # get all fields and save to dict
                        dict_insert(msg_fields, attribute_list, node.value.n)

                # assign target is a tuple
                elif isinstance(node.targets[0], ast.Tuple):
                    for idx, item in enumerate(node.targets[0].elts):
                        if isinstance(item, ast.Attribute):
                            # check if first part of attribute is a msgType Object
                            result, attribute_list = attribute_in_var_names_list(item, var_names_list)
                            if result:
                                # get all fields and save to dict
                                dict_insert(msg_fields, attribute_list, node.value.elts[idx].n)

                # TODO: what if the value is a variable

    except Exception as e:
        traceback.print_exc()
        print 'Directory: {0}'.format(fname)

    test_logger.debug("var_names_list: {0}".format(var_names_list))
    test_logger.debug("msg_fields: {0}".format(msg_fields))
    return msg_fields