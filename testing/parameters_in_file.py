import ast
import traceback
import codegen
import logging
from collections import deque
import traceback

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
    """
    This function checks if part of a given attribute is in the var_names_list.
    It splits the attribute and returns a list if that's the case.
    """
    attributevisitor = topics_in_file.FuncCallVisitor()
    attributevisitor.visit(ast_attribute_obj)
    test_logger.log(2, "Attribute name: {0}".format(attributevisitor.name))
    # check if part of attribute is a msgType Object
    if any([x in attributevisitor.name for x in var_names_list]):
        test_logger.log(2, "Save Parameter: TRUE in var_names_list:{0}".format(var_names_list))
        attribute_name_list = attributevisitor.name.replace(x+'.','').split('.')
        return True, attribute_name_list
    else:
        test_logger.log(2, "Save Parameter: FALSE in var_names_list:{0}".format(var_names_list))
        return False, []


def get_parameters_in_file(fname, msgTypeList, msg_fields_dict={}):
    var_names_list = []

    try:
        with open(fname) as f:
            a = ast.parse(f.read())
        rv = ROSParameterVisitor(msgTypeList, msg_fields_dict)
        rv.visit(a)

    except Exception as e:
        traceback.print_exc()
        print 'Directory: {0}\n-----------'.format(fname)

    test_logger.log(4, "var_names_list: {0}".format(rv.var_names_list))
    test_logger.log(4, "msg_fields_dict: {0}".format(rv.msg_fields_dict))
    return rv.msg_fields_dict


def save_attribute_and_value_to_dict(ast_attribute_obj, ast_value_obj, var_names_list, msg_fields_dict, scope=None):
    """
    This function updates msg_fields_dict if attribute(ast_attribute_obj)
    matches variables in var_names_list.
    The value of the assignment comes from ast_value_obj or scope
    ast_attribute_obj: object on the left side of the assignment
    ast_value_obj: object on the right side of the assignment
    var_names_list: variables that are associated with the msg type of interest
    msg_fields_dict: dictionary storing all possible values for the msg type of interest
    scope: previously saved variables
    """
    result, attribute_list = attribute_in_var_names_list(ast_attribute_obj, var_names_list)
    if result:
        # get all fields and save to dict
        if isinstance(ast_value_obj, ast.Num):
            dict_insert(msg_fields_dict, attribute_list, ast_value_obj.n)
        elif isinstance(ast_value_obj, ast.Name) and scope:
            if scope.find(ast_value_obj.id):
                dict_insert(msg_fields_dict, attribute_list, scope.find(ast_value_obj.id))
        else:
            if not scope:
                test_logger.warning('scope is None!')
            else:
                test_logger.warning('not sure what to do with this: {0}'.format(type(ast_value_obj)))


class ROSParameterVisitor(ast.NodeVisitor):
    def __init__(self, msgTypeList, msg_fields_dict={}):
        self.scopes = deque()
        self.scopes.append(topics_in_file.Scope(None))
        self.msgTypeList = msgTypeList
        self.var_names_list = []
        self.msg_fields_dict = msg_fields_dict

    def visit_Assign(self, node):
        # save variables
        if len(node.targets) == 1 and isinstance(node.targets[0], ast.Name) \
                and isinstance(node.value, ast.Num):
            self.scopes[0].add(node.targets[0].id, node.value.n)


        # FIXIT: skips Subscript objects, cannot use codegen
        #if isinstance(node.targets[0], ast.Subscript):
        #    continue
        #test_logger.log(2, codegen.to_source(node))
        test_logger.log(2, "------")
        test_logger.log(2, "node targets:{0}, node value: {1}".format(node.targets, node.value))

        # check if it is initializing a msgType object
        if isinstance(node.value, ast.Call):
            callvisitor = topics_in_file.FuncCallVisitor()
            callvisitor.visit(node.value.func)
            if callvisitor.name in self.msgTypeList:
                # store name to look for in the tree
                namevisitor = topics_in_file.FuncCallVisitor()
                namevisitor.visit(node.targets[0])
                self.var_names_list.append(namevisitor.name)

            #TODO: some does assignemnt when initialing - do that too

        # assign target is an attribute
        elif isinstance(node.targets[0], ast.Attribute):
            # check if first part of attribute is a msgType Object
            save_attribute_and_value_to_dict(node.targets[0], node.value, self.var_names_list, self.msg_fields_dict, self.scopes[0])

        # assign target is a tuple
        elif isinstance(node.targets[0], ast.Tuple):
            for idx, item in enumerate(node.targets[0].elts):
                if isinstance(item, ast.Attribute):
                    # check if first part of attribute is a msgType Object
                    save_attribute_and_value_to_dict(item, node.value.elts[idx], self.var_names_list, self.msg_fields_dict, self.scopes[0])

        # TODO: what if the value is an equation

        self.generic_visit(node)


if __name__ == "__main__":
    with open("../turtlebot/processed/wander.py") as f:
        a = ast.parse(f.read())
    rv = ROSParameterVisitor(['geometry_msgs.msg.Twist','Twist'])
    rv.visit(a)
    print("All variables: {0}".format(rv.scopes[0]))
    print("Parameters of interest: {0}".format(rv.msg_fields_dict))
    print("Message type: {0}".format(rv.var_names_list))