import ast
import os
import traceback
import codegen
import logging
from collections import deque
import traceback
import getpass

import topics_in_file
import logging_config
parameters_logger = logging.getLogger("parameters_logger")


def retrieve_parameters(file_path, msg_type, msg_fields_dict={}):
    """
    Retrieve parameters based on a msg type
    file_path: path to python files (not recursive)
    msg_type: type of msg
    """
    for root, dirs, files in os.walk(file_path):
        # traverse all directories
        for directory in dirs:
            msg_fields = retrieve_parameters(os.path.join(root, directory), msg_type, msg_fields_dict)

        # find all files
        for file in files:
            if file.endswith(".py"):
                #find parameters with msg_type
                #msg_fields = parameters_in_file.get_parameters_in_file(os.path.join(root, file), msg_type, {})
                msg_fields = get_parameters_in_file(os.path.join(root, file), msg_type, msg_fields_dict)
                #test_logger.debug("File: {0}, fields: {1}".format(file, msg_fields))

    # clean up to remove sublists
    update_dict_with_flattened_list(msg_fields_dict)

    return msg_fields_dict

def update_dict_with_flattened_list(target_dict):
    """
    update target_dict such that there are no sublists
    """
    for key in target_dict.keys():
        if not isinstance(target_dict[key], list):
            update_dict_with_flattened_list(target_dict[key])
        else:
            temp_list = []
            for item in target_dict[key]:
                if not isinstance(item, list):
                    temp_list.append(item)
                else:
                    temp_list.extend(item)

            target_dict[key] = temp_list

def dict_insert(cur, cur_list, value):
    if len(cur_list) == 1:
        if cur_list[0] in cur.keys():
            if value not in cur[cur_list[0]]: # don't want any duplicates
                cur[cur_list[0]].append(value)
        else:
            cur[cur_list[0]] = [value]
        return
    if not cur.has_key(cur_list[0]):
        cur[cur_list[0]] = {}
    dict_insert(cur[cur_list[0]], cur_list[1:], value)

def dict_get_values_from_key_list(cur, list):
    """
    get values from a dict and a list of subsequent keys
    """
    if len(list) == 1:
        if list[0] in cur.keys():
            return cur[list[0]]
        else:
            parameters_logger.warning("Key: {0} not found in dict {1}".format(cur, list[0]))
    value = dict_get_values_from_key_list(cur[list[0]], list[1:])
    return value


def attribute_in_var_names_list(ast_attribute_obj, var_names_list):
    """
    This function checks if part of a given attribute is in the var_names_list.
    It splits the attribute and returns a list if that's the case.
    """
    attributevisitor = topics_in_file.FuncCallVisitor()
    attributevisitor.visit(ast_attribute_obj)
    parameters_logger.log(2, "Attribute name: {0}".format(attributevisitor.name))
    # check if part of attribute is a msgType Object
    if any([x in attributevisitor.name for x in var_names_list]):
        parameters_logger.log(2, "Save Parameter: TRUE in var_names_list:{0}".format(var_names_list))
        attribute_name_list = attributevisitor.name.replace(x+'.','').split('.')
        return True, attribute_name_list
    else:
        parameters_logger.log(2, "Save Parameter: FALSE in var_names_list:{0}".format(var_names_list))
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

    parameters_logger.log(4, "var_names_list: {0}".format(rv.var_names_list))
    parameters_logger.log(4, "msg_fields_dict: {0}".format(rv.msg_fields_dict))
    return rv.msg_fields_dict


def check_and_save_attribute_and_value_to_dict(ast_attribute_obj, ast_value_obj, var_names_list, msg_fields_dict, \
                                     scope=None, attribute_list=[], limits_dict={}, call_func_list=[], call_func_hist=None):
    """
    This function updates msg_fields_dict if attribute(ast_attribute_obj)
    matches variables in var_names_list.
    The value of the assignment comes from ast_value_obj or scope
    ast_attribute_obj: object on the left side of the assignment
    ast_value_obj: object on the right side of the assignment
    var_names_list: variables that are associated with the msg type of interest
    msg_fields_dict: dictionary storing all possible values for the msg type of interest
    scope: previously saved variables
    attribute_list: a.b to ['a','b'], can be feed into the function to bypass 'attribute_in_var_names_list'
    limit_dict: a dictionary of value obj limits
    call_func_list: valid func call that we will save parameters e.g.:['JointTrajectoryPoint']
    call_func_hist: history of related function call to the variable:
                       e.g.: Twist(Vector3(-,1,-),....) -> ['Twist', [0, 'Vector3'], 1]
                       for parameter replacement later
    """
    out_of_bound_list = []
    result = False
    if attribute_list:
        result = True

    if not result:
        result, attribute_list = attribute_in_var_names_list(ast_attribute_obj, var_names_list)
        parameters_logger.debug('result:{0}, attribute_list:{1}'.format(result, attribute_list)) # TODO: maybe we want the full list instead? including the name?

    if result:
        # get all fields and save to dict
        if isinstance(ast_value_obj, ast.Num):
            dict_insert(msg_fields_dict, attribute_list, ast_value_obj.n)
            # check limits
            if limits_dict:
                out_of_bound_tuple = check_limits_wrapper(attribute_list, ast_value_obj.n, limits_dict)
                if out_of_bound_tuple: out_of_bound_list.append(out_of_bound_tuple + (ast_value_obj.n, call_func_hist))

        elif isinstance(ast_value_obj, ast.Name) and scope:
            if scope.find(ast_value_obj.id):
                dict_insert(msg_fields_dict, attribute_list, ast_value_obj.id)
                #dict_insert(msg_fields_dict, attribute_list, scope.find(ast_value_obj.id))
                # check limits
                if limits_dict:
                    for value in scope.find(ast_value_obj.id):
                        out_of_bound_tuple = check_limits_wrapper(attribute_list, value, limits_dict)
                        if out_of_bound_tuple: out_of_bound_list.append(out_of_bound_tuple + (ast_value_obj.id, call_func_hist))

        elif isinstance(ast_value_obj, ast.List) and scope:
            parameters_logger.debug('List elts: {0}, ctx: {1}'.format(ast_value_obj.elts, ast_value_obj.ctx))
            for ast_elts_obj in ast_value_obj.elts:

                #recursive for different fields
                check_and_save_attribute_and_value_to_dict(None, ast_elts_obj, var_names_list, msg_fields_dict, \
                                     scope=scope, attribute_list=attribute_list, limits_dict=limits_dict,
                                     call_func_list=call_func_list, call_func_hist=call_func_hist)

        elif isinstance(ast_value_obj, ast.Call) and scope:

            callvisitor = topics_in_file.FuncCallVisitor()
            callvisitor.visit(ast_value_obj.func)

            # check if the function cal is valid for consideration
            if not call_func_list or callvisitor.name in call_func_list:

                for ast_args_obj in ast_value_obj.args:
                    pass

                # only dealing with keywords now
                for ast_keywords_obj in ast_value_obj.keywords:
                    check_and_save_attribute_and_value_to_dict(None, ast_keywords_obj.value, var_names_list, msg_fields_dict, \
                                         scope=scope, attribute_list=attribute_list + [ast_keywords_obj.arg], limits_dict=limits_dict,\
                                         call_func_list=call_func_list, call_func_hist=call_func_hist)

                parameters_logger.debug("Call Func name:{0}, Args:{1}, Keywords:{2}".format(\
                                          callvisitor.name, ast_value_obj.args, ast_value_obj.keywords))

        else:
            if not scope:
                parameters_logger.warning('scope is None!')
            else:
                parameters_logger.warning('not sure what to do with this: {0}'.format(type(ast_value_obj)))

    return out_of_bound_list

def check_limits_wrapper(attribute_list, value, limits_dict):
    """
    Check if value is within the limits given by limits_dict and the keys from attribute_list
    The limits dict follows the following format:
    limits_dict = {'??':{'?':{'lower': -10, 'upper':10}}}
    Return over-limit or below-limit tuple
    """
    limits = dict_get_values_from_key_list(limits_dict, attribute_list)
    lower_limit, upper_limit = limits['lower'], limits['upper']

    if limits['lower'] != 'None' and lower_limit > value:
        parameters_logger.warning("{0} value: {1} is below the lower limit: {2}".format('.'.join(attribute_list), value, lower_limit))
        return (attribute_list, 'lower', lower_limit, value)

    if limits['upper'] != 'None' and upper_limit < value:
        parameters_logger.warning("{0} value: {1} is above the upper limit: {2}".format('.'.join(attribute_list), value, upper_limit))
        return (attribute_list, 'upper', upper_limit, value)

    if limits['lower'] != 'None' and limits['upper'] != 'None':
        parameters_logger.info("{0} value: {1} is within the lower limit: {2} and the upper limit: {3}".format(\
                                '.'.join(attribute_list), value, lower_limit, upper_limit))
    return None



class ROSParameterVisitor(ast.NodeVisitor):
    def __init__(self, msgTypeList, limits_dict={}, msg_fields_dict={}):
        """
        msgTypeList: list of msgTypes. e.g: ['Twist','geometry_msgs.msg.Twist']
        limits_dict: a dictionary of limits
        msg_fields_dict: feed in an existing dictionary of fields. e.g: for Twist msgs:
                         {'linear': {'y': [0], 'x': [0], 'z': [0]}, 'angular': {'y': [0], 'x': [0], 'z': [-20]}}
        """
        self.scopes = deque()
        self.scopes.append(topics_in_file.Scope(None))
        self.msgTypeList = msgTypeList
        self.var_names_list = []
        self.msg_fields_dict = msg_fields_dict
        self.limits_dict = limits_dict
        self.out_of_bound_list = [] # from checking of limits_dict
        self.possible_subcall_func = []
        self.function_def = {}

        if any(['JointTrajectory' in x for x in self.msgTypeList]):
            self.possible_subcall_func_list = ['trajectory_msgs.msg.JointTrajectory', 'JointTrajectory', \
                                               'trajectory_msgs.msg.JointTrajectoryPoint','JointTrajectoryPoint']
        else:
            self.possible_subcall_func_list = []

    def visit_FunctionDef(self,node):
        parameters_logger.log(4, "Function definition: {0}, args:{1}, decorator_list:{2}".format(\
                                 node.name, node.args.args, node.decorator_list))

        self.function_def[node.name]=[]
        for arg in node.args.args:
            self.function_def[node.name].append(arg.id)

        # run Another ROSParameterVisitor in this?!

        self.generic_visit(node)

    def visit_Call(self, node):
        callvisitor = topics_in_file.FuncCallVisitor()
        callvisitor.visit(node.func)

        if callvisitor.name in self.msgTypeList:
            ####################
            ### Instantiation ##
            ####################
            # Twist msg
            if 'Twist' in callvisitor.name and len(node.args) == 2:
                self.save_vel_instantiation(node.args)


        self.generic_visit(node)

    def visit_Assign(self, node):
       # FIXIT: skips Subscript objects, cannot use codegen
        #if isinstance(node.targets[0], ast.Subscript):
        #    continue
        #parameters_logger.log(2, codegen.to_source(node))
        parameters_logger.log(2, "------")
        parameters_logger.log(2, "node targets:{0}, node value: {1}".format(node.targets, node.value))

        # save variables 'a = 1' to scope (Name -> Num)
        if len(node.targets) == 1 and isinstance(node.targets[0], ast.Name) \
                and isinstance(node.value, ast.Num):
            self.scopes[0].add(node.targets[0].id, node.value.n)

        # save variables 'a = [1,2,3]' to scope (Name -> List)
        elif len(node.targets) == 1 and isinstance(node.targets[0], ast.Name) \
                and isinstance(node.value, ast.List)\
                and (isinstance(node.value.ctx, ast.Load) or isinstance(node.value.ctx, ast.Store)):
            listVisitor = topics_in_file.ListVisitor(self.scopes[0])
            listVisitor.visit(node.value)
            self.scopes[0].add(node.targets[0].id, listVisitor._name)


        # check if it is initializing a msgType object (new Func Obj with msg_type)
        elif isinstance(node.value, ast.Call):
            callvisitor = topics_in_file.FuncCallVisitor()
            callvisitor.visit(node.value.func)
            if callvisitor.name in self.msgTypeList:
                # store name to look for in the tree
                namevisitor = topics_in_file.FuncCallVisitor()
                namevisitor.visit(node.targets[0])
                if namevisitor.name not in self.var_names_list:
                    self.var_names_list.append(namevisitor.name)
                    parameters_logger.debug(node.value.args)

                ####################
                ### Instantiation ##
                ####################
                # Twist msg
                if 'Twist' in callvisitor.name and len(node.value.args) == 2:
                    self.save_vel_instantiation(node.value.args)


        # assign target is an attribute (Attribute -> ???)
        elif isinstance(node.targets[0], ast.Attribute):
            # check if first part of attribute is a msgType Object
            self.out_of_bound_list.extend(check_and_save_attribute_and_value_to_dict(node.targets[0], node.value, self.var_names_list, \
                                                self.msg_fields_dict, self.scopes[0], limits_dict=self.limits_dict,\
                                                call_func_list=self.possible_subcall_func_list))

        # assign target is a tuple (Tuple -> ???)
        elif isinstance(node.targets[0], ast.Tuple):
            for idx, item in enumerate(node.targets[0].elts):
                if isinstance(item, ast.Attribute):
                    # check if first part of attribute is a msgType Object
                    self.out_of_bound_list.extend(check_and_save_attribute_and_value_to_dict(item, node.value.elts[idx], self.var_names_list, \
                                                        self.msg_fields_dict, self.scopes[0],limits_dict=self.limits_dict,\
                                                        call_func_list=self.possible_subcall_func_list))


        # assign target is a list
        elif isinstance(node.value, ast.ListComp):
            parameters_logger.debug('NEEDS TO BE DONE: assign target is a listComp. node target[0]: {0}'.format(node.targets[0]))

        else:
            if len(node.targets) == 1:
                parameters_logger.debug("This assignment is not handled: target: {0}, value: {1}".format(\
                                            type(node.targets[0]), type(node.value)))
            else:
                 parameters_logger.debug("This assignment is not handled: target: {0}, value: {1}".format(\
                                            type(node.targets), type(node.value)))


        # TODO: what if the value is an equation
        self.generic_visit(node)

    def save_vel_instantiation(self, node_args_list):
        attribute_field = ['x','y','z']

        # for twist msg
        if isinstance(node_args_list[0], ast.Call):  # instantiate with Vector3
            callvisitor_twist_linear = topics_in_file.FuncCallVisitor()
            callvisitor_twist_linear.visit(node_args_list[0].func)
            for idx, ast_value_obj in enumerate(node_args_list[0].args):
                # lienar x, y, z
                self.out_of_bound_list.extend(check_and_save_attribute_and_value_to_dict(None, ast_value_obj, self.var_names_list, self.msg_fields_dict, \
                    scope=self.scopes[0], attribute_list=['linear', attribute_field[idx]], limits_dict=self.limits_dict,\
                    call_func_list=self.possible_subcall_func_list, call_func_hist=['Twist', [0, 'Vector3'], idx]))

                # TODO: get line number and replace later?


        if isinstance(node_args_list[1], ast.Call):  # instantiate with Vector3
            callvisitor_twist_angular = topics_in_file.FuncCallVisitor()
            callvisitor_twist_angular.visit(node_args_list[1].func)
            for idx, ast_value_obj in enumerate(node_args_list[1].args):
                # angular x, y, z
               self.out_of_bound_list.extend(check_and_save_attribute_and_value_to_dict(None, ast_value_obj, self.var_names_list, self.msg_fields_dict, \
                    scope=self.scopes[0], attribute_list=['angular', attribute_field[idx]], limits_dict=self.limits_dict,\
                    call_func_list=self.possible_subcall_func_list, call_func_hist=['Twist', [1, 'Vector3'], idx]))



if __name__ == "__main__":
    with open("../turtlebot/processed/wander.py") as f:
        a = ast.parse(f.read())

    vel_limits = {'linear':{'x':{'lower': -0.2, 'upper':0.2}, \
                            'y':{'lower': -0.2, 'upper':0.2}, \
                            'z':{'lower': -0.2, 'upper':0.2}},\
                  'angular':{'x':{'lower': -0.2, 'upper':0.2}, \
                             'y':{'lower': -0.2, 'upper':0.2}, \
                             'z':{'lower': -0.2, 'upper':0.2}}}
    rv = ROSParameterVisitor(['geometry_msgs.msg.Twist','Twist'], limits_dict=vel_limits)
    rv.visit(a)
    print("All variables: {0}".format(rv.scopes[0]))
    print("Parameters of interest: {0}".format(rv.msg_fields_dict))
    print("Message type: {0}".format(rv.var_names_list))
    print("Out of bound list: {0}".format(rv.out_of_bound_list))



    # for arm
    #with open("files/move_robot_ur5.py") as f:
    #with open("../Examples/jaco_to_ur5 (move_robot)/move_robot_jaco.py") as f:
    with open("../Examples/ur5_to_jaco (test_move)/test_move_jaco.py") as f:
        a = ast.parse(f.read())
    rv = ROSParameterVisitor(['trajectory_msgs.msg.JointTrajectory','JointTrajectory'],\
                              limits_dict={}, msg_fields_dict={})
    rv.visit(a)
    print("All variables: {0}".format(rv.scopes[0]))
    print("Parameters of interest: {0}".format(rv.msg_fields_dict))
    print("Message type: {0}".format(rv.var_names_list))
    print("Out of bound list: {0}".format(rv.out_of_bound_list))
    print("Function Definition: {0}".format(rv.function_def))
