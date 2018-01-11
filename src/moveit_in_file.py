import ast
import getpass
import logging
from collections import deque

import topics_in_file

import logging_config
moveit_logger = logging.getLogger("moveit_logger")


def process_node_arg(target_dict, dict_key, node, arg_idx, scopes):
    if isinstance(node.args[arg_idx], ast.Str):
        target_dict[dict_key] = node.args[arg_idx].s #original

    elif isinstance(node.args[arg_idx],ast.Name):
        if scopes[0].find(node.args[arg_idx].id):
            target_dict[dict_key] = node.args[arg_idx].id #original
        else:
            moveit_logger.warning('Cannot find variable:{0} in scopes!'.format(node.args[arg_idx].id))

    elif isinstance(node.args[arg_idx],ast.Attribute):
        attr_value_visitor = topics_in_file.FuncCallVisitor()
        attr_value_visitor.visit(node.args[arg_idx])
        attr_name = attr_value_visitor.name
        if scopes[0].find(attr_name):
            target_dict[dict_key] = scopes[0].find(attr_name) #original

    elif isinstance(node.args[0],ast.Add):
        pass



class ROSMoveItVisitor(ast.NodeVisitor):
    def __init__(self, msg_fields_dict={}):
        """
        interface_list: list of msgTypes. e.g: ['Twist','geometry_msgs.msg.Twist']
        msg_fields_dict: feed in an existing dictionary of fields. e.g: for Twist msgs:
                         {'linear': {'y': [0], 'x': [0], 'z': [0]}, 'angular': {'y': [0], 'x': [0], 'z': [-20]}}
        """
        self.scopes = deque()
        self.scopes.append(topics_in_file.Scope(None))
        self.interface_list = ['moveit_python.MoveGroupInterface','MoveGroupInterface']
        self.move_to_pose_cmd = 'moveToPose'
        self.interface_dict = {}  # Interface Obj Name: group used
        self.pose_cmd_frame_dict = {} # Interface Obj Name: frame
        self.msg_fields_dict = msg_fields_dict

    def visit_Call(self, node):
        callvisitor = topics_in_file.FuncCallVisitor()
        callvisitor.visit(node.func)

        # moveToPose
        if self.move_to_pose_cmd in callvisitor.name:
            # save value to self.pose_cmd_frame_dict
            process_node_arg(self.pose_cmd_frame_dict, callvisitor.name.split('.')[0], node, 1, self.scopes)

        self.generic_visit(node)

    def visit_Assign(self, node):
       # FIXIT: skips Subscript objects, cannot use codegen
        #if isinstance(node.targets[0], ast.Subscript):
        #    continue
        #moveit_logger.log(2, codegen.to_source(node))
        moveit_logger.log(2, "------")
        moveit_logger.log(2, "node targets:{0}, node value: {1}".format(node.targets, node.value))

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

        # save variables 'a = "hello" to scope (Name -> str)
        elif len(node.targets) == 1 and isinstance(node.targets[0], ast.Name) \
                and isinstance(node.value, ast.Str):
            self.scopes[0].add(node.targets[0].id, node.value.s)

        # check if it is MoveGroupInterface Instantiation
        elif isinstance(node.value, ast.Call):

            callvisitor = topics_in_file.FuncCallVisitor()
            callvisitor.visit(node.value.func)

            if callvisitor.name in self.interface_list:
                # store name to look for in the tree
                namevisitor = topics_in_file.FuncCallVisitor()
                namevisitor.visit(node.targets[0])

                # save value to self.interface_dict
                process_node_arg(self.interface_dict, namevisitor.name, node.value, 0, self.scopes)


        # assign target is an attribute (Attribute -> ???)
        elif isinstance(node.targets[0], ast.Attribute):
            moveit_logger.debug('NEEDS TO BE DONE: assign target is a Attribute. node target[0]: {0}'.format(node.targets[0]))


        # assign target is a tuple (Tuple -> ???)
        elif isinstance(node.targets[0], ast.Tuple):
            moveit_logger.debug('NEEDS TO BE DONE: assign target is a Tuple. node target[0]: {0}'.format(node.targets[0]))

        # assign target is a list
        elif isinstance(node.value, ast.ListComp):
            moveit_logger.debug('NEEDS TO BE DONE: assign target is a listComp. node target[0]: {0}'.format(node.targets[0]))

        else:
            if len(node.targets) == 1:
                moveit_logger.debug("This assignment is not handled: target: {0}, value: {1}".format(\
                                            type(node.targets[0]), type(node.value)))
            else:
                 moveit_logger.debug("This assignment is not handled: target: {0}, value: {1}".format(\
                                            type(node.targets), type(node.value)))


if __name__ == "__main__":
    with open('/home/{0}/ros_examples/Examples/fetch_to_pr2 (wave)/wave_fetch.py'.format(getpass.getuser())) as f:
        a = ast.parse(f.read())


    rv = ROSMoveItVisitor()
    rv.visit(a)
    print("scopes: All variables: {0}".format(rv.scopes[0]))
    print("Parameters of interest: {0}".format(rv.msg_fields_dict))
    print("MoveGroupInterface Instantiated: {0}".format(rv.interface_dict))
    print("moveToPose frames by objs: {0}".format(rv.pose_cmd_frame_dict))

