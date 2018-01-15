import redbaron
import json
import logging
import os
import ast
import copy


import logging_config
replace_logger = logging.getLogger("replace_logger")

# see if we can do active check with rostopic
try:
    import rostopic
    import rosgraph
    rosgraph.Master('/rostopic').getPid()
    ROSTOPIC_IMPORT=True
except:
    ROSTOPIC_IMPORT=False
    replace_logger.warning("No active ROS Master Found. We will not search for topics/actions if needed.")

import find_active_channels

def creat_redbaron_obj(filename):
    with open(filename, "r") as f:
        red_obj = redbaron.RedBaron(f.read())
    f.closed
    return red_obj


def replace_topic(red_obj, channel_type, target_topic_dict):
    replace_logger.log(8, '----------------------------------------')
    replace_logger.log(8, 'Channel_type:{0}'.format(channel_type))
    replace_logger.log(8, 'Target_topic_dict:{0}'.format(target_topic_dict))
    replace_logger.info('========================================')

    # finding all calls (e.g publishers/subscribers)
    for atomtrailersNode in red_obj.find_all("AtomtrailersNode"):

        # check if it contains channel_type (only direct children)
        if atomtrailersNode.value.find("NameNode", value=channel_type, recursive=False):

            callNode = atomtrailersNode.value.find("CallNode", recursive=False)
            # TODO: may have a problem if topic is not an obj?

            ###################
            ## find msg_type ##
            ###################
            # AtomtrailersNode -> CallNode -> CallArgumentNode -> NameNode (msgType) or other Nodes
            # short library name (e.g: Twist)
            if isinstance(callNode.value[1].value, redbaron.NameNode):
                msg_type = callNode.value[1].value.value
            # full library name (e.g: geometry_msgs.msg.Twist)
            elif isinstance(callNode.value[1].value, redbaron.AtomtrailersNode):
                msg_type = ".".join([x.value for x in callNode.value[1].value.find_all("NameNode")])
            else:
                # message type is not a Name Node or AtomtrailersNode
                replace_logger.warning('We found {0} but the msg_type is {1}'.format(channel_type, type(callNode.value[1].value)))
                return

            ######################
            # replace topic name #
            ######################
            if ("Bool" in msg_type) and \
               ((channel_type == 'Publisher' and 'node_publish_topic' in str(callNode.value[0].value)) or \
                (channel_type == 'Subscriber' and 'node_subscribe_topic' in str(callNode.value[0].value))):

                # ignore node_subscribe_topic and node_publish_topic
                replace_logger.info("NOT replacing {0}: {1}".format(channel_type, callNode.value[0].value))

            elif msg_type in target_topic_dict.keys():
                replace_logger.info("Replaced {0}: {1} with {2}".format(channel_type, callNode.value[0].value,\
                                                                target_topic_dict[msg_type]))
                # replace topic name with new Name Node
                callNode.value[0].value = target_topic_dict[msg_type]

            elif ROSTOPIC_IMPORT:
                # scan current action channels (topics, actions) for replacement
                topic_name = find_active_channels.find_channel_name(callNode.value[0].value, channel_type, msg_type)
                if topic_name:
                    callNode.value[0].value = topic_name
                    replace_logger.info("Replaced {0}: {1} with {2}".format(channel_type, callNode.value[0].value,\
                                                topic_name))
                else:
                    # throw a warning
                    replace_logger.warning('{0} is not found and thus not replaced!'.format(msg_type))

            else:
                # throw a warning
                replace_logger.warning('{0} is not found and thus not replaced!'.format(msg_type))

def get_value_from_node(ast_value_node):
     # get value from node
    if isinstance(ast_value_node, redbaron.nodes.IntNode) or \
       isinstance(ast_value_node, redbaron.nodes.FloatNode):
        ast_value = ast.literal_eval(ast_value_node.value)

    elif isinstance(ast_value_node, redbaron.nodes.UnitaryOperatorNode):
        # get the full value 'operator' + 'value'
        ast_value = ast.literal_eval(ast_value_node.value + ast_value_node.target.value)

    else:
        replace_logger.warning("We don't know how to get evaluate this value: {0}".format(ast_value_node)) #.help(deep=True)
        ast_value = None

    return ast_value

def recursive_find_call_names(red_obj, out_of_bound_list):
    """
    This function recursively find the cur_value to replace based on call_name_list
    out_of_bound_list = [attribute_list, limit_violation, replace_value, cur_value, var_name, call_name_list]
    call_name_list example: ['Twist', [0, 'Vector3'], 0]
    code example: self.move(Twist(Vector3(0.5, 0, 0), Vector3(0, 0 ,0)))

    """
    replace_logger.log(8, 'out_of_bound_list: {0}'.format(out_of_bound_list))
    var_name = out_of_bound_list[0][4]
    cur_value = out_of_bound_list[0][3]
    replace_value = out_of_bound_list[0][2]
    call_name_list = out_of_bound_list[0][5]

    for atomtrailersNode in red_obj.find_all("AtomtrailersNode"):
        # AtomtrailersNode -> NameNode
        #                  -> CallNode -> CallArgumentNode (target, value)-> AtomtrailersNode
        if atomtrailersNode.value[0].find("NameNode", value=call_name_list[0], recursive=False):

            # need to keep going down the node
            if len(call_name_list) > 2:
                # manipulate call_name_list
                new_out_of_bound_list = copy.deepcopy(out_of_bound_list)
                callArg_idx = call_name_list[1][0]
                new_out_of_bound_list[0][5][1] = new_out_of_bound_list[0][5][1][1]
                new_out_of_bound_list[0][5].pop(0) # remove first element

                # go to the next level
                if len(atomtrailersNode.value) >= 2 and len(atomtrailersNode.value[1].value) >= callArg_idx+1:
                    recursive_find_call_names(atomtrailersNode.value[1].value[callArg_idx],\
                        new_out_of_bound_list)

            else:
                # process and replace value
                callArg_idx = call_name_list[1]
                ast_value_node = atomtrailersNode.value[1].value[callArg_idx].value

                # get value from node
                ast_value = get_value_from_node(ast_value_node)

                # atomtrailersNode.value[1] = CallNode
                # atomtrailersNode.value[1].value = CallArgumentNode
                # atomtrailersNode.value[1].value[callArg_idx].value = IntNode
                if ast_value == cur_value:
                    replace_logger.info("Replaced value in {0}: {1} with {2}".format(atomtrailersNode, cur_value, replace_value))
                    atomtrailersNode.value[1].value[callArg_idx].value = str(replace_value)
                else:
                    replace_logger.debug("DID NOT replace value in {0}: {1} with {2} in field {3}".format(\
                        atomtrailersNode, cur_value, replace_value, call_name_list))


def replace_parameters(red_obj, out_of_bound_list):
    """
    out_of_bound_list = [(attribute_list, limit_violation, replace_value, cur_value, var_name, call_name_list)]
    """
    attribute_list = out_of_bound_list[0][0]
    var_name = out_of_bound_list[0][4]
    cur_value = out_of_bound_list[0][3]
    replace_value = out_of_bound_list[0][2]
    call_name_list = out_of_bound_list[0][5]

    # mainly in instantiation
    if isinstance(var_name, float) or isinstance(var_name, int):
        # checking replacement with calllist
        if call_name_list is not None:
            recursive_find_call_names(red_obj, out_of_bound_list)

    elif isinstance(var_name, str):
        replace_logger.debug('out_of_bound_list: {0}'.format(out_of_bound_list))
        # finding all calls (e.g publishers/subscribers)
        for assignmentNode in red_obj.find_all("AssignmentNode"):

            # check if it contains channel_type (only direct children)
            if assignmentNode.target.find("NameNode", value=var_name, recursive=False):

                # int and float replacement
                if isinstance(assignmentNode.value, redbaron.nodes.IntNode) or \
                   isinstance(assignmentNode.value, redbaron.nodes.FloatNode) or \
                   isinstance(assignmentNode.value, redbaron.nodes.StringNode):

                    if ast.literal_eval(assignmentNode.value.value) == cur_value:
                        replace_logger.debug("value match: {0}, var_name: {1}, cur_value: {2}".format(assignmentNode, var_name, cur_value))
                        replace_logger.info("Replaced value in {0}: {1} with {2}".format(assignmentNode, cur_value, replace_value))
                        # replace value
                        assignmentNode.value.value = str(replace_value)

                # list replacement
                elif isinstance(assignmentNode.value, redbaron.nodes.ListNode):
                    valid_list = True
                    redbaron_list = []

                    # first check the list is all floats or ints
                    for red_obj in assignmentNode.value.value:
                        if isinstance(red_obj, redbaron.nodes.IntNode) or \
                            isinstance(red_obj, redbaron.nodes.FloatNode) or \
                            isinstance(red_obj, redbaron.nodes.StringNode):
                            redbaron_list.append(ast.literal_eval(red_obj.value))

                        elif isinstance(red_obj, redbaron.nodes.UnitaryOperatorNode):
                            # get the full value 'operator' + 'value'
                            redbaron_list.append(ast.literal_eval(red_obj.value + red_obj.target.value))

                        else:
                            valid_list = False
                            break

                    # check if it's the same as cur_value
                    if valid_list and redbaron_list:
                        replace_logger.debug('Same:{0}: redbaron_list:{1}, cur_value:{2}.'.format(\
                            redbaron_list == cur_value, redbaron_list, cur_value))

                        # check if the assignment is the same
                        if redbaron_list == cur_value:
                            # replace value
                            assignmentNode.value.replace(redbaron.RedBaron(str(replace_value))[0])
                            replace_logger.info("Replaced value in {0}: {1} with {2}".format(assignmentNode, cur_value, replace_value))


            # deal with a.b.c = value
            elif isinstance(assignmentNode.target, redbaron.nodes.AtomtrailersNode):
                # if variable matches
                if var_name+'.'+'.'.join(attribute_list) == assignmentNode.target.dumps():
                    # value matches
                    ast_value = get_value_from_node(assignmentNode.value)

                    if ast_value == cur_value:
                        assignmentNode.value = str(replace_value)
                        replace_logger.info("Replaced value in {0}: {1} with {2}".format(assignmentNode, cur_value, replace_value))
                    else:
                        replace_logger.debug("DID NOT replace value in {0}: {1} with {2} with var_name {3}".format(\
                            assignmentNode, cur_value, replace_value, var_name))

    else:
        replace_logger.warning("Not Replaced: {0}".format(out_of_bound_list))


def replace_moveit_variable(red_obj, info_dict):
    # structure - dict keys:
    # func_type, obj, orig_group_name, target_group_name, func_idx
    idx = info_dict['func_idx']

    replace_logger.debug('info_dict: {0}'.format(info_dict))
    # finding all calls (e.g publishers/subscribers)
    for atomtrailersNode in red_obj.find_all("AtomtrailersNode"):

        # check if it contains channel_type (only direct children)
        if atomtrailersNode.value.find("NameNode", value=info_dict['func_type'], recursive=False):

            callNode = atomtrailersNode.value.find("CallNode", recursive=False)
            # AtomtrailersNode -> CallNode -> CallArgumentNode -> NameNode (msgType) or other Nodes

            #######################
            # replace moveit name #
            #######################
            # str replacement
            if not info_dict['var_name'] and isinstance(callNode.value[idx].value, redbaron.StringNode) and \
            callNode.value[idx].value.value == info_dict['orig_name']:
                replace_logger.info("Replaced {0}: {1} with {2}".format(info_dict['func_type'], callNode.value[idx].value,\
                                                                info_dict['target_name']))
                # replace topic name with new Name Node
                callNode.value[idx].value = info_dict['target_name']


            # variable replacement
            elif info_dict['var_name'] and isinstance(callNode.value[idx].value, redbaron.NameNode) and \
            callNode.value[idx].value.value == info_dict['var_name']:

                # finding all calls (e.g publishers/subscribers)
                for assignmentNode in red_obj.find_all("AssignmentNode"):

                    # find variables
                    nameNode = assignmentNode.target.find("NameNode", value=info_dict['var_name'], recursive=False)
                    if nameNode and isinstance(assignmentNode.value, redbaron.StringNode) and\
                    assignmentNode.value.value .replace("'",'"') == info_dict['orig_name']:

                        # replace value
                        assignmentNode.value.value = info_dict['target_name']
                        replace_logger.info("Replaced value in {0}: {1} with {2}".format(assignmentNode, \
                                                        info_dict['orig_name'], info_dict['target_name']))
            else:
                # throw a warning
                replace_logger.warning('{0} is not found and thus not replaced!'.format(callNode.value[idx].value))


def replace_moveit_moveToPose(red_obj, pose_cmd_frame_dict, frame_replacement_dict, scopes):
    for group_obj, orig_name in pose_cmd_frame_dict.iteritems():
        # it's a variable
        if scopes[0].find(orig_name):
            for value in scopes[0].find(orig_name):
                replace_moveit_variable(red_obj,
                    {'func_type': 'moveToPose', 'obj': group_obj, 'orig_name': '"'+value+'"', \
                     'target_name': '"'+frame_replacement_dict[value]+'"',\
                     'func_idx':1, 'var_name': orig_name})
        else:
            replace_moveit_variable(red_obj,
                {'func_type': 'moveToPose', 'obj': group_obj, 'orig_name': '"'+orig_name+'"', \
                 'target_name': '"'+frame_replacement_dict[orig_name]+'"',\
                 'func_idx':1, 'var_name': None})

def replace_moveit_interface(red_obj, interface_dict, group_replacement_dict, scopes):
    for group_obj, orig_name in interface_dict.iteritems():
        # it's a variable
        if scopes[0].find(orig_name):
            for value in scopes[0].find(orig_name):
                replace_moveit_variable(red_obj,
                    {'func_type': 'MoveGroupInterface', 'obj': group_obj, 'orig_name': '"'+value+'"', \
                     'target_name': '"'+group_replacement_dict[value]+'"',\
                     'func_idx':0, 'var_name': orig_name})
        else:
            replace_moveit_variable(red_obj,
                {'func_type': 'MoveGroupInterface', 'obj': group_obj, 'orig_name': '"'+orig_name+'"', \
                 'target_name': '"'+group_replacement_dict[orig_name]+'"',\
                 'func_idx':0, 'var_name': None})


def save_redbardon_obj_to_file(red_obj, dest_file):
    dirname = os.path.dirname(dest_file)
    if not os.path.exists(dirname):
        os.makedirs(dirname)
    with open(dest_file, "w") as f:
        f.write(red_obj.dumps())

if __name__ == "__main__":
    filename_list = []
    out_of_bound_list_list =[]

    #out_of_bound_list = [(attribute_list, limit_violation, replace_value, cur_value, var_name, call_name_list)]
    # FORMAT: Twist(Vector3(0,0,0),Vector3(0,0,0)) _GOOD
    filename_list.append(os.path.dirname(os.path.abspath(__file__))+'/../examples/jackal_to_turtlebot (controller)/jackal_controller.py')
    out_of_bound_list_list.append([(['linear', 'x'], '--', 0.2, 0.5, 0.5, ['Twist', [0, 'Vector3'], 0])])

    # FORMAT: variables, e.g: twist.linear.x = vel
    filename_list.append(os.path.dirname(os.path.abspath(__file__))+"/testfiles/wander.py")
    out_of_bound_list_list.append([(['linear', 'x'], 'upper', 0.2, 0.3, 'vel', None)])

    # FORMAT: variables, e.g: twist.linear.x = 0.4
    filename_list.append(os.path.dirname(os.path.abspath(__file__))+"/testfiles/wander.py")
    out_of_bound_list_list.append([(['linear', 'x'], 'upper', 0.2, 0.4, 'twist', None)])

    filename_list.append(os.path.dirname(os.path.abspath(__file__))+"/../examples/fetch_to_pr2 (wave)/wave_fetch.py")
    out_of_bound_list_list.append({'func_type': 'MoveGroupInterface', \
                                    'obj':'move_group', 'orig_name': '"arm_with_torso"', 'target_name': '"arms_with_torso"',\
                                    'func_idx':0, 'var_name': None})


    filename_list.append(os.path.dirname(os.path.abspath(__file__))+"/../examples/fetch_to_pr2 (wave)/wave_fetch.py")
    out_of_bound_list_list.append({'func_type': 'moveToPose',\
                                    'obj':'move_group', 'orig_name': '"wrist_roll_link"', 'target_name': '"l_wrist_roll_link"',\
                                    'func_idx':1, 'var_name': 'gripper_frame'})


    dest_file = os.path.dirname(os.path.abspath(__file__))+"/testfiles/code.py"

    #!! note the special string formating!
    target_topic_dict = {"Twist": '"/cmd_vel_short"', \
                         "geometry_msgs.msg.Twist": '"/cmd_vel_long"'}
    channel_type = "Publisher"
    #channel_type = "Subscriber"

    for idx, item in enumerate(out_of_bound_list_list):

        # create object
        red_obj = creat_redbaron_obj(filename_list[idx])

        # operations
        replace_topic(red_obj, channel_type, target_topic_dict)

        if isinstance(item, list):
            replace_parameters(red_obj, item)

        if isinstance(item, dict):
            replace_moveit_variable(red_obj, item)

        # save to file
        save_redbardon_obj_to_file(red_obj,dest_file)