import redbaron
import json
import logging
import os
import ast

import logging_config
replace_logger = logging.getLogger("replace_logger")


def creat_redbaron_obj(filename):
    with open(filename, "r") as f:
        red_obj = redbaron.RedBaron(f.read())
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
            else:
                # throw a warning
                replace_logger.warning('{0} is not found and thus not replaced!'.format(msg_type))


def replace_parameters(red_obj, out_of_bound_list):
    """
    out_of_bound_list = [attribute_list, limit_violation, replace_value, cur_value, var_name]
    """
    replace_logger.debug('out_of_bound_list: {0}'.format(out_of_bound_list))
    var_name = out_of_bound_list[0][4]
    cur_value = out_of_bound_list[0][3]
    replace_value = out_of_bound_list[0][2]

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

                    # replace value
                    assignmentNode.value.value = str(replace_value)
                    replace_logger.info("Replaced value in {0}: {1} with {2}".format(assignmentNode, cur_value, replace_value))

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


            # TODO: what if it's an attribute?

def save_redbardon_obj_to_file(red_obj, dest_file):
    dirname = os.path.dirname(dest_file)
    if not os.path.exists(dirname):
        os.makedirs(dirname)
    with open(dest_file, "w") as f:
        f.write(red_obj.dumps())

if __name__ == "__main__":
    filename = "files/jackal_auto_drive.py"
    dest_file = "files/code.py"

    #!! note the special string formating!
    target_topic_dict = {"Twist": '"/cmd_vel_short"', \
                         "geometry_msgs.msg.Twist": '"/cmd_vel_long"'}
    channel_type = "Publisher"
    #channel_type = "Subscriber"

    # create object
    red_obj = creat_redbaron_obj(filename)

    # operations
    replace_topic(red_obj, channel_type, target_topic_dict)

    out_of_bound_list = [(['linear', 'x'], 'upper', 0.2, 0.3, 'vel')]
    replace_parameters(red_obj, out_of_bound_list)

    # save to file
    save_redbardon_obj_to_file(red_obj,dest_file)