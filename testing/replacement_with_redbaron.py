import redbaron
import json
import logging
import os

import logging_config
replace_logger = logging.getLogger("replace_logger")


def creat_redbaron_obj(filename):
    with open(filename, "r") as f:
        red_obj = redbaron.RedBaron(f.read())
    return red_obj


def replace_topic(red_obj, channel_type, target_topic_dict):
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
            if msg_type in target_topic_dict.keys():
                replace_logger.info("Replaced {0}:{1} with {2}".format(channel_type, callNode.value[0].value,\
                                                                target_topic_dict[msg_type]))
                # replace topic name with new Name Node
                callNode.value[0].value = target_topic_dict[msg_type]
            else:
                # throw a warning
                replace_logger.warning('{0} is not found and thus not replaced!\nTarget_topic_dict:{1}'.format(msg_type, target_topic_dict))


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

    # save to file
    save_redbardon_obj_to_file(red_obj,dest_file)