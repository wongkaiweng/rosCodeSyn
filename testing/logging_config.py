"""
logging for this package temporarily. PIPE to ros later?
Modified from LTLMoP
"""

import logging
import ConfigParser
import sys, os
import time

LOG_CLEAN = False
OVERALL_LEVEL = "INFO"

if LOG_CLEAN:
    prob_from_files_level = OVERALL_LEVEL
    replacement_with_redbaron_level = OVERALL_LEVEL
    topics_in_file_level = OVERALL_LEVEL
    parameters_in_file_level = OVERALL_LEVEL
    process_limits_level = OVERALL_LEVEL
    process_urdf_level = OVERALL_LEVEL
    code_synthesis_level = OVERALL_LEVEL
    find_active_channels_level = OVERALL_LEVEL
    process_yaml_level = OVERALL_LEVEL
else:
    prob_from_files_level = 'DEBUG'
    replacement_with_redbaron_level = 'DEBUG'
    topics_in_file_level = 'DEBUG'
    parameters_in_file_level = 'DEBUG'
    process_limits_level = 'DEBUG'
    process_urdf_level = 'DEBUG'
    code_synthesis_level = 'DEBUG'
    find_active_channels_level = 'DEBUG'
    process_yaml_level = 'DEBUG'

loggerLevel = {"prob_from_files":prob_from_files_level,
               "replacement_with_redbaron":replacement_with_redbaron_level,
               "topics_in_file":topics_in_file_level,
               "parameters_in_file":parameters_in_file_level,
               "process_limits":process_limits_level,
               "process_urdf":process_urdf_level,
               "code_synthesis":code_synthesis_level,
               "find_active_channels":find_active_channels_level,
               "process_yaml":process_yaml_level}

def setupLogging(loggerLevel=None):
    # Set up loggers for printing error messages
    class ColorLogFormatter(logging.Formatter):
        def __init__(self, *args, **kwds):
            super(ColorLogFormatter, self).__init__(*args, **kwds)
            self.plain_formatter = logging.Formatter("%(levelname)5s[%(filename)s:%(lineno)s] %(message)s", "%H:%M:%S")
            self.debug_formatter = logging.Formatter("%(levelname)5s[%(filename)s:%(lineno)s ] %(message)s", "%H:%M:%S")
            self.detailed_formatter = logging.Formatter("%(levelname)5s[%(filename)s:%(lineno)s] %(message)s", "%H:%M:%S")

        def colorize(self, level, string):
            if sys.platform in ['win32', 'cygwin']:
                # Message with color is not yet supported in Windows
                return string

            else:
                colors = {'ERROR': 91, 'WARNING': 93, 'INFO': 97, 'DEBUG': 94, 'Level 1': 90, 'Level 2': 95, 'Level 4': "7;95", 'Level 6': 96, 'Level 8': 92}
                return "\033[{0}m{1}\033[0m".format(colors[level], string)

        def format(self, record):
            if record.levelname == "INFO":
                precolor = self.plain_formatter.format(record)
            elif record.levelname == "DEBUG":
                precolor = self.debug_formatter.format(record)
            else:
                precolor = self.detailed_formatter.format(record)

            string =  self.colorize(record.levelname, precolor)
            header, footer = string.split(record.message)
            string = string.replace('\n', '\n' + ' '*len(header))
            return string

    loggers = {"prob_from_files": logging.getLogger("probs_logger"), \
               "replacement_with_redbaron": logging.getLogger("replace_logger"),
               "topics_in_file": logging.getLogger("topics_logger"),
               "parameters_in_file": logging.getLogger("parameters_logger"),\
               "process_limits":logging.getLogger("limits_logger"),\
               "process_urdf":logging.getLogger("urdf_logger"),\
               "code_synthesis":logging.getLogger("synthesis_logger"),\
               "find_active_channels":logging.getLogger("channel_logger"),\
               "process_yaml":logging.getLogger("yaml_logger")}

    h = logging.StreamHandler()
    f = ColorLogFormatter()
    h.setFormatter(f)
    for logger in loggers.values():
        if not logger.handlers:
            logger.addHandler(h)

    # also save to file
    # read from terminal: tail -f /var/log/syslog -f /var/tmp/contoller_logger.log
    for logger_name, logger in loggers.iteritems():
        h_file = logging.FileHandler('/var/tmp/'+logger_name+'_logger.log', mode='w')
        h_file.setFormatter(f)
        logger.addHandler(h_file)

    cfg = ConfigParser.ConfigParser()

    for logger_name, logger in loggers.iteritems():
        if loggerLevel[logger_name].lower() == 'error':
            logger.setLevel(logging.ERROR)
        elif loggerLevel[logger_name].lower() == 'warning':
            logger.setLevel(logging.WARNING)
        elif loggerLevel[logger_name].lower() == 'info':
            logger.setLevel(logging.INFO)
        elif loggerLevel[logger_name].lower() == 'debug':
            logger.setLevel(logging.DEBUG)
        elif loggerLevel[logger_name].lower() == 'notset':
            #logger.setLevel(logging.NOTSET)
            # for some reason logging.NOTSET does not work
            logger.setLevel(int(1))
        else:
            logger.setLevel(int(loggerLevel[logger_name]))

# Choose the timer func with maximum accuracy for given platform
if sys.platform in ['win32', 'cygwin']:
    best_timer = time.clock
else:
    best_timer = time.time

# Set-up logging automatically on import
setupLogging(loggerLevel)
