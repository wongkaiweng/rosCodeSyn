import ast
from collections import deque
import traceback
import logging
import sys

import logging_config
test_logger = logging.getLogger("test_logger")


if not sys.platform == 'darwin': # not Mac OS
    import rosmsg
    LINUX = True
else:
    LINUX = False

class FuncCallVisitor(ast.NodeVisitor):
    def __init__(self):
        self._name = deque()

    @property
    def name(self):
        return '.'.join(self._name)

    @property
    def simplename(self):
        return self._simplename


    @property
    def shortname(self):
        return self._name[-1]

    @name.deleter
    def name(self):
        self._name.clear()

    def visit_Name(self, node):
        self._name.appendleft(node.id)

    def visit_Attribute(self, node):
        try:
            self._name.appendleft(node.attr)
            self._name.appendleft(node.value.id)
        except AttributeError:
            self.generic_visit(node)


class Scope():
    def __init__(self, parent):
        self.variables = {}
        self.parent = parent

    def find(self, name):
        if name in self.variables:
            return self.variables[name]
        elif self.parent:
            return self.parent.find(name)
        else:
            return None

    def add(self, name, value):
        if name in self.variables:
            self.variables[name].append(value)
        else:
            self.variables[name] = [value]

    def __str__(self):
        return str(self.variables)


class ROSCodeVisitor(ast.NodeVisitor):
    def __init__(self):
        self.ret = {}
        self.scopes = deque()
        self.scopes.append(Scope(None))
        self.names_to_import_module = {}

    def visit_Assign(self, node):
        if len(node.targets) == 1 and isinstance(node.targets[0], ast.Name) \
                and isinstance(node.value, ast.Str):
            self.scopes[0].add(node.targets[0].id,node.value.s)
        self.generic_visit(node)

    def visit_FunctionDef(self,node):
        self.scopes.appendleft(Scope(self.scopes[0]))
        self.generic_visit(node)
        self.scopes.popleft()

    # def visit_If(self,node):
    #     self.scopes.appendleft(Scope(self.scopes[0]))
    #     for n in node.body:
    #         self.visit(n)
    #     self.scopes.popleft()
    #     if hasattr(node,'orelse'):
    #         self.scopes.appendleft(Scope(self.scopes[0]))
    #         for n in node.orelse:
    #             self.visit(n)
    #         self.scopes.popleft()

    def visit_Call(self, node):
        SHORT_FLAG = False
        LONG_FLAG = False
        callvisitor = FuncCallVisitor()
        callvisitor.visit(node.func)
        if callvisitor.name == 'rospy.Publisher':

            if len(node.args) > 1 and (isinstance(node.args[1], ast.Name) or isinstance(node.args[1], ast.Attribute)):
                if isinstance(node.args[1], ast.Name): # short name
                    msgType = node.args[1].id

                    # create also msg long form
                    if msgType in self.names_to_import_module.keys():
                        SHORT_FLAG = True
                        long_msgType = self.names_to_import_module[msgType]+'.'+msgType
                        if long_msgType not in self.ret:
                            self.ret[long_msgType] = []

                elif isinstance(node.args[1], ast.Attribute): #long name
                    attrvisitor = FuncCallVisitor()
                    attrvisitor.visit(node.args[1])
                    msgType = attrvisitor.name

                    # also create msg short form
                    short_msgType = attrvisitor.shortname
                    LONG_FLAG = True
                    if short_msgType not in self.ret:
                        self.ret[short_msgType] = []

                # add new msgType
                if msgType not in self.ret: self.ret[msgType] = []

                if isinstance(node.args[0], ast.Str):
                    self.ret[msgType].append(node.args[0].s) #original
                    if SHORT_FLAG: self.ret[long_msgType].append(node.args[0].s) #short to long
                    if LONG_FLAG: self.ret[short_msgType].append(node.args[0].s) #long to short
                elif isinstance(node.args[0],ast.Name):
                    if self.scopes[0].find(node.args[0].id):
                        self.ret[msgType].extend(self.scopes[0].find(node.args[0].id)) #original
                        if SHORT_FLAG: self.ret[long_msgType].extend(self.scopes[0].find(node.args[0].id)) #short to long
                        if LONG_FLAG: self.ret[short_msgType].extend(self.scopes[0].find(node.args[0].id)) #long to short
                elif isinstance(node.args[0],ast.Add):
                    pass


    """
    def visit_Import(self, node):
        print node, node.names
        for x in node.names:
            print x.name
        self.generic_visit(node)
    """

    def visit_ImportFrom(self, node):
        # node.module, node.names, node.level

        #first check if it's a ROS import
        if "_msgs" in node.module or "_srv" in node.module:

            for x in node.names:

                # code just import all, need to check
                if "*" == x.name:
                    # TODO:test here
                    if LINUX:
                        msg_name_list = rosmsg.list_types(node.module.split('.')[0])
                        for msg_name in msg_name_list:
                            # save mapping of name to module
                            self.names_to_import_module[msg_name] = node.module

                        #TODO: does it work with srv msg?

                elif x.name in self.names_to_import_module.keys():
                    # if it's different from before
                    if self.names_to_import_module[x.name] != node.module:
                        test_logger.warning("This msg_type {0} currently belongs to module {1}. But we are changing it to module {2}".format(\
                            x.name, self.names_to_import_module[x.name], node.module))

                        # save mapping of name to module
                        self.names_to_import_module[x.name] = node.module
                else:
                    # save mapping of name to module
                    self.names_to_import_module[x.name] = node.module

        self.generic_visit(node)



def get_topics_in_file(fname):
    ret = []
    try:
        with open(fname) as f:
            a = ast.parse(f.read())
        rv = ROSCodeVisitor()
        rv.visit(a)
        ret = rv.ret
    except (IndentationError,SyntaxError) as e:
        # traceback.print_exc()
        print 'Compilation error: {0}'.format(fname)
    except Exception as e:
        traceback.print_exc()
        print 'Directory: {0}'.format(fname)
    return ret


if __name__ == "__main__":
    file = "files/jackal_auto_drive.py"
    #with open(file) as f:
    #    # with open("topics_in_file.py") as f:
    #    a = ast.parse(f.read())
    #rv = ROSCodeVisitor()
    #rv.visit(a)
    #print(rv.scopes[0])
    #print(rv.ret)
    #print(rv.names_to_import_module)
    print("topics in file:{0}".format(get_topics_in_file(file)))

