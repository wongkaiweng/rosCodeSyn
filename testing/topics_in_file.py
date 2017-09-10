import ast
from collections import deque
import traceback

class FuncCallVisitor(ast.NodeVisitor):
    def __init__(self):
        self._name = deque()

    @property
    def name(self):
        return '.'.join(self._name)

    @property
    def simplename(self):
        return self._simplename

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
        self.ret = list()
        self.scopes = deque()
        self.scopes.append(Scope(None))

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
        callvisitor = FuncCallVisitor()
        callvisitor.visit(node.func)
        if callvisitor.name == 'rospy.Publisher':
            if len(node.args) > 1 and \
                    isinstance(node.args[1], ast.Name):  # \
                # and node.args[1].id in msgTypeList:
                if isinstance(node.args[0], ast.Str):
                    self.ret.append(node.args[0].s)
                elif isinstance(node.args[0],ast.Name):
                    if self.scopes[0].find(node.args[0].id):
                        self.ret.extend(self.scopes[0].find(node.args[0].id))
                elif isinstance(node.args[0],ast.Add):
                    pass


# with open("test/jackal_auto_drive.py") as f:
#     # with open("topics_in_file.py") as f:
#     a = ast.parse(f.read())
# rv = ROSCodeVisitor()
# rv.visit(a)
# print(rv.scopes[0])
# print(rv.ret)


def get_topics_in_file(fname, msgTypeList):
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
