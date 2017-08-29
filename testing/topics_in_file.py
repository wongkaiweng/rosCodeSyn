import ast
from collections import deque


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


def get_topics_in_file(fname, msgType):
    with open(fname) as f:
        a = ast.parse(f.read())
    ret = []
    for node in ast.walk(a):
        if isinstance(node, ast.Call):
            callvisitor = FuncCallVisitor()
            callvisitor.visit(node.func)
            if callvisitor.name == 'rospy.Publisher':
                if len(node.args) > 1 and \
                        isinstance(node.args[1], ast.Name) \
                        and node.args[1].id == 'Twist':
                    ret.append(node.args[0].s)
    return ret
