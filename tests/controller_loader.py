"""Load executable controller algorithms without connecting to Space Teams PRO."""
import ast
from collections import deque
from threading import Lock
import numpy as np


def load_namespace(source):
    tree = ast.parse(source)
    protected = {'custom_exception_handler', 'GetNearestDebrisLocations', 'PanTiltCamera', 'SetCommand'}
    nodes = [node for node in tree.body if
             (isinstance(node, (ast.FunctionDef, ast.ClassDef)) and node.name not in protected)
             or (isinstance(node, ast.Assign) and all(isinstance(t, ast.Name) and t.id.isupper()
                                                     for t in node.targets))]
    namespace = dict(np=np, deque=deque, Lock=Lock)
    exec(compile(ast.Module(body=nodes, type_ignores=[]), '<controller algorithms>', 'exec'), namespace)
    return namespace
