import ast

def get_all_imports(filename):
    modules = set()

    def visit_Import(node):
        for name in node.names:
            modules.add(name.name.split(".")[0])

    def visit_ImportFrom(node):
        # if node.module is missing it's a "from . import ..." statement
        # if level > 0 it's a "from .submodule import ..." statement
        if node.module is not None and node.level == 0:
            modules.add(node.module.split(".")[0])

    node_iter = ast.NodeVisitor()
    node_iter.visit_Import = visit_Import
    node_iter.visit_ImportFrom = visit_ImportFrom

    with open(filename) as f:
        node_iter.visit(ast.parse(f.read()))
    
    return modules