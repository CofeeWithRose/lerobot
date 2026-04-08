"""
Bridge module to safely import functions from move.py without triggering
its top-level execution code (robot init, pick(), disconnect(), etc.).

We achieve this by loading the source, parsing the AST, and only executing
function/class/import definitions — skipping bare expressions and assignments
at the module level that would trigger hardware operations.
"""

import ast
import sys
import types
import os

# Path to the original move.py
_MOVE_PY_PATH = os.path.join(os.path.dirname(__file__), "..", "move.py")


def _load_move_functions():
    """
    Parse move.py's AST, keep only safe top-level nodes
    (imports, function defs, class defs), compile and exec them
    into a synthetic module namespace.
    """
    with open(_MOVE_PY_PATH, "r", encoding="utf-8") as f:
        source = f.read()

    tree = ast.parse(source, filename=_MOVE_PY_PATH)

    # Keep only safe node types at the top level
    safe_types = (ast.Import, ast.ImportFrom, ast.FunctionDef,
                  ast.AsyncFunctionDef, ast.ClassDef)
    safe_body = [node for node in tree.body if isinstance(node, safe_types)]
    tree.body = safe_body
    ast.fix_missing_locations(tree)

    code = compile(tree, filename=_MOVE_PY_PATH, mode="exec")

    # Create a synthetic module to exec into
    mod = types.ModuleType("move_safe")
    mod.__file__ = _MOVE_PY_PATH
    # Pre-populate with builtins so imports work
    mod.__builtins__ = __builtins__

    # Ensure the parent directory of move.py is on sys.path so its imports resolve
    move_dir = os.path.dirname(os.path.abspath(_MOVE_PY_PATH))
    if move_dir not in sys.path:
        sys.path.insert(0, move_dir)

    exec(code, mod.__dict__)
    return mod


# Load once at import time
_move = _load_move_functions()

# Re-export the functions we need
initRobot = _move.initRobot
getAllMotorMoveRange = _move.getAllMotorMoveRange
getRobotState = _move.getRobotState
setRobotState = _move.setRobotState
send_action_timed = _move.send_action_timed
send_action_timed_with_normalized = _move.send_action_timed_with_normalized
stand_up = _move.stand_up


def disconnectRobot(robot):
    """Safely disconnect the robot."""
    try:
        robot.disconnect()
    except Exception as e:
        print(f"Warning: error during disconnect: {e}")
