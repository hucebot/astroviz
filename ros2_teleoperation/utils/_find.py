import os

def _find_src_config():
    cwd = os.getcwd()
    while True:
        candidate = os.path.join(cwd, 'src', 'ros2_teleoperation', 'config')
        if os.path.isdir(candidate):
            return candidate
        parent = os.path.dirname(cwd)
        if parent == cwd:
            break
        cwd = parent
    return None

def _find_pkg():
    cwd = os.getcwd()
    while True:
        candidate = os.path.join(cwd, 'src', 'ros2_teleoperation')
        if os.path.isdir(candidate):
            return candidate
        parent = os.path.dirname(cwd)
        if parent == cwd:
            break
        cwd = parent
    return None
