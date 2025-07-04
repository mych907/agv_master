import importlib
import sys
import types
import numpy as np
from pathlib import Path

def import_library_agv():
    modules = [
        'rospy', 'rospy.numpy_msg',
        'nav_msgs', 'nav_msgs.msg',
        'std_msgs', 'std_msgs.msg',
        'geometry_msgs', 'geometry_msgs.msg',
        'marvelmind_nav', 'marvelmind_nav.msg',
        'sensor_msgs', 'sensor_msgs.msg',
        'visualization_msgs', 'visualization_msgs.msg',
        'darknet_ros_msgs', 'darknet_ros_msgs.msg',
        'update_markers', 'path_planning', 'path_tracking', 'initializer',
    ]
    repo_root = Path(__file__).resolve().parents[1]
    if str(repo_root) not in sys.path:
        sys.path.insert(0, str(repo_root))

    for name in modules:
        if name not in sys.modules:
            module = types.ModuleType(name)
            if name == 'rospy.numpy_msg':
                module.numpy_msg = lambda x: x
            if name.endswith('.msg'):
                module.__all__ = []
            if '.' in name:
                parent_name, child_name = name.rsplit('.', 1)
                parent = sys.modules.setdefault(parent_name, types.ModuleType(parent_name))
                setattr(parent, child_name, module)
            sys.modules[name] = module
    # Provide placeholders required by explicit imports
    geom = sys.modules['geometry_msgs.msg']
    geom.PoseStamped = type('PoseStamped', (), {})
    geom.Point32 = type('Point32', (), {})

    sensor = sys.modules['sensor_msgs.msg']
    sensor.Imu = type('Imu', (), {})
    sensor.PointCloud = type('PointCloud', (), {})

    viz = sys.modules['visualization_msgs.msg']
    viz.Marker = type('Marker', (), {})

    darknet = sys.modules['darknet_ros_msgs.msg']
    darknet.BoundingBoxes = type('BoundingBoxes', (), {})

    nav = sys.modules['nav_msgs.msg']
    nav.OccupancyGrid = type('OccupancyGrid', (), {})

    marvel = sys.modules['marvelmind_nav.msg']
    marvel.hedge_pos_ang = type('hedge_pos_ang', (), {})

    return importlib.import_module('scripts.library_agv')


def test_euler_to_quat_zero():
    lib = import_library_agv()
    quat = lib.euler_to_quat(0, 0, 0)
    assert np.allclose(quat, [0, 0, 0, 1])

