import sys
from python_qt_binding import QT_BINDING
if QT_BINDING == 'pyqt':
    from . import rviz2_py_sip as bindings    
    #from bindings.rviz2_py import *
    #from bindings.rviz_common import *
    viz = bindings.rviz2_py
    common = bindings.rviz_common
else:
    raise ImportError('Qt binding name "%s" is unknown.' % QT_BINDING)