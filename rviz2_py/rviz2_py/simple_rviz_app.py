#!/usr/bin/env python3

from PyQt5.QtWidgets import *
from PyQt5 import QtCore, QtGui
from PyQt5.QtGui import *
from PyQt5.QtCore import *

import threading

from pathlib import Path

import rclpy
from rclpy.utilities import *

import rviz2_py

import sys
 
 
class MainWindow(QWidget):
    
    def __init__(self):
        
        super(MainWindow,self).__init__()
        self.setup_ui()
        
    def setup_ui(self):
        
        self.setWindowTitle("PyQt Rviz Simple Window ")
        self.setGeometry(100, 100, 500, 400)
        
        self.main_layout = QVBoxLayout(self)
        
        self.rviz_visualization_frame = rviz2_py.viz.RvizVisualizationFrame('rviz2','')
        self.rviz_visualization_frame.setSplashPath('') 
        self.rviz_visualization_frame.setApp(QApplication.instance())       
        self.rviz_visualization_frame.initialize('')    
        print('Initialized rviz')
        
        self.rviz_visualization_frame.setMenuBar(None)
        self.rviz_visualization_frame.setStatusBar(None)
        self.rviz_visualization_frame.setHideButtonVisibility(False)
        
        #self.set_theme_style()
        
        # loading rviz config
        rviz_config_reader = rviz2_py.common.YamlConfigReader()
        rviz_config = rviz2_py.common.Config()
        rviz_config_reader.readFile(rviz_config, os.path.join( str(Path.home()) , '.rviz2/default.rviz') )
        self.rviz_visualization_frame.load(rviz_config)
        
        # adding to layout
        self.main_layout.addWidget(self.rviz_visualization_frame) # Uncomment this to show all of Rviz
        self.main_layout.addWidget(self.rviz_visualization_frame.getRenderPanel())            
        self.setLayout(self.main_layout)
        
    def set_theme_style(self):
        
        self.manager = self.rviz_visualization_frame.getManager()
        self.root_display = self.manager.getRootDisplayGroup()
        
        self.root_display.subProp('Global Options').subProp("Background Color").setValue(_back_color)
        self.root_display.subProp("Grid").subProp("Color").setValue(_grid_color)
        self.root_display.subProp("Grid").subProp("Alpha").setValue(_grid_alpha)
        self.root_display.subProp("Grid").subProp("Line Style").setValue('Lines')
        self.root_display.subProp("Grid").subProp("Plane Cell Count").setValue(20)
        self.root_display.subProp("Grid").subProp("Cell Size").setValue(0.5)
        
if __name__ == '__main__':
    
    # Strip the command line to remove any ROS run-time inserted arguments
    argv = remove_ros_args(args = sys.argv)
    rclpy.init(args = sys.argv)
    
    #node = rclpy.create_node('quick_rviz_node')
    
    # setting up Qt5
    qt_app = QApplication(argv)
    qt_app.setApplicationDisplayName('quick_rviz_app')
    #qt_app.beep()
    #qt_app.aboutQt()
    
    # declaring variable
    main_window : MainWindow;
    
    # launch thread
    #rclpy_spin_thread = threading.Thread(target = rclpy.spin, args = (node,), daemon = True )
    #rclpy_spin_thread.start()
    

    try:        
        main_window = MainWindow()
        main_window.show()
        
        # launch qt
        qt_app.exec()
        
    except KeyboardInterrupt as ex:
        pass
    
    print("Closing qt app")
        
    rclpy.shutdown()
    #rclpy_spin_thread.join()        
    sys.exit(0)
        
        
    
    
    