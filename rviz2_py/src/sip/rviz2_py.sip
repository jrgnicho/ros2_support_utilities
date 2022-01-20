%Module rviz2_py_sip

%Import QtCore/QtCoremod.sip
%Import QtGui/QtGuimod.sip
%Import QtWidgets/QtWidgetsmod.sip

%DefaultSupertype sip.simplewrapper

%Include property.sip
%Include config.sip
%Include display.sip
%Include view_controller.sip
%Include view_manager.sip
%Include display_group.sip
%Include visualization_manager.sip
%Include panel_dock_widget.sip
%Include tool.sip
%Include yaml_config_reader.sip
%Include rviz_visualization_frame.sip

//%Include bool_property.sip
//%Include tool_manager.sip
//%Include ogre_logging.sip
//%Include yaml_config_writer.sip
