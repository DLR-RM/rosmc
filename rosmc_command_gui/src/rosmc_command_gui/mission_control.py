#
# The BSD 3-Clause License
#
# Copyright (c) 2022, DLR-RM All rights reserved.
#
# Redistribution and use in source and binary forms, with or without 
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, 
#    this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice, 
#    this list of conditions and the following disclaimer in the documentation 
#    and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors 
#    may be used to endorse or promote products derived from this software 
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE 
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# 
# Contributors:
# Ryo Sakagami <ryo.sakagami@dlr.de>

import copy
from distutils.version import LooseVersion
import functools
import numpy as np
import os
import pickle
import qdarkstyle
import sys
from threading import RLock
import time
import yaml

import matplotlib
from matplotlib.collections import LineCollection
from matplotlib.collections import PathCollection
from matplotlib.collections import PolyCollection
from matplotlib.colors import colorConverter
from matplotlib.figure import Figure
import matplotlib.pyplot as plt

import rospy
import rospkg
import geometry_msgs.msg
import tf

from visualization_msgs.msg import InteractiveMarkerInit, InteractiveMarkerFeedback, InteractiveMarker, InteractiveMarkerUpdate

from rosmc_interface_msgs.msg import AgentActions, Action, ActionStatus
from rosmc_interface_msgs.srv import RegisterToServer, RegisterToServerRequest, Mission, MissionRequest, MissionResponse, TriggerMissionExecutor, \
    UpdateActionStatus, UpdateActionStatusRequest
from rosmc_msgs.srv import AddAction, EditAction,\
    SaveMission, SaveMissionRequest, LoadMission, LoadMissionRequest, NewMission, DeleteAction, DeleteActionRequest,\
    ReorderActions, AddSync, AddSyncRequest, DeleteSync, DeleteSyncRequest,\
    UpdateIntMarker, UpdateIntMarkerRequest, GetIntMarker, GetIntMarkerRequest, GetIntMarkerVertices, GetIntMarkerVerticesRequest,\
    RenameMarker, RenameMarkerRequest
from ScheduleWidget import ScheduleWidget, ActionTableView, SyncActionTableView, NonSyncActionTableView, \
    show_error_msg

from qt_gui.plugin import Plugin

import python_qt_binding
from python_qt_binding import loadUi
from python_qt_binding.QtCore import pyqtSignal
from python_qt_binding.QtCore import QObject
from python_qt_binding.QtCore import pyqtSlot
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtCore import QTimer
from python_qt_binding.QtCore import qWarning
from python_qt_binding.QtCore import Slot
from python_qt_binding.QtCore import QDir
from python_qt_binding.QtCore import QModelIndex
from python_qt_binding.QtCore import QFile
from python_qt_binding.QtCore import QTextStream
from python_qt_binding.QtGui import QColor
from python_qt_binding.QtGui import QIcon
from python_qt_binding.QtGui import QCursor


# Support both qt4 and qt5
QT_VERSION = LooseVersion(python_qt_binding.QT_BINDING_VERSION).version[0]
if QT_VERSION >= 5:
    from python_qt_binding.QtWidgets import QAction
    from python_qt_binding.QtWidgets import QFrame
    from python_qt_binding.QtWidgets import QHBoxLayout
    from python_qt_binding.QtWidgets import QMenu
    from python_qt_binding.QtWidgets import QPushButton
    from python_qt_binding.QtWidgets import QSizePolicy
    from python_qt_binding.QtWidgets import QTreeWidget
    from python_qt_binding.QtWidgets import QVBoxLayout
    from python_qt_binding.QtWidgets import QWidget
    from python_qt_binding.QtWidgets import QLabel
    from python_qt_binding.QtWidgets import QLineEdit
    from python_qt_binding.QtWidgets import QListWidget
    from python_qt_binding.QtWidgets import QListWidgetItem
    from python_qt_binding.QtWidgets import QComboBox
    from python_qt_binding.QtWidgets import QTreeWidgetItem
    from python_qt_binding.QtWidgets import QCheckBox
    from python_qt_binding.QtGui import QStandardItemModel
    from python_qt_binding.QtGui import QStandardItem
    from python_qt_binding.QtWidgets import QTableView
    from python_qt_binding.QtWidgets import QTableWidget
    from python_qt_binding.QtWidgets import QTableWidgetItem
    from python_qt_binding.QtWidgets import QHeaderView
    from python_qt_binding.QtWidgets import QFileDialog
    from python_qt_binding.QtWidgets import QMessageBox
    from python_qt_binding.QtWidgets import QApplication
    from python_qt_binding.QtCore import QItemSelectionModel
    from python_qt_binding.QtWidgets import QItemDelegate
    try:
        from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg \
            as FigureCanvas
    except ImportError:
        # work around bug in dateutil
        import thread
        sys.modules['_thread'] = thread
        from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg \
            as FigureCanvas
    try:
        from matplotlib.backends.backend_qt5agg \
            import NavigationToolbar2QTAgg as NavigationToolbar
    except ImportError:
        from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT \
            as NavigationToolbar

else:
    from python_qt_binding.QtGui import QAction
    from python_qt_binding.QtGui import QFrame
    from python_qt_binding.QtGui import QHBoxLayout
    from python_qt_binding.QtGui import QMenu
    from python_qt_binding.QtGui import QPushButton
    from python_qt_binding.QtGui import QSizePolicy
    from python_qt_binding.QtGui import QTreeWidget
    from python_qt_binding.QtGui import QVBoxLayout
    from python_qt_binding.QtGui import QWidget
    from python_qt_binding.QtGui import QLabel
    from python_qt_binding.QtGui import QLineEdit
    from python_qt_binding.QtGui import QListWidget
    from python_qt_binding.QtGui import QListWidgetItem
    from python_qt_binding.QtGui import QComboBox
    from python_qt_binding.QtGui import QTreeWidgetItem
    from python_qt_binding.QtGui import QCheckBox
    from python_qt_binding.QtGui import QStandardItemModel
    from python_qt_binding.QtGui import QStandardItem
    from python_qt_binding.QtGui import QTableView
    from python_qt_binding.QtGui import QTableWidget
    from python_qt_binding.QtGui import QTableWidgetItem
    from python_qt_binding.QtGui import QHeaderView
    from python_qt_binding.QtGui import QFileDialog
    from python_qt_binding.QtGui import QMessageBox
    from python_qt_binding.QtGui import QApplication
    from python_qt_binding.QtGui import QItemSelectionModel
    from python_qt_binding.QtGui import QItemDelegate

    try:
        from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg \
            as FigureCanvas
    except ImportError:
        # work around bug in dateutil
        import thread
        sys.modules['_thread'] = thread
        from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg \
            as FigureCanvas
    try:
        from matplotlib.backends.backend_qt4agg \
            import NavigationToolbar2QTAgg as NavigationToolbar
    except ImportError:
        from matplotlib.backends.backend_qt4agg import NavigationToolbar2QT \
            as NavigationToolbar


def rsetattr(obj, attr, val):
    pre, _, post = attr.rpartition('.')
    return setattr(rgetattr(obj, pre) if pre else obj, post, val)


def rgetattr(obj, attr, *args):
    def _getattr(obj, attr):
        return getattr(obj, attr, *args)
    return functools.reduce(_getattr, [obj] + attr.split('.'))


class ParameterTreeDelegate(QItemDelegate):

    def createEditor(self, parent, option, index):
        if index.column() == 2:
            return super(ParameterTreeDelegate, self).createEditor(parent, option, index)
        return None


class IntMarkerUpdateFullSubscriber(QObject):
    # Custom signal for updating options in combobox
    # parameters: topic, int_marker_name_list
    int_marker_list_update_full_signal = pyqtSignal(str, list)

    def __init__(self, topic):
        super(IntMarkerUpdateFullSubscriber, self).__init__()
        rospy.logdebug("Function IntMarkerUpdateFullSubscriber.__init__ is called")
        self.topic = topic
        self.int_marker_update_full_subscriber = rospy.Subscriber(topic + '/update_full', InteractiveMarkerInit,
                                                                  self.int_marker_update_full_callback)
        self.int_marker_name_list = []
        self.update_full_mutex = False

    def int_marker_update_full_callback(self, msg):
        rospy.logdebug("Function IntMarkerUpdateFullSubscriber.int_marker_update_full_callback is called")

        rospy.logdebug("Function IntMarkerUpdateFullSubscriber.int_marker_update_full_callback: Wait for mutex")
        while self.update_full_mutex:
            pass
        rospy.logdebug("Function IntMarkerUpdateFullSubscriber.int_marker_update_full_callback: Mutex acquired")
        self.update_full_mutex = True

        int_marker_name_list = [marker.name for marker in msg.markers]
        int_marker_name_list.sort()
        # If marker is added or deleted,
        if self.int_marker_name_list != int_marker_name_list:
            # Update member self.int_marker_name_list
            self.int_marker_name_list = int_marker_name_list
            # Emit signal
            self.int_marker_list_update_full_signal.emit(self.topic, self.int_marker_name_list)

        rospy.logdebug("Function IntMarkerUpdateFullSubscriber.int_marker_update_full_callback: Mutex unlocked")
        self.update_full_mutex = False


class MissionControlPlugin(Plugin):
    # Custom signal for re-selecting action in nonSyncActionTableView or syncActionTableView
    # parameters: agent_name, ActionTableView.objectName(), index
    action_table_item_reselection_signal = pyqtSignal(str, str, QModelIndex)
    action_table_clear_selection_signal = pyqtSignal(str, str)  # agent name, sync/nonsync
    schedule_widget_set_enabled_signal = pyqtSignal(bool)  # True for enabling
    synchronise_to_agent_signal = pyqtSignal(list, str)  # list of index, agent_name
    unsynchronise_from_agent_signal = pyqtSignal(list, str)  # list of index, agent_name
    update_visualization_signal = pyqtSignal(MissionRequest)  # MissionRequest

    def __init__(self, context):
        rospy.logdebug("Function MissionControlPlugin.__init__ is called")
        super(MissionControlPlugin, self).__init__(context)

        self.process_name = 'mcg'

        # Setup members for sending any modifications in command widget to Mission Server.
        self.selected_action = None
        self.selected_action_agent = None

        ############################################################
        # Mutexes
        self.command_widget_mutex = False
        self.selection_changed_callback_mutex = False

        ############################################################
        # Basic GUI setup

        # Give QObjects reasonable names
        self.setObjectName('MissionControlPlugin')
        rp = rospkg.RosPack()

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                            dest="quiet",
                            help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # Create QWidget
        self._widget = QWidget()

        # Apply dark theme if Qt4 is used
        if QT_VERSION <= 4:
            self._widget.setStyleSheet(qdarkstyle.load_stylesheet_pyqt())
        # else then use BreezeStyleSheets
        else:
            # import dark theme
            theme_path = rp.get_path('rosmc_command_gui') + '/BreezeStyleSheets/'
            theme_path = os.path.realpath(theme_path)
            print("Add " + theme_path + " to python system path.")
            sys.path.append(theme_path)
            import breeze_resources
            # set stylesheet
            file_ = QFile(":/dark.qss")
            file_.open(QFile.ReadOnly | QFile.Text)
            stream = QTextStream(file_)
            self._widget.setStyleSheet(stream.readAll())

        # Get path to UI file which is a sibling of this file
        # in this example the .ui and .py file are in the same folder
        ui_file = os.path.join(rp.get_path('rosmc_command_gui'), 'resource', 'MissionControlPlugin.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('MissionControlPluginUi')
        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        # Setup slots for signals of run/pause/stop buttons
        self._widget.runPushButton.clicked.connect(self.runButtonCallback)
        self._widget.pausePushButton.clicked.connect(self.pauseButtonCallback)
        self._widget.stopPushButton.clicked.connect(self.stopButtonCallback)
        self._widget.resetPushButton.clicked.connect(self.resetButtonCallback)

        # Setup slots for signals of new/save/saveas/load buttons
        self.mission_req = None
        self.save_path = None
        self._widget.newPushButton.clicked.connect(self.newButtonCallback)
        self._widget.savePushButton.clicked.connect(self.saveButtonCallback)
        self._widget.saveasPushButton.clicked.connect(self.saveasButtonCallback)
        self._widget.loadPushButton.clicked.connect(self.loadButtonCallback)

        # Setup slots for signals of QLineEdit of action label
        self._widget.commandWidget.findChild(QLineEdit, "actionLabelVariableLineEdit").editingFinished.connect(
            lambda tree_widget_item=None: self.send_edited_action_to_server(tree_widget_item)
        )

        # Setup headers for parameter tree widget
        tree_widget = self._widget.commandWidget.findChild(QTreeWidget, "parametersTreeWidget")
        # tree_widget.header().setStretchLastSection(False)
        delegate = ParameterTreeDelegate()
        tree_widget.setItemDelegate(delegate)
        tree_widget.itemChanged.connect(self.parameters_tree_item_changed_callback)
        # NOTE: Header labels are set after ensuring that Mission Server exists

        ############################################################
        # Communications between MissionControlGUI and Mission Server

        # Wait until MissionServer is available
        rospy.loginfo("Wait for MissionServer to be ready...")
        register_to_server_name = 'register_to_mission_server'
        rospy.wait_for_service(register_to_server_name)
        rospy.loginfo("MissionServer is ready.")
        register_to_server = rospy.ServiceProxy(register_to_server_name, RegisterToServer)
        # Establish connection
        rospy.loginfo("Start connection establishment between MissionServer and MissionControlGUI")
        self.send_mission_service = rospy.Service('send_mission', Mission, self.handle_send_mission)
        rospy.loginfo("Ready to send mission by calling services externally")
        try:
            req = RegisterToServerRequest()
            req.process_name = self.process_name
            req.namespace = rospy.get_namespace()
            req.mission_receive_mode = RegisterToServerRequest.FULL
            resp = register_to_server(req)  # This will set self.mission_req
            if resp.success:
                rospy.loginfo("Connection establishment succeed")
                self.mission_server_namespace = resp.mission_server_namespace
            else:
                rospy.logerr("Connection establishment failed")
        except rospy.ServiceException as exc:
            rospy.logerr("Service did not process request: " + str(exc))
            rospy.logerr("Connection establishment failed")

        # NOTE: Header labels are set after ensuring that Mission Server exists
        if rospy.get_param('{}use_interactive_markers'.format(self.mission_server_namespace)):
            tree_widget.setColumnCount(4)
            tree_widget.setHeaderLabels(['Name', 'Data Type', 'Value', 'Marker'])
        else:
            tree_widget.setColumnCount(3)
            tree_widget.setHeaderLabels(['Name', 'Data Type', 'Value'])

        # ScheduleWidgets setup
        # NOTE: This code is placed here because rosparam 'agent_action_dict'
        #       must be provided by MissionServer beforehand.
        try:
            h = QHBoxLayout()
            self._widget.scrollAreaWidgetContents.setLayout(h)
            sync_ids_list = self.mission_req.sync_ids_list
            for agent_actions in self.mission_req.agents_actions:
                # Create new ScheduleWidget
                schedule_widget = ScheduleWidget(agent_actions, sync_ids_list, self.mission_server_namespace)
                # Setup slots from ScheduleWidget and ActionTableView
                schedule_widget.non_sync_selection_changed.connect(self.non_sync_selection_changed_callback)
                schedule_widget.sync_selection_changed.connect(self.sync_selection_changed_callback)
                schedule_widget.selection_cleared.connect(self.clear_command_widget)
                self.action_table_item_reselection_signal.connect(
                    schedule_widget.action_table_item_reselection_signal_callback)
                self.schedule_widget_set_enabled_signal.connect(
                    schedule_widget.schedule_widget_set_enabled_signal_callback)
                self.action_table_clear_selection_signal.connect(
                    schedule_widget.action_table_clear_selection_signal_callback)
                self.synchronise_to_agent_signal.connect(
                    schedule_widget.synchronise_to_agent_callback)
                self.unsynchronise_from_agent_signal.connect(
                    schedule_widget.unsynchronise_from_agent_callback)
                self.update_visualization_signal.connect(
                    schedule_widget.update_visualization_callback)
                sync_action_table_view = schedule_widget.findChild(SyncActionTableView, 'syncActionTableView')
                sync_action_table_view.context_menu_event_signal.connect(self.sync_context_menu_event_callback)
                non_sync_action_table_view = schedule_widget.findChild(NonSyncActionTableView, 'nonSyncActionTableView')
                non_sync_action_table_view.context_menu_event_signal.connect(self.non_sync_context_menu_event_callback)
                # Add the ScheduleWidget to MissionControlGUI
                h.addWidget(schedule_widget)
                # Update visualization forcefully
                # NOTE: this style of function call is actually thread-unsafe! Use signal-slot as much as possible
                sync_action_table_view.update_visualization_forcefully(agent_actions.actions, sync_ids_list)
                non_sync_action_table_view.update_visualization_forcefully(agent_actions.actions, sync_ids_list)
        except KeyError:
            rospy.logerr("Rosparam key %s does not exist" % 'agent_action_dict')
            pass

        ############################################################
        # Assignment of service_proxy
        # For MissionControl (self)
        self.save_mission_service_proxy = rospy.ServiceProxy('{}save_mission'.format(self.mission_server_namespace), SaveMission)
        self.load_mission_service_proxy = rospy.ServiceProxy('{}load_mission'.format(self.mission_server_namespace), LoadMission)
        self.new_mission_service_proxy = rospy.ServiceProxy('{}new_mission'.format(self.mission_server_namespace), NewMission)
        self.add_sync_service_proxy = rospy.ServiceProxy('{}add_sync'.format(self.mission_server_namespace), AddSync)
        self.delete_sync_service_proxy = rospy.ServiceProxy('{}delete_sync'.format(self.mission_server_namespace), DeleteSync)

        # For each ScheduleWidget
        for schedule_widget in self._widget.scrollAreaWidgetContents.findChildren(ScheduleWidget):
            add_action_service_proxy\
                = rospy.ServiceProxy('{}add_action'.format(self.mission_server_namespace),
                                     AddAction)
            trigger_mission_executor_service_proxy\
                = rospy.ServiceProxy('/{}/trigger_mission_executor'.format(schedule_widget.objectName()),
                                     TriggerMissionExecutor) # TODO: do not directly call services of agents
            delete_action_service_proxy\
                = rospy.ServiceProxy('{}delete_action'.format(self.mission_server_namespace),
                                     DeleteAction)
            reorder_actions_service_proxy\
                = rospy.ServiceProxy('{}reorder_actions'.format(self.mission_server_namespace),
                                     ReorderActions)
            update_action_status_service_proxy\
                = rospy.ServiceProxy('{}update_action_status'.format(self.mission_server_namespace),
                                     UpdateActionStatus)
            edit_action_service_proxy\
                = rospy.ServiceProxy('{}edit_action'.format(self.mission_server_namespace),
                                     EditAction)
            # NOTE: this style of function call is actually thread-unsafe! Use signal-slot as much as possible
            schedule_widget.setServiceProxies(add_action_service_proxy,
                                              trigger_mission_executor_service_proxy,
                                              delete_action_service_proxy,
                                              reorder_actions_service_proxy,
                                              update_action_status_service_proxy,
                                              edit_action_service_proxy)
            rospy.loginfo("Service proxies for ScheduleWidget %s are set" % schedule_widget.objectName())

        # For CommandWidget
        self.edit_action_service_proxy = rospy.ServiceProxy('{}edit_action'.format(self.mission_server_namespace), EditAction)

        ############################################################
        # Read rosparam related to configurations
        # TODO: remove all self.mission_server_namespace?
        # agent action dictionary
        self.agent_action_dict = rospy.get_param('{}agent_action_dict'.format(self.mission_server_namespace))

        # agent library dictionary
        self.agent_library_dict = rospy.get_param('{}agent_library_dict'.format(self.mission_server_namespace))

        # parameter groups
        self.lib_action_parameter_groups_dict = rospy.get_param('{}lib_action_parameter_groups_dict'.format(self.mission_server_namespace))

        # use interactive markers
        self.use_interactive_markers = rospy.get_param('{}use_interactive_markers'.format(self.mission_server_namespace))

        # frame_id
        self.frame_id = rospy.get_param('frame_id', 'map')  # use relative namespace for defining frame_id used in ROSMC

        ############################################################
        # Interactive Markers
        self.int_marker_feedback_subscriber_list = []
        self.update_int_marker_service_proxy_dict = {}
        self.get_int_marker_service_proxy_dict = {}
        self.get_int_marker_list_service_proxy_dict = {}  # for vertices
        self.rename_marker_service_proxy_dict = {}
        self.int_marker_update_full_subscriber_dict = {}
        self.update_int_marker_feedback_mutex_lock = RLock()
        if self.use_interactive_markers:
            # Read config dicts related to int_markers
            self.common_int_marker = rospy.get_param('{}common_int_marker'.format(self.mission_server_namespace))
            self.lib_action_int_marker_dict = rospy.get_param('{}lib_action_int_marker_dict'.format(self.mission_server_namespace))
            for topic in self.common_int_marker['topics']:
                topic_name = topic['name']
                topic_type = topic['type']
                int_marker_feedback_subscriber = rospy.Subscriber(topic_name + '/feedback', InteractiveMarkerFeedback,
                                                                  self.int_marker_feedback_callback)
                self.int_marker_feedback_subscriber_list.append(int_marker_feedback_subscriber)
                update_int_marker_service_proxy = rospy.ServiceProxy(topic_name + '/update_int_marker', UpdateIntMarker)
                self.update_int_marker_service_proxy_dict[topic_name] = update_int_marker_service_proxy
                get_int_marker_service_proxy = rospy.ServiceProxy(topic_name + '/get_int_marker', GetIntMarker)
                self.get_int_marker_service_proxy_dict[topic_name] = get_int_marker_service_proxy
                rename_marker_service_proxy = rospy.ServiceProxy(topic_name + '/rename_marker', RenameMarker)
                self.rename_marker_service_proxy_dict[topic_name] = rename_marker_service_proxy
                int_marker_update_full_subscriber = IntMarkerUpdateFullSubscriber(topic_name)
                self.int_marker_update_full_subscriber_dict[topic_name] = int_marker_update_full_subscriber

                # If we use topic type TASK_LIST_POSEDICT, we also subscribes information about vertices
                if topic_type == 'TASK_LIST_POSEDICT':
                    # TODO: we need to update mission so that we could increase/decrease the number of children
                    topic_name += '_vertices'  # Implicit assumption here
                    int_marker_feedback_subscriber = rospy.Subscriber(topic_name + '/feedback',
                                                                      InteractiveMarkerFeedback,
                                                                      self.int_marker_feedback_callback)
                    self.int_marker_feedback_subscriber_list.append(int_marker_feedback_subscriber)
                    # update_int_marker_service_proxy?
                    get_int_marker_service_proxy = rospy.ServiceProxy(topic_name + '/get_int_marker', GetIntMarker)
                    self.get_int_marker_service_proxy_dict[topic_name] = get_int_marker_service_proxy
                    # NOTE: prepare special additional service for vertices!!
                    get_int_marker_list_service_proxy = rospy.ServiceProxy(topic_name + '/get_int_marker_list',
                                                                           GetIntMarkerVertices)
                    self.get_int_marker_list_service_proxy_dict[topic_name] = get_int_marker_list_service_proxy
                    int_marker_update_full_subscriber = IntMarkerUpdateFullSubscriber(topic_name)
                    self.int_marker_update_full_subscriber_dict[topic_name] = int_marker_update_full_subscriber

            # Members that are used for dealing with int_markers
            self.INT_MARKER = InteractiveMarker()
            self.ROLL = 0.0
            self.PITCH = 0.0
            self.YAW = 0.0

        ############################################################
        rospy.loginfo("MissionControlGUI is ready")

    def int_marker_feedback_callback(self, msg):
        """
        This function
            - sets an action selected when MOUSE_DOWN
            - updates all pose parameter values bound to interactive markers whose topic is self.topic
            - TODO: updates all other parameters bound to interactive markers whose topic is self.topic
                ex. task_marker_2d_area
            - sends the above updates to Mission Server
            - TODO: deals with menu feedback, such as AddAction
        :param msg:
        :return:
        """
        rospy.logdebug("Function MissionControlPlugin.int_marker_feedback_callback is called")
        rospy.logdebug("Function MissionControlPlugin.int_marker_feedback_callback: client_id = %s, msg.marker_name = %s" % (msg.client_id, msg.marker_name))

        rospy.logdebug(
            "Function MissionControlPlugin.int_marker_feedback_callback: Wait for mutex lock")
        with self.update_int_marker_feedback_mutex_lock:
            rospy.logdebug(
                "Function MissionControlPlugin.int_marker_feedback_callback: Mutex lock acquired")

            # case MOUSE_DOWN
            if msg.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
                rospy.logdebug(
                    "Function MissionControlPlugin.int_marker_feedback_callback: msg.event_type == InteractiveMarkerFeedback.MOUSE_DOWN")
                # Iterate over self.mission_req.agents_actions
                # and select the first action that uses the clicked marker for parametrization

                is_found = False
                for agent_actions in self.mission_req.agents_actions:
                    if is_found:
                        break
                    agent_name = agent_actions.agent_name

                    sync_action_id_list = []  # These id lists are used for finding index in ActionTableView
                    non_sync_action_id_list = []
                    for action in agent_actions.actions:
                        if is_found:
                            break
                        if action.action_content.status.value != ActionStatus.IDLE:
                            sync_action_id_list.append(action.action_id)
                        else:
                            non_sync_action_id_list.append(action.action_id)

                        action_parameters = yaml.safe_load(action.action_content.parameters_yaml)
                        for parameter_name in action_parameters:
                            if is_found:
                                break
                            try:
                                marker_name = action_parameters[parameter_name]['marker']
                            except KeyError:
                                continue
                            if marker_name == "":  # skip the following if marker name is not filled out
                                continue
                            if msg.marker_name.startswith(marker_name):  # This is ultimate condition we want after all
                                # Make this action selected in ActionTableView
                                schedule_widget = self._widget.scrollAreaWidgetContents.findChild(ScheduleWidget,
                                                                                                    agent_name)
                                if action.action_content.status.value != ActionStatus.IDLE:
                                    action_table_view = schedule_widget.findChild(SyncActionTableView,
                                                                                    'syncActionTableView')
                                    row = sync_action_id_list.index(action.action_id)
                                    index = action_table_view.model.index(row, 0)
                                else:
                                    action_table_view = schedule_widget.findChild(NonSyncActionTableView,
                                                                                    'nonSyncActionTableView')
                                    row = non_sync_action_id_list.index(action.action_id)
                                    index = action_table_view.model.index(row, 0)
                                self.action_table_item_reselection_signal.emit(agent_name,
                                                                                action_table_view.objectName(), index)
                                rospy.loginfo("Marker %s is clicked, thus selected action %s" % (msg.marker_name,
                                                                                                    action.action_id))
                                is_found = True

            # case MOUSE_UP
            elif msg.event_type == InteractiveMarkerFeedback.MOUSE_UP:
                rospy.logdebug(
                    "Function MissionControlPlugin.int_marker_feedback_callback: msg.event_type == InteractiveMarkerFeedback.MOUSE_UP")
                # Get corresponding int_marker
                for topic_name, int_marker_update_full_subscriber in self.int_marker_update_full_subscriber_dict.iteritems():
                    if msg.marker_name in int_marker_update_full_subscriber.int_marker_name_list:
                        req = GetIntMarkerRequest()
                        req.marker_name = msg.marker_name
                        res = self.get_int_marker_service_proxy_dict[topic_name](req)
                        self.INT_MARKER = res.int_marker
                        self.INT_MARKER.pose.position.z = 0.0  # override
                        euler = tf.transformations.euler_from_quaternion((self.INT_MARKER.pose.orientation.x,
                                                                            self.INT_MARKER.pose.orientation.y,
                                                                            self.INT_MARKER.pose.orientation.z,
                                                                            self.INT_MARKER.pose.orientation.w))
                        self.ROLL = round(euler[0], 4)
                        self.PITCH = round(euler[1], 4)
                        self.YAW = round(euler[2], 4)
                        break

                # Iterate over self.mission_req.agents_actions
                # and update all actions that are included in self.lib_action_parameter_int_marker_dict
                for agent_actions in self.mission_req.agents_actions:
                    agent_name = agent_actions.agent_name
                    library = self.agent_library_dict[agent_name]
                    try:
                        action_int_marker_dict = self.lib_action_int_marker_dict[library]
                    except KeyError:
                        continue
                    for action in agent_actions.actions:
                        action_name = action.action_content.action_name
                        try:
                            int_marker_info_list = action_int_marker_dict[action_name]
                        except KeyError:
                            continue
                        edited_action = action
                        edited_parameters = yaml.safe_load(edited_action.action_content.parameters_yaml)
                        is_parameter_changed = False
                        for int_marker_info in int_marker_info_list:
                            group_or_parameter_name = int_marker_info['name']
                            int_marker_type = int_marker_info['type']
                            topics = int_marker_info['topics']
                            if int_marker_type == 'TASK_POSEDICT':
                                # special logic with hard-coding
                                parameter_name = group_or_parameter_name
                                if msg.marker_name == edited_parameters[parameter_name]['marker']:
                                    posedict = {
                                        'x': self.INT_MARKER.pose.position.x,
                                        'y': self.INT_MARKER.pose.position.y,
                                        'z': 0.0,  # override
                                        'roll': self.ROLL,
                                        'pitch': self.PITCH,
                                        'yaw': self.YAW
                                    }
                                    edited_parameters[parameter_name]['value'] = posedict
                                    is_parameter_changed = True
                            elif int_marker_type == 'TASK_LIST_POSEDICT':
                                # special logic with hard-coding
                                parameter_name = group_or_parameter_name
                                marker_name_center = edited_parameters[parameter_name]['marker']  # marker name of center
                                if msg.marker_name.startswith(marker_name_center):
                                    if msg.marker_name == marker_name_center:
                                        topic_name_vertices = topic_name + '_vertices'
                                    else:
                                        topic_name_vertices = topic_name
                                    # Get all vertices markers
                                    req = GetIntMarkerVerticesRequest()
                                    req.marker_name_center = marker_name_center
                                    res = self.get_int_marker_list_service_proxy_dict[topic_name_vertices](req)  # Implicit assumption
                                    int_marker_vertices = res.int_marker_vertices
                                    # Calculate value
                                    list_posedict = []
                                    for int_marker_vertex in int_marker_vertices:
                                        euler = tf.transformations.euler_from_quaternion((int_marker_vertex.pose.orientation.x,
                                                                                            int_marker_vertex.pose.orientation.y,
                                                                                            int_marker_vertex.pose.orientation.z,
                                                                                            int_marker_vertex.pose.orientation.w))
                                        posedict = {
                                            'x': int_marker_vertex.pose.position.x,
                                            'y': int_marker_vertex.pose.position.y,
                                            'z': 0.0,  # override
                                            'roll': euler[0],
                                            'pitch': euler[1],
                                            'yaw': euler[2]
                                        }
                                        list_posedict.append(posedict)

                                    edited_parameters[parameter_name]['value'] = list_posedict
                                    is_parameter_changed = True
                            else:
                                # Check if this int_marker is assigned to single parameter or group of parameters
                                # If this marker is assigned to multiple parameters (group),
                                if 'members' in int_marker_info:
                                    members = int_marker_info['members']
                                    for member in members:
                                        parameter_name = member['name']
                                        marker_attr = member['value']
                                        if not (marker_attr.startswith('INT_MARKER.pose') or marker_attr in ['ROLL', 'PITCH', 'YAW']):  # Update parameter only if attribute is pose
                                            continue
                                        marker_name = edited_parameters[parameter_name]['marker']
                                        if marker_name == msg.marker_name:
                                            edited_parameters[parameter_name]['value'] = rgetattr(self, marker_attr)
                                            is_parameter_changed = True
                                # Else if this marker is assigned to single parameter
                                elif 'value' in int_marker_info:
                                    parameter_name = group_or_parameter_name
                                    marker_attr = int_marker_info['value']
                                    if not (marker_attr.startswith('INT_MARKER.pose') or marker_attr in ['ROLL', 'PITCH', 'YAW']):  # Update parameter only if attribute is pose
                                        continue
                                    marker_name = edited_parameters[parameter_name]['marker']
                                    if marker_name == msg.marker_name:
                                        edited_parameters[parameter_name]['value'] = rgetattr(self, marker_attr)
                                        is_parameter_changed = True
                                else:
                                    rospy.logerr(
                                        "ERROR: either 'members' or 'value' key is required for int_marker assignment!")
                        if is_parameter_changed:
                            edited_action.action_content.parameters_yaml = yaml.dump(edited_parameters)
                            res = self.edit_action_service_proxy(edited_action)
                            show_error_msg(res.error_msg)
                            rospy.loginfo("Applied changes to action (id: %s)" % edited_action.action_id)

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    def clear_command_widget(self):
        """
        Clear the contents of command widget.
        This function is connected to signal selection_cleared of ScheduleWidget.
        This function is also called after loading new mission file.
        :return:
        """
        rospy.logdebug("Function MissionControlPlugin.clear_command_widget is called")

        # Disable updates of ActionTableViews so that no action is selected by clicking during this operation
        self.schedule_widget_set_enabled_signal.emit(False)

        rospy.logdebug("Function MissionControlPlugin.clear_command_widget: Wait for mutex")
        while self.command_widget_mutex:
            pass
        rospy.logdebug("Function MissionControlPlugin.clear_command_widget: Mutex acquired")
        self.command_widget_mutex = True

        # Agent name
        self._widget.commandWidget.findChild(QLabel, "agentNameVariableLabel").setText('')

        # ID
        self._widget.commandWidget.findChild(QLabel, "idLabel").setText('')

        # Action name
        self._widget.commandWidget.findChild(QLabel, "actionNameVariableLabel").setText('')

        # Action label
        self._widget.commandWidget.findChild(QLineEdit, "actionLabelVariableLineEdit").setText('')

        # Status
        self._widget.commandWidget.findChild(QLabel, "statusVariableLabel").setText('')

        # Parameters
        tree_widget = self._widget.commandWidget.findChild(QTreeWidget, 'parametersTreeWidget')
        tree_widget.clear()
        # TODO: we may need to delete IntMarkerComboBox explicitly
        # for i in range(tree_widget.rowCount()):
        #     # Need to delete IntMarkerComboBox explicitly
        #     combo = tree_widget.cellWidget(i, 3)
        #     if combo is not None:
        #         combo.unregister()
        #         tree_widget.removeCellWidget(i, 3)
        # tree_widget.setRowCount(0)

        # reset debug string
        self.set_debug_int_marker_name_label_signal_callback("debug", "")

        # Reactive options
        self._widget.commandWidget.findChild(QCheckBox, "batteryCheckBox").setChecked(False)
        self._widget.commandWidget.findChild(QCheckBox, "wlanCheckBox").setChecked(False)
        self._widget.commandWidget.findChild(QCheckBox, "manualCheckBox").setChecked(False)

        # Reset members
        self.selected_action = None
        self.selected_action_agent = None

        rospy.loginfo("Cleared command widget")

        rospy.logdebug("Function MissionControlPlugin.clear_command_widget: Mutex unlocked")
        self.command_widget_mutex = False

        # Enable updates of ActionTableViews so that new action is selected by clicking
        self.schedule_widget_set_enabled_signal.emit(True)

    def non_sync_selection_changed_callback(self, agent_name, action):
        """
        Handle selectionChanged signal from NonSyncActionTableView in ScheduleWidgets to
            - show information in commandWidget
            - create synchronization for different actions
        :param action: rosmc_msgs.msg.Action
        :return:
        """
        rospy.logdebug("Function MissionControlPlugin.non_sync_selection_changed_callback is called")

        rospy.logdebug("Function MissionControlPlugin.non_sync_selection_changed_callback: Wait for mutex")
        while self.selection_changed_callback_mutex:
            pass
        rospy.logdebug("Function MissionControlPlugin.non_sync_selection_changed_callback: Mutex acquired")
        self.selection_changed_callback_mutex = True

        modifiers = QApplication.keyboardModifiers()
        if modifiers == Qt.ShiftModifier or modifiers == Qt.ControlModifier \
                or modifiers == (Qt.ShiftModifier | Qt.ControlModifier):
            # If the action is selected with Control/Shift modifiers, do not clear selection in other ScheduleWidgets.
            # This is for making it possible to create synchronization among actions.
            rospy.loginfo("Action %s (id: %s, label: %s) is selected with Ctrl/Shift modifier."
                          % (action.action_content.action_name, action.action_id, action.action_content.action_label))

        else:
            rospy.loginfo("Action %s (id: %s, label: %s) is selected." % (action.action_content.action_name, action.action_id, action.action_content.action_label))

            # Clear selection in other agents' ScheduleWidgets
            for schedule_widget in self._widget.scrollAreaWidgetContents.findChildren(ScheduleWidget):
                if schedule_widget.objectName() == agent_name:
                    continue
                self.action_table_clear_selection_signal.emit(schedule_widget.objectName(), 'nonsync')
                self.action_table_clear_selection_signal.emit(schedule_widget.objectName(), 'sync')

        # Clear selection in all SyncActionTableView
        for schedule_widget in self._widget.scrollAreaWidgetContents.findChildren(ScheduleWidget):
            self.action_table_clear_selection_signal.emit(schedule_widget.objectName(), 'sync')

        # Store the action information
        self.selected_action = action
        self.selected_action_agent = agent_name
        # Update command widget
        self.update_command_widget(agent_name, action)
        # Update whether command widget is editable or not
        # ActionStatus.IDLE is the only state that is editable
        if action.action_content.status.value == ActionStatus.IDLE:
            self._widget.commandWidget.setEnabled(True)
        else:
            self._widget.commandWidget.setEnabled(False)

        rospy.logdebug("Function MissionControlPlugin.non_sync_selection_changed_callback: Mutex unlocked")
        self.selection_changed_callback_mutex = False

    def sync_selection_changed_callback(self, agent_name, action):
        """
        Handle selectionChanged signal from SyncActionTableView in ScheduleWidgets to
            - show information in commandWidget
        :param action: rosmc_msgs.msg.Action
        :return:
        """
        rospy.logdebug("Function MissionControlPlugin.sync_selection_changed_callback is called")

        rospy.logdebug("Function MissionControlPlugin.sync_selection_changed_callback: Wait for mutex")
        while self.selection_changed_callback_mutex:
            pass
        rospy.logdebug("Function MissionControlPlugin.sync_selection_changed_callback: Mutex acquired")
        self.selection_changed_callback_mutex = True

        # In SyncActionTableView, we do not allow multiple selection from different lists
        rospy.loginfo("Action %s (id: %s, label: %s) is selected." % (action.action_content.action_name, action.action_id, action.action_content.action_label))

        # Clear selection in other ActionTableView in ScheduleWidgets
        for schedule_widget in self._widget.scrollAreaWidgetContents.findChildren(ScheduleWidget):
            if schedule_widget.objectName() != agent_name:
                self.action_table_clear_selection_signal.emit(schedule_widget.objectName(), 'sync')
            self.action_table_clear_selection_signal.emit(schedule_widget.objectName(), 'nonsync')

        # Store the action information
        self.selected_action = action
        self.selected_action_agent = agent_name
        # Update command widget
        self.update_command_widget(agent_name, action)
        # Update whether command widget is editable or not
        # ActionStatus.IDLE is the only state that is editable
        if action.action_content.status.value == ActionStatus.IDLE:
            self._widget.commandWidget.setEnabled(True)
        else:
            self._widget.commandWidget.setEnabled(False)

        rospy.logdebug("Function MissionControlPlugin.sync_selection_changed_callback: Mutex unlocked")
        self.selection_changed_callback_mutex = False

    def sync_context_menu_event_callback(self, event):
        rospy.logdebug("Function MissionControlPlugin.sync_context_menu_event_callback is called")
        # example: selection_dict['turtle1'][index.row()] returns action
        selection_dict = {}
        for schedule_widget in self._widget.scrollAreaWidgetContents.findChildren(ScheduleWidget):
            agent_name = schedule_widget.objectName()
            sync_action_table_view = schedule_widget.findChild(SyncActionTableView, 'syncActionTableView')
            indexes = sync_action_table_view.selectionModel().selectedRows()
            # Sort selected indices because selected indices are not sorted when ctrl modifier is used
            indexes = sorted(indexes)
            if len(indexes) > 0:
                selected_actions = {}
                for index in indexes:
                    action = sync_action_table_view.actions[index.row()]
                    selected_actions[index.row()] = action
                selection_dict[agent_name] = selected_actions
        rospy.loginfo("Right-clicked the following action(s) %s" % str(selection_dict))

        # For convenience, create the following lists/dicts from selection_dict
        selected_statuses = []
        selected_ids = []
        max_selection_per_agent = 0
        for key in selection_dict:
            selected_actions = selection_dict[key]
            selected_statuses = selected_statuses + [selected_actions[index_row].action_content.status.value for
                                                     index_row in selected_actions]
            selected_ids = selected_ids + [selected_actions[index_row].action_id for index_row in selected_actions]
            max_selection_per_agent = max(max_selection_per_agent, len(selected_actions))

        # Create tree menu
        menu = QMenu(self._widget.scrollAreaWidgetContents)

        # Show action for delete action if at least one action is selected whose status is IDLE_SYNCHRONISED
        if len(selection_dict) > 0 and \
                all(status == ActionStatus.IDLE_SYNCHRONISED for status in selected_statuses):
            # menu for synchronizing to agents
            unsyncFromAgentAction = QAction('Unsynchronize from agent', self)
            unsyncFromAgentAction.triggered.connect(lambda: self.unsynchronize_from_agent(selection_dict))
            menu.addAction(unsyncFromAgentAction)
        menu.popup(QCursor.pos())

    def non_sync_context_menu_event_callback(self, event):
        rospy.logdebug("Function MissionControlPlugin.non_sync_context_menu_event_callback is called")
        # example: selection_dict['turtle1'][index.row()] returns action
        selection_dict = {}
        for schedule_widget in self._widget.scrollAreaWidgetContents.findChildren(ScheduleWidget):
            agent_name = schedule_widget.objectName()
            non_sync_action_table_view = schedule_widget.findChild(NonSyncActionTableView, 'nonSyncActionTableView')
            sync_action_table_view = schedule_widget.findChild(SyncActionTableView, 'syncActionTableView')
            indexes = non_sync_action_table_view.selectionModel().selectedRows()
            # Sort selected indices because selected indices are not sorted when ctrl modifier is used
            indexes = sorted(indexes)
            if len(indexes) > 0:
                selected_actions = {}
                for index in indexes:
                    action = non_sync_action_table_view.actions[index.row() + sync_action_table_view.model.rowCount()]
                    selected_actions[index.row()] = action
                selection_dict[agent_name] = selected_actions
        rospy.loginfo("Right-clicked the following action(s) %s" % str(selection_dict))

        # For convenience, create the following lists/dicts from selection_dict
        selected_statuses = []
        selected_ids = []
        max_selection_per_agent = 0
        for key in selection_dict:
            selected_actions = selection_dict[key]
            selected_statuses = selected_statuses + [selected_actions[index_row].action_content.status.value for index_row in selected_actions]
            selected_ids = selected_ids + [selected_actions[index_row].action_id for index_row in selected_actions]
            max_selection_per_agent = max(max_selection_per_agent, len(selected_actions))

        # Create tree menu
        menu = QMenu(self._widget.scrollAreaWidgetContents)

        # Show action for delete action if at least one action is selected whose status is IDLE
        if len(selection_dict) > 0 and \
                all(status == ActionStatus.IDLE for status in selected_statuses):
            # menu for deleting
            deleteActionAction = QAction('Delete action', self)
            deleteActionAction.triggered.connect(lambda: self.delete_actions(selected_ids))
            menu.addAction(deleteActionAction)
            # menu for synchronizing to agents
            syncToAgentAction = QAction('Synchronize to agent', self)
            syncToAgentAction.triggered.connect(lambda: self.synchronize_to_agent(selection_dict))
            menu.addAction(syncToAgentAction)
        # Show action for adding synchronization if multiple IDLE actions from different agents are selected
        if len(selection_dict) > 1 and max_selection_per_agent == 1 and\
                all(status == ActionStatus.IDLE for status in selected_statuses):
            addSyncAction = QAction('Add synchronization', self)
            addSyncAction.triggered.connect(lambda: self.add_synchronization(selected_ids))
            menu.addAction(addSyncAction)
        # Show action for deleting synchronization if at least one of the selected actions are included in sync_ids_list
        for id in selected_ids:
            for sync_ids in self.mission_req.sync_ids_list:
                if id in sync_ids.sync_ids:
                    deleteSyncAction = QAction('Delete synchronization', self)
                    deleteSyncAction.triggered.connect(lambda: self.delete_synchronization(selected_ids))
                    menu.addAction(deleteSyncAction)
                    break
            else:
                continue
            break
        menu.popup(QCursor.pos())

    def add_synchronization(self, ids):
        """
        Connected to a signal emitted from non_sync_context_menu_event_callback()
        :param ids:
        :return:
        """
        rospy.logdebug("Function MissionControlPlugin.add_synchronization is called")
        res = self.add_sync_service_proxy(ids)
        show_error_msg(res.error_msg)

    def delete_synchronization(self, ids):
        """
        Connected to a signal emitted from non_sync_context_menu_event_callback()
        :param ids:
        :return:
        """
        rospy.logdebug("Function MissionControlPlugin.delete_synchronization is called")
        res = self.delete_sync_service_proxy(ids)
        show_error_msg(res.error_msg)

    def delete_actions(self, ids):
        rospy.logdebug("Function MissionControlPlugin.delete_actions is called")
        schedule_widgets = self._widget.scrollAreaWidgetContents.findChildren(ScheduleWidget)
        schedule_widget = schedule_widgets[0]
        for id in ids:
            req = DeleteActionRequest()
            req.action_id = id
            res = schedule_widget.delete_action_service_proxy(req)  # TODO: USE SIGNAL-SLOT!!
            show_error_msg(res.error_msg)
        # clear the command widget
        self.clear_command_widget()

    def synchronize_to_agent(self, selection_dict):
        """
        Connected to a signal emitted from non_sync_context_menu_event_callback()
        :param selection_dict:
        :return:
        """
        rospy.logdebug("Function MissionControlPlugin.synchronize_to_agent is called")
        for agent_name in selection_dict:
            self.synchronise_to_agent_signal.emit(list(selection_dict[agent_name]), agent_name)

    def unsynchronize_from_agent(self, selection_dict):
        """
        Connected to a signal emitted from sync_context_menu_event_callback()
        :param selection_dict:
        :return:
        """
        rospy.logdebug("Function MissionControlPlugin.unsynchronize_from_agent is called")
        for agent_name in selection_dict:
            self.unsynchronise_from_agent_signal.emit(list(selection_dict[agent_name]), agent_name)

    def update_command_widget(self, agent_name, action):
        rospy.logdebug("Function MissionControlPlugin.update_command_widget is called")

        rospy.logdebug("Function MissionControlPlugin.update_command_widget: Wait for mutex")
        while self.command_widget_mutex:
            pass
        rospy.logdebug("Function MissionControlPlugin.update_command_widget: Mutex acquired")
        self.command_widget_mutex = True

        # Agent name
        self._widget.commandWidget.findChild(QLabel, "agentNameVariableLabel").setText(agent_name)

        # ID
        self._widget.commandWidget.findChild(QLabel, "idLabel").setText(action.action_id)

        # Action list
        self._widget.commandWidget.findChild(QLabel, "actionNameVariableLabel").setText(action.action_content.action_name)
        self._widget.commandWidget.findChild(QLineEdit, "actionLabelVariableLineEdit").setText(action.action_content.action_label)

        # Status
        status = action.action_content.status.value
        if status == ActionStatus.IDLE:
            status_name = "IDLE"
        elif status == ActionStatus.SUCCESS:
            status_name = "SUCCESS"
        elif status == ActionStatus.FAILURE:
            status_name = "FAILURE"
        elif status == ActionStatus.RUNNING:
            status_name = "RUNNING"
        elif status == ActionStatus.PAUSED:
            status_name = "PAUSED"
        elif status == ActionStatus.STOPPED:
            status_name = "STOPPED"
        elif status == ActionStatus.RESUMED:
            status_name = "RESUMED"
        elif status == ActionStatus.SYNCWAITING:
            status_name = "SYNCWAITING"
        elif status == ActionStatus.IDLE_SYNCHRONISED:
            status_name = "IDLE_SYNCHRONISED"
        else:
            status_name = ""
        self._widget.commandWidget.findChild(QLabel, "statusVariableLabel").setText(status_name)

        # Parameters
        parameters_dict = yaml.safe_load(action.action_content.parameters_yaml)
        self.update_parameter_tree(parameters_dict, agent_name, action.action_content.action_name)

        # Reactive options
        self._widget.commandWidget.findChild(QCheckBox, "batteryCheckBox").setChecked(
            action.action_content.battery_flag)
        self._widget.commandWidget.findChild(QCheckBox, "wlanCheckBox").setChecked(
            action.action_content.wlan_flag)
        self._widget.commandWidget.findChild(QCheckBox, "manualCheckBox").setChecked(
            action.action_content.manual_flag)

        rospy.loginfo("Updated command widget")

        rospy.logdebug("Function MissionControlPlugin.update_command_widget: Mutex unlocked")
        self.command_widget_mutex = False

    def create_parameter_tree_item_from_parameter(self, parameters_dict, parameter_name):
        # data type
        data_type = parameters_dict[parameter_name]['data_type']
        if data_type.startswith("__builtin__."):
            data_type = data_type[len("__builtin__."):]
        elif data_type.startswith("<type '"):
            data_type = data_type[len("<type '"):-2]
        # value
        value = parameters_dict[parameter_name]['value']
        # create item
        item = self.create_parameter_tree_item(parameter_name, data_type, value)
        return item

    def create_parameter_tree_item(self, name, data_type, value):
        # create parameter level tree widget
        item = QTreeWidgetItem()
        # set parameter name
        item.setData(0, Qt.DisplayRole, name)
        # set parameter data_type
        item.setData(1, Qt.DisplayRole, data_type)
        # set parameter value
        item.setData(2, Qt.EditRole, value)
        # make item editable
        item.setFlags(item.flags() | Qt.ItemIsEditable)
        return item

    def read_parameter_tree_item(self, item):
        item_name = item.text(0)
        item_datatype = item.text(1)
        item_value = item.text(2)
        if item_datatype == 'str':
            value = str(item_value)
        elif item_datatype == 'float':
            value = float(item_value)
        elif item_datatype == 'int':
            value = int(item_value)
        elif item_datatype == 'bool':
            value = bool(item_value)
            # TODO: Add the flow for the rest of data types
        else:
            rospy.logerr(
                "Data Type %s is not implemented in send_edited_action_to_server()" % item_datatype)
            value = None
        return str(item_name), item_datatype, value

    def create_parameter_tree_item_break_down_mode(self, parameters_dict, parameter_name, parse_type):
        # create parameter level tree widget
        item_top_level = QTreeWidgetItem()
        # action name
        item_top_level.setData(0, Qt.DisplayRole, parameter_name)

        # get parameter data
        data_type = parameters_dict[parameter_name]['data_type']
        if data_type.startswith("__builtin__."):
            data_type = data_type[len("__builtin__."):]
        elif data_type.startswith("<type '"):
            data_type = data_type[len("<type '"):-2]
        value = parameters_dict[parameter_name]['value']

        # assert that data_type is list
        try:
            assert data_type == 'list' or data_type == 'dict'
        except AssertionError:
            rospy.logerr('ERROR: Unexpected data_type of input parameter: {}'.format(data_type))
            return item_top_level

        # create children item following parse_type
        if parse_type == 'dict':
            assert type(value) == dict
            for key, v in value.iteritems():
                item = self.create_parameter_tree_item(key, 'float', v)
                item_top_level.addChild(item)
        elif parse_type == 'posedict':
            for i, posedict in enumerate(value):
                # posedict is dictionary with key 'x', 'y', 'z', 'roll', 'pitch', 'yaw'
                assert type(posedict) == dict
                item_second_level = QTreeWidgetItem()
                item_second_level.setData(0, Qt.DisplayRole, 'pose' + str(i))
                for key, v in posedict.iteritems():
                    item = self.create_parameter_tree_item(key, 'float', v)
                    item_second_level.addChild(item)
                item_top_level.addChild(item_second_level)
        else:
            rospy.logerr('ERROR: Unexpected parse_type: {}'.format(parse_type))
        return item_top_level

    def read_parameter_tree_item_break_down_mode(self, item_top_level, parse_type):
        value = []
        if parse_type == 'dict':
            child_count = item_top_level.childCount()
            value = {}
            for child_index in range(child_count):
                item = item_top_level.child(child_index)
                item_name, item_datatype, item_value = self.read_parameter_tree_item(item)
                value[item_name] = item_value
            return value
        elif parse_type == 'posedict':
            child_count = item_top_level.childCount()
            for child_index in range(child_count):
                item_second_level = item_top_level.child(child_index)
                second_child_count = item_second_level.childCount()
                pose_dict = {}
                for second_child_index in range(second_child_count):
                    item = item_second_level.child(second_child_index)
                    item_name, item_datatype, item_value = self.read_parameter_tree_item(item)
                    pose_dict[item_name] = item_value
                value.append(pose_dict)
            return value
        else:
            rospy.logerr('ERROR: Unexpected parse_type: {}'.format(parse_type))
            return value

    def update_parameter_tree(self, parameters_dict, agent_name, action_name):
        """
        Update visualization of parameter tree view.
        This function is called inside of self.update_command_widget()
        :param parameters_dict: {
            parameter_name1: {
                'value': 0.1,
                'data_type': float
            },
            ...
        }
        :return:
        """
        rospy.logdebug("Function MissionControlPlugin.update_parameter_tree is called")
        tree_widget = self._widget.commandWidget.findChild(QTreeWidget, 'parametersTreeWidget')

        # Disable update signals
        tree_widget.blockSignals(True)

        tree_widget.clear()
        # TODO: we may need to delete IntMarkerComboBox explicitly
        # for i in range(tree_widget.rowCount()):
        #     # Need to delete IntMarkerComboBox explicitly
        #     combo = tree_widget.cellWidget(i, 3)
        #     if combo is not None:
        #         combo.unregister()
        #         tree_widget.removeCellWidget(i, 3)
        # tree_widget.setRowCount(0)

        ########################################
        # Fill in parameter tree

        # Get config-related information
        parameter_groups = self.get_parameter_groups(agent_name, action_name)

        # create parameter tree widget and insert it
        # Append parameter once inserted into parameter tree
        # in order to process parameters that do not appear in parameter_groups
        parameters_inserted = []

        if parameter_groups is not None:

            for parameter_group in parameter_groups:
                mode = parameter_group['mode']
                group = parameter_group['group']

                if mode == 'build_up':
                    members = parameter_group['members']

                    # TODO: Currently, we implicitly assume that there is no nested group
                    # create top level tree widget item
                    item_top_level = QTreeWidgetItem()
                    # group name
                    item_top_level.setText(0, group)

                    for member in members:
                        # create and add parameter level tree widget item
                        item = self.create_parameter_tree_item_from_parameter(parameters_dict, member)
                        item_top_level.addChild(item)
                        parameters_inserted.append(member)

                    # insert top level item into tree widget
                    tree_widget.addTopLevelItem(item_top_level)

                elif mode == 'break_down':
                    t = parameter_group['type']
                    item_top_level = self.create_parameter_tree_item_break_down_mode(parameters_dict, group, t)
                    parameters_inserted.append(group)
                    tree_widget.addTopLevelItem(item_top_level)
                else:
                    rospy.logerr('ERROR: Unexpected parameter_group mode')

        # Insert parameters that do not appear in parameter_groups into tree widget
        for member in parameters_dict:
            if member not in parameters_inserted:
                item = self.create_parameter_tree_item_from_parameter(parameters_dict, member)
                tree_widget.addTopLevelItem(item)
                parameters_inserted.append(member)

        # interactive markers (only when the flag is set True)
        if self.use_interactive_markers:
            # Get config-related information
            int_marker_info_list = self.get_int_marker_info_list(agent_name, action_name)
            if int_marker_info_list is not None:
                for int_marker_info in int_marker_info_list:
                    name = int_marker_info['name']
                    int_marker_type = int_marker_info['type']
                    topics = int_marker_info['topics']
                    if int_marker_type == 'TASK_POSEDICT':
                        # special combobox with hard-coding
                        # actually, this is the same as the process of else: elif 'value' in int_marker_info
                        item = tree_widget.findItems(name, Qt.MatchExactly)[0]
                        marker_name = parameters_dict[name]['marker']
                        combo = self.IntMarkerComboBoxPoseDict(self, None, topics, marker_name, item, int_marker_type)
                        combo.set_debug_int_marker_name_label_signal.connect(self.set_debug_int_marker_name_label_signal_callback)
                        tree_widget.setItemWidget(item, 3, combo)
                    elif int_marker_type == 'TASK_LIST_POSEDICT':
                        # special combobox with hard-coding
                        item = tree_widget.findItems(name, Qt.MatchExactly)[0]
                        marker_name = parameters_dict[name]['marker']
                        combo = self.IntMarkerComboBoxListPoseDict(self, None, topics, marker_name, item, int_marker_type)
                        combo.set_debug_int_marker_name_label_signal.connect(self.set_debug_int_marker_name_label_signal_callback)
                        tree_widget.setItemWidget(item, 3, combo)
                        # set vertex name in column 3
                        child_count = item.childCount()
                        for child_index in range(child_count):
                            # Assume that poses are stored
                            item_child = item.child(child_index)
                            item_child.setData(3, Qt.DisplayRole, marker_name + ' (vertex {})'.format(child_index))
                    else:
                        # Check if this int_marker is assigned to single parameter or group of parameters
                        # If this marker is assigned to multiple parameters (group),
                        if 'members' in int_marker_info:
                            item = tree_widget.findItems(name, Qt.MatchExactly)[0]
                            marker_name = parameters_dict[int_marker_info['members'][0]['name']]['marker']
                            combo = self.IntMarkerComboBoxGroup(self, None, topics, marker_name, item, int_marker_type)
                            combo.set_debug_int_marker_name_label_signal.connect(self.set_debug_int_marker_name_label_signal_callback)
                            tree_widget.setItemWidget(item, 3, combo)
                        # Else if this marker is assigned to single parameter
                        elif 'value' in int_marker_info:
                            item = tree_widget.findItems(name, Qt.MatchExactly)[0]
                            marker_name = parameters_dict[name]['marker']
                            combo = self.IntMarkerComboBox(self, None, topics, marker_name, item, int_marker_type)
                            combo.set_debug_int_marker_name_label_signal.connect(self.set_debug_int_marker_name_label_signal_callback)
                            tree_widget.setItemWidget(item, 3, combo)
                        else:
                            rospy.logerr("ERROR: either 'members' or 'value' key is required for int_marker assignment!")

        # Expand all
        tree_widget.expandAll()

        # Enable update signals
        tree_widget.blockSignals(False)

    class IntMarkerComboBox(QComboBox):
        set_debug_int_marker_name_label_signal = pyqtSignal(str, str)  # ['debug', 'info', 'warn', 'error'], 'text to show'

        def __init__(self, mcg_instance, parent, topics, initial_marker_name, tree_widget_item, int_marker_type):
            # NOTE: mcg_instance is required only for
            #   - connecting signals from IntMarkerFullUpdateSubscriber
            rospy.logdebug("Function MissionControlPlugin.IntMarkerComboBox.__init__ is called")
            super(MissionControlPlugin.IntMarkerComboBox, self).__init__(parent)

            # Make combobox editable and set insert policy
            if int_marker_type == 'AGENT':
                self.setEditable(False)
            else:
                self.setEditable(True)
                self.setInsertPolicy(QComboBox.InsertAtCurrent)  # replace current one

            self.topics = topics  # type: list
            self.tree_widget_item = tree_widget_item
            self.initial_marker_name = initial_marker_name
            self.previous_marker_name = self.initial_marker_name  # this is updated after index changed or marker name edited

            # Get config files
            self.agent_library_dict = mcg_instance.agent_library_dict
            self.lib_action_parameter_groups_dict = mcg_instance.lib_action_parameter_groups_dict
            self.lib_action_int_marker_dict = mcg_instance.lib_action_int_marker_dict

            # Connect signal of IntMarkerUpdateFullSubscriber so that combobox can get updated list of int_markers
            for topic in self.topics:
                mcg_instance.int_marker_update_full_subscriber_dict[topic].int_marker_list_update_full_signal.connect(
                    self.int_marker_list_update_full_signal_callback)

            # Services for getting int_marker with topic
            self.get_int_marker_service_proxy_dict = mcg_instance.get_int_marker_service_proxy_dict

            # Services for renaming int marker
            self.rename_marker_service_proxy_dict = mcg_instance.rename_marker_service_proxy_dict

            # Initialize combobox selection
            self.marker_list = [self.initial_marker_name]
            self.addItems(self.marker_list)
            self.setCurrentIndex(0)
            self.topic_marker_names_dict = {}
            for topic in self.topics:
                self.topic_marker_names_dict[topic] = mcg_instance.int_marker_update_full_subscriber_dict[topic].int_marker_name_list
            self.update_marker_list()

            # Mutexes
            self.current_index_changed_callback_mutex = False
            self.edit_text_changed_callback_lock = RLock()

            # Members for int_markers
            self.INT_MARKER = InteractiveMarker()
            self.ROLL = 0.0
            self.PITCH = 0.0
            self.YAW = 0.0

            self.currentIndexChanged.connect(self.current_index_changed_callback)
            if int_marker_type != 'AGENT':
                self.lineEdit().editingFinished.connect(self.edit_text_changed_callback)
            self.feedback_publisher_dict = {}
            for topic in self.topics:
                feedback_publisher = rospy.Publisher(topic + '/feedback', InteractiveMarkerFeedback,
                                                     queue_size=10)
                self.feedback_publisher_dict[topic] = feedback_publisher

        def int_marker_list_update_full_signal_callback(self, topic_name, int_marker_name_list):
            self.topic_marker_names_dict[topic_name] = int_marker_name_list
            self.update_marker_list()

        def update_marker_list(self):
            self.blockSignals(True)
            # clear combobox
            current_text = self.currentText()
            self.marker_list = [""]
            self.clear()
            # update combobox
            for topic, marker_names in self.topic_marker_names_dict.iteritems():
                self.marker_list.extend(marker_names)
            self.addItems(self.marker_list)
            # reselect option
            try:
                self.setCurrentIndex(self.marker_list.index(current_text))
            except ValueError:
                rospy.logwarn("Corresponding parameter not found in combobox list.")
                pass
            self.blockSignals(False)

        def unregister(self):
            rospy.logdebug("Function MissionControlPlugin.IntMarkerComboBox.unregister is called")
            while self.update_full_mutex:
                pass
            for update_full_subscriber in self.update_full_subscriber_dict:
                update_full_subscriber.unregister()
            for feedback_publisher in self.feedback_publisher_dict:
                feedback_publisher.unregister()
            rospy.loginfo("All publishers/subscribers in IntMarkerComboBox with topic %s are unregistered." % self.topic)

        def update_tree_widget(self):
            # Function specific for updating tree widget
            # Override this function for different types of combobox

            # Get widget information
            tree_widget = self.tree_widget_item.treeWidget()
            agent_name = str(tree_widget.parent().findChild(QLabel, "agentNameVariableLabel").text())
            action_name = str(tree_widget.parent().findChild(QLabel, "actionNameVariableLabel").text())
            parameter_name = self.tree_widget_item.text(0)
            parameter_datatype = self.tree_widget_item.text(1)
            parameter_value_str = self.tree_widget_item.text(2)
            int_marker_name = str(self.currentText())

            # Get config information
            library = self.agent_library_dict[agent_name]
            int_marker_info_list = self.lib_action_int_marker_dict[library][action_name]

            # Update tree widget, which triggers update in mission file as well
            for int_marker_info in int_marker_info_list:
                # Check if parameter_name matches 'name' entry of config file
                if parameter_name != int_marker_info['name']:
                    continue
                # Check that this combobox is assigned to single parameter
                assert 'value' in int_marker_info

                # Get corresponding int_marker
                self.INT_MARKER = self.get_current_marker()
                euler = tf.transformations.euler_from_quaternion((self.INT_MARKER.pose.orientation.x,
                                                                  self.INT_MARKER.pose.orientation.y,
                                                                  self.INT_MARKER.pose.orientation.z,
                                                                  self.INT_MARKER.pose.orientation.w))
                self.ROLL = euler[0]
                self.PITCH = euler[1]
                self.YAW = euler[2]

                # Update value in tree item
                value = int_marker_info['value']
                # The following automatically trigger parameters_tree_item_changed_callback
                self.tree_widget_item.setData(2, Qt.EditRole, rgetattr(self, value))

        def get_topic(self, marker_name):
            for topic, marker_names in self.topic_marker_names_dict.iteritems():
                if marker_name in marker_names:
                    return topic
            else:
                return None

        def get_current_topic(self):
            topic = self.get_topic(str(self.currentText()))
            if topic:
                return topic
            else:
                rospy.loginfo('WARNING: current selected text does not exist in marker list')
                return None

        def get_current_marker(self):
            int_marker_name = str(self.currentText())
            current_topic = self.get_current_topic()
            if current_topic is None:
                return None
            int_marker = self.get_int_marker_service_proxy_dict[current_topic](int_marker_name).int_marker
            return int_marker

        def current_index_changed_callback(self, index):
            rospy.logdebug("Function MissionControlPlugin.IntMarkerComboBox.current_index_changed_callback is called")

            rospy.logdebug(
                "Function MissionControlPlugin.IntMarkerComboBox.current_index_changed_callback waits for mutex")
            while self.current_index_changed_callback_mutex:
                pass
            rospy.logdebug(
                "Function MissionControlPlugin.IntMarkerComboBox.current_index_changed_callback: Mutex acquired")
            self.current_index_changed_callback_mutex = True

            if not str(self.currentText()) == "":
                rospy.loginfo("Function MissionControlPlugin.IntMarkerComboBox.current_index_changed_callback: "
                              "combobox selection changed with valid option")
                self.update_tree_widget()
                self.previous_marker_name = str(self.currentText())  # update previous marker name after index changed
            else:
                rospy.logwarn("Function MissionControlPlugin.IntMarkerComboBox.current_index_changed_callback: "
                              "combobox selection changed but blank string selected")

            rospy.logdebug(
                "Function MissionControlPlugin.IntMarkerComboBox.current_index_changed_callback: Mutex unlocked")
            self.current_index_changed_callback_mutex = False

        def edit_text_changed_callback(self):
            text = self.currentText()
            with self.edit_text_changed_callback_lock:
                rospy.loginfo("New marker name: {}".format(text))
                rospy.loginfo("Previous marker name: {}".format(self.previous_marker_name))
                if self.get_current_topic():  # if a marker with the edited name already exists,
                    rospy.logdebug(self.get_current_topic())
                    self.set_debug_int_marker_name_label_signal.emit("info", "A marker with the same name is found!")
                    return
                else:  # if a marker is assigned with a new name,
                    self.set_debug_int_marker_name_label_signal.emit("info", "Marker name is changed!")
                    previous_topic = self.get_topic(self.previous_marker_name)
                    # get the previous marker
                    int_marker = self.get_int_marker_service_proxy_dict[previous_topic](self.previous_marker_name).int_marker
                    # control_visual = int_marker.controls[control_name_list.index('visualization_with_menu')]
                    # control_visual.markers[0].color
                    # rename it
                    int_marker.name = text
                    # update the marker with the new name in marker server
                    self.rename_marker_service_proxy_dict[previous_topic](previous_topic, self.previous_marker_name, text)
                    # updating marker name used in mission server should be triggered by marker servers
                    # update self.previous_marker_name
                    self.previous_marker_name = text


    class IntMarkerComboBoxGroup(IntMarkerComboBox):

        def __init__(self, mcg_instance, parent, topics, initial_marker_name, tree_widget_item, int_marker_type):
            super(MissionControlPlugin.IntMarkerComboBoxGroup, self).__init__(mcg_instance, parent, topics, initial_marker_name, tree_widget_item, int_marker_type)

        def update_tree_widget(self):
            # Function specific for updating tree widget
            # Override this function for different types of combobox

            # Get widget information
            tree_widget = self.tree_widget_item.treeWidget()
            agent_name = str(tree_widget.parent().findChild(QLabel, "agentNameVariableLabel").text())
            action_name = str(tree_widget.parent().findChild(QLabel, "actionNameVariableLabel").text())
            group_name = self.tree_widget_item.text(0)
            int_marker_name = str(self.currentText())

            # Get config information
            library = self.agent_library_dict[agent_name]
            int_marker_info_list = self.lib_action_int_marker_dict[library][action_name]

            # Update tree widget, which triggers update in mission file as well
            for int_marker_info in int_marker_info_list:
                # Check if parameter_name matches 'name' entry of config file
                if group_name != int_marker_info['name']:
                    continue
                # Check that this combobox is assigned to single parameter
                assert 'members' in int_marker_info
                members = int_marker_info['members']

                # Get corresponding int_marker
                self.INT_MARKER = self.get_current_marker()
                euler = tf.transformations.euler_from_quaternion((self.INT_MARKER.pose.orientation.x,
                                                                  self.INT_MARKER.pose.orientation.y,
                                                                  self.INT_MARKER.pose.orientation.z,
                                                                  self.INT_MARKER.pose.orientation.w))
                self.ROLL = euler[0]
                self.PITCH = euler[1]
                self.YAW = euler[2]

                # Update value in tree item
                child_count = self.tree_widget_item.childCount()
                for child_index in range(child_count):
                    item = self.tree_widget_item.child(child_index)
                    for member in members:
                        if item.text(0) != member['name']:
                            continue
                        value = member['value']
                        # The following automatically trigger parameters_tree_item_changed_callback
                        item.setData(2, Qt.EditRole, rgetattr(self, value))

    class IntMarkerComboBoxPoseDict(IntMarkerComboBox):

        def __init__(self, mcg_instance, parent, topics, initial_marker_name, tree_widget_item, int_marker_type):
            super(MissionControlPlugin.IntMarkerComboBoxPoseDict, self).__init__(mcg_instance, parent, topics, initial_marker_name, tree_widget_item, int_marker_type)

        def update_tree_widget(self):
            # Get widget information
            tree_widget = self.tree_widget_item.treeWidget()
            agent_name = str(tree_widget.parent().findChild(QLabel, "agentNameVariableLabel").text())
            action_name = str(tree_widget.parent().findChild(QLabel, "actionNameVariableLabel").text())
            group_name = self.tree_widget_item.text(0)
            int_marker_name = str(self.currentText())

            # Get config information
            library = self.agent_library_dict[agent_name]
            int_marker_info_list = self.lib_action_int_marker_dict[library][action_name]

            # Update tree widget, which triggers update in mission file as well
            for int_marker_info in int_marker_info_list:
                # Check if int_marker_type matches 'TASK_POSEDICT' or not
                if int_marker_info['type'] != 'TASK_POSEDICT':
                    continue

                # Get corresponding int_marker
                self.INT_MARKER = self.get_current_marker()
                euler = tf.transformations.euler_from_quaternion((self.INT_MARKER.pose.orientation.x,
                                                                  self.INT_MARKER.pose.orientation.y,
                                                                  self.INT_MARKER.pose.orientation.z,
                                                                  self.INT_MARKER.pose.orientation.w))
                self.ROLL = euler[0]
                self.PITCH = euler[1]
                self.YAW = euler[2]

                # Update value in tree item
                child_count = self.tree_widget_item.childCount()
                for child_index in range(child_count):
                    item = self.tree_widget_item.child(child_index)
                    for key_name in ['x', 'y', 'z', 'roll', 'pitch', 'yaw']:
                        if item.text(0) != key_name:
                            continue
                        if key_name == 'x' or key_name == 'y' or key_name == 'z':
                            marker_attr = 'INT_MARKER.pose.position.' + key_name
                        elif key_name.upper() in ['YAW', 'PITCH', 'ROLL']:
                            marker_attr = key_name.upper()
                        else:
                            rospy.logerr("unexpected sequence")
                            continue
                        # The following automatically trigger parameters_tree_item_changed_callback
                        item.setData(2, Qt.EditRole, rgetattr(self, marker_attr))

    class IntMarkerComboBoxListPoseDict(IntMarkerComboBox):

        def __init__(self, mcg_instance, parent, topics, initial_marker_name, tree_widget_item, int_marker_type):
            super(MissionControlPlugin.IntMarkerComboBoxListPoseDict, self).__init__(mcg_instance, parent, topics, initial_marker_name, tree_widget_item, int_marker_type)

            # Additional logic for vertices
            self.get_int_marker_list_service_proxy_dict = mcg_instance.get_int_marker_list_service_proxy_dict
            self.vertex_topic_marker_names_dict = {}
            self.vertex_feedback_publisher_dict = {}

            for topic in self.topics:
                topic_name = topic + '_vertices'  # Implicit assumption here

                # Initialization
                self.vertex_topic_marker_names_dict[topic_name] = mcg_instance.int_marker_update_full_subscriber_dict[topic_name].int_marker_name_list

                # Connect signal of IntMarkerUpdateFullSubscriber so that combobox can get updated list of int_markers
                mcg_instance.int_marker_update_full_subscriber_dict[
                    topic_name].int_marker_list_update_full_signal.connect(
                    self.vertex_int_marker_list_update_full_signal_callback)

                # Feedback publisher
                feedback_publisher = rospy.Publisher(topic_name + '/feedback', InteractiveMarkerFeedback,
                                                     queue_size=10)
                self.vertex_feedback_publisher_dict[topic_name] = feedback_publisher

        def vertex_int_marker_list_update_full_signal_callback(self, topic_name, int_marker_name_list):
            self.vertex_topic_marker_names_dict[topic_name] = int_marker_name_list

        def get_current_vertex_topic(self):
            current_topic = self.get_current_topic()
            if current_topic is None:
                return None
            return current_topic + '_vertices'  # Implicit assumption here

        def get_current_vertex_markers(self):
            current_vertex_topic = self.get_current_vertex_topic()
            center_int_marker_name = str(self.currentText())
            int_marker_current_list = self.get_int_marker_list_service_proxy_dict[current_vertex_topic](center_int_marker_name).int_marker_vertices
            # sort vertex marker list by name
            int_marker_current_list.sort(key=lambda x: x.name)
            return int_marker_current_list

        def update_tree_widget(self):
            # Function specific for updating tree widget
            # Override this function for different types of combobox

            # Get widget information
            tree_widget = self.tree_widget_item.treeWidget()
            agent_name = str(tree_widget.parent().findChild(QLabel, "agentNameVariableLabel").text())
            action_name = str(tree_widget.parent().findChild(QLabel, "actionNameVariableLabel").text())
            param_name = self.tree_widget_item.text(0)
            int_marker_name = str(self.currentText())

            # Get config information
            library = self.agent_library_dict[agent_name]
            int_marker_info_list = self.lib_action_int_marker_dict[library][action_name]

            # Block signal from tree_widget
            tree_widget.blockSignals(True)

            # Update tree widget, which triggers update in mission file as well
            for int_marker_info in int_marker_info_list:
                # Check if int_marker_type matches 'TASK_LIST_POSEDICT' or not
                if int_marker_info['type'] != 'TASK_LIST_POSEDICT':
                    continue
                # Get corresponding list of vertex int_marker
                vertex_int_marker_list = self.get_current_vertex_markers()

                # Assert the number of vertices and the number of children matches.
                # TODO: This is achieved by...
                try:
                    assert len(vertex_int_marker_list) == self.tree_widget_item.childCount()
                except AssertionError:
                    rospy.logerr('ERROR: number of vertex and number of children of this item do not match.')

                # Update value in tree item
                child_count = self.tree_widget_item.childCount()
                for child_index in range(child_count):
                    vertex_int_marker = vertex_int_marker_list[child_index]
                    x = vertex_int_marker.pose.position.x
                    y = vertex_int_marker.pose.position.y
                    z = vertex_int_marker.pose.position.z
                    euler = tf.transformations.euler_from_quaternion((vertex_int_marker.pose.orientation.x,
                                                                      vertex_int_marker.pose.orientation.y,
                                                                      vertex_int_marker.pose.orientation.z,
                                                                      vertex_int_marker.pose.orientation.w))
                    roll = euler[0]
                    pitch = euler[1]
                    yaw = euler[2]

                    item_second_level = self.tree_widget_item.child(child_index)

                    # item_second_level.setData(0, Qt.DisplayRole, vertex_int_marker.name)
                    item_second_level.setData(3, Qt.DisplayRole, vertex_int_marker.name)

                    ground_child_count = item_second_level.childCount()
                    for ground_child_index in range(ground_child_count):
                        item_parameter_level = item_second_level.child(ground_child_index)
                        if item_parameter_level.text(0) == 'x':
                            item_parameter_level.setData(2, Qt.EditRole, x)
                        elif item_parameter_level.text(0) == 'y':
                            item_parameter_level.setData(2, Qt.EditRole, y)
                        elif item_parameter_level.text(0) == 'z':
                            item_parameter_level.setData(2, Qt.EditRole, z)
                        elif item_parameter_level.text(0) == 'roll':
                            item_parameter_level.setData(2, Qt.EditRole, roll)
                        elif item_parameter_level.text(0) == 'pitch':
                            item_parameter_level.setData(2, Qt.EditRole, pitch)
                        elif item_parameter_level.text(0) == 'yaw':
                            item_parameter_level.setData(2, Qt.EditRole, yaw)

            # Unblock signal from tree_widget
            tree_widget.blockSignals(False)
            # Manually emit signal that tree_widget is modified
            tree_widget.itemChanged.emit(self.tree_widget_item, 3)

    def parameters_tree_item_changed_callback(self, tree_widget_item):
        """
        Callback function for changing values in parametersTreeWidget.
        This function is called when
            1. New TreeWidgetItem is added to parametersTreeWidget
            2. Parameters are edited by entering a new value to parametersTreeWidget
            (NOTE: This function is NOT called when IntMarkerComboBox is added to parametersTableWidget)
        However, the current implementation does not let parametersTreeWidget to emit signal in case 1.
        Thus, this function is currently only called in case 2.
        This function is used for
            - Updating interactive markers.
            - Updating mission data instantly after parameters are changed.
        """
        rospy.logdebug("Function MissionControlPlugin.parameters_tree_item_changed_callback is called")
        rospy.loginfo("Parameter Tree is edited!")

        agent_name = str(
            self._widget.commandWidget.findChild(QLabel, "agentNameVariableLabel").text())
        action_name = str(
            self._widget.commandWidget.findChild(QLabel, "actionNameVariableLabel").text())
        tree_widget = tree_widget_item.treeWidget()

        # Interactive marker interactions
        # Interactive markers are dealt with only when the flag is set True
        if self.use_interactive_markers:
            # First, read config information
            library = self.agent_library_dict[agent_name]
            try:
                int_marker_info_list = self.lib_action_int_marker_dict[library][action_name]
            except KeyError:
                # Early return because this action is not parametrized by int_markers
                self.send_edited_action_to_server(tree_widget_item)
                return

            # TODO: fix this for loop logic! This code does not fully consider the situation
            #       where multiple parameters are used for a single skill!
            # Choose int_marker_info in which tree_widget_item is used
            for int_marker_info in int_marker_info_list:
                int_marker_type = int_marker_info['type']
                # Check interactive_marker type for this action
                if int_marker_type == 'TASK_POSEDICT':
                    key_name = tree_widget_item.text(0)
                    if key_name == 'x' or key_name == 'y' or key_name == 'z':
                        marker_attr = 'INT_MARKER.pose.position.' + key_name
                    elif key_name.upper() in ['YAW', 'PITCH', 'ROLL']:
                        marker_attr = key_name.upper()
                    else:
                        continue
                    # datatype = tree_widget_item.text(1)
                    new_value = float(tree_widget_item.text(2))
                    item_top_level = tree_widget_item.parent()
                    combo = tree_widget.itemWidget(item_top_level, 3)
                    self.publish_int_marker_feedback(combo, marker_attr, new_value, tree_widget_item,
                                                     InteractiveMarkerFeedback.POSE_UPDATE)
                    break
                elif int_marker_type == 'TASK_LIST_POSEDICT':
                    # Case 1: modified parameter is one of the element which composes TASK_LIST_POSEDICT int_marker
                    # Hard coding required for TASK_LIST_POSEDICT
                    # Check if the modified tree_widget_item is top_level or parameter_level
                    # If tree_widget_item is top_level, it has multiple children
                    # whose name starts with 'pose' (implicit assumption).
                    # If tree_widget_item is paramter_level, its value at column 0 should be
                    # one of ['x', 'y', 'z', 'roll', 'pitch', 'yaw']
                    # and its grand father has combobox at column 3 (implicit assumption).
                    is_top_level = False
                    is_parameter_level = False
                    child_count = tree_widget_item.childCount()
                    if child_count > 0:
                        is_name_startswith_pose = []
                        for child_index in range(child_count):
                            item_second_level = tree_widget_item.child(child_index)
                            is_name_startswith_pose.append(str(item_second_level.text(0)).startswith('pose'))
                        is_top_level = all(is_name_startswith_pose)
                    else:
                        if str(tree_widget_item.text(0)) in ['x', 'y', 'z', 'roll', 'pitch', 'yaw']:
                            item_top_level = tree_widget_item.parent().parent()
                            if item_top_level is not None:
                                combo = tree_widget.itemWidget(item_top_level, 3)
                                if combo is not None:
                                    is_parameter_level = True

                    # If is_top_level,
                    if is_top_level:
                        combo = tree_widget.itemWidget(tree_widget_item, 3)  # IntMarkerComboBox
                        center_marker_name = combo.currentText()
                        vertex_marker_name_dict = {}  # Once store values because item_second_level could be updated by self.publish_int_marker_area_feedback
                        for child_index in range(child_count):
                            item_second_level = tree_widget_item.child(child_index)
                            vertex_marker_name = str(item_second_level.text(3))
                            vertex_marker_name_dict[child_index] = vertex_marker_name
                        for child_index, vertex_marker_name in vertex_marker_name_dict.iteritems():
                            item_second_level = tree_widget_item.child(child_index)
                            # self.publish_int_marker_area_feedback(combo, vertex_marker_name, item_second_level, InteractiveMarkerFeedback.POSE_UPDATE)
                        break
                    # If is_parameter_level
                    if is_parameter_level:
                        # Publish feedback on this vertex
                        item_second_level = tree_widget_item.parent()
                        vertex_marker_name = str(item_second_level.text(3))
                        # First, block signals for tree_widget and update geometry of int_marker safely
                        tree_widget.blockSignals(True)
                        self.publish_int_marker_area_feedback(combo, vertex_marker_name, item_second_level, InteractiveMarkerFeedback.POSE_UPDATE)
                        tree_widget.blockSignals(False)
                        # Then, publish MOUSE_UP in order to update values of different vertices used in parameter
                        self.publish_int_marker_area_feedback(combo, vertex_marker_name, item_second_level, InteractiveMarkerFeedback.MOUSE_UP)
                        # NOTE: In this case, updated parameters are sent to Mission Server by
                        # self.int_marker_feedback_callback(), so we do early return without
                        # self.send_edited_action_to_server.
                        return

                else:
                    if 'members' in int_marker_info:
                        members = int_marker_info['members']
                        param_attr_dict = {}
                        for member in members:
                            param_attr_dict[member['name']] = member['value']
                        if tree_widget_item.text(0) in param_attr_dict:
                            # Case 2: modified parameter is one of the element which composes build_up mode
                            # Get element in tree_widget_item
                            datatype = tree_widget_item.text(1)
                            new_value = tree_widget_item.text(2)
                            # TODO: currently assume that build_up has no nested structure
                            item_top_level = tree_widget_item.parent()
                            combo = tree_widget.itemWidget(item_top_level, 3)  # IntMarkerComboBoxGroup

                            # Cast read new_value into correct type
                            if datatype == 'str':
                                new_value = str(new_value)
                            elif datatype == 'float':
                                new_value = float(new_value)
                            elif datatype == 'int':
                                new_value = int(new_value)
                            elif datatype == 'bool':
                                new_value = bool(new_value)
                                # TODO: Add the flow for the rest of data types
                            else:
                                rospy.logerr(
                                    "Data Type %s is not implemented in parameters_table_cell_changed_callback()"
                                    % datatype)
                                # Early return
                                self.send_edited_action_to_server(tree_widget_item)
                                return

                            # Feedback logic from here
                            marker_attr = param_attr_dict[tree_widget_item.text(0)]
                            self.publish_int_marker_feedback(combo, marker_attr, new_value, tree_widget_item,
                                                             InteractiveMarkerFeedback.POSE_UPDATE)
                            break

                    elif 'value' in int_marker_info:
                        if tree_widget_item.text(0) == int_marker_info['name']:
                            # Case 3: modified parameter is assigned to single parameter
                            # Get element in tree_widget_item
                            datatype = tree_widget_item.text(1)
                            new_value = tree_widget_item.text(2)
                            combo = tree_widget.itemWidget(tree_widget_item, 3)  # IntMarkerComboBox

                            # Cast read new_value into correct type
                            if datatype == 'str':
                                new_value = str(new_value)
                            elif datatype == 'float':
                                new_value = float(new_value)
                            elif datatype == 'int':
                                new_value = int(new_value)
                            elif datatype == 'bool':
                                new_value = bool(new_value)
                                # TODO: Add the flow for the rest of data types
                            else:
                                rospy.logerr("Data Type %s is not implemented in parameters_table_cell_changed_callback()"
                                             % datatype)
                                # Early return
                                self.send_edited_action_to_server(tree_widget_item)
                                return

                            # Feedback logic from here
                            marker_attr = int_marker_info['value']
                            self.publish_int_marker_feedback(combo, marker_attr, new_value, tree_widget_item,
                                                             InteractiveMarkerFeedback.MOUSE_UP)
                            break
                    else:
                        rospy.logerr("ERROR: either 'members' or 'value' key is required for int_marker assignment!")

        # Send edited action to Mission Server
        self.send_edited_action_to_server(tree_widget_item)

    def publish_int_marker_feedback(self, combo, marker_attr, new_value, tree_widget_item, event_type):
        assert event_type == InteractiveMarkerFeedback.POSE_UPDATE or event_type == InteractiveMarkerFeedback.MOUSE_UP
        # Get int_marker in current selection
        self.INT_MARKER = combo.get_current_marker()
        euler = tf.transformations.euler_from_quaternion((self.INT_MARKER.pose.orientation.x,
                                                          self.INT_MARKER.pose.orientation.y,
                                                          self.INT_MARKER.pose.orientation.z,
                                                          self.INT_MARKER.pose.orientation.w,))
        self.ROLL = euler[0]
        self.PITCH = euler[1]
        self.YAW = euler[2]

        # Case: Pose or ROLL, PITCH, YAW
        if marker_attr.startswith('INT_MARKER.pose') or marker_attr in ['ROLL', 'PITCH', 'YAW']:
            # Fill in common values
            feedback = InteractiveMarkerFeedback()
            feedback.header.frame_id = self.frame_id
            feedback.client_id = self.process_name
            feedback.marker_name = str(combo.currentText())
            feedback.control_name = ''
            feedback.event_type = event_type
            # Update value in pose
            rsetattr(self, marker_attr, new_value)
            # Update value in pose.orientation
            quaternion = tf.transformations.quaternion_from_euler(self.ROLL, self.PITCH, self.YAW)
            self.INT_MARKER.pose.orientation.x = quaternion[0]
            self.INT_MARKER.pose.orientation.y = quaternion[1]
            self.INT_MARKER.pose.orientation.z = quaternion[2]
            self.INT_MARKER.pose.orientation.w = quaternion[3]
            feedback.pose = self.INT_MARKER.pose
            current_topic = combo.get_current_topic()
            if current_topic is None:
                return
            combo.feedback_publisher_dict[current_topic].publish(feedback)
            rospy.loginfo("Published pose feedback to interactive marker")

        # Case: name
        elif marker_attr.startswith('INT_MARKER.name'):
            # Change the current selection in combo box
            # If the requested name is not found in combo box, discard changes
            if new_value in combo.marker_list:
                combo.setCurrentIndex(combo.findText(new_value))
                rospy.loginfo("Modified current selection in marker combo box")
            else:
                tree_widget_item.setText(2, combo.currentText())
        else:
            rospy.logerr('ERROR: feedback process for int_marker is not implemented when tree is modified')

    def publish_int_marker_area_feedback(self, combo, vertex_marker_name, item_second_level, event_type):
        assert event_type == InteractiveMarkerFeedback.POSE_UPDATE or event_type == InteractiveMarkerFeedback.MOUSE_UP
        # Hard-coded logic for publishing feedback to area markers
        # Fill in common values
        feedback = InteractiveMarkerFeedback()
        feedback.header.frame_id = self.frame_id
        feedback.client_id = self.process_name
        feedback.marker_name = vertex_marker_name
        feedback.control_name = ''
        feedback.event_type = event_type
        # Update value in pose
        child_count = item_second_level.childCount()
        x = 0.
        y = 0.
        z = 0.
        roll = 0.
        pitch = 0.
        yaw = 0.
        for child_index in range(child_count):
            item_parameter_level = item_second_level.child(child_index)
            if item_parameter_level.text(0) == 'x':
                x = float(item_parameter_level.text(2))
            elif item_parameter_level.text(0) == 'y':
                y = float(item_parameter_level.text(2))
            elif item_parameter_level.text(0) == 'z':
                z = float(item_parameter_level.text(2))
            elif item_parameter_level.text(0) == 'roll':
                roll = float(item_parameter_level.text(2))
            elif item_parameter_level.text(0) == 'pitch':
                pitch = float(item_parameter_level.text(2))
            elif item_parameter_level.text(0) == 'yaw':
                yaw = float(item_parameter_level.text(2))
        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        feedback.pose.position.x = x
        feedback.pose.position.y = y
        feedback.pose.position.z = z
        feedback.pose.orientation.x = quaternion[0]
        feedback.pose.orientation.y = quaternion[1]
        feedback.pose.orientation.z = quaternion[2]
        feedback.pose.orientation.w = quaternion[3]
        current_topic = combo.get_current_topic()
        if current_topic is None:
            return
        combo.vertex_feedback_publisher_dict[current_topic + '_vertices'].publish(feedback)  # Implicit assumption
        rospy.loginfo("Published pose feedback to interactive marker")

    def runButtonCallback(self):
        """
        Callback function for clicking "Run" button.
        This function virtually clicks all "Run" buttons in ScheduleWidgets.
        :return:
        """
        rospy.logdebug("Function MissionControlPlugin.runButtonCallback is called")
        for schedule_widget in self._widget.scrollAreaWidgetContents.findChildren(ScheduleWidget):
            schedule_widget.findChild(QPushButton, 'runButton').click()  # TODO: USE SIGNAL-SLOT!!

    def pauseButtonCallback(self):
        """
        Callback function for clicking "Pause" button.
        This function virtually clicks all "Pause" buttons in ScheduleWidgets.
        :return:
        """
        rospy.logdebug("Function MissionControlPlugin.pauseButtonCallback is called")
        for schedule_widget in self._widget.scrollAreaWidgetContents.findChildren(ScheduleWidget):
            schedule_widget.findChild(QPushButton, 'pauseButton').click()  # TODO: USE SIGNAL-SLOT!!

    def stopButtonCallback(self):
        """
        Callback function for clicking "Stop" button.
        This function virtually clicks all "Stop" buttons in ScheduleWidgets.
        :return:
        """
        rospy.logdebug("Function MissionControlPlugin.stopButtonCallback is called")
        for schedule_widget in self._widget.scrollAreaWidgetContents.findChildren(ScheduleWidget):
            schedule_widget.findChild(QPushButton, 'stopButton').click()  # TODO: USE SIGNAL-SLOT!!

    def resetButtonCallback(self):
        """
        Callback function for clicking "reset" button.
        This function virtually clicks all "reset" buttons in ScheduleWidgets.
        :return:
        """
        rospy.logdebug("Function MissionControlPlugin.resetButtonCallback is called")
        for schedule_widget in self._widget.scrollAreaWidgetContents.findChildren(ScheduleWidget):
            schedule_widget.findChild(QPushButton, 'resetButton').click()  # TODO: USE SIGNAL-SLOT!!

    def new_mission(self):
        rospy.logdebug("Function MissionControlPlugin.new_mission is called")
        res = self.new_mission_service_proxy()
        show_error_msg(res.error_msg)
        self.save_path = None
        rospy.loginfo("Created new mission data")
        self.clear_command_widget()

    def newButtonCallback(self):
        rospy.logdebug("Function MissionControlPlugin.newButtonCallback is called")
        # Check if the current self.mission_req is the same as the one saved at last time
        # If there is lastly saved file,
        if self.save_path is not None:
            with open(self.save_path + '/mission.pkl', 'rb') as stream:
                last_save_mission_req = pickle.load(stream)
                # If the latest mission data is the same as the one saved last time, create new data
                if self.is_mission_same(self.mission_req, last_save_mission_req):
                    self.new_mission()
                # Otherwise, show a warning message box
                else:
                    msg = QMessageBox()
                    msg.setIcon(QMessageBox.Warning)
                    msg.setText("Do you want to save the changes made to the mission data before creating new one?")
                    msg.setInformativeText("Your changes will be lost if you don't save them.")
                    msg.setStandardButtons(QMessageBox.Yes | QMessageBox.No | QMessageBox.Cancel)
                    retval = msg.exec_()
                    if retval == QMessageBox.Yes:
                        self.saveButtonCallback()
                        self.new_mission()
                    elif retval == QMessageBox.No:
                        self.new_mission()
                    else:
                        pass
        else:
            self.new_mission()

    def saveButtonCallback(self):
        rospy.logdebug("Function MissionControlPlugin.saveButtonCallback is called")
        if self.save_path is not None:
            self.save_mission_service_proxy(self.save_path)
            rospy.loginfo("Saved mission data to the latest path %s" % self.save_path.encode('utf-8'))
        else:
            self.saveasButtonCallback()

    def saveasButtonCallback(self):
        rospy.logdebug("Function MissionControlPlugin.saveasButtonCallback is called")
        dialog = QFileDialog()
        dialog.setWindowTitle('Select a folder to save Mission Data')
        dialog.setFilter(dialog.filter())
        dialog.setFileMode(QFileDialog.Directory)
        dialog.setAcceptMode(QFileDialog.AcceptSave)
        if dialog.exec_() == QFileDialog.Accepted:
            folder_path = dialog.selectedFiles()[0]
            req = SaveMissionRequest()
            req.folder_path = folder_path
            self.save_mission_service_proxy(req)
            self.save_path = folder_path
            rospy.loginfo("Saved mission data to %s" % self.save_path.encode('utf-8'))
        else:
            rospy.loginfo("Canceled saving mission data")

    def load_mission(self):
        rospy.logdebug("Function MissionControlPlugin.load_mission is called")
        dialog = QFileDialog()
        dialog.setWindowTitle('Select a folder to load Mission Data')
        dialog.setFilter(dialog.filter())
        dialog.setFileMode(QFileDialog.Directory)
        dialog.setAcceptMode(QFileDialog.AcceptOpen)
        if dialog.exec_() == QFileDialog.Accepted:
            folder_path = dialog.selectedFiles()[0]
            req = LoadMissionRequest()
            req.folder_path = folder_path
            res = self.load_mission_service_proxy(req)
            show_error_msg(res.error_msg)
            self.save_path = folder_path
            rospy.loginfo("Loaded mission data from %s" % self.save_path.encode('utf-8'))
            self.clear_command_widget()
        else:
            rospy.loginfo("Canceled loading mission data")

    def loadButtonCallback(self):
        rospy.logdebug("Function MissionControlPlugin.loadButtonCallback is called")
        # Check if the current self.mission_req is the same as the one saved at last time.
        # TODO: How to check the existing agent and available actions?
        # If there is lastly saved file,
        if self.save_path is not None:
            with open(self.save_path + '/mission.pkl', 'rb') as stream:
                last_save_mission_req = pickle.load(stream)
                # If the latest mission data is the same as the one saved last time, load data
                if self.is_mission_same(self.mission_req, last_save_mission_req):
                    self.load_mission()
                # Otherwise, show a warning message box
                else:
                    msg = QMessageBox()
                    msg.setIcon(QMessageBox.Warning)
                    msg.setText("Do you want to save the changes made to the mission data before loading one?")
                    msg.setInformativeText("Your changes will be lost if you don't save them.")
                    msg.setStandardButtons(QMessageBox.Yes | QMessageBox.No | QMessageBox.Cancel)
                    retval = msg.exec_()
                    if retval == QMessageBox.Yes:
                        self.saveButtonCallback()
                        self.load_mission()
                    elif retval == QMessageBox.No:
                        self.load_mission()
                    else:
                        pass
        else:
            self.load_mission()

    def set_debug_int_marker_name_label_signal_callback(self, mode, text):
        label = self._widget.commandWidget.findChild(QLabel, "debugIntMarkerNameLabel")
        label.setText(text)
        if mode == 'debug':
            label.setStyleSheet('color: white')
            rospy.logdebug(text)
        elif mode == 'info':
            label.setStyleSheet('color: white')
            rospy.loginfo(text)
        elif mode == 'warn':
            label.setStyleSheet('color: yellow')
            rospy.logwarn(text)
        elif mode =='error':
            label.setStyleSheet('color: red')
            rospy.logerr(text)
        else:
            rospy.logerr("ERROR: mode {} is not implemented in set_debug_int_marker_name_label_signal_callback")

    def send_edited_action_to_server(self, tree_widget_item):
        rospy.logdebug("Function MissionControlPlugin.send_edited_action_to_server is called")
        if self.selected_action is not None:
            edited_action = copy.deepcopy(self.selected_action)  # NOTE: this needs to be copy; otherwise reference is sent to Mission Sever as well as to this GUI.
            action_name = str(self._widget.commandWidget.findChild(QLabel, "actionNameVariableLabel").text())
            action_label = str(self._widget.commandWidget.findChild(QLineEdit, "actionLabelVariableLineEdit").text())
            edited_action.action_content.action_label = action_label

            # Read reactive behaviors
            edited_action.action_content.battery_flag = self._widget.commandWidget.findChild(QCheckBox, "batteryCheckBox").isChecked()
            edited_action.action_content.wlan_flag = self._widget.commandWidget.findChild(QCheckBox, "wlanCheckBox").isChecked()
            edited_action.action_content.manual_flag = self._widget.commandWidget.findChild(QCheckBox, "manualCheckBox").isChecked()

            # Update parameter_yaml if tree_widget_item is passed
            if tree_widget_item is not None:
                # Load parameters from action before update
                parameters_dict = yaml.safe_load(edited_action.action_content.parameters_yaml)

                # Get config-related information
                parameter_groups = self.get_parameter_groups(self.selected_action_agent, action_name)

                # Get parameter tree
                tree_widget = tree_widget_item.treeWidget()

                # Append parameter once read from parameter tree
                # in order to process parameters that do not appear in parameter_groups
                parameters_read = []

                if parameter_groups is not None:
                    for parameter_group in parameter_groups:
                        mode = parameter_group['mode']
                        group = parameter_group['group']

                        if mode == 'build_up':
                            item_top_level = tree_widget.findItems(group, Qt.MatchExactly)[0]
                            child_count = item_top_level.childCount()
                            # TODO: Currently, we implicitly assume that there is no nested group
                            for child_index in range(child_count):
                                item = item_top_level.child(child_index)
                                item_name, item_datatype, item_value = self.read_parameter_tree_item(item)
                                parameters_dict[item_name]['value'] = item_value
                                parameters_read.append(item_name)

                            # Interactive markers (only when the flag is set True)
                            if self.use_interactive_markers:
                                item_int_marker_combobox = tree_widget.itemWidget(item_top_level, 3)
                                if item_int_marker_combobox is not None:
                                    item_int_marker_name = str(item_int_marker_combobox.currentText())
                                    item_int_marker_topic = item_int_marker_combobox.get_current_topic()
                                    for child_index in range(child_count):
                                        item = item_top_level.child(child_index)
                                        item_name, item_datatype, item_value = self.read_parameter_tree_item(item)
                                        parameters_dict[item_name]['marker'] = item_int_marker_name
                                        parameters_dict[item_name]['marker_topic'] = item_int_marker_topic

                        elif mode == 'break_down':
                            t = parameter_group['type']
                            item_top_level = tree_widget.findItems(group, Qt.MatchExactly)[0]
                            item_value = self.read_parameter_tree_item_break_down_mode(item_top_level, t)
                            parameters_dict[group]['value'] = item_value
                            parameters_read.append(group)

                            # Interactive markers (only when the flag is set True)
                            if self.use_interactive_markers:
                                # Hard-coded logic for break_down mode
                                item_int_marker_combobox = tree_widget.itemWidget(item_top_level, 3)
                                if item_int_marker_combobox is not None:
                                    item_int_marker_name = str(item_int_marker_combobox.currentText())
                                    item_int_marker_topic = item_int_marker_combobox.get_current_topic()
                                    parameters_dict[group]['marker'] = item_int_marker_name
                                    parameters_dict[group]['marker_topic'] = item_int_marker_topic

                        else:
                            rospy.logerr('ERROR: Unexpected parameter_group mode')

                # Read parameters that do not appear in parameter_groups into tree widget
                for member in parameters_dict:
                    if member not in parameters_read:
                        item = tree_widget.findItems(member, Qt.MatchExactly)[0]
                        item_name, item_datatype, item_value = self.read_parameter_tree_item(item)
                        parameters_dict[item_name]['value'] = item_value
                        parameters_read.append(member)

                        # Interactive markers (only when the flag is set True)
                        if self.use_interactive_markers:
                            item_int_marker_combobox = tree_widget.itemWidget(item, 3)
                            if item_int_marker_combobox is not None:
                                item_int_marker_name = str(item_int_marker_combobox.currentText())
                                item_int_marker_topic = item_int_marker_combobox.get_current_topic()
                                parameters_dict[item_name]['marker'] = item_int_marker_name
                                parameters_dict[item_name]['marker_topic'] = item_int_marker_topic

                edited_action.action_content.parameters_yaml = yaml.dump(parameters_dict)

            res = self.edit_action_service_proxy(edited_action)
            show_error_msg(res.error_msg)
            rospy.loginfo("Applied changes to action (id: %s)" % edited_action.action_id)
        else:
            rospy.logwarn("No action is selected.")

    def is_mission_same(self, new_mission, old_mission):
        """
        Check if new_mission is the same mission as old_mission.
        This function takes it into consideration that action_id could be different when the mission is loaded.
        :param new_mission:
        :param old_mission:
        :return:
        """
        rospy.logdebug("Function MissionControlPlugin.is_mission_same is called")
        if new_mission == old_mission:
            return True
        else:
            # Check actions
            is_same = len(new_mission.agents_actions) == len(old_mission.agents_actions)
            if is_same:
                for new_agent_actions, old_agent_actions in zip(new_mission.agents_actions, old_mission.agents_actions):
                    is_same = is_same and new_agent_actions.agent_name == old_agent_actions.agent_name
                    is_same = is_same and len(new_agent_actions.actions) == len(old_agent_actions.actions)
                    if is_same:
                        for new_action, old_action in zip(new_agent_actions.actions, old_agent_actions.actions):
                            # do not check id
                            is_same = is_same and new_action.action_content == old_action.action_content
            # Check synchronization points
            is_same = is_same and len(new_mission.sync_ids_list) == len(old_mission.sync_ids_list)
            if is_same:
                for new_sync_ids, old_sync_ids in zip(new_mission.sync_ids_list, old_mission.sync_ids_list):
                    # FIXME: currently, just check the number of each synchronization point is the same or not.
                    #  Therefore, this function could misjudge that the new_mission is the same as the old_mission
                    #  if you do not modify the number of synchronization points but modified the actions to be
                    #  synchronized.
                    is_same = is_same and len(new_sync_ids.sync_ids) == len(old_sync_ids.sync_ids)
            return is_same

    def handle_send_mission(self, req):
        """
        Handle service call from MissionServer that send updated mission data.
        This function updates visualization of this GUI.
        :param req: MissionRequest()
        :return: MissionResponse()
        """
        rospy.logdebug("Function MissionControlPlugin.handle_send_mission is called")
        self.mission_req = req

        # If this function is called before initialization of ScheduleWidget
        if len(self._widget.scrollAreaWidgetContents.findChildren(ScheduleWidget)) == 0:
            rospy.loginfo("Received mission only for initialization. Early return.\nReceived mission is:{}".format(req))
            return MissionResponse()

        # Update ScheduleWidget of each agent
        rospy.logdebug("Function MissionControlPlugin.handle_send_mission: send update_visualization_signal")
        self.update_visualization_signal.emit(self.mission_req)

        # If we have selected one of the actions before receiving mission_req, re-select the action
        if self.selected_action is not None and self.selected_action_agent is not None:
            schedule_widget = self._widget.scrollAreaWidgetContents.findChild(ScheduleWidget,
                                                                              self.selected_action_agent)
            action_table_views = schedule_widget.findChildren(ActionTableView)
            for action_table_view in action_table_views:
                action_id_list = []
                id_column = action_table_view.get_column_from_label('id')
                for row in range(action_table_view.model.rowCount()):
                    id = str(action_table_view.model.index(row, id_column).data())
                    action_id_list.append(id)
                if self.selected_action.action_id in action_id_list:
                    row = action_id_list.index(self.selected_action.action_id)
                    index = action_table_view.model.index(row, 0)
                    self.action_table_item_reselection_signal.emit(self.selected_action_agent, action_table_view.objectName(), index)
                    rospy.loginfo("Re-selected action %s" % self.selected_action.action_id)
                    break

        return MissionResponse()

    def get_parameter_groups(self, agent_name, action_name):
        # Get config-related information
        library = self.agent_library_dict[agent_name]
        try:
            parameter_groups = self.lib_action_parameter_groups_dict[library][action_name]
        except KeyError:
            parameter_groups = None
        return parameter_groups

    def get_int_marker_info_list(self, agent_name, action_name):
        # Get config-related information of int_markers
        library = self.agent_library_dict[agent_name]
        try:
            int_marker_info_list = self.lib_action_int_marker_dict[library][action_name]
        except KeyError:
            int_marker_info_list = None
        return int_marker_info_list



    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
