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

from distutils.version import LooseVersion
import sys
import functools

import geometry_msgs.msg
from visualization_msgs.msg import InteractiveMarker

from rosmc_msgs.msg import MarkerStatus
from rosmc_interface_msgs.msg import Action, ActionStatus, AgentActions
from rosmc_interface_msgs.srv import TriggerMissionExecutorRequest, UpdateActionStatusRequest, MissionRequest
from rosmc_msgs.srv import AddActionRequest, DeleteActionRequest, ReorderActionsRequest
import rosmc_msgs.srv
import rospy
import tf
import qdarkstyle
import yaml

import python_qt_binding
from python_qt_binding import loadUi
from python_qt_binding.QtCore import pyqtSignal
from python_qt_binding.QtCore import pyqtSlot
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtCore import QModelIndex
from python_qt_binding.QtCore import QTimer
from python_qt_binding.QtCore import qWarning
from python_qt_binding.QtCore import Slot
from python_qt_binding.QtCore import QModelIndex
from python_qt_binding.QtGui import QColor
from python_qt_binding.QtGui import QIcon
from python_qt_binding.QtGui import QBrush
from python_qt_binding.QtGui import QPen
from python_qt_binding.QtGui import QPainter
from python_qt_binding.QtGui import QContextMenuEvent
from python_qt_binding.QtGui import QMouseEvent
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
    from python_qt_binding.QtWidgets import QListWidget
    from python_qt_binding.QtWidgets import QListWidgetItem
    from python_qt_binding.QtGui import QStandardItem
    from python_qt_binding.QtGui import QStandardItemModel
    # from python_qt_binding.QtWidgets import QProxyStyle
    from python_qt_binding.QtWidgets import QStyleOption
    from python_qt_binding.QtWidgets import QTableView
    from python_qt_binding.QtWidgets import QAbstractItemView
    from python_qt_binding.QtWidgets import QHeaderView
    from python_qt_binding.QtWidgets import QMainWindow
    from python_qt_binding.QtWidgets import QApplication
    from python_qt_binding.QtWidgets import QSpacerItem
    from python_qt_binding.QtWidgets import QCheckBox
    from python_qt_binding.QtWidgets import QMessageBox
    from python_qt_binding.QtWidgets import QStyledItemDelegate
    from python_qt_binding.QtCore import QItemSelectionModel
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
    from python_qt_binding.QtGui import QListWidget
    from python_qt_binding.QtGui import QListWidgetItem
    from python_qt_binding.QtGui import QStandardItem
    from python_qt_binding.QtGui import QStandardItemModel
    # from python_qt_binding.QtGui import QProxyStyle
    from python_qt_binding.QtGui import QStyleOption
    from python_qt_binding.QtGui import QTableView
    from python_qt_binding.QtGui import QAbstractItemView
    from python_qt_binding.QtGui import QHeaderView
    from python_qt_binding.QtGui import QMainWindow
    from python_qt_binding.QtGui import QApplication
    from python_qt_binding.QtGui import QSpacerItem
    from python_qt_binding.QtGui import QCheckBox
    from python_qt_binding.QtGui import QMessageBox
    from python_qt_binding.QtGui import QStyledItemDelegate
    from python_qt_binding.QtGui import QItemSelectionModel
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


def show_error_msg(error_msg):
    """
    Function to show pop-up window with error_msg.
    :param error_msg:
    :return:
    """
    if error_msg is None or error_msg == "":
        return

    error_msg = str(error_msg)
    msg = QMessageBox()
    msg.setIcon(QMessageBox.Critical)
    # msg.setText(error_msg)
    msg.setInformativeText(error_msg)
    msg.setStandardButtons(QMessageBox.Ok)
    retval = msg.exec_()


class ActionTableModel(QStandardItemModel):
    reorderActions = pyqtSignal(int)

    def dropMimeData(self, data, action, row, col, parent):
        """
        Always move the entire row, and don't allow column "shifting"
        """
        rospy.logdebug("Function ActionTableModel.dropMimeData is called")
        print 'dropMimeData %s %s %s %s' % (data.data('text/xml'), action, row, parent)
        result = super(ActionTableModel, self).dropMimeData(data, action, row, 0, parent)
        if result:
            self.reorderActions.emit(row)
        return result

    def data(self, item, role):
        rospy.logdebug("Function ActionTableModel.data is called")
        if role == Qt.BackgroundRole:
            # Set background color depending on its action status
            if QStandardItemModel.data(self, self.index(item.row(), 5), Qt.DisplayRole) == "1":
                return QBrush(QColor(19, 97, 105))
            elif QStandardItemModel.data(self, self.index(item.row(), 5), Qt.DisplayRole) == "2":
                return QBrush(QColor(170, 30, 0))
            elif QStandardItemModel.data(self, self.index(item.row(), 5), Qt.DisplayRole) == "3":
                return QBrush(QColor(44, 148, 108))
            elif QStandardItemModel.data(self, self.index(item.row(), 5), Qt.DisplayRole) == "4":
                return QBrush(QColor(69, 69, 69))
            elif QStandardItemModel.data(self, self.index(item.row(), 5), Qt.DisplayRole) == "7":
                return QBrush(QColor(120, 117, 71))
        return QStandardItemModel.data(self, item, role)


class ActionTableDelegate(QStyledItemDelegate):

    def __init__(self, action_table_view):
        rospy.logdebug("Function ActionTableDelegate.__init__ is called")
        super(ActionTableDelegate, self).__init__()
        self.action_table_view = action_table_view
        self.sync_pen = QPen()
        self.sync_pen.setColor(QColor(232, 222, 59))

    def paint(self, painter, option, index):
        rospy.logdebug("Function ActionTableDelegate.paint is called")
        super(ActionTableDelegate, self).paint(painter, option, index)
        id_column = self.action_table_view.get_column_from_label('id')
        action_id = str(self.action_table_view.model.index(index.row(), id_column).data())
        # If actions have synchronization points, draw a line on top of it
        for i, sync_ids in enumerate(self.action_table_view.sync_ids_list):
            if action_id in sync_ids.sync_ids:
                old_pen = painter.pen()
                painter.setPen(self.sync_pen)
                painter.drawLine(option.rect.topLeft(), option.rect.topRight())
                # Draw text with sync number on top left corner
                if index.column() == self.action_table_view.get_column_from_label('WLAN'):
                    painter.drawText(option.rect.topLeft(), "sync#%d" % i)
                painter.setPen(old_pen)

"""
class ActionTableStyle(QProxyStyle):

    def drawPrimitive(self, element, option, painter, widget=None):
        if element == self.PE_IndicatorItemViewItemDrop and not option.rect.isNull():
            option_new = QStyleOption(option)
            option_new.rect.setLeft(0)
            if widget:
                option_new.rect.setRight(widget.width())
            option = option_new
        super(ActionTableStyle, self).drawPrimitive(element, option, painter, widget)
"""


class ActionTableView(QTableView):
    context_menu_event_signal = pyqtSignal(QContextMenuEvent)

    def __init__(self, parent, actions, sync_ids_list):
        rospy.logdebug("Function ActionTableView.__init__ is called")
        super(ActionTableView, self).__init__(parent)
        # Initialize members
        self.drag_mouse_index = None

        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        # self.verticalHeader().hide()
        # self.horizontalHeader().hide()

        # Enable users to select whole row, not each column
        self.setSelectionBehavior(self.SelectRows)
        # Enable users to select multiple rows contiguously (shift key)
        self.setSelectionMode(self.ExtendedSelection)

        self.setShowGrid(False)
        self.setDragDropMode(self.InternalMove)
        self.setDragDropOverwriteMode(False)

        # Set our custom style - this draws the drop indicator across the whole row
        # self.setStyle(ActionTableStyle())

        # Set our custom model - this prevents row "shifting"
        self.model = ActionTableModel()
        self.setModel(self.model)

        # Set our custom delegate for drawing synchronization lines
        self.setItemDelegate(ActionTableDelegate(self))

        # Mutexes
        self.update_visualization_mutex = False

        # Store mission data
        # NOTE: THIS MISSION DATA WILL BE UPDATED EVERY TIME
        self.actions = actions
        self.sync_ids_list = sync_ids_list
        self.update_visualization(actions, sync_ids_list)

        header_labels = ['id', 'name', 'battery', 'WLAN', 'manual', 'status', 'parameters']
        self.model.setHorizontalHeaderLabels(header_labels)

        # Hide unnecessary columns
        # Comment out the followings for debugging
        self.setColumnHidden(0, True)
        self.setColumnHidden(4, True)
        self.setColumnHidden(5, True)
        self.setColumnHidden(6, True)

        # Column icon_size
        self.setColumnWidth(2, 60)
        self.setColumnWidth(3, 60)
        self.setColumnWidth(4, 60)

        # Set horizontal header
        # Use the following in qt5
        if QT_VERSION >= 5:
            self.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        # Use the following in qt4
        else:
            self.horizontalHeader().setResizeMode(1, QHeaderView.Stretch)

        # Re-apply dark theme if Qt4 is used (otherwise, the header is somehow not dark-colored)
        if QT_VERSION <= 4:
            self.horizontalHeader().setStyleSheet(qdarkstyle.load_stylesheet_pyqt())

    def dragMoveEvent(self, event):
        """
        Override to store index of table which mouse is currently pointing
        :param event:
        :return:
        """
        rospy.logdebug("Function ActionTableView.dragMoveEvent is called")
        item = self.indexAt(event.pos())
        self.drag_mouse_index = item  # used in painDropIndicator
        super(ActionTableView, self).dragMoveEvent(event)

    def mousePressEvent(self, event):
        """
        Override to implement deselection.
        :param event:
        :return:
        """
        rospy.logdebug("Function ActionTableView.mousePressEvent is called")
        item = self.indexAt(event.pos())
        is_selected = self.selectionModel().isSelected(item)
        QTableView.mousePressEvent(self, event)
        if item.row() == -1 and item.column() == -1:
            self.clearSelection()
            index = QModelIndex()
            self.selectionModel().setCurrentIndex(index, QItemSelectionModel.Select)

    def contextMenuEvent(self, event):
        """
        Override to implement multiple selections among different ScheduleWidgets.
        Mainly used for creating synchronizations.
        :param event:
        :return:
        """
        rospy.logdebug("Function ActionTableView.contextMenuEvent is called")
        # This signal will be connected to context_menu_event_callback() in mission_control.py
        self.context_menu_event_signal.emit(event)

    def paintEvent(self, event):
        """
        Override to show drop indicator for all columns
        :param event:
        :return:
        """
        rospy.logdebug("Function ActionTableView.paintEvent is called")
        # 1. bad content in the event
        super(ActionTableView, self).paintEvent(event)
        # 2. wait for qt GUI loop
        painter = QPainter(self.viewport())
        # 3. following function fails
        self.paintDropIndicator(painter)

    def paintDropIndicator(self, painter):
        rospy.logdebug("Function ActionTableView.paintDropIndicator is called")
        if self.state() == QAbstractItemView.DraggingState:
            brush = QBrush(QColor(Qt.white))
            pen = QPen(brush, 2, Qt.SolidLine)
            painter.setPen(pen)

            for column in range(self.model.columnCount()):
                index = self.model.createIndex(self.drag_mouse_index.row(), column)
                opt = QStyleOption()
                if QT_VERSION < 5:
                    opt.init(self)
                opt.rect = self.visualRect(index)
                rect = opt.rect

                indicator_pos = self.dropIndicatorPosition()
                if indicator_pos == QAbstractItemView.OnItem:
                    pass
                elif indicator_pos == QAbstractItemView.AboveItem:
                    painter.drawLine(rect.topLeft(), rect.topRight())
                elif indicator_pos == QAbstractItemView.BelowItem:
                    painter.drawLine(rect.bottomLeft(), rect.bottomRight())
                elif indicator_pos == QAbstractItemView.OnViewport:
                    pass

            """
            brush = QBrush(QColor(Qt.black))

            if rect.height() == 0:
                print("hoge")
                pen = QPen(brush, 2, Qt.SolidLine)
                painter.setPen(pen)
                painter.drawLine(rect.topLeft(), rect.topRight())
            """

    def update_visualization(self, actions, sync_ids_list):
        raise NotImplementedError()

    def update_visualization_forcefully(self, actions, sync_ids_list):
        raise NotImplementedError()

    def generate_item(self, value, is_editable=False):
        """
        Generate an item for ActionTableView.
        :param value: string, integer, etc.
        :return: QStandardItem
        """
        rospy.logdebug("Function ActionTableView.generate_item is called with parameter: " + str(value) + " of type " + str(type(value)))
        if type(value) is str:
            item = QStandardItem(value)
            item.setEditable(is_editable)
            item.setDropEnabled(False)
            return item
        elif type(value) is bool:
            item = QStandardItem("")
            item.setCheckable(False)  # checkbox is not checkable by default
            item.setCheckState(value)
            item.setEditable(is_editable)
            item.setDropEnabled(False)
            return item
        else:
            rospy.logerr("Input type %s is not supported in function generate_item" % type(value))
            return None

    def get_column_from_label(self, label):
        rospy.logdebug("Function ActionTableView.get_column_from_label is called")
        model = self.horizontalHeader().model()
        for column in range(model.columnCount()):
            if model.headerData(column, Qt.Horizontal) == label:
                return column
        return -1

    def clear_selection_signal_callback(self):
        rospy.logdebug("Function ActionTableView.clear_selection_signal_callback is called")
        self.clearSelection()

    def update_visualization_callback(self, actions, sync_ids_list):
        """
        Callback function triggered by signal emit by ScheduleWidget
        to update table visualization.
        """
        rospy.logdebug("Function ActionTableView.update_visualization_callback is called")
        self.update_visualization(actions, sync_ids_list)


class SyncActionTableView(ActionTableView):

    def __init__(self, parent, actions, sync_ids_list):
        rospy.logdebug("Function SyncActionTableView.__init__ is called")
        super(SyncActionTableView, self).__init__(parent, actions, sync_ids_list)
        self.setDragDropMode(self.NoDragDrop)

    def update_visualization(self, actions, sync_ids_list):
        """
        Only shows actions whose status is not IDLE
        :param actions:
        :param sync_ids_list:
        :return:
        """
        rospy.logdebug("Function SyncActionTableView.update_visualization is called")

        rospy.logdebug("Function SyncActionTableView.update_visualization: Waits mutex")
        while self.update_visualization_mutex:
            pass
        rospy.logdebug("Function SyncActionTableView.update_visualization: Mutex acquired")
        self.update_visualization_mutex = True

        # Reset the table only when
        #   - the number of actions whose status is NOT IDLE has changed, or
        #   - the status of any action has changed, or
        #   - the label of any action has changed, or
        #   - the order of actions has changed
        # NOTE: Update in flags are sent to Mission Server when changed, but its visualization is kept in GUI
        action_list_old = [action for action in self.actions if action.action_content.status.value != ActionStatus.IDLE]
        action_list_new = [action for action in actions if action.action_content.status.value != ActionStatus.IDLE]
        is_number_of_actions_changed = len(action_list_old) != len(action_list_new)
        action_status_list_old = [action.action_content.status.value for action in action_list_old]
        action_status_list_new = [action.action_content.status.value for action in action_list_new]
        is_status_changed = action_status_list_old != action_status_list_new
        action_label_list_old = [action.action_content.action_label for action in action_list_old]
        action_label_list_new = [action.action_content.action_label for action in action_list_new]
        is_label_changed = action_label_list_old != action_label_list_new
        action_id_list_old = [action.action_id for action in action_list_old]
        action_id_list_new = [action.action_id for action in action_list_new]
        is_id_order_changed = action_id_list_old != action_id_list_new
        if is_number_of_actions_changed or is_status_changed or is_label_changed or is_id_order_changed:
            self.model.removeRows(0, self.model.rowCount())
            # self.model.clearContents()
            # self.model.setHorizontalHeaderLabels(['id', 'name', 'B', 'W', 'M', 'status', 'parameters'])
            for i, action in enumerate(actions):
                if action.action_content.status.value != ActionStatus.IDLE:
                    rospy.logdebug("Function SyncActionTableView.update_visualization: append row %d" % i)
                    item_list = [self.generate_item(action.action_id),
                                 self.generate_item(action.action_content.action_label, is_editable=False),  # use action_label for visualization in tables
                                 self.generate_item(action.action_content.battery_flag),
                                 self.generate_item(action.action_content.wlan_flag),
                                 self.generate_item(action.action_content.manual_flag),
                                 self.generate_item(str(action.action_content.status.value)),
                                 self.generate_item(action.action_content.parameters_yaml)]
                    self.model.appendRow(item_list)

        # Enable/disable buttons of run/pause/stop/reset
        if self.model.rowCount() > 0:
            is_button_enabled = True
        else:
            is_button_enabled = False
        buttons = self.parent().findChildren(QPushButton)
        for button in buttons:
            if button.objectName() in self.parent().trigger_button_names:
                button.setEnabled(is_button_enabled)

        # Update the mission data inside
        # NOTE: These members should be updated even if the table is not updated
        self.actions = actions
        self.sync_ids_list = sync_ids_list

        rospy.logdebug("Function SyncActionTableView.update_visualization: Mutext unlocked")
        self.update_visualization_mutex = False

    def update_visualization_forcefully(self, actions, sync_ids_list):
        rospy.logdebug("Function SyncActionTableView.update_visualization_forcefully is called")

        rospy.logdebug("Function SyncActionTableView.update_visualization_forcefully: Waits mutex")
        while self.update_visualization_mutex:
            pass
        rospy.logdebug("Function SyncActionTableView.update_visualization_forcefully: Mutex acquired")
        self.update_visualization_mutex = True

        self.model.removeRows(0, self.model.rowCount())
        # self.model.clearContents()
        # self.model.setHorizontalHeaderLabels(['id', 'name', 'B', 'W', 'M', 'status', 'parameters'])
        for i, action in enumerate(actions):
            if action.action_content.status.value != ActionStatus.IDLE:
                rospy.logdebug("Function SyncActionTableView.update_visualization_forcefully: append row %d" % i)
                item_list = [self.generate_item(action.action_id),
                             self.generate_item(action.action_content.action_label, is_editable=False),  # use action_label for visualization in tables
                             self.generate_item(action.action_content.battery_flag),
                             self.generate_item(action.action_content.wlan_flag),
                             self.generate_item(action.action_content.manual_flag),
                             self.generate_item(str(action.action_content.status.value)),
                             self.generate_item(action.action_content.parameters_yaml)]
                self.model.appendRow(item_list)

        # Update the mission data inside
        self.actions = actions
        self.sync_ids_list = sync_ids_list

        rospy.logdebug("Function SyncActionTableView.update_visualization_forcefully: Mutext unlocked")
        self.update_visualization_mutex = False


class NonSyncActionTableDelegate(ActionTableDelegate):

    def editorEvent(self, event, model, option, index):
        checked = index.data(Qt.CheckStateRole)
        ret = QStyledItemDelegate.editorEvent(self, event, model, option, index)
        if checked != index.data(Qt.CheckStateRole):
            self.action_table_view.checked.emit(index)
        return ret


class NonSyncActionTableView(ActionTableView):
    checked = pyqtSignal(QModelIndex)

    def __init__(self, parent, actions, sync_ids_list):
        rospy.logdebug("Function NonSyncActionTableView.__init__ is called")
        super(NonSyncActionTableView, self).__init__(parent, actions, sync_ids_list)
        # set custom delegate to detect checked state modification (used for updating reactive behavior flags)
        non_sync_action_table_delegate = NonSyncActionTableDelegate(self)
        self.setItemDelegate(non_sync_action_table_delegate)
        # signals to detect check state modification (used for updating reactive behavior flags)
        self.checked.connect(self.onChecked)
        # signals to detect string modification (used for updating action label)
        non_sync_action_table_delegate.closeEditor.connect(self.onEdited)

    def update_visualization(self, actions, sync_ids_list):
        """
        Only shows actions whose status is IDLE
        :param actions:
        :param sync_ids_list:
        :return:
        """
        rospy.logdebug("Function NonSyncActionTableView.update_visualization is called")

        rospy.logdebug("Function NonSyncActionTableView.update_visualization is called")

        rospy.logdebug("Function NonSyncActionTableView.update_visualization: Waits mutex")
        while self.update_visualization_mutex:
            pass
        rospy.logdebug("Function NonSyncActionTableView.update_visualization: Mutex acquired")
        self.update_visualization_mutex = True

        # Reset the table only when
        #   - the number of actions whose status is IDLE has changed, or
        #   - the status of any action has changed, or
        #   - the label of any action has changed, or
        #   - the order of actions has changed
        # NOTE: Update in flags are sent to Mission Server when changed, but its visualization is kept in GUI
        print(self.actions)
        print(actions)
        action_list_old = [action for action in self.actions if action.action_content.status.value == ActionStatus.IDLE]
        action_list_new = [action for action in actions if action.action_content.status.value == ActionStatus.IDLE]
        is_number_of_actions_changed = len(action_list_old) != len(action_list_new)
        action_status_list_old = [action.action_content.status.value for action in action_list_old]
        action_status_list_new = [action.action_content.status.value for action in action_list_new]
        is_status_changed = action_status_list_old != action_status_list_new
        action_label_list_old = [action.action_content.action_label for action in action_list_old]
        action_label_list_new = [action.action_content.action_label for action in action_list_new]
        is_label_changed = action_label_list_old != action_label_list_new
        action_id_list_old = [action.action_id for action in action_list_old]
        action_id_list_new = [action.action_id for action in action_list_new]
        is_id_order_changed = action_id_list_old != action_id_list_new
        if is_number_of_actions_changed or is_status_changed or is_label_changed or is_id_order_changed:
            self.model.removeRows(0, self.model.rowCount())
            # self.model.clearContents()
            # self.model.setHorizontalHeaderLabels(['id', 'name', 'B', 'W', 'M', 'status', 'parameters'])
            for i, action in enumerate(actions):
                if action.action_content.status.value == ActionStatus.IDLE:
                    item_list = [self.generate_item(action.action_id),
                                 self.generate_item(action.action_content.action_label, is_editable=True),  # use action_label for visualization in tables
                                 self.generate_item(action.action_content.battery_flag),
                                 self.generate_item(action.action_content.wlan_flag),
                                 self.generate_item(action.action_content.manual_flag),
                                 self.generate_item(str(action.action_content.status.value)),
                                 self.generate_item(action.action_content.parameters_yaml)]
                    self.model.appendRow(item_list)

        # Update the mission data inside
        # NOTE: These members should be updated even if the table is not updated
        self.actions = actions
        self.sync_ids_list = sync_ids_list

        rospy.logdebug("Function NonSyncActionTableView.update_visualization: Mutext unlocked")
        self.update_visualization_mutex = False

    def update_visualization_forcefully(self, actions, sync_ids_list):
        rospy.logdebug("Function NonSyncActionTableView.update_visualization_forcefully is called")

        rospy.logdebug("Function NonSyncActionTableView.update_visualization_forcefully: Waits mutex")
        while self.update_visualization_mutex:
            pass
        rospy.logdebug("Function NonSyncActionTableView.update_visualization_forcefully: Mutex acquired")
        self.update_visualization_mutex = True

        self.model.removeRows(0, self.model.rowCount())
        # self.model.clearContents()
        # self.model.setHorizontalHeaderLabels(['id', 'name', 'B', 'W', 'M', 'status', 'parameters'])
        for i, action in enumerate(actions):
            if action.action_content.status.value == ActionStatus.IDLE:
                item_list = [self.generate_item(action.action_id),
                             self.generate_item(action.action_content.action_label, is_editable=True),  # use action_label for visualization in tables
                             self.generate_item(action.action_content.battery_flag),
                             self.generate_item(action.action_content.wlan_flag),
                             self.generate_item(action.action_content.manual_flag),
                             self.generate_item(str(action.action_content.status.value)),
                             self.generate_item(action.action_content.parameters_yaml)]
                self.model.appendRow(item_list)

        # Update the mission data inside
        self.actions = actions
        self.sync_ids_list = sync_ids_list

        rospy.logdebug("Function NonSyncActionTableView.update_visualization_forcefully: Mutext unlocked")
        self.update_visualization_mutex = False

    def generate_item(self, value, is_editable=False):
        """
        Overriding parent's function for making checkbox editable.
        :param value: string, integer, etc.
        :return: QStandardItem
        """
        rospy.logdebug("Function ActionTableView.generate_item is called with parameter: " + str(value) + " of type " + str(type(value)))
        if type(value) is str:
            item = QStandardItem(value)
            item.setEditable(is_editable)
            item.setDropEnabled(False)
            return item
        elif type(value) is bool:
            item = QStandardItem("")
            item.setCheckable(True)  # checkbox is now checkable!
            # NOTE: item.setCheckState(True) does set item state as checked, but visualization is corrupted!
            if value:
                item.setCheckState(Qt.Checked)
            else:
                item.setCheckState(Qt.Unchecked)
            item.setEditable(is_editable)
            item.setDropEnabled(False)
            return item
        else:
            rospy.logerr("Input type %s is not supported in function generate_item" % type(value))
            return None

    def onChecked(self, index):
        """
        This function is used for updating reactive behavior flags.
        :param index: QModelIndex
        :return:
        """
        # Extract non-synchronized actions
        actions = [action for action in self.actions if action.action_content.status.value == ActionStatus.IDLE]
        # Extract the action in which reactive flag is changed
        action = actions[index.row()]
        item = self.model.item(index.row(), index.column())
        # Update flags
        for label in ['battery', 'WLAN', 'manual']:
            column = self.get_column_from_label(label)
            if column != index.column():
                continue
            if label == 'battery':
                action.action_content.battery_flag = bool(item.checkState())
                break
            elif label == 'WLAN':
                action.action_content.wlan_flag = bool(item.checkState())
                break
            elif label == 'manual':
                action.action_content.manual_flag = bool(item.checkState())
                break
            else:
                rospy.logerr("Unexpected reactive behavior flag modification")
                break
        # Send server edited action
        rospy.loginfo("Reactive behavior flag is modified. Call EditAction service to update!")
        print self.parent().edit_action_service_proxy(action)

    def onEdited(self, editor, hint):
        """
        This function is used for updating action labels.
        :param editor: QWidget
               hint: QAbstractItemDelegate::EndEditHint
        :return:
        """
        # Get index
        index = self.indexAt(editor.pos())
        # Extract non-synchronized actions
        actions = [action for action in self.actions if action.action_content.status.value == ActionStatus.IDLE]
        # Extract the action in which reactive flag is changed
        action = actions[index.row()]
        action.action_content.action_label = str(editor.text())
        # Send server edited action
        rospy.loginfo("Action label is modified. Call EditAction service to update!")
        print self.parent().edit_action_service_proxy(action)


class ScheduleWidget(QWidget):
    # Create custom selection changed signal
    non_sync_selection_changed = pyqtSignal(str, Action)
    sync_selection_changed = pyqtSignal(str, Action)
    selection_cleared = pyqtSignal()
    non_sync_action_table_clear_selection_signal = pyqtSignal()
    sync_action_table_clear_selection_signal = pyqtSignal()
    non_sync_action_table_update_visualization_signal = pyqtSignal(list, list)  # actions, sync_ids_list
    sync_action_table_update_visualization_signal = pyqtSignal(list, list)  # actions, sync_ids_list

    def __init__(self, agent_actions, sync_ids_list, mission_server_namespace):
        """
        Schedule widget for each agent.
        :param agent_actions: AgentActions()
        """
        rospy.logdebug("Function ScheduleWidget.__init__ is called")
        super(ScheduleWidget, self).__init__()

        # Store mission data
        # NOTE: THIS MISSION DATA IS ONLY USED FOR INITIALIZATION AND NEVER UPDATED
        self.agent_actions = agent_actions
        self.sync_ids_list = sync_ids_list

        # Store parameters
        self.mission_server_namespace = mission_server_namespace

        # Read config dicts
        self.agent_library_dict = rospy.get_param('{}agent_library_dict'.format(self.mission_server_namespace))
        self.lib_action_parameter_groups_dict = rospy.get_param('{}lib_action_parameter_groups_dict'.format(self.mission_server_namespace))
        self.use_interactive_markers = rospy.get_param('{}use_interactive_markers'.format(self.mission_server_namespace))
        if self.use_interactive_markers:
            self.common_int_marker = rospy.get_param('{}common_int_marker'.format(self.mission_server_namespace))
            self.lib_action_int_marker_dict = rospy.get_param('{}lib_action_int_marker_dict'.format(self.mission_server_namespace))

        # Set object name of self
        # NOTE: even though type(self.agent_actions.agent_name) is str,
        #       self.objectName() returns unicode. Be careful when sending srv with self.objectName()
        self.setObjectName(self.agent_actions.agent_name)
        # print str(self.objectName())
        # print type(str(self.objectName()))

        # Service proxies
        # NOTE: service proxies must be set after service advertisement with function self.setServiceProxies().
        self.add_action_service_proxy = None
        self.trigger_mission_executor_service_proxy = None
        self.delete_action_service_proxy = None
        self.reorder_actions_service_proxy = None
        self.update_action_status_service_proxy = None
        self.edit_action_service_proxy = None

        # Set main layout
        v_l = QVBoxLayout()
        self.setLayout(v_l)

        # Set agent name
        agent_name_label = QLabel(self.agent_actions.agent_name)
        agent_name_label.setSizePolicy(QSizePolicy.Minimum, QSizePolicy.Minimum)
        v_l.addWidget(agent_name_label)

        # Set ActionTableView for synchronised actions
        sync_action_table_view = SyncActionTableView(self, self.agent_actions.actions, self.sync_ids_list)
        # Connect to selection changed signal
        selection_model = sync_action_table_view.selectionModel()
        selection_model.selectionChanged.connect(self.sync_selection_changed_callback)
        # Connect signal to clear selection
        self.sync_action_table_clear_selection_signal.connect(sync_action_table_view.clear_selection_signal_callback)
        # Connect signal to update visualization
        self.sync_action_table_update_visualization_signal.connect(sync_action_table_view.update_visualization_callback)
        sync_action_table_view.setObjectName('syncActionTableView')
        v_l.addWidget(sync_action_table_view)

        # Set ActionTableView for non-synchronised actions
        non_sync_action_table_view = NonSyncActionTableView(self, self.agent_actions.actions, self.sync_ids_list)
        # Connect to selection changed signal
        selection_model = non_sync_action_table_view.selectionModel()
        selection_model.selectionChanged.connect(self.non_sync_selection_changed_callback)
        # Connect signal to clear selection
        self.non_sync_action_table_clear_selection_signal.connect(non_sync_action_table_view.clear_selection_signal_callback)
        # Connect signal to update visualization
        self.non_sync_action_table_update_visualization_signal.connect(non_sync_action_table_view.update_visualization_callback)
        # Connect to rows moved signal
        model = non_sync_action_table_view.model
        model.reorderActions.connect(self.reorder_actions_callback)
        non_sync_action_table_view.setObjectName('nonSyncActionTableView')
        v_l.addWidget(non_sync_action_table_view)

        # Set buttons for add/delete/synchronize and run/pause/stop
        widget_buttons_1 = QWidget()
        widget_buttons_1.setObjectName('buttonsWidget')
        h_l_buttons_1 = QHBoxLayout()
        widget_buttons_1.setLayout(h_l_buttons_1)
        v_l.addWidget(widget_buttons_1)
        h_l_buttons_1.addWidget(QPushButton('Add', objectName='addButton', clicked=self.addButtonCallback), 1)
        h_l_buttons_1.addWidget(QPushButton('Delete', objectName='deleteButton', clicked=self.deleteButtonCallback), 1)
        h_l_buttons_1.addWidget(QPushButton('Sync', objectName='syncButton', clicked=self.syncButtonCallback), 1)
        h_l_buttons_1.addWidget(QPushButton('Unsync', objectName='unsyncButton', clicked=self.unsyncButtonCallback), 1)
        h_l_buttons_1.addItem(QSpacerItem(2000, 20, QSizePolicy.Expanding, QSizePolicy.Minimum))
        h_l_buttons_1.addWidget(QPushButton('Run', objectName='runButton', clicked=self.runButtonCallback), 1)
        h_l_buttons_1.addWidget(QPushButton('Pause', objectName='pauseButton', clicked=self.pauseButtonCallback), 1)
        h_l_buttons_1.addWidget(QPushButton('Stop', objectName='stopButton', clicked=self.stopButtonCallback), 1)
        reset_button = QPushButton('Reset', objectName='resetButton', clicked=self.resetButtonCallback)
        reset_button.setStyleSheet("background-color:rgb(126, 0, 0)")
        self.trigger_button_names = ['runButton', 'pauseButton', 'stopButton', 'resetButton']
        h_l_buttons_1.addWidget(reset_button, 1)

        # Mutexes
        self.action_table_item_reselection_signal_mutex = False

        # Local variable required for interactive marekrs
        if self.use_interactive_markers:
            self.INT_MARKER = None
            self.ROLL = None
            self.PITCH = None
            self.YAW = None

    def non_sync_selection_changed_callback(self, selected, deselected):
        """
        Emit custom signal "self.non_sync_selection_changed", whose contents is class Action()
        or emit custom signal "self.selection_cleared", whose content is None.
        :param selected:
        :param deselected:
        :return:
        """
        rospy.logdebug("Function ScheduleWidget.non_sync_selection_changed_callback is called")
        # TODO: we should not store action inside of ActionTableView.
        #  We should just communicate action_id, and obtain its information from Mission Server!
        non_sync_action_table_view = self.findChild(NonSyncActionTableView, 'nonSyncActionTableView')
        indexes = non_sync_action_table_view.selectionModel().selectedRows()
        if len(indexes) > 0:
            # get information of the row selected last time
            index = indexes[-1]
            action = non_sync_action_table_view.actions[index.row() + self.findChild(SyncActionTableView, 'syncActionTableView').model.rowCount()]
            self.non_sync_selection_changed.emit(self.objectName(), action)
        else:
            self.selection_cleared.emit()

    def sync_selection_changed_callback(self, selected, deselected):
        """
        Emit custom signal "self.sync_selection_changed", whose contents is class Action()
        or emit custom signal "self.selection_cleared", whose content is None.
        :param selected:
        :param deselected:
        :return:
        """
        rospy.logdebug("Function ScheduleWidget.sync_selection_changed_callback is called")
        # TODO: we should not store action inside of ActionTableView.
        #  We should just communicate action_id, and obtain its information from Mission Server!
        sync_action_table_view = self.findChild(SyncActionTableView, 'syncActionTableView')
        indexes = sync_action_table_view.selectionModel().selectedRows()
        if len(indexes) > 0:
            # get information of the row selected last time
            index = indexes[-1]
            action = sync_action_table_view.actions[index.row()]
            self.sync_selection_changed.emit(self.objectName(), action)
        else:
            self.selection_cleared.emit()

    def reorder_actions_callback(self, new_index):
        """
        This function deals with the behavior of dragging & dropping actions.
        - Reorder
        # TODO: make it possible to synchronise or unsynchronise actions by dragging and dropping
        :param req:
        :return:
        """
        rospy.logdebug("Function ScheduleWidget.reorder_actions_callback is called")
        if self.reorder_actions_service_proxy is not None:
            # Reorder is only possible in NonSyncActionTableView
            non_sync_action_table_view = self.findChild(NonSyncActionTableView, 'nonSyncActionTableView')
            old_indices = [index.row() for index in non_sync_action_table_view.selectionModel().selectedRows()]
            # Sort selected indices because selected indices are not sorted when ctrl modifier is used
            old_indices = sorted(old_indices)
            # During dragging, the rowCount returns the value incremented by the number of selected rows
            list_before = range(non_sync_action_table_view.model.rowCount() - len(old_indices))
            list_after = range(non_sync_action_table_view.model.rowCount() - len(old_indices))
            # Allows reordering only if
            #   - the old_indices is a list of sequential numbers
            #   - new_index is larger than -1
            # When we do not allow reordering, we have to forcefully update visualization in ActionTableViews, because
            # they are updated on several specific cases in order to avoid core dumps
            is_force_visualization_update = False
            if new_index > -1:
                if old_indices == list(range(min(old_indices), max(old_indices) + 1)):
                    rospy.loginfo("old_indices:")
                    rospy.loginfo(old_indices)
                    rospy.loginfo("new_index: %d" % new_index)

                    status_column = non_sync_action_table_view.get_column_from_label('status')
                    # old_status = int(non_sync_action_table_view.model.index(old_index, status_column).data())
                    old_statuses = [int(non_sync_action_table_view.model.index(old_i, status_column).data()) for old_i in old_indices]
                    # Only the actions whose status are IDLE are movable
                    if all(old_status == ActionStatus.IDLE for old_status in old_statuses):
                        if max(old_indices) < new_index:
                            rospy.loginfo(
                                "Reorder actions from %s to %d in agent %s" %
                                (''.join(str(e) + ', ' for e in old_indices), new_index, self.objectName()))
                            for old_index in reversed(old_indices):
                                list_after.insert(new_index, list_after[old_index])
                            for old_index in reversed(old_indices):
                                list_after.pop(old_index)
                        elif new_index < min(old_indices):
                            # When moving actions in forward, make sure that the next action is IDLE
                            new_next_status = int(non_sync_action_table_view.model.index(new_index + len(old_indices), status_column).data())
                            if new_next_status == ActionStatus.IDLE:
                                rospy.loginfo(
                                    "Reorder actions from %s to %d in agent %s" %
                                    (''.join(str(e) + ', ' for e in old_indices), new_index, self.objectName()))
                                for old_index in reversed(old_indices):
                                    list_after.pop(old_index - len(old_indices))
                                for old_index in reversed(old_indices):
                                    list_after.insert(new_index, old_index - len(old_indices))
                            else:
                                rospy.logwarn("Unable to move selected action because its next status will be %d" % new_next_status)
                                list_after = list_before
                                is_force_visualization_update = True
                        else:
                            rospy.logerr("Error in reorder_action_callback: old_index == new_index")
                            list_after = list_before
                            is_force_visualization_update = True
                    else:
                        rospy.logwarn("Unable to move selected action because its status is %s" %
                                      ''.join(str(e) + ', ' for e in old_indices))
                        list_after = list_before
                        is_force_visualization_update = True
                else:
                    rospy.logwarn("Unable to move a non-sequential list of actions.")
                    list_after = list_before
                    is_force_visualization_update = True
            else:
                rospy.logwarn("Unable to move action(s) because destination is not valid.")
                list_after = list_before
                is_force_visualization_update = True

            # Arrange list_before and and list_after
            sync_action_table_view = self.findChild(SyncActionTableView, 'syncActionTableView')
            sync_action_num = sync_action_table_view.model.rowCount()
            list_after = [n + sync_action_num for n in list_after]
            list_after = range(sync_action_num) + list_after

            req = ReorderActionsRequest()
            req.agent_name = str(self.objectName())
            req.new_order = list_after
            res = self.reorder_actions_service_proxy(req)
            show_error_msg(res.error_msg)

            if is_force_visualization_update:
                non_sync_action_table_view.update_visualization_forcefully(non_sync_action_table_view.actions,
                                                                           non_sync_action_table_view.sync_ids_list)

        else:
            rospy.logwarn("Service proxy to reorder actions is not set yet." +
                          "Reordering is NOT reflected to Mission Server.")

    def addButtonCallback(self):
        rospy.logdebug("Function ScheduleWidget.addButtonCallback is called")
        rospy.loginfo("Add Button is clicked")
        # Create context menu for addButton
        add_button_menu = QMenu(self.findChild(QWidget, 'buttonsWidget').findChild(QPushButton, 'addButton'))
        actions_dict = rospy.get_param('{}agent_action_dict/{}'.format(self.mission_server_namespace, self.objectName()))
        action_names = list(actions_dict)
        action_names.sort()
        for action_name in action_names:
            actn = QAction(action_name, add_button_menu)
            if QT_VERSION >= 5:
                actn.triggered.connect(self.action_menuitem_clicked)
            else:
                actn.triggered[()].connect(lambda action_name=action_name: self.add_action(action_name))
            add_button_menu.addAction(actn)
        add_button_menu.popup(QCursor.pos())

    @python_qt_binding.QtCore.pyqtSlot(bool)
    def action_menuitem_clicked(self, checked):
        action = self.sender()
        print('Action: ', action.text())
        self.add_action(action.text())

    def add_action(self, action_name):
        rospy.logdebug("Function ScheduleWidget.add_action is called")
        print(action_name)
        if self.add_action_service_proxy is not None:
            add_action_req = AddActionRequest()
            add_action_req.agent_name = str(self.objectName())

            # Find the first action in alphabetical order as a default action
            actions_dict = rospy.get_param('{}agent_action_dict/{}'.format(self.mission_server_namespace, str(self.objectName())))

            # Fill in the initial contents
            add_action_req.action_content.action_name = action_name
            add_action_req.action_content.action_label = action_name
            add_action_req.action_content.battery_flag = True
            add_action_req.action_content.wlan_flag = True

            # parameter specification
            parameters_dict = {}
            if actions_dict[action_name] != 'None':  # In case this action requires no parameter, this block is skipped.
                parameters_dict = actions_dict[action_name].copy()
            for param_name in parameters_dict:
                parameters_dict[param_name]['value'] = parameters_dict[param_name]['default_value']
                del parameters_dict[param_name]['default_value']

            # Interactive markers are dealt with only when the flag is set True
            if self.use_interactive_markers:
                # Check if the action requires parametrization by interactive marker
                # If required, create a new interactive marker for each kind of type

                # Get config data from dicts
                library = self.agent_library_dict[str(self.objectName())]
                try:
                    parameter_groups = self.lib_action_parameter_groups_dict[library][action_name]
                except KeyError:
                    parameter_groups = None
                try:
                    marker_for_parametrization_list = self.lib_action_int_marker_dict[library][action_name]
                except KeyError:
                    marker_for_parametrization_list = []

                # Add TASK interactive markers in int_marker_list
                for i, marker_for_parametrization in enumerate(marker_for_parametrization_list):
                    if marker_for_parametrization['type'] == 'TASK':
                        # Retrieve information from marker_for_parametrization
                        service_type_for_addition = marker_for_parametrization['service_type_for_addition']
                        service_topic_for_addition = marker_for_parametrization['service_topic_for_addition']
                        service_req_attr = marker_for_parametrization['service_req_attr']
                        service_req_value = marker_for_parametrization['service_req_value']

                        # Obtain the corresponding service classes for adding markers
                        service_type = getattr(rosmc_msgs.srv, service_type_for_addition)
                        req = getattr(rosmc_msgs.srv, service_type_for_addition + 'Request')()
                        req.status.value = MarkerStatus.IDLE

                        # Create int_marker from the parameters in this group
                        self.INT_MARKER = InteractiveMarker()
                        self.ROLL = 0.0
                        self.PITCH = 0.0
                        self.YAW = 0.0
                        # If this marker is assigned to multiple parameters (group),
                        if 'members' in marker_for_parametrization:
                            members = marker_for_parametrization['members']
                            for member in members:
                                param_name = member['name']
                                value = member['value']
                                rsetattr(self, value, parameters_dict[param_name]['value'])
                        # Else if this marker is assigned to single parameter
                        elif 'value' in marker_for_parametrization:
                            param_name = marker_for_parametrization['name']
                            value = marker_for_parametrization['value']
                            rsetattr(self, value, parameters_dict[param_name]['value'])
                        else:
                            rospy.logerr("ERROR: either 'members' or 'value' key is required for int_marker assignment!")
                            return
                        quaternion = tf.transformations.quaternion_from_euler(self.ROLL, self.PITCH, self.YAW)
                        self.INT_MARKER.pose.orientation.x = quaternion[0]
                        self.INT_MARKER.pose.orientation.y = quaternion[1]
                        self.INT_MARKER.pose.orientation.z = quaternion[2]
                        self.INT_MARKER.pose.orientation.w = quaternion[3]

                        # Enter the values for service req
                        value = rgetattr(self, service_req_value)
                        rsetattr(req, service_req_attr, value)

                        # Add marker
                        add_task_marker_service_proxy = \
                            rospy.ServiceProxy('{}/add_task_marker'.format(service_topic_for_addition), service_type)
                        res = add_task_marker_service_proxy(req)
                        # Store the information about the name of the added marker
                        # NOTE: Every request for adding task markers should return the name of the marker added.
                        marker_for_parametrization_list[i]['marker_name'] = res.name

                    elif marker_for_parametrization['type'] == 'TASK_POSEDICT':
                        # Hard-coded mode for pose dict
                        param_name = marker_for_parametrization['name']
                        service_type_for_addition = marker_for_parametrization['service_type_for_addition']
                        service_topic_for_addition = marker_for_parametrization['service_topic_for_addition']
                        service_type = getattr(rosmc_msgs.srv, service_type_for_addition)
                        req = getattr(rosmc_msgs.srv, service_type_for_addition + 'Request')()
                        req.status.value = MarkerStatus.IDLE
                        # Convert pose into geometry_msgs/Pose
                        pose = parameters_dict[param_name]['value']
                        geometry_pose = geometry_msgs.msg.Pose()
                        geometry_pose.position.x = pose['x']
                        geometry_pose.position.y = pose['y']
                        geometry_pose.position.z = pose['z']
                        quaternion = tf.transformations.quaternion_from_euler(pose['roll'], pose['pitch'],
                                                                              pose['yaw'])
                        geometry_pose.orientation.x = quaternion[0]
                        geometry_pose.orientation.y = quaternion[1]
                        geometry_pose.orientation.z = quaternion[2]
                        geometry_pose.orientation.w = quaternion[3]
                        req.pose = geometry_pose
                        add_task_marker_service_proxy = \
                            rospy.ServiceProxy('{}/add_task_marker'.format(service_topic_for_addition), service_type)
                        res = add_task_marker_service_proxy(req)
                        # Store the information about the name of the added marker
                        # NOTE: Every request for adding task markers should return the name of the marker added.
                        marker_for_parametrization_list[i]['marker_name'] = res.name

                    elif marker_for_parametrization['type'] == 'TASK_LIST_POSEDICT':
                        # Hard-coded mode for area marker
                        param_name = marker_for_parametrization['name']
                        service_type_for_addition = marker_for_parametrization['service_type_for_addition']
                        service_topic_for_addition = marker_for_parametrization['service_topic_for_addition']
                        service_type = getattr(rosmc_msgs.srv, service_type_for_addition)
                        req = getattr(rosmc_msgs.srv, service_type_for_addition + 'Request')()
                        req.status.value = MarkerStatus.IDLE

                        # Convert list_pose into list of geometry_msgs/Pose
                        list_pose = parameters_dict[param_name]['value']
                        list_geometry_pose = []
                        for pose in list_pose:
                            geometry_pose = geometry_msgs.msg.Pose()
                            geometry_pose.position.x = pose['x']
                            geometry_pose.position.y = pose['y']
                            geometry_pose.position.z = pose['z']
                            quaternion = tf.transformations.quaternion_from_euler(pose['roll'], pose['pitch'], pose['yaw'])
                            geometry_pose.orientation.x = quaternion[0]
                            geometry_pose.orientation.y = quaternion[1]
                            geometry_pose.orientation.z = quaternion[2]
                            geometry_pose.orientation.w = quaternion[3]
                            list_geometry_pose.append(geometry_pose)

                        # NOTE: implicitly assumes that req has key 'poses', which is list of geometry_msgs/Pose
                        req.poses = list_geometry_pose
                        add_task_marker_service_proxy = \
                            rospy.ServiceProxy('{}/add_task_marker'.format(service_topic_for_addition), service_type)
                        res = add_task_marker_service_proxy(req)
                        # Store the information about the name of the added marker
                        # NOTE: Every request for adding task markers should return the name of the marker added.
                        marker_for_parametrization_list[i]['marker_name'] = res.name

                # Add the name and the topic_name of interactive markers to the parameters
                for param_name in parameters_dict:
                    for marker_for_parametrization in marker_for_parametrization_list:
                        if marker_for_parametrization['type'] == 'TASK':
                            service_topic_for_addition = marker_for_parametrization['service_topic_for_addition']
                            # If this marker is assigned to multiple parameters (group),
                            if 'members' in marker_for_parametrization:
                                members = marker_for_parametrization['members']
                                for member in members:
                                    if param_name == member['name']:
                                        marker_name = marker_for_parametrization['marker_name']
                                        parameters_dict[param_name]['marker'] = marker_name
                                        parameters_dict[param_name]['marker_topic'] = service_topic_for_addition
                                        break
                            # Else if this marker is assigned to single parameter
                            elif 'value' in marker_for_parametrization:
                                if param_name == marker_for_parametrization['name']:
                                    marker_name = marker_for_parametrization['marker_name']
                                    parameters_dict[param_name]['marker'] = marker_name
                                    parameters_dict[param_name]['marker_topic'] = service_topic_for_addition
                                    break

                        elif marker_for_parametrization['type'] == 'TASK_LIST_POSEDICT' or \
                                marker_for_parametrization['type'] == 'TASK_POSEDICT':
                            # hard-coded behavior for adding markers
                            service_topic_for_addition = marker_for_parametrization['service_topic_for_addition']
                            if param_name == marker_for_parametrization['name']:
                                marker_name = marker_for_parametrization['marker_name']
                                parameters_dict[param_name]['marker'] = marker_name
                                parameters_dict[param_name]['marker_topic'] = service_topic_for_addition
                                break

                        elif marker_for_parametrization['type'] == 'TASK_OBJECT':
                            service_topic_for_addition = marker_for_parametrization['service_topic_for_addition']
                            # If this marker is assigned to multiple parameters (group),
                            if 'members' in marker_for_parametrization:
                                members = marker_for_parametrization['members']
                                for member in members:
                                    if param_name == member['name']:
                                        marker_name = parameters_dict[param_name]['value']  # Assign default value here
                                        parameters_dict[param_name]['marker'] = marker_name
                                        parameters_dict[param_name]['marker_topic'] = service_topic_for_addition
                                        break
                            # Else if this marker is assigned to single parameter
                            elif 'value' in marker_for_parametrization:
                                if param_name == marker_for_parametrization['name']:
                                    marker_name = parameters_dict[param_name]['value']  # Assign default value here
                                    parameters_dict[param_name]['marker'] = marker_name
                                    parameters_dict[param_name]['marker_topic'] = service_topic_for_addition
                                    break

                        elif marker_for_parametrization['type'] == 'AGENT':
                            # If this marker is assigned to multiple parameters (group),
                            if 'members' in marker_for_parametrization:
                                members = marker_for_parametrization['members']
                                for member in members:
                                    if param_name == member['name']:
                                        marker_name = parameters_dict[param_name]['value']  # Assign default value here
                                        parameters_dict[param_name]['marker'] = marker_name
                                        # TODO: do we need to fill in here? - Maybe not.
                                        #  This is because key 'marker_topic' is only used for update_marker_status() in mission_server.py
                                        #  and update_marker_status() takes no effect for AGENT markers.
                                        parameters_dict[param_name]['marker_topic'] = None
                                        break
                            # Else if this marker is assigned to single parameter
                            elif 'value' in marker_for_parametrization:
                                if param_name == marker_for_parametrization['name']:
                                    marker_name = parameters_dict[param_name]['value']  # Assign default value here
                                    parameters_dict[param_name]['marker'] = marker_name
                                    # TODO: do we need to fill in here? - Maybe not.
                                    #  This is because key 'marker_topic' is only used for update_marker_status() in mission_server.py
                                    #  and update_marker_status() takes no effect for AGENT markers.
                                    parameters_dict[param_name]['marker_topic'] = None
                                    break

            add_action_req.action_content.parameters_yaml = yaml.dump(parameters_dict)
            resp = self.add_action_service_proxy(add_action_req)
            show_error_msg(resp.error_msg)
            rospy.loginfo("Add new action %s with id %s" % (add_action_req.action_content.action_name, resp.action_id))
        else:
            rospy.logwarn("Service proxy to add action is not set yet. New action is not added.")

    def deleteButtonCallback(self):
        rospy.logdebug("Function ScheduleWidget.deleteButtonCallback is called")
        # This function should literally delete action from mission server.
        # This function allows to delete actions whose state is IDLE
        # Deletion is only possible in NonSyncActionTableView
        if self.delete_action_service_proxy is not None:
            req = DeleteActionRequest()
            non_sync_action_table_view = self.findChild(NonSyncActionTableView, 'nonSyncActionTableView')
            indexes = non_sync_action_table_view.selectionModel().selectedRows()
            id_column = non_sync_action_table_view.get_column_from_label('id')
            status_column = non_sync_action_table_view.get_column_from_label('status')
            selected_action_id_list = [str(non_sync_action_table_view.model.index(index.row(), id_column).data())
                                       for index in sorted(indexes)]
            selected_action_status_list = [int(non_sync_action_table_view.model.index(index.row(), status_column).data())
                                           for index in sorted(indexes)]
            for action_id, status in zip(selected_action_id_list, selected_action_status_list):
                if status != ActionStatus.IDLE:
                    rospy.logwarn("Cannot delete actions whose state is not IDLE.")
                else:
                    req.action_id = action_id
                    res = self.delete_action_service_proxy(req)
                    show_error_msg(res.error_msg)
                    rospy.loginfo("Delete selected action (id: %s)" % req.action_id)
            # clear the command widget
            self.selection_cleared.emit()
        else:
            rospy.logwarn("Service proxy to delete action is not set yet. Action is not deleted.")

    def syncButtonCallback(self):
        """
        Callback function for pressing sync button
        :return:
        """
        rospy.logdebug("Function ScheduleWidget.syncButtonCallback is called")
        if self.update_action_status_service_proxy is not None and self.reorder_actions_service_proxy is not None:
            non_sync_action_table_view = self.findChild(NonSyncActionTableView, 'nonSyncActionTableView')
            indexes = non_sync_action_table_view.selectionModel().selectedRows()
            # Sort selected indices because selected indices are not sorted when ctrl modifier is used
            indexes = sorted(indexes)
            if len(indexes) == 0:
                # If there is no action selected, synchronize all actions
                self.synchronise_to_agent(list(range(non_sync_action_table_view.model.rowCount())))
            elif len(indexes) > 0:
                index_rows = [index.row() for index in indexes]
                index_rows = sorted(index_rows)
                self.synchronise_to_agent(index_rows)
            else:
                rospy.logwarn("No IDLE actions are selected. Synchronization failed.")
        else:
            rospy.logwarn("Service proxy to update action status is not set yet. Synchronization failed.")

    def synchronise_to_agent(self, index_rows):
        """
        It changes status of the actions from IDLE to IDLE_SYNCHRONIZED if they are in NonSyncActionTableView.
        It also reorders actions so that the selected actions are located next to the actions in SyncActionTableView.
        If multiple rows are selected, all IDLE actions existing before the last one are synchronised.
        :param index_rows: list of int
        :return:
        """
        rospy.logdebug("Function ScheduleWidget.synchronise_to_agent is called")
        rospy.logdebug("Function ScheduleWidget.synchronise_to_agent is called: index_rows = " + str(index_rows))

        # Ensure that the index_rows are sorted
        index_rows = sorted(index_rows)
        # If the index_rows is empty (i.e. no actions inside non-sync part), do nothing
        if len(index_rows) == 0:
            rospy.loginfo("There is no action in non-sync part. Skip synchronization.")
            return

        last_row = index_rows[-1]

        non_sync_action_table_view = self.findChild(NonSyncActionTableView, 'nonSyncActionTableView')

        sync_actions_row_count = self.findChild(SyncActionTableView, 'syncActionTableView').model.rowCount()
        non_sync_actions_row_count = non_sync_action_table_view.model.rowCount()
        sync_rows_order = range(sync_actions_row_count)
        non_sync_rows_order = [i + sync_actions_row_count for i in range(non_sync_actions_row_count)]

        # Candidate rows to be synchronised are all rows before last_row
        index_rows = range(non_sync_actions_row_count)[:last_row + 1]
        print(index_rows)

        id_column = non_sync_action_table_view.get_column_from_label('id')
        status_column = non_sync_action_table_view.get_column_from_label('status')
        rospy.logdebug("Function ScheduleWidget.synchronise_to_agent: row_count = %d, column_count = %d" % (non_sync_action_table_view.model.rowCount(), non_sync_action_table_view.model.columnCount()))
        for i in range(non_sync_action_table_view.model.rowCount()):
            for j in range(non_sync_action_table_view.model.columnCount()):
                rospy.logdebug("Function ScheduleWidget.synchronise_to_agent: hasIndex(%d, %d) = %s" %
                               (i, j, non_sync_action_table_view.model.hasIndex(i, j)))
                rospy.logdebug("Function ScheduleWidget.synchronise_to_agent: index(%d, %d).data() = %s" %
                               (i, j, str(non_sync_action_table_view.model.index(i, j).data())))

        ids = [str(non_sync_action_table_view.model.index(index_row, id_column).data()) for index_row in index_rows]
        statuses = [int(non_sync_action_table_view.model.index(index_row, status_column).data()) for index_row in
                    index_rows]

        # Find the last valid row to be synchornised
        last_row_to_be_unsynced = None
        for index_row, (id, status) in zip(reversed(index_rows), zip(reversed(ids), reversed(statuses))):
            if status == ActionStatus.IDLE:
                last_row_to_be_unsynced = index_row
                break

        # If no valid row is found, do nothing
        if last_row_to_be_unsynced is None:
            rospy.logwarn("Unable to synchornise actions. No row is able to be synchronised.")
            return

        # Create arranged index_rows_ so that all front rows in NonSyncActionTableView are synchronised
        index_rows_ = range(non_sync_actions_row_count)[:last_row_to_be_unsynced + 1]
        ids_ = [str(non_sync_action_table_view.model.index(index_row, id_column).data()) for index_row in index_rows_]
        statuses_ = [int(non_sync_action_table_view.model.index(index_row, status_column).data()) for index_row in
                     index_rows_]
        rows_to_be_synced = []

        for index_row, (id, status) in zip(index_rows_, zip(ids_, statuses_)):
            if status == ActionStatus.IDLE:
                req = UpdateActionStatusRequest()
                req.action_id = id
                req.updated_status.value = ActionStatus.IDLE_SYNCHRONISED
                res = self.update_action_status_service_proxy(req)
                show_error_msg(res.error_msg)
                rows_to_be_synced.append(index_row + sync_actions_row_count)
                rospy.loginfo("Synchronised action (id: %s) to %s" % (id, self.objectName()))
        # Reorder them after updating status
        for i in reversed(rows_to_be_synced):
            del non_sync_rows_order[non_sync_rows_order.index(i)]
            non_sync_rows_order = [i] + non_sync_rows_order
        req = ReorderActionsRequest()
        req.agent_name = str(self.objectName())
        req.new_order = sync_rows_order + non_sync_rows_order
        res = self.reorder_actions_service_proxy(req)
        show_error_msg(res.error_msg)
        rospy.loginfo("Reordered synchronised actions")

    def synchronise_to_agent_callback(self, index_rows, agent_name):
        """
        Callback function triggered by signal from MissionControlPlugin
        to synchronise actions to agent
        """
        rospy.logdebug("Function ScheduleWidget.synchronise_to_agent_callback is called")
        if agent_name != str(self.objectName()):
            return
        self.synchronise_to_agent(index_rows)

    def unsyncButtonCallback(self):
        """
        Callback function for pressing unsync button
        :return:
        """
        rospy.logdebug("Function ScheduleWidget.unsyncButtonCallback is called")
        if self.update_action_status_service_proxy is not None and self.reorder_actions_service_proxy is not None:
            sync_action_table_view = self.findChild(SyncActionTableView, 'syncActionTableView')
            indexes = sync_action_table_view.selectionModel().selectedRows()
            # Sort selected indices because selected indices are not sorted when ctrl modifier is used
            indexes = sorted(indexes)
            if len(indexes) == 0:
                self.unsynchronise_from_agent([0])  # This automatically checks if action can be unsynchronised or not
            elif len(indexes) > 0:
                index_rows = [index.row() for index in indexes]
                index_rows = sorted(index_rows)
                self.unsynchronise_from_agent(index_rows)
            else:
                rospy.logwarn("No IDLE_SYNCHRONISED actions are selected. Synchronization failed.")
        else:
            rospy.logwarn("Service proxy to update action status is not set yet. Unsynchronization failed.")

    def unsynchronise_from_agent(self, index_rows, forcefully=False):
        """
        It changes status of the actions from IDLE_SYNCHRONIZED to IDLE if they are in SyncActionTableView.
        It also reorders actions so that the selected actions are located before the actions in NonSyncActionTableView.
        If multiple rows are selected, all IDLE_SYNCHRONISED actions following after the first one are unsynchronised.
        If the option 'forcefully' is set True, it unsynchronise all actions regardless of their status.
        This option should be used to allow uses to somehow reset the mission regardless of action status,
        e.g. the mission executor crashed and users want to re-compose a mission again.
        The option is utilized by the reset button.
        :param index_rows: list of int
        :param forcefully: bool
        :return:
        """
        rospy.logdebug("Function ScheduleWidget.unsynchronise_from_agent is called")

        index_rows = sorted(index_rows)
        # If the index_rows is empty (i.e. no actions inside sync part), do nothing
        if len(index_rows) == 0:
            rospy.loginfo("There is no action in sync part. Skip synchronization.")
            return
        first_row = index_rows[0]

        sync_action_table_view = self.findChild(SyncActionTableView, 'syncActionTableView')

        sync_actions_row_count = sync_action_table_view.model.rowCount()
        non_sync_actions_row_count = self.findChild(NonSyncActionTableView, 'nonSyncActionTableView').model.rowCount()
        sync_rows_order = range(sync_actions_row_count)
        non_sync_rows_order = [i + sync_actions_row_count for i in range(non_sync_actions_row_count)]

        # Candidate rows to be unsynchronised are all rows that follow first_row
        index_rows = sync_rows_order[first_row:]

        id_column = sync_action_table_view.get_column_from_label('id')
        status_column = sync_action_table_view.get_column_from_label('status')

        ids = [str(sync_action_table_view.model.index(index_row, id_column).data()) for index_row in index_rows]
        statuses = [int(sync_action_table_view.model.index(index_row, status_column).data()) for index_row in
                    index_rows]

        # Find the first valid row to be unsynchornised
        first_row_to_be_unsynced = None
        if forcefully:
            first_row_to_be_unsynced = 0
        else:
            for index_row, (id, status) in zip(index_rows, zip(ids, statuses)):
                if status == ActionStatus.IDLE_SYNCHRONISED:
                    first_row_to_be_unsynced = index_row
                    break

        # If no valid row is found, do nothing
        if first_row_to_be_unsynced is None:
            rospy.logwarn("Unable to unsynchornise actions. No row is able to be unsynchronised.")
            return

        # Create arranged index_rows_ so that the rest of all rows in SyncActionTableView are unsynchronised
        index_rows_ = sync_rows_order[first_row_to_be_unsynced:]
        ids_ = [str(sync_action_table_view.model.index(index_row, id_column).data()) for index_row in index_rows_]
        statuses_ = [int(sync_action_table_view.model.index(index_row, status_column).data()) for index_row in
                     index_rows_]
        rows_to_be_unsynced = []
        for index_row, (id, status) in zip(index_rows_, zip(ids_, statuses_)):
            req = UpdateActionStatusRequest()
            req.action_id = id
            req.updated_status.value = ActionStatus.IDLE
            res = self.update_action_status_service_proxy(req)
            show_error_msg(res.error_msg)
            rows_to_be_unsynced.append(index_row)
            rospy.loginfo("Unsynchronised action (id: %s) to %s" % (id, self.objectName()))

        # Reorder them after updating status
        for i in rows_to_be_unsynced:
            del sync_rows_order[sync_rows_order.index(i)]
            sync_rows_order = sync_rows_order + [i]
        req = ReorderActionsRequest()
        req.agent_name = str(self.objectName())
        req.new_order = sync_rows_order + non_sync_rows_order
        res = self.reorder_actions_service_proxy(req)
        show_error_msg(res.error_msg)
        rospy.loginfo("Reordered synchronised actions")

    def unsynchronise_from_agent_callback(self, index_rows, agent_name):
        """
        Callback function triggered by signal from MissionControlPlugin
        to synchronise actions to agent
        """
        rospy.logdebug("Function ScheduleWidget.unsynchronise_from_agent_callback is called")
        if agent_name != str(self.objectName()):
            return
        self.unsynchronise_from_agent(index_rows)

    def runButtonCallback(self):
        rospy.logdebug("Function ScheduleWidget.runButtonCallback is called")
        if self.trigger_mission_executor_service_proxy is not None:
            req = TriggerMissionExecutorRequest()
            req.trigger = TriggerMissionExecutorRequest.RUN
            req.agent_name = str(self.objectName())
            try:
                res = self.trigger_mission_executor_service_proxy(req)
            except rospy.ServiceException as e:
                rospy.logerr(e)
                show_error_msg(e)
                return
            self.process_trigger_mission_executor_result(res)
            rospy.loginfo("Triggered run command.")
        else:
            rospy.logwarn("Service proxy to run actions is not set yet. Actions are not run.")

    def pauseButtonCallback(self):
        rospy.logdebug("Function ScheduleWidget.pauseButtonCallback is called")
        if self.trigger_mission_executor_service_proxy is not None:
            req = TriggerMissionExecutorRequest()
            req.trigger = TriggerMissionExecutorRequest.PAUSE
            req.agent_name = str(self.objectName())
            try:
                res = self.trigger_mission_executor_service_proxy(req)
            except rospy.ServiceException as e:
                rospy.logerr(e)
                show_error_msg(e)
                return
            self.process_trigger_mission_executor_result(res)
            rospy.loginfo("Triggered pause command.")
        else:
            rospy.logwarn("Service proxy to pause actions is not set yet. Actions are not paused.")

    def stopButtonCallback(self):
        rospy.logdebug("Function ScheduleWidget.stopButtonCallback is called")
        if self.trigger_mission_executor_service_proxy is not None:
            req = TriggerMissionExecutorRequest()
            req.trigger = TriggerMissionExecutorRequest.STOP
            req.agent_name = str(self.objectName())
            try:
                res = self.trigger_mission_executor_service_proxy(req)
            except rospy.ServiceException as e:
                rospy.logerr(e)
                show_error_msg(e)
                return
            self.process_trigger_mission_executor_result(res)
            rospy.loginfo("Triggered stop command.")
        else:
            rospy.logwarn("Service proxy to run actions is not set yet. Actions are not run.")

    def resetButtonCallback(self):
        rospy.logdebug("Function ScheduleWidget.resetButtonCallback is called")
        #if self.trigger_mission_executor_service_proxy is not None:
        #    req = TriggerMissionExecutorRequest()
        #    req.trigger = TriggerMissionExecutorRequest.RESET
        #    req.agent_name = str(self.objectName())
        #    try:
        #        res = self.trigger_mission_executor_service_proxy(req)
        #    except rospy.ServiceException as e:
        #        rospy.logerr(e)
        #        show_error_msg(e)
        #        return
        #    self.process_trigger_mission_executor_result(res)
        #    rospy.loginfo("Triggered reset command.")
        #else:
        #    rospy.logwarn("Service proxy to run actions is not set yet. Actions are not run.")

        # If we press reset, it changes all the execution status to idle so that we can unsynchronize
        # and then restart mission executor safely
        sync_action_table_view = self.findChild(SyncActionTableView, 'syncActionTableView')
        sync_actions_row_count = sync_action_table_view.model.rowCount()
        index_rows = range(sync_actions_row_count)
        self.unsynchronise_from_agent(index_rows, forcefully=True)

    def process_trigger_mission_executor_result(self, trigger_mission_executor_result):
        """
        Pop up message box if trigger_mission_executor_result.success is False
        :param trigger_mission_executor_result:
        :return:
        """
        if trigger_mission_executor_result.success:
            return
        msg = QMessageBox()
        msg.setIcon(QMessageBox.Critical)
        msg.setText("Service TriggerMissionExecutor failed.")
        msg.setInformativeText(trigger_mission_executor_result.msg)
        msg.setStandardButtons(QMessageBox.Ok)
        retval = msg.exec_()

    def setServiceProxies(self, add_action_service_proxy, trigger_mission_executor_service_proxy,
                          delete_action_service_proxy, reorder_actions_service_proxy,
                          update_action_status_service_proxy, edit_action_service_proxy):
        rospy.logdebug("Function ScheduleWidget.setServiceProxies is called")
        self.add_action_service_proxy = add_action_service_proxy
        self.trigger_mission_executor_service_proxy = trigger_mission_executor_service_proxy
        self.delete_action_service_proxy = delete_action_service_proxy
        self.reorder_actions_service_proxy = reorder_actions_service_proxy
        self.update_action_status_service_proxy = update_action_status_service_proxy
        self.edit_action_service_proxy = edit_action_service_proxy

    def action_table_item_reselection_signal_callback(self, agent_name, table_view_name, index):
        rospy.logdebug("Function ScheduleWidget.action_table_item_reselection_signal_callback is called")

        rospy.logdebug("Function ScheduleWidget.action_table_item_reselection_signal_callback: Waits for mutex")
        while self.action_table_item_reselection_signal_mutex:
            pass
        rospy.logdebug("Function ScheduleWidget.action_table_item_reselection_signal_callback: Mutex acquired")
        self.action_table_item_reselection_signal_mutex = True

        if agent_name == str(self.objectName()):
            action_table_view = self.findChild(ActionTableView, table_view_name)
            action_table_view.clearSelection()  # For emitting signals selectionChanged
            action_table_view.setCurrentIndex(index)

        rospy.logdebug("Function ScheduleWidget.action_table_item_reselection_signal_callback: Mutex unlocked")
        self.action_table_item_reselection_signal_mutex = False

    def action_table_clear_selection_signal_callback(self, agent_name, sync_or_nonsync):
        rospy.logdebug("Function ScheduleWidget.action_table_clear_selection_signal_callback is called")

        if agent_name != str(self.objectName()):
            return

        if sync_or_nonsync == 'sync':
            self.sync_action_table_clear_selection_signal.emit()
        elif sync_or_nonsync == 'nonsync':
            self.non_sync_action_table_clear_selection_signal.emit()
        else:
            rospy.logerr("Unexpected value of sync_or_nonsync: {}".format(sync_or_nonsync))

    def schedule_widget_set_enabled_signal_callback(self, flag):
        rospy.logdebug("Function ScheduleWidget.schedule_widget_set_enabled_signal_callback is called")
        self.setEnabled(flag)

    def update_visualization_callback(self, mission_req):
        rospy.logdebug("Function ScheduleWidget.update_visualization_callback is called")

        # Extract corresponding agent_actions
        for agent_actions in mission_req.agents_actions:
            agent_name = agent_actions.agent_name
            if agent_name == str(self.objectName()):
                break

        # Update each table
        self.sync_action_table_update_visualization_signal.emit(agent_actions.actions, mission_req.sync_ids_list)
        self.non_sync_action_table_update_visualization_signal.emit(agent_actions.actions, mission_req.sync_ids_list)


class Testing(QMainWindow):

    def __init__(self, agent_actions):
        super(Testing, self).__init__()
        w = ScheduleWidget(agent_actions)
        self.setCentralWidget(w)
        self.setWindowTitle('Testing')
        self.show()


if __name__ == '__main__':
    from rosmc_msgs.msg import *

    # Prepare dummy agent_actions
    agent_actions = AgentActions()
    agent_actions.agent_name = 'turtle1'

    action = Action()
    action.action_id = 'test-test-test-001'
    action_content = ActionContent()
    action_content.action_name = 'explore'
    action_content.action_label = 'explore_1'
    action_content.battery_flag = 'True'
    action_content.wlan_flag = 'False'
    action_content.manual_flag = 'False'
    action_content.status = ActionStatus.SUCCESS
    action.action_content = action_content
    agent_actions.actions.append(action)

    action = Action()
    action.action_id = 'test-test-test-002'
    action_content = ActionContent()
    action_content.action_name = 'follow_turtle'
    action_content.action_label = 'follow_turtle_1'
    action_content.battery_flag = 'False'
    action_content.wlan_flag = 'True'
    action_content.manual_flag = 'True'
    action_content.status = ActionStatus.IDLE
    action.action_content = action_content
    agent_actions.actions.append(action)

    app = QApplication([])
    test = Testing(agent_actions)
    sys.exit(app.exec_())
