# Copyright 2020 RUVU Robotics B.V.

from python_qt_binding.QtWidgets import QInputDialog, QLineEdit


def get_double_from_input(parent, label, initial_value, min_value, max_value, decimals=1):
    """
    Get a double from user input
    :param parent: Parent widget
    :param label: Label of the value
    :param initial_value: Initial value
    :param min_value: Minimum bound
    :param max_value: Maximum bound
    :return: The chosen value
    """
    value, ok_pressed = QInputDialog.getDouble(parent, "Please enter the {}".format(label), "{}:".format(label),
                                               initial_value, min_value, max_value, decimals)
    if ok_pressed:
        return value
    else:
        return initial_value


def get_string_from_input(parent, label, initial_value):
    """
    Get a string from use input
    :param parent: Parent widget
    :param label: Label of the value
    :param initial_value: Initial value
    :return: The chosen value
    """
    value, ok_pressed = QInputDialog.getText(parent, "Please enter the {}".format(label), "{}:".format(label),
                                             QLineEdit.Normal, initial_value)
    if ok_pressed:
        return value
    else:
        return initial_value
