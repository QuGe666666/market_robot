"""Small Qt binding compatibility layer.

The robot image currently ships PyQt5. PySide6 is preferred when available,
without making mock mode depend on it.
"""

try:
    from PySide6 import QtCore, QtGui, QtWidgets

    Signal = QtCore.Signal
    Slot = QtCore.Slot
    QT_BINDING = "PySide6"

    def app_exec(app):
        return app.exec()

except ImportError:
    from PyQt5 import QtCore, QtGui, QtWidgets

    Signal = QtCore.pyqtSignal
    Slot = QtCore.pyqtSlot
    QT_BINDING = "PyQt5"

    def app_exec(app):
        return app.exec_()

