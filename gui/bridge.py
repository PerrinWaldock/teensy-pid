
from PySide6.QtQml import QmlElement
from PySide6.QtCore import QObject, Slot

QML_IMPORT_NAME = "io.qt.textproperties"
QML_IMPORT_MAJOR_VERSION = 1

@QmlElement
class Bridge(QObject):

    @Slot(str, result=str)
    def getColor(self, s):
        if s.lower() == "red":
            return "#ef9a9a"
        if s.lower() == "green":
            return "#a5d6a7"
        if s.lower() == "blue":
            return "#90caf9"
        return "white"

    @Slot(float, result=int)
    def getSize(self, s):
        size = int(s * 34)
        return max(1, size)

    @Slot(str, result=bool)
    def getItalic(self, s):
        return s.lower() == "italic"

    @Slot(str, result=bool)
    def getBold(self, s):
        return s.lower() == "bold"

    @Slot(str, result=bool)
    def getUnderline(self, s):
        return s.lower() == "underline"