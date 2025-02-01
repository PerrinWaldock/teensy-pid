import QtQuick
import QtQuick.Layouts
import QtQuick.Controls
import QtQuick.Window
import QtQuick.Controls.Material
import io.qt.textproperties

ApplicationWindow {
    id: page
    width: 800
    height: 400
    visible: true
    Material.theme: Material.Light
    Material.accent: Material.Red

    Bridge {
        id: bridge
    }

    Rectangle {
        id: main
        width: 200
        height: 200
        color: "green"

        Text {
            text: "Hello World"
            anchors.centerIn: main
        }
    }
}
