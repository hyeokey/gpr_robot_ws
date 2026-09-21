import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Layouts 1.3

Rectangle {
  Layout.minimumWidth: 360
  Layout.minimumHeight: 300
  color: "#f3f3f3"

  ColumnLayout {
    anchors.fill: parent
    anchors.margins: 12
    spacing: 6

    Label {
      text: "Lateral Position Y [m]"
      font.bold: true
    }
    RowLayout {
      spacing: 8
      TextField {
        id: yField
        Layout.preferredWidth: 90
        validator: DoubleValidator { bottom: PiperPlatformGui.minY; top: PiperPlatformGui.maxY; decimals: 3 }
        onEditingFinished: PiperPlatformGui.targetY = parseFloat(text)
      }
      Binding {
        target: yField; property: "text"
        value: PiperPlatformGui.targetY.toFixed(2)
        when: !yField.activeFocus
      }
      Slider {
        id: ySlider
        Layout.fillWidth: true
        from: PiperPlatformGui.minY
        to: PiperPlatformGui.maxY
        onMoved: PiperPlatformGui.targetY = value
      }
      Binding {
        target: ySlider; property: "value"
        value: PiperPlatformGui.targetY
        when: !ySlider.pressed
      }
    }
    Label {
      text: "Target Y: " + PiperPlatformGui.targetY.toFixed(2) +
          " m   Current Y: " + PiperPlatformGui.currentY.toFixed(2) + " m"
    }

    Label {
      text: "Platform Height Z [m]"
      font.bold: true
      Layout.topMargin: 10
    }
    RowLayout {
      spacing: 8
      TextField {
        id: zField
        Layout.preferredWidth: 90
        validator: DoubleValidator { bottom: PiperPlatformGui.minZ; top: PiperPlatformGui.maxZ; decimals: 3 }
        onEditingFinished: PiperPlatformGui.targetZ = parseFloat(text)
      }
      Binding {
        target: zField; property: "text"
        value: PiperPlatformGui.targetZ.toFixed(2)
        when: !zField.activeFocus
      }
      Slider {
        id: zSlider
        Layout.fillWidth: true
        from: PiperPlatformGui.minZ
        to: PiperPlatformGui.maxZ
        onMoved: PiperPlatformGui.targetZ = value
      }
      Binding {
        target: zSlider; property: "value"
        value: PiperPlatformGui.targetZ
        when: !zSlider.pressed
      }
    }
    Label {
      text: "Target Z: " + PiperPlatformGui.targetZ.toFixed(2) +
          " m   Current Z: " + PiperPlatformGui.currentZ.toFixed(2) + " m"
    }

    RowLayout {
      Layout.topMargin: 12
      spacing: 8
      Button { text: "Apply"; onClicked: PiperPlatformGui.Apply() }
      Button { text: "Center"; onClicked: PiperPlatformGui.Center() }
      Button { text: "Reset"; onClicked: PiperPlatformGui.Reset() }
    }

    Label {
      Layout.topMargin: 8
      Layout.fillWidth: true
      wrapMode: Text.WordWrap
      text: PiperPlatformGui.statusMessage
      color: PiperPlatformGui.statusMessage === "정상" ? "#1b7a1b" : "#b45309"
    }

    Item { Layout.fillHeight: true }
  }
}
