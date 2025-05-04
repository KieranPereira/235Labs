import sys
import os
import time
import math
import re
import serial

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import (
    QApplication, QWidget, QHBoxLayout, QVBoxLayout,
    QSizePolicy, QSlider, QPushButton, QLabel
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QThread
from PyQt5.QtGui import (
    QPainter, QPen, QFont, QPixmap, QImage,
    QPalette, QColor
)
from PIL import Image, ImageDraw, ImageChops

# Configuration
PORT = "COM8"
BAUD = 115200
ROLL_THRESHOLD    = 10
PITCH_THRESHOLD   = 10
MAX_POINTS        = 1000
FLASH_DURATION_MS = 200  # ms for quadrant flash

# Single serial instance for read/write
ser = serial.Serial(PORT, BAUD, timeout=0.01)

class SerialWorker(QThread):
    newData = pyqtSignal(float, float, float)  # t, pitch, roll

    def __init__(self, port, baud):
        super().__init__()
        self._running = True

    def run(self):
        # 1) timestamp
        # 2) back.roll
        # 3) top.pitch
        pattern = re.compile(
            r'^\s*([0-9]+\.[0-9]+),'      # 1) timestamp
            r'\s*([+-]?\d+(?:\.\d+)?),'   # 2) roll lower back
            r'\s*([+-]?\d+(?:\.\d+)?)'    # 3) pitch neck
        )

        while self._running:
            line = ser.readline().decode('utf-8', 'ignore').strip()
            m = pattern.match(line)
            if not m:
                continue

            t    = float(m.group(1))
            roll = float(m.group(2))
            pitch= float(m.group(3))

            # our GUI slot expects (t, pitch, roll)
            self.newData.emit(t, pitch, roll)

    def stop(self):
        self._running = False
        self.wait()

def make_silhouette_mask(img: Image.Image) -> Image.Image:
    gray   = img.convert("L")
    stroke = gray.point(lambda p: 255 if p < 128 else 0)
    white  = gray.point(lambda p: 255 if p > 128 else 0)
    ImageDraw.floodfill(white, (0, 0), 0)
    return ImageChops.add(stroke, white)

class HeatmapWidget(QWidget):
    sectionClicked = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        img_path    = os.path.join(os.path.dirname(__file__), "human_outline.jpg")
        self.orig   = Image.open(img_path).convert("RGBA")
        self.mask   = make_silhouette_mask(self.orig)
        self.roll   = 0.0
        self.pitch  = 0.0
        self.flash_section = None

    def setAngles(self, roll, pitch):
        self.roll, self.pitch = roll, pitch
        self.update()

    def mousePressEvent(self, event):
        x, y = event.x(), event.y()
        w, h = self.width(), self.height()
        if x < w/2 and y < h/2:
            section, msg = 'TL', 'Q1\n'
        elif x >= w/2 and y < h/2:
            section, msg = 'TR', 'Q2\n'
        elif x < w/2 and y >= h/2:
            section, msg = 'BL', 'Q3\n'
        else:
            section, msg = 'BR', 'Q4\n'
        try:
            ser.write(msg.encode('utf-8'))
        except Exception as e:
            print("Serial write failed:", e)
        self.flash_section = section
        QTimer.singleShot(FLASH_DURATION_MS, self.clearFlash)
        self.update()
        self.sectionClicked.emit(section)
        super().mousePressEvent(event)

    def clearFlash(self):
        self.flash_section = None
        self.update()

    def paintEvent(self, event):
        painter     = QPainter(self)
        w, h        = self.width(), self.height()
        mask_scaled = self.mask.resize((w, h), Image.NEAREST)
        mask_data   = mask_scaled.load()

        out = Image.new("RGBA", (w, h), (0,0,0,0))
        pix = out.load()

        for yy in range(h):
            top  = yy < h//2
            for xx in range(w):
                if mask_data[xx, yy] != 255:
                    continue
                left = xx < w//2
                if top and left:
                    value, threshold = -self.roll, ROLL_THRESHOLD
                elif top and not left:
                    value, threshold = self.roll, ROLL_THRESHOLD
                elif not top and left:
                    value, threshold = -self.pitch, PITCH_THRESHOLD
                else:
                    value, threshold = self.pitch, PITCH_THRESHOLD
                if value > threshold:
                    pix[xx, yy] = (255, 0, 0)

        data = out.convert("RGBA").tobytes("raw","RGBA")
        qimg = QImage(data, w, h, QImage.Format_RGBA8888)
        painter.drawPixmap(0, 0, QPixmap.fromImage(qimg))

        if self.flash_section:
            overlay = QColor(0,0,255,100)
            half_w, half_h = w//2, h//2
            rects = {
                'TL': (0,0,half_w,half_h),
                'TR': (half_w,0,half_w,half_h),
                'BL': (0,half_h,half_w,half_h),
                'BR': (half_w,half_h,half_w,half_h)
            }
            painter.fillRect(*rects[self.flash_section], overlay)

class AngleHistoryWidget(QWidget):
    def __init__(self, label, color, threshold, parent=None):
        super().__init__(parent)
        self.label, self.color, self.threshold = label, color, threshold
        self.size = 300
        self.center = self.size // 2
        self.length = 200
        self.half = self.length / 2
        self.value = 0.0
        self.setFixedSize(self.size, self.size)

    def setValue(self, v):
        self.value = v
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.fillRect(0,0,self.size,self.size, QColor(45,45,45))
        painter.setPen(QColor(200,200,200))
        painter.setFont(QFont("Arial",16))
        painter.drawText(self.center-30,30,self.label)

        rad = math.radians(self.value)
        dx = self.half * math.cos(rad)
        dy = self.half * math.sin(rad)
        x1, y1 = self.center-dx, self.center+dy
        x2, y2 = self.center+dx, self.center-dy

        painter.setPen(QPen(self.color,4))
        painter.drawLine(int(x1),int(y1),int(x2),int(y2))

        tex_color = Qt.red if abs(self.value) > self.threshold else QColor(220,220,220)
        painter.setPen(QPen(tex_color))
        painter.setFont(QFont("Arial",12))
        painter.drawText(self.center-20,self.size-20,f"{self.value:.1f}°")

class LeftPanel(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.roll_display  = AngleHistoryWidget("Roll", Qt.cyan,   ROLL_THRESHOLD)
        self.pitch_display = AngleHistoryWidget("Pitch",Qt.magenta,PITCH_THRESHOLD)
        self.slider        = QSlider(Qt.Horizontal)
        self.time_label    = QLabel("0.0 s")
        self.sync_button   = QPushButton("Sync to Live")
        self.reboot_button = QPushButton("Reboot")

        self.sync_button.clicked.connect(self.sync_to_live)
        self.reboot_button.clicked.connect(self.sendReboot)

        top = QHBoxLayout()
        top.addWidget(self.roll_display)
        top.addWidget(self.pitch_display)
        bottom = QHBoxLayout()
        bottom.addWidget(self.slider)
        bottom.addWidget(self.time_label)
        
        bottom.addWidget(self.sync_button)
        bottom.addWidget(self.reboot_button)


        layout = QVBoxLayout(self)
        layout.addLayout(top)
        layout.addLayout(bottom)

        self.time_data, self.roll_data, self.pitch_data = [], [], []
        self.follow = True

        self.slider.sliderPressed.connect(self.on_slider_pressed)

    def on_slider_pressed(self):
        self.follow = False

    def sync_to_live(self):
        if self.time_data:
            self.slider.setMaximum(len(self.time_data)-1)
            self.slider.setValue(self.slider.maximum())
            self.follow = True

    def sendReboot(self):
        try:
            ser.write(b'REBOOT\n')
        except Exception as e:
            print("Failed to send reboot:", e)

    # def sendStop(self):
    #     try:
    #         ser.write(b'STOP\n')
    #     except Exception as e:
    #         print("Failed to send stop:", e)

    @QtCore.pyqtSlot(float, float, float)
    def onSerialData(self, t, pitch, roll):
        self.time_data.append(t)
        self.pitch_data.append(pitch)
        self.roll_data.append(roll)

        if len(self.time_data) > MAX_POINTS:
            self.time_data.pop(0)
            self.pitch_data.pop(0)
            self.roll_data.pop(0)

        cnt = len(self.time_data)
        self.slider.setMinimum(0)
        self.slider.setMaximum(cnt - 1)
        if self.follow:
            self.slider.setValue(cnt - 1)

        self.redraw()

    def redraw(self):
        idx = self.slider.value()
        if 0 <= idx < len(self.roll_data):
            self.roll_display.setValue(self.roll_data[idx])
            self.pitch_display.setValue(self.pitch_data[idx])
            self.time_label.setText(f"{self.time_data[idx]:.1f} s")

class SwimMonitor(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Swim Form Monitor")
        self.resize(900, 400)

        self.left_panel = LeftPanel()
        self.heatmap    = HeatmapWidget()

        layout = QHBoxLayout(self)
        layout.addWidget(self.left_panel, 1)
        layout.addWidget(self.heatmap,    1)

        timer = QTimer(self)
        timer.timeout.connect(self.update_heatmap)
        timer.start(100)

    def update_heatmap(self):
        if self.left_panel.roll_data:
            roll  = self.left_panel.roll_data[-1]
            pitch = self.left_panel.pitch_data[-1]
        else:
            roll, pitch = 0, 0
        self.heatmap.setAngles(roll, pitch)

if __name__ == "__main__":
    # Dark theme setup
    app = QApplication(sys.argv)
    dark = QPalette()
    dark.setColor(QPalette.Window,        QColor(53, 53, 53))
    dark.setColor(QPalette.WindowText,    Qt.white)
    dark.setColor(QPalette.Base,          QColor(35, 35, 35))
    dark.setColor(QPalette.AlternateBase, QColor(53, 53, 53))
    dark.setColor(QPalette.Text,          Qt.white)
    dark.setColor(QPalette.Button,        QColor(53, 53, 53))
    dark.setColor(QPalette.ButtonText,    Qt.white)
    dark.setColor(QPalette.Highlight,     QColor(42, 130, 218))
    dark.setColor(QPalette.HighlightedText, Qt.black)
    app.setPalette(dark)

    # Main window and serial thread
    window = SwimMonitor()
    window.show()

    ser_thread = SerialWorker(PORT, BAUD)
    ser_thread.newData.connect(window.left_panel.onSerialData)
    ser_thread.start()

    exit_code = app.exec_()
    ser_thread.stop()
    sys.exit(exit_code)

