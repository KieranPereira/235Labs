import sys
import os
import time
import random
import math
from PyQt5.QtWidgets import (
    QApplication, QWidget, QHBoxLayout, QVBoxLayout,
    QSizePolicy, QSlider, QPushButton, QLabel
)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import (
    QPainter, QPen, QFont, QPixmap, QImage,
    QPalette, QColor
)
from PIL import Image, ImageDraw, ImageChops

# Configuration
ROLL_THRESHOLD = 10
PITCH_THRESHOLD = 10
MAX_POINTS = 1000

def make_silhouette_mask(img: Image.Image) -> Image.Image:
    gray = img.convert("L")
    stroke = gray.point(lambda p: 255 if p < 128 else 0)
    white = gray.point(lambda p: 255 if p > 128 else 0)
    ImageDraw.floodfill(white, (0, 0), 0)
    return ImageChops.add(stroke, white)

class HeatmapWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumWidth(200)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        img_path = os.path.join(os.path.dirname(__file__), "human_outline.jpg")
        self.orig = Image.open(img_path).convert("RGBA")
        self.mask = make_silhouette_mask(self.orig)
        self.roll = 0.0
        self.pitch = 0.0

    def setAngles(self, roll, pitch):
        self.roll = roll
        self.pitch = pitch
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        w, h = self.width(), self.height()
        orig_scaled = self.orig.resize((w, h), Image.BILINEAR)
        mask_scaled = self.mask.resize((w, h), Image.NEAREST)
        mask_data = mask_scaled.load()

        out = Image.new("RGBA", (w, h), (0, 0, 0, 0))
        pix = out.load()
        for y in range(h):
            top = (y < h // 2)
            for x in range(w):
                if mask_data[x, y] != 255:
                    continue
                left = (x < w // 2)
                intensity = 0.0
                if top and left and self.roll < 0:
                    intensity = min(-self.roll / 45.0, 1.0)
                elif top and not left and self.roll > 0:
                    intensity = min(self.roll / 45.0, 1.0)
                elif not top and left and self.pitch < 0:
                    intensity = min(-self.pitch / 45.0, 1.0)
                elif not top and not left and self.pitch > 0:
                    intensity = min(self.pitch / 45.0, 1.0)
                if intensity > 0:
                    pix[x, y] = (255, 0, 0, int(255 * intensity))

        data = out.convert("RGBA").tobytes("raw", "RGBA")
        qimg = QImage(data, w, h, QImage.Format_RGBA8888)
        painter.drawPixmap(0, 0, QPixmap.fromImage(qimg))

class AngleHistoryWidget(QWidget):
    def __init__(self, label, color, threshold, parent=None):
        super().__init__(parent)
        self.label = label
        self.color = color
        self.threshold = threshold
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
        painter.fillRect(0, 0, self.size, self.size, QColor(45,45,45))
        painter.setPen(QColor(200,200,200))
        painter.setFont(QFont("Arial", 16))
        painter.drawText(self.center - 30, 30, self.label)

        rad = math.radians(self.value)
        dx = self.half * math.cos(rad)
        dy = self.half * math.sin(rad)
        x1, y1 = self.center - dx, self.center + dy
        x2, y2 = self.center + dx, self.center - dy

        pen = QPen(self.color, 4)
        painter.setPen(pen)
        painter.drawLine(int(x1), int(y1), int(x2), int(y2))

        txt = f"{self.value:.1f}°"
        pen_text = QPen(Qt.red if abs(self.value) > self.threshold else QColor(220,220,220))
        painter.setPen(pen_text)
        painter.setFont(QFont("Arial", 12))
        painter.drawText(self.center - 20, self.size - 20, txt)

class LeftPanel(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.roll_display = AngleHistoryWidget("Roll", Qt.cyan, ROLL_THRESHOLD)
        self.pitch_display = AngleHistoryWidget("Pitch", Qt.magenta, PITCH_THRESHOLD)
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setMinimum(0)
        self.slider.setMaximum(0)
        self.slider.sliderPressed.connect(self.on_slider_pressed)
        self.slider.valueChanged.connect(self.redraw)
        self.time_label = QLabel("0.0 s")
        self.sync_button = QPushButton("Sync to Live")
        self.sync_button.clicked.connect(self.sync_to_live)
        self.sync_button.setStyleSheet(
            "QPushButton { background-color: #353535; color: #FFFFFF; border: 1px solid #2A82DA; padding: 5px 10px; border-radius:4px }"
            "QPushButton:hover { background-color: #454545; }"
        )
        top_layout = QHBoxLayout()
        top_layout.addWidget(self.roll_display)
        top_layout.addWidget(self.pitch_display)
        bottom_layout = QHBoxLayout()
        bottom_layout.addWidget(self.slider)
        bottom_layout.addWidget(self.time_label)
        bottom_layout.addWidget(self.sync_button)
        main_layout = QVBoxLayout(self)
        main_layout.addLayout(top_layout)
        main_layout.addLayout(bottom_layout)
        self.time_data = []
        self.roll_data = []
        self.pitch_data = []
        self.follow = True
        self.start_time = time.time()
        timer = QTimer(self)
        timer.timeout.connect(self.update_loop)
        timer.start(100)

    def on_slider_pressed(self):
        self.follow = False

    def sync_to_live(self):
        if self.time_data:
            self.slider.setMaximum(len(self.time_data) - 1)
            self.slider.setValue(self.slider.maximum())
            self.follow = True

    def update_loop(self):
        now = time.time() - self.start_time
        roll = random.uniform(-30, 30)
        pitch = random.uniform(-30, 30)
        self.time_data.append(now)
        self.roll_data.append(roll)
        self.pitch_data.append(pitch)
        if len(self.time_data) > MAX_POINTS:
            self.time_data.pop(0)
            self.roll_data.pop(0)
            self.pitch_data.pop(0)
        cnt = len(self.time_data)
        self.slider.setMaximum(cnt - 1)
        if self.follow:
            self.slider.setValue(cnt - 1)
        self.redraw()

    def redraw(self):
        idx = self.slider.value()
        if idx < 0 or idx >= len(self.roll_data):
            return
        self.roll_display.setValue(self.roll_data[idx])
        self.pitch_display.setValue(self.pitch_data[idx])
        self.time_label.setText(f"{self.time_data[idx]:.1f} s")

class SwimMonitor(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Swim Form Monitor")
        self.resize(900, 400)
        self.left_panel = LeftPanel()
        self.heatmap = HeatmapWidget()
        layout = QHBoxLayout(self)
        layout.addWidget(self.left_panel, 1)
        layout.addWidget(self.heatmap, 1)
        timer = QTimer(self)
        timer.timeout.connect(self.update_heatmap)
        timer.start(100)

    def update_heatmap(self):
        if self.left_panel.roll_data:
            roll = self.left_panel.roll_data[-1]
            pitch = self.left_panel.pitch_data[-1]
        else:
            roll, pitch = 0, 0
        self.heatmap.setAngles(roll, pitch)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    dark = QPalette()
    dark.setColor(QPalette.Window, QColor(53,53,53))
    dark.setColor(QPalette.WindowText, Qt.white)
    dark.setColor(QPalette.Base, QColor(35,35,35))
    dark.setColor(QPalette.AlternateBase, QColor(53,53,53))
    dark.setColor(QPalette.Text, Qt.white)
    dark.setColor(QPalette.Button, QColor(53,53,53))
    dark.setColor(QPalette.ButtonText, Qt.white)
    dark.setColor(QPalette.Highlight, QColor(42,130,218))
    dark.setColor(QPalette.HighlightedText, Qt.black)
    app.setPalette(dark)
    window = SwimMonitor()
    window.show()
    sys.exit(app.exec_())
