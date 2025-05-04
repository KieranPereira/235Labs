import sys
import os
import time
import math
import re
import logging
import serial
from serial import SerialException

from PyQt5 import QtCore
from PyQt5.QtWidgets import (
    QApplication, QWidget, QHBoxLayout, QVBoxLayout,
    QSlider, QPushButton, QLabel
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QThread, pyqtSlot
from PyQt5.QtGui import (
    QPainter, QPen, QFont, QPixmap, QImage,
    QPalette, QColor
)
from PIL import Image, ImageDraw, ImageChops

# ——— Logging setup ———
logging.basicConfig(
    level=logging.DEBUG,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S"
)
logger = logging.getLogger(__name__)

# Configuration
PORT               = "COM8"
BAUD               = 115200
ROLL_THRESHOLD     = 10
PITCH_THRESHOLD    = 10
MAX_POINTS         = 1000
FLASH_DURATION_MS  = 200   # ms for quadrant flash
RETRY_DELAY        = 5     # seconds between retry attempts


class SerialWorker(QThread):
    newData     = pyqtSignal(float, float, float)  # t, pitch, roll
    sendCommand = pyqtSignal(bytes)

    def __init__(self, port, baud):
        super().__init__()
        self.port = port
        self.baud = baud
        self._running = True
        self.ser = None
        self.sendCommand.connect(self.onSend)

    def open_serial(self):
        """Block until we can open the port, retrying every RETRY_DELAY."""
        while self._running:
            try:
                logger.info(f"Opening serial port {self.port} @ {self.baud}")
                self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
                logger.info("Serial port opened successfully")
                return
            except (SerialException, PermissionError, OSError) as e:
                logger.warning(f"Failed to open {self.port}: {e}. Retrying in {RETRY_DELAY}s")
                time.sleep(RETRY_DELAY)

    @pyqtSlot(bytes)
    def onSend(self, data: bytes):
        """Handle outgoing commands; special-case REBOOT to close port."""
        if data == b'REBOOT\n':
            logger.info("→ REBOOT command sent, closing port to allow ESP restart")
            try:
                if self.ser and self.ser.is_open:
                    self.ser.write(data)
                    logger.debug(f"Sent reboot: {data!r}")
            except Exception as e:
                logger.error(f"Error sending reboot: {e}")
            # close and clear handle; run() will detect next iteration
            try:
                if self.ser:
                    self.ser.close()
                    logger.debug("Serial port closed for reboot")
            except Exception as e:
                logger.error(f"Error closing port for reboot: {e}")
            self.ser = None
            return

        # all other commands
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(data)
                logger.debug(f"Sent command: {data!r}")
            except SerialException as e:
                logger.error(f"Error writing to serial: {e}")

    def run(self):
        pattern = re.compile(
            r'^\s*([0-9]+\.[0-9]+),'      # 1) timestamp
            r'\s*([+-]?\d+(?:\.\d+)?),'   # 2) roll
            r'\s*([+-]?\d+(?:\.\d+)?)'    # 3) pitch
        )

        self.open_serial()

        while self._running:
            try:
                raw = self.ser.readline()
                line = raw.decode('utf-8', 'ignore').strip()
            except Exception as e:
                logger.error(f"Connection lost: {e}. Retrying in {RETRY_DELAY}s")
                try:
                    if self.ser:
                        self.ser.close()
                        logger.debug("Serial port closed after loss")
                except:
                    pass
                self.ser = None
                time.sleep(RETRY_DELAY)
                self.open_serial()
                continue

            if not line:
                continue

            logger.debug(f"Received line: {line}")
            m = pattern.match(line)
            if not m:
                logger.debug("Line did not match telemetry pattern")
                continue

            t     = float(m.group(1))
            roll  = float(m.group(2))
            pitch = float(m.group(3))
            logger.debug(f"Parsed data → t:{t}, roll:{roll}, pitch:{pitch}")
            self.newData.emit(t, pitch, roll)

    def stop(self):
        self._running = False
        try:
            if self.ser:
                self.ser.close()
                logger.info("Serial port closed on stop()")
        except:
            pass
        self.wait()


def make_silhouette_mask(img: Image.Image) -> Image.Image:
    gray   = img.convert("L")
    stroke = gray.point(lambda p: 255 if p < 128 else 0)
    white  = gray.point(lambda p: 255 if p > 128 else 0)
    ImageDraw.floodfill(white, (0, 0), 0)
    return ImageChops.add(stroke, white)


class HeatmapWidget(QWidget):
    sectionClicked = pyqtSignal(str)

    def __init__(self, serial_thread, parent=None):
        super().__init__(parent)
        self.serial_thread = serial_thread
        img_path  = os.path.join(os.path.dirname(__file__), "human_outline.jpg")
        self.mask  = make_silhouette_mask(Image.open(img_path).convert("RGBA"))
        self.roll  = 0.0
        self.pitch = 0.0
        self.flash_section = None

    def setAngles(self, roll, pitch):
        self.roll, self.pitch = roll, pitch
        self.update()

    def mousePressEvent(self, event):
        x, y = event.x(), event.y()
        w, h = self.width(), self.height()
        if x < w/2 and y < h/2:
            section, msg = 'TL', b'Q1\n'
        elif x >= w/2 and y < h/2:
            section, msg = 'TR', b'Q2\n'
        elif x < w/2 and y >= h/2:
            section, msg = 'BL', b'Q3\n'
        else:
            section, msg = 'BR', b'Q4\n'

        logger.info(f"Click in {section}, sending {msg!r}")
        self.serial_thread.sendCommand.emit(msg)

        self.flash_section = section
        QTimer.singleShot(FLASH_DURATION_MS, self.clearFlash)
        self.update()
        self.sectionClicked.emit(section)

    def clearFlash(self):
        self.flash_section = None
        self.update()

    def paintEvent(self, event):
        painter     = QPainter(self)
        w, h        = self.width(), self.height()
        mask_data   = self.mask.resize((w, h), Image.NEAREST).load()
        out         = Image.new("RGBA", (w, h), (0,0,0,0))
        pix         = out.load()

        for yy in range(h):
            top = yy < h//2
            for xx in range(w):
                if mask_data[xx, yy] != 255:
                    continue
                left = xx < w//2
                if top and left:
                    value, thresh = -self.roll, ROLL_THRESHOLD
                elif top and not left:
                    value, thresh = self.roll, ROLL_THRESHOLD
                elif not top and left:
                    value, thresh = -self.pitch, PITCH_THRESHOLD
                else:
                    value, thresh = self.pitch, PITCH_THRESHOLD
                if value > thresh:
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
        self.half = self.size // 2
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
        c = QColor(220,0,0) if abs(self.value)>self.threshold else QColor(220,220,220)
        painter.setPen(QPen(c))
        painter.setFont(QFont("Arial",12))
        painter.drawText(self.center-20,self.size-20,f"{self.value:.1f}°")    


class LeftPanel(QWidget):
    def __init__(self, serial_thread, parent=None):
        super().__init__(parent)
        self.serial_thread = serial_thread

        self.roll_display  = AngleHistoryWidget("Roll", Qt.cyan,   ROLL_THRESHOLD)
        self.pitch_display = AngleHistoryWidget("Pitch",Qt.magenta,PITCH_THRESHOLD)
        self.slider        = QSlider(Qt.Horizontal)
        self.time_label    = QLabel("0.0 s")
        self.status_label  = QLabel("Status: Disconnected")
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
        bottom.addWidget(self.status_label)
        bottom.addWidget(self.sync_button)
        bottom.addWidget(self.reboot_button)

        layout = QVBoxLayout(self)
        layout.addLayout(top)
        layout.addLayout(bottom)

        self.time_data, self.roll_data, self.pitch_data = [], [], []
        self.last_data_time = 0
        self.follow = True
        self.slider.sliderPressed.connect(lambda: setattr(self, 'follow', False))

        # status checker
        self.check_timer = QTimer(self)
        self.check_timer.timeout.connect(self.check_status)
        self.check_timer.start(1000)

    def sync_to_live(self):
        if self.time_data:
            self.slider.setRange(0, len(self.time_data)-1)
            self.slider.setValue(self.slider.maximum())
            self.follow = True

    def sendReboot(self):
        logger.info("GUI → sending REBOOT")
        self.serial_thread.sendCommand.emit(b'REBOOT\n')

    @pyqtSlot(float, float, float)
    def onSerialData(self, t, pitch, roll):
        logger.debug(f"GUI received data: t={t}, pitch={pitch}, roll={roll}")
        self.time_data.append(t)
        self.pitch_data.append(pitch)
        self.roll_data.append(roll)
        self.last_data_time = time.time()
        self.status_label.setText("Status: Live")

        if len(self.time_data) > MAX_POINTS:
            self.time_data.pop(0)
            self.pitch_data.pop(0)
            self.roll_data.pop(0)

        cnt = len(self.time_data)
        self.slider.setRange(0, cnt-1)
        if self.follow:
            self.slider.setValue(cnt-1)
        self.redraw()

    def check_status(self):
        if time.time() - self.last_data_time > RETRY_DELAY:
            self.status_label.setText("Status: No Data")

    def redraw(self):
        idx = self.slider.value()
        if 0 <= idx < len(self.roll_data):
            self.roll_display.setValue(self.roll_data[idx])
            self.pitch_display.setValue(self.pitch_data[idx])
            self.time_label.setText(f"{self.time_data[idx]:.1f} s")


class SwimMonitor(QWidget):
    def __init__(self, serial_thread):
        super().__init__()
        self.serial_thread = serial_thread
        self.setWindowTitle("Swim Form Monitor")
        self.resize(900, 400)

        self.left_panel = LeftPanel(serial_thread, parent=self)
        self.heatmap    = HeatmapWidget(serial_thread, parent=self)

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
    logger.info("Starting application")
    app = QApplication(sys.argv)

    # Dark theme
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

    ser_thread = SerialWorker(PORT, BAUD)
    ser_thread.start()
    logger.info("SerialWorker thread started")

    window = SwimMonitor(ser_thread)
    window.show()

    ser_thread.newData.connect(window.left_panel.onSerialData)

    ret = app.exec_()
    ser_thread.stop()
    logger.info("Application exiting")
    sys.exit(ret)
