import sys, os, time, math, re, logging, socket, queue
from PyQt5 import QtCore
from PyQt5.QtWidgets import (
    QApplication, QWidget, QHBoxLayout, QVBoxLayout,
    QSlider, QPushButton, QLabel
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QThread, pyqtSlot
from PyQt5.QtGui import QPainter, QPen, QFont, QPixmap, QImage, QPalette, QColor
from PIL import Image, ImageDraw, ImageChops

# ——— Logging setup ———
logging.basicConfig(
    level=logging.DEBUG,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S"
)
logger = logging.getLogger(__name__)
# ——— Configuration ———

BAUD               = 115200     # your ESP32 baud rate
ROLL_THRESHOLD     = 10         # tilt beyond which top zones light up
PITCH_THRESHOLD    = 10         # tilt beyond which bottom zones light up
MAX_POINTS         = 1000       # how many history points to keep
FLASH_DURATION_MS  = 200        # how long the clicked quadrant flashes
RETRY_DELAY        = 5          # seconds between serial‐retry attempts


# near the top of your file, before BLEWorker
pattern = re.compile(
    r'^\s*([0-9]+\.[0-9]+),'      # timestamp
    r'\s*([+-]?\d+(?:\.\d+)?),'   # roll
    r'\s*([+-]?\d+(?:\.\d+)?)'    # pitch
)



class TCPWorker(QThread):
    newData     = pyqtSignal(float, float, float)
    sendCommand = pyqtSignal(str)

    def __init__(self, host: str, port: int):
        super().__init__()
        self.host      = host
        self.port      = port
        self.cmd_queue = queue.Queue()
        self.sendCommand.connect(lambda cmd: self.cmd_queue.put(cmd))

    def run(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        while True:
            try:
                sock.connect((self.host, self.port))
                logger.info(f"TCP connected to {self.host}:{self.port}")
                break
            except OSError:
                logger.warning(f"TCP connect failed, retry in {RETRY_DELAY}s")
                time.sleep(RETRY_DELAY)

        sock.setblocking(False)
        buf = ""
        while True:
            # send queued commands
            try:
                cmd = self.cmd_queue.get_nowait()
                sock.sendall(cmd.encode())
                logger.debug(f"Sent command: {cmd.strip()}")
            except queue.Empty:
                pass

            # receive telemetry
            try:
                data = sock.recv(1024).decode('utf-8', 'ignore')
                if data:
                    buf += data
                    while '\n' in buf:
                        line, buf = buf.split('\n', 1)
                        m = pattern.match(line.strip())
                        if m:
                            t, roll, pitch = map(float, m.groups())
                            self.newData.emit(t, pitch, roll)
            except BlockingIOError:
                pass

            time.sleep(0.01)



def make_silhouette_mask(img: Image.Image) -> Image.Image:
    gray   = img.convert("L")
    stroke = gray.point(lambda p: 255 if p < 128 else 0)
    white  = gray.point(lambda p: 255 if p > 128 else 0)
    ImageDraw.floodfill(white, (0, 0), 0)
    return ImageChops.add(stroke, white)


class HeatmapWidget(QWidget):
    sectionClicked = pyqtSignal(str)

    # fraction down from top where the neck line should lie
    NECK_FRACTION = 0.2

    def __init__(self, serial_thread, parent=None):
        super().__init__(parent)
        self.serial_thread = serial_thread

        img_path = os.path.join(os.path.dirname(__file__), "human_outline.jpg")
        self.mask = make_silhouette_mask(Image.open(img_path).convert("RGBA"))
        self.roll, self.pitch = 0.0, 0.0
        self.flash_section = None

    def setAngles(self, roll, pitch):
        self.roll, self.pitch = roll, pitch
        self.update()

    def mousePressEvent(self, event):
        x, y = event.x(), event.y()
        w, h = self.width(), self.height()

        neck_y  = int(h * self.NECK_FRACTION)
        top_mid = neck_y // 2
        half_w  = w // 2

        if y < top_mid:
            section, msg = 'T1', 'Q1\n'
        elif y < neck_y:
            section, msg = 'T2', 'Q2\n'
        elif x < half_w:
            section, msg = 'BL', 'Q3\n'
        else:
            section, msg = 'BR', 'Q4\n'

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
        painter   = QPainter(self)
        w, h      = self.width(), self.height()
        neck_y    = int(h * self.NECK_FRACTION)
        top_mid   = neck_y // 2
        half_w    = w // 2

        # 1) build the colored heatmap as before
        mask_data = self.mask.resize((w, h), Image.NEAREST).load()
        out       = Image.new("RGBA", (w, h), (0,0,0,0))
        pix       = out.load()

        for yy in range(h):
            for xx in range(w):
                if mask_data[xx, yy] != 255:
                    continue
                if yy < top_mid:
                    value, thresh = self.roll, ROLL_THRESHOLD
                elif yy < neck_y:
                    value, thresh = self.pitch, PITCH_THRESHOLD
                elif xx < half_w:
                    value, thresh = -self.roll, ROLL_THRESHOLD
                else:
                    value, thresh = -self.pitch, PITCH_THRESHOLD
                if abs(value) > thresh:
                    pix[xx, yy] = (255, 0, 0)

        data = out.convert("RGBA").tobytes("raw","RGBA")
        qimg = QImage(data, w, h, QImage.Format_RGBA8888)
        painter.drawPixmap(0, 0, QPixmap.fromImage(qimg))

        # 2) draw white border around the silhouette
        pen = QPen(Qt.white)
        pen.setWidth(1)            # border thickness: tune as needed
        painter.setPen(pen)

        # outline: any mask pixel adjacent to a non-mask pixel
        for yy in range(h):
            for xx in range(w):
                if mask_data[xx, yy] != 255:
                    continue
                # check 4-neighborhood for edge
                if ((xx > 0    and mask_data[xx-1, yy] != 255) or
                    (xx < w-1  and mask_data[xx+1, yy] != 255) or
                    (yy > 0    and mask_data[xx, yy-1] != 255) or
                    (yy < h-1  and mask_data[xx, yy+1] != 255)):
                    painter.drawPoint(xx, yy)

        # 3) flash overlay if required
        if self.flash_section:
            overlay = QColor(0,0,255,100)
            rects = {
                'T1': (0,        0,        w,       top_mid),
                'T2': (0,   top_mid,        w,       neck_y - top_mid),
                'BL': (0,      neck_y,      half_w,   h - neck_y),
                'BR': (half_w, neck_y,      half_w,   h - neck_y)
            }
            painter.fillRect(*rects[self.flash_section], overlay)


class AngleHistoryWidget(QWidget):
    def __init__(self, label, color, threshold, parent=None):
        super().__init__(parent)
        self.label, self.color, self.threshold = label, color, threshold
        self.size   = 300
        self.center = self.size // 2
        self.half   = self.size // 2
        self.value  = 0.0
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
        dx  = self.half * math.cos(rad)
        dy  = self.half * math.sin(rad)
        x1, y1 = self.center - dx, self.center + dy
        x2, y2 = self.center + dx, self.center - dy

        painter.setPen(QPen(self.color,4))
        painter.drawLine(int(x1),int(y1),int(x2),int(y2))

        c = QColor(220,0,0) if abs(self.value) > self.threshold else QColor(220,220,220)
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
        self.serial_thread.sendCommand.emit('REBOOT\n')

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
        timer.start(100)  # ms between heatmap refreshes

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

    # Dark theme palette (same as before)
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

    app.setStyleSheet("""
        QPushButton {
            background-color: #353535;
            color:            white;
            border:           1px solid #444;
            border-radius:    4px;
            padding:          4px 8px;
        }
        QPushButton:hover {
            background-color: #454545;
        }
        QPushButton:pressed {
            background-color: #2d2d2d;
        }
    """)

    # ——— Start TCP thread instead of BLE ———
    tcp_thread = TCPWorker("192.168.4.1", 3333)
    tcp_thread.start()

    window = SwimMonitor(tcp_thread)
    tcp_thread.newData.connect(window.left_panel.onSerialData)

    window.show()
    ret = app.exec_()

    tcp_thread.wait()  # clean up
    logger.info("Application exiting")
    sys.exit(ret)