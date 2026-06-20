# MaixCAM2 / MaixPy (требуется прошивка >= v4.12.5 для YOLO26)
# YOLO26 obstacle detection + symmetric trapezoid corridor with TWO LEFT handles:
#   Handle A (left-bottom = D): X -> AD (bottom width), Y -> AB + vertical position of AD
#   Handle B (left-top = C):    X -> BC (top width),    Y -> AB + vertical position of BC
# AB and CD are symmetric automatically.

from maix import camera, display, image, nn, app, sys, time, network, http
from maix.touchscreen import TouchScreen
import socket
import gc
import json
import os
import threading
import queue as _queue

device_id = sys.device_id()

CONFIG_PATH = "/root/aog_zone/zone_config.json"

def save_config(zone, excl):
    data = {
        "zone": {
            "yA_ratio":        zone.yA_ratio,
            "yB_ratio":        zone.yB_ratio,
            "near_half_ratio": zone.near_half_ratio,
            "far_half_ratio":  zone.far_half_ratio,
            "lmid_y_ratio":    zone.lmid_y_ratio,
            "lmid_half_ratio": zone.lmid_half_ratio,
            "obstacle_detection_enabled": zone.obstacle_detection_enabled,
        },
        "exclusion": {
            "yA_ratio":        excl.yA_ratio,
            "yB_ratio":        excl.yB_ratio,
            "near_half_ratio": excl.near_half_ratio,
            "far_half_ratio":  excl.far_half_ratio,
        }
    }
    try:
        os.makedirs(os.path.dirname(CONFIG_PATH), exist_ok=True)
        with open(CONFIG_PATH, "w") as f:
            json.dump(data, f)
        print(f"💾 Настройки сохранены: {CONFIG_PATH}")
    except Exception as e:
        print(f"❌ Ошибка сохранения: {e}")

def load_config(zone, excl):
    if not os.path.exists(CONFIG_PATH):
        print("ℹ️ Файл настроек не найден, используются значения по умолчанию")
        return
    try:
        with open(CONFIG_PATH, "r") as f:
            data = json.load(f)
        z = data.get("zone", {})
        zone.yA_ratio        = z.get("yA_ratio",        zone.yA_ratio)
        zone.yB_ratio        = z.get("yB_ratio",        zone.yB_ratio)
        zone.near_half_ratio = z.get("near_half_ratio", zone.near_half_ratio)
        zone.far_half_ratio  = z.get("far_half_ratio",  zone.far_half_ratio)
        zone.lmid_y_ratio    = z.get("lmid_y_ratio",    zone.lmid_y_ratio)
        zone.lmid_half_ratio = z.get("lmid_half_ratio", zone.lmid_half_ratio)
        zone.obstacle_detection_enabled = z.get("obstacle_detection_enabled", zone.obstacle_detection_enabled)
        e = data.get("exclusion", {})
        excl.yA_ratio        = e.get("yA_ratio",        excl.yA_ratio)
        excl.yB_ratio        = e.get("yB_ratio",        excl.yB_ratio)
        excl.near_half_ratio = e.get("near_half_ratio", excl.near_half_ratio)
        excl.far_half_ratio  = e.get("far_half_ratio",  excl.far_half_ratio)
        print(f"✅ Настройки загружены: {CONFIG_PATH}")
    except Exception as e:
        print(f"❌ Ошибка загрузки настроек: {e}")

# =========================
# TouchScreen init
# =========================
try:
    touchscreen = TouchScreen()
    print("✅ TouchScreen инициализирован")
except Exception as e:
    print(f"❌ Ошибка инициализации TouchScreen: {e}")
    touchscreen = None

# =========================
# Wi-Fi manager
# =========================
class WiFiManager:
    def __init__(self):
        self.wifi = network.wifi.Wifi()
        self.udp_socket = None
        self.esp32_ip = "192.168.4.1"
        self.esp32_port = 8888
        self.connected = False
        self.ssid = None
        self.password = None
        self._reconnecting = False

    def connect(self, ssid, password, timeout=30):
        self.ssid = ssid
        self.password = password
        print(f"📡 Подключение к Wi-Fi: {ssid}")
        try:
            e = self.wifi.connect(ssid, password, wait=True, timeout=timeout)
            if e == 0:
                self.connected = True
                new_ip = self.wifi.get_ip()
                print(f"✅ Wi-Fi подключен! IP: {new_ip}")
                self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                print(f"✅ UDP сокет создан для отправки на {self.esp32_ip}:{self.esp32_port}")
                return True
            print(f"❌ Ошибка подключения Wi-Fi: {e}")
            return False
        except Exception as e:
            print(f"❌ Ошибка настройки Wi-Fi: {e}")
            return False

    def send_obstacle_data(self, has_obstacle, obstacle_count, steering_angle):
        if (not self.connected) or (self.udp_socket is None):
            return False
        try:
            msg = f"OBSTACLE:{1 if has_obstacle else 0}:COUNT:{obstacle_count}:ANGLE:{steering_angle:.1f}"
            self.udp_socket.sendto(msg.encode("utf-8"), (self.esp32_ip, self.esp32_port))
            return True
        except Exception as e:
            print(f"❌ Ошибка отправки UDP: {e}")
            return False

    def _is_alive(self):
        try:
            ip = self.wifi.get_ip()
            return bool(ip) and ip not in ("0.0.0.0", "")
        except Exception:
            return False

    def start_auto_reconnect(self, check_interval_ms=8000):
        """Фоновый поток: если Wi-Fi отвалился (или не поднялся при старте),
        периодически пытается переподключиться, не требуя перезапуска скрипта."""
        if self._reconnecting:
            return
        self._reconnecting = True

        def _loop():
            while True:
                try:
                    if not self._is_alive():
                        self.connected = False
                        if self.ssid:
                            print("📡 Wi-Fi не подключён, пробую переподключиться...")
                            self.connect(self.ssid, self.password, timeout=15)
                    else:
                        self.connected = True
                except Exception as e:
                    print(f"⚠️ Wi-Fi reconnect: {e}")
                time.sleep_ms(check_interval_ms)

        threading.Thread(target=_loop, daemon=True).start()
        print("🔁 Авто-переподключение Wi-Fi включено")

# =========================
# Angle receiver
# =========================
class AngleReceiver:
    def __init__(self, listen_port=8889):
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket.settimeout(0.02)
        self.udp_socket.bind(("0.0.0.0", listen_port))
        self.current_angle = 0.0

    def receive_angle(self):
        try:
            data, _ = self.udp_socket.recvfrom(64)
            msg = data.decode("utf-8").strip()
            if msg.startswith("ANGLE:"):
                s = msg.replace("ANGLE:", "").strip()
                try:
                    self.current_angle = float(s)
                    return True
                except ValueError:
                    return False
        except socket.timeout:
            return False
        except Exception as e:
            print(f"❌ Ошибка приема угла: {e}")
            return False

# =========================
# Touch calibration (simple scaling)
# =========================
class TouchCalibrator:
    def __init__(self, display_width, display_height):
        self.display_width = display_width
        self.display_height = display_height
        print(f"📐 Калибратор: {display_width}x{display_height}")

    def transform_coordinates(self, x, y):
        display_x = int(x * (self.display_width / 640.0))
        display_y = int(y * (self.display_height / 480.0))
        display_x = max(0, min(display_x, self.display_width - 1))
        display_y = max(0, min(display_y, self.display_height - 1))
        return display_x, display_y

# =========================
# Geometry: point in convex quad
# =========================
def point_in_quad(px, py, quad):
    def cross(ax, ay, bx, by, cx, cy):
        return (bx-ax)*(cy-ay) - (by-ay)*(cx-ax)

    x1,y1 = quad[0]
    x2,y2 = quad[1]
    x3,y3 = quad[2]
    x4,y4 = quad[3]

    c1 = cross(x1,y1, x2,y2, px,py)
    c2 = cross(x2,y2, x3,y3, px,py)
    c3 = cross(x3,y3, x4,y4, px,py)
    c4 = cross(x4,y4, x1,y1, px,py)

    return (c1 >= 0 and c2 >= 0 and c3 >= 0 and c4 >= 0) or \
           (c1 <= 0 and c2 <= 0 and c3 <= 0 and c4 <= 0)

def point_in_convex_polygon(px, py, pts):
    n = len(pts)
    pos = neg = 0
    for i in range(n):
        ax, ay = pts[i]
        bx, by = pts[(i + 1) % n]
        c = (bx - ax) * (py - ay) - (by - ay) * (px - ax)
        if c > 0: pos += 1
        else:     neg += 1
    return pos == 0 or neg == 0

def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v

# =========================
# ZoneConfig: symmetric trapezoid with two LEFT handles (B=top-left, A=bottom-left)
# =========================
class ZoneConfig:
    def __init__(self, width, height):
        self.width = width
        self.height = height

        self.shift_far_k = 1.0 # коэффициэнт движения вершины

        # base geometry in pixels derived from ratios (init)
        self.yA_ratio = 0.95  # bottom line y (AD) — почти у нижнего края
        self.yB_ratio = 0.04  # top line y (BC)    — у самого верха

        self.near_half_ratio = 0.47  # half width of bottom AD — широкий низ
        self.far_half_ratio  = 0.08  # half width of top BC    — узкий верх

        # steering shift behavior
        self.max_shift_ratio = 0.4
        self.max_steer_for_max = 30.0  # +/-30° -> max shift

        # UI / edit
        self.edit_mode = False
        self.selected = None   # 'A' or 'B'
        self.touch_threshold = 60
        self.last_touch_time = 0
        self.touch_cooldown = 70

        self.obstacle_detection_enabled = True

        # side kink handle (L): point on left side that can bow outward
        self.lmid_y_ratio    = 0.50   # vertical position between yB and yA
        self.lmid_half_ratio = 0.28   # half-width at kink (> interp → outward bow)

        # constraints
        self.min_half_ratio = 0.06
        self.max_half_ratio = 0.49
        self.min_height_px = 40

    def toggle_obstacle_detection(self):
        self.obstacle_detection_enabled = not self.obstacle_detection_enabled
        status = "ВКЛЮЧЕНО" if self.obstacle_detection_enabled else "ВЫКЛЮЧЕНО"
        print(f"🎯 Обнаружение препятствий: {status}")
        return self.obstacle_detection_enabled

    def _steer_norm(self, steering_angle):
        return clamp(steering_angle / self.max_steer_for_max, -1.0, 1.0)

    def _get_params_px(self):
        W, H = self.width, self.height
        cx = W // 2

        yA = int(H * self.yA_ratio)
        yB = int(H * self.yB_ratio)

        # ensure valid ordering & min height
        yA = clamp(yA, int(H * 0.50), H - 1)        # bottom must be lower half
        yB = clamp(yB, 0, int(H * 0.80))            # top must be upper-ish
        if yA - yB < self.min_height_px:
            yB = max(0, yA - self.min_height_px)

        near_half = int(W * self.near_half_ratio)
        far_half  = int(W * self.far_half_ratio)

        near_half = clamp(near_half, int(W * self.min_half_ratio), int(W * self.max_half_ratio))
        far_half  = clamp(far_half,  int(W * self.min_half_ratio), int(W * self.max_half_ratio))

        return cx, yA, yB, near_half, far_half

    def get_trapezoid(self, steering_angle=0.0):
        """
        Returns A,B,C,D in your naming:
          A = right-bottom
          B = right-top
          C = left-top
          D = left-bottom
        Symmetric about centerline, then shifted by steering.
        """
        W = self.width
        cx, yA, yB, near_half, far_half = self._get_params_px()

        max_shift = int(W * self.max_shift_ratio)
        s = self._steer_norm(steering_angle)

        shift_far = int(s * max_shift * self.shift_far_k)     # ВЕРХ ДВИГАЕМ ПО КОЭФФ
        shift_near = int(0.1 * shift_far)                     # НИЗ  ДВИГАЕМ МЕНЬШЕ
        
        # bottom (AD) — без сдвига
        D = (cx - near_half + shift_near, yA)
        A = (cx + near_half + shift_near, yA)

        # top (BC) — со сдвигом
        C = (cx - far_half + shift_far, yB)
        B = (cx + far_half + shift_far, yB)

        # clamp bounds
        def clamp_pt(p):
            return (int(clamp(p[0], 0, self.width - 1)), int(clamp(p[1], 0, self.height - 1)))

        return clamp_pt(A), clamp_pt(B), clamp_pt(C), clamp_pt(D)

    def get_hexagon(self, steering_angle=0.0):
        """
        Returns 6 vertices: A (bottom-right), Rv (right kink), B (top-right),
        C (top-left), Lv (left kink), D (bottom-left).
        Lv/Rv are midpoints on the sides that can bow outward.
        """
        W, H = self.width, self.height
        cx = W // 2
        _, yA, yB, near_half, far_half = self._get_params_px()

        max_shift = int(W * self.max_shift_ratio)
        s = self._steer_norm(steering_angle)
        shift_far  = int(s * max_shift * self.shift_far_k)
        shift_near = int(0.1 * shift_far)

        # kink y — clamped between yB+5 and yA-5
        Ly = int(H * clamp(self.lmid_y_ratio, self.yB_ratio + 0.02, self.yA_ratio - 0.02))

        # steering shift at kink y (linear interpolation between near and far shifts)
        span = yB - yA
        t = clamp((Ly - yA) / span, 0.0, 1.0) if span != 0 else 0.5
        shift_L = int(shift_near + t * (shift_far - shift_near))

        # interpolated half on the straight side at kink y (min bound — no inward bow)
        line_half = int(near_half + t * (far_half - near_half))
        Lhalf = max(int(W * clamp(self.lmid_half_ratio, 0.01, self.max_half_ratio + 0.15)), line_half)

        def cp(p):
            return (int(clamp(p[0], 0, W - 1)), int(clamp(p[1], 0, H - 1)))

        A  = cp((cx + near_half + shift_near, yA))
        Rv = cp((cx + Lhalf     + shift_L,   Ly))
        B  = cp((cx + far_half  + shift_far,  yB))
        C  = cp((cx - far_half  + shift_far,  yB))
        Lv = cp((cx - Lhalf     + shift_L,   Ly))
        D  = cp((cx - near_half + shift_near, yA))
        return A, Rv, B, C, Lv, D

    def get_quad_for_tests(self, steering_angle=0.0):
        A, Rv, B, C, Lv, D = self.get_hexagon(steering_angle)
        # polygon CCW/CW — point_in_convex_polygon handles both
        return [D, Lv, C, B, Rv, A], (A, Rv, B, C, Lv, D)

    # ---- handles: two points on LEFT (B=top-left (C), A=bottom-left (D)) ----
    def get_left_handles(self, steering_angle=0.0):
        A, B, C, D = self.get_trapezoid(steering_angle)
        # handle A = left-bottom corner D
        # handle B = left-top corner C
        return {
            "A": (D[0], D[1]),
            "B": (C[0], C[1]),
        }

    # ---- side handle: left kink point Lv; right is symmetric ----
    def get_side_handles(self, steering_angle=0.0):
        A, Rv, B, C, Lv, D = self.get_hexagon(steering_angle)
        return {"L": Lv}

    # ---- update rules per your requirements ----
    def _set_near_from_x(self, x):
        W = self.width
        cx = W // 2
        half = abs(cx - x)
        half = clamp(half, int(W * self.min_half_ratio), int(W * self.max_half_ratio))
        self.near_half_ratio = half / W
        print(f"📐 AD (низ) half={half}px  ratio={self.near_half_ratio:.3f}")

    def _set_far_from_x(self, x):
        W = self.width
        cx = W // 2
        half = abs(cx - x)
        half = clamp(half, int(W * self.min_half_ratio), int(W * self.max_half_ratio))
        self.far_half_ratio = half / W
        print(f"📐 BC (верх) half={half}px  ratio={self.far_half_ratio:.3f}")

    def _set_yA_from_y(self, y):
        H = self.height
        y = clamp(y, int(H * 0.55), H - 1)
        self.yA_ratio = y / H
        print(f"📐 yA (AD по вертикали) = {y}px  ratio={self.yA_ratio:.3f}")

    def _set_yB_from_y(self, y):
        H = self.height
        y = clamp(y, 0, int(H * 0.80))
        self.yB_ratio = y / H
        print(f"📐 yB (BC по вертикали) = {y}px  ratio={self.yB_ratio:.3f}")

    def handle_touch(self, x, y, pressed, steering_angle=0.0):
        now = time.ticks_ms()
        if now - self.last_touch_time < self.touch_cooldown:
            return False
        self.last_touch_time = now

        # Buttons
        btn_w, btn_h = 120, 32
        btn_y = 0
        edit_x = 0
        det_x = self.width - btn_w

        if not self.edit_mode:
            if pressed == 1:
                if edit_x <= x <= edit_x + btn_w and btn_y <= y <= btn_y + btn_h:
                    self.edit_mode = True
                    self.selected = None
                    print("📐 EDIT ON")
                    return True
                if det_x <= x <= det_x + btn_w and btn_y <= y <= btn_y + btn_h:
                    self.toggle_obstacle_detection()
                    return True
            return False

        # edit mode: обе зоны редактируются одновременно
        if pressed == 1:
            if edit_x <= x <= edit_x + btn_w and btn_y <= y <= btn_y + btn_h:
                self.edit_mode = False
                self.selected = None
                exclusion_zone.selected = None
                save_config(self, exclusion_zone)
                return True
            if det_x <= x <= det_x + btn_w and btn_y <= y <= btn_y + btn_h:
                self.toggle_obstacle_detection()
                return True

        handles = {**self.get_left_handles(steering_angle), **self.get_side_handles(steering_angle)}

        # pick nearest handle
        nearest = None
        best_d = 1e9
        for k, (hx, hy) in handles.items():
            d = ((x - hx) ** 2 + (y - hy) ** 2) ** 0.5
            if d < best_d:
                best_d = d
                nearest = k

        if pressed == 1:
            if best_d < self.touch_threshold:
                self.selected = nearest
                print(f"🎯 Выбрана точка {self.selected}")
            else:
                self.selected = None
                return False

        # apply drag logic even if pressed==0 (на некоторых прошивках так идет движение)
        if self.selected == "A":
            # X -> AD, Y -> vertical position of AD
            self._set_near_from_x(x)
            self._set_yA_from_y(y)
            return True

        if self.selected == "B":
            # X -> BC, Y -> vertical position of BC
            self._set_far_from_x(x)
            self._set_yB_from_y(y)
            return True

        if self.selected in ("L", "R"):
            # Y -> position along the side (between yB and yA)
            H, W = self.height, self.width
            y_min = int(H * self.yB_ratio) + 10
            y_max = int(H * self.yA_ratio) - 10
            self.lmid_y_ratio = clamp(y, y_min, y_max) / H
            # X -> outward bow (half-width at kink)
            cx_px = W // 2
            half = abs(cx_px - x)
            half = clamp(half, int(W * 0.01), int(W * (self.max_half_ratio + 0.15)))
            self.lmid_half_ratio = half / W
            return True

        return False

# =========================
# ExclusionZone: трапеция-маска капота (объекты внутри игнорируются)
# =========================
class ExclusionZone:
    def __init__(self, width, height):
        self.width  = width
        self.height = height
        self.enabled = True

        self.yA_ratio        = 1.00  # нижняя граница (низ кадра)
        self.yB_ratio        = 0.65  # верхняя граница капота
        self.near_half_ratio = 0.22  # полуширина низа — уже главной зоны
        self.far_half_ratio  = 0.12  # полуширина верха

        self.selected = None

    def get_trapezoid(self):
        W, H = self.width, self.height
        cx = W // 2
        yA = int(H * self.yA_ratio)
        yB = int(H * self.yB_ratio)
        na = int(W * self.near_half_ratio)
        fa = int(W * self.far_half_ratio)
        A = (cx + na, yA);  B = (cx + fa, yB)
        C = (cx - fa, yB);  D = (cx - na, yA)
        def cp(p): return (int(clamp(p[0],0,W-1)), int(clamp(p[1],0,H-1)))
        return cp(A), cp(B), cp(C), cp(D)

    def get_quad(self):
        A, B, C, D = self.get_trapezoid()
        return [D, C, B, A]

    def contains(self, px, py):
        if not self.enabled:
            return False
        return point_in_quad(px, py, self.get_quad())

    # ручки справа (A=right-bottom, B=right-top)
    def get_right_handles(self):
        A, B, C, D = self.get_trapezoid()
        return {"hA": (A[0], A[1]), "hB": (B[0], B[1])}

    def apply_drag(self, x, y):
        W, H = self.width, self.height
        cx = W // 2
        if self.selected == "hA":
            self.near_half_ratio = clamp(abs(cx - x) / W, 0.06, 0.45)
            self.yA_ratio        = clamp(y / H, 0.50, 1.00)
            return True
        if self.selected == "hB":
            self.far_half_ratio = clamp(abs(cx - x) / W, 0.04, 0.44)
            self.yB_ratio       = clamp(y / H, 0.10, 0.90)
            return True
        return False

# =========================
# Obstacle stabilizer (дебаунс по кадрам)
# =========================
class ObstacleStabilizer:
    def __init__(self, confirm_frames=3, clear_frames=5):
        self.confirm_frames = confirm_frames  # кадров подряд "есть" → stable=True
        self.clear_frames   = clear_frames    # кадров подряд "нет"  → stable=False
        self._detect_cnt = 0
        self._clear_cnt  = 0
        self.stable      = False             # стабильное состояние для отправки

    def update(self, raw):
        if raw:
            self._detect_cnt += 1
            self._clear_cnt  = 0
            if self._detect_cnt >= self.confirm_frames:
                self.stable = True
        else:
            self._clear_cnt  += 1
            self._detect_cnt  = 0
            if self._clear_cnt >= self.clear_frames:
                self.stable = False
        return self.stable

# =========================
# Model / Camera / Display
# =========================
# YOLO26 поддерживается нативно (MaixPy >= 4.12.5).
# Модель .mud должна быть собрана под MaixCAM2 (axmodel), иначе загрузка упадёт.
MODEL_PATH = "/root/models/yolo26n.mud"
try:
    detector = nn.YOLO26(model=MODEL_PATH, dual_buff=True)
    print(f"✅ Модель загружена: {MODEL_PATH}")
except Exception as e:
    # nn.YOLO26(...) бросает err.Exception при неудачной загрузке (см. API maix.nn)
    print(f"❌ Не удалось загрузить модель {MODEL_PATH}: {e}")
    print("   Проверь: файл существует и собран под MaixCAM2 (NPU AX630C), а не под старую MaixCAM.")
    raise
# MaixCAM2 / SC850SL: 1280x720, RGB888.
# detect() сам отресайзит кадр под вход модели (FIT_CONTAIN) и пересчитает координаты рамок.
cam = camera.Camera(1280, 720, detector.input_format())
disp = display.Display()

# =========================
# Web control server (порт 8765) — приём команд от браузера
# =========================
_cmd_queue = _queue.Queue(maxsize=30)
_state_cache = {}

class WebControlServer:
    def __init__(self, port=8765):
        self.port = port
        self._running = False

    def start(self):
        self._running = True
        t = threading.Thread(target=self._serve, daemon=True)
        t.start()
        try:
            ips = sys.ip_address()  # maix.sys.ip_address() -> dict, e.g. {"wlan0": "192.168.x.x"}
            ip = ips.get("wlan0") or ips.get("eth0") or "<device-ip>"
        except Exception:
            ip = "<device-ip>"
        # Веб-интерфейс (видео + ползунки) отдаёт JpegStreamer (обычно порт 8000),
        # а этот сервер (self.port) обрабатывает только /state и /set.
        ui_port = 8000
        try:
            if stream is not None:
                ui_port = stream.port()
        except Exception:
            pass
        print(f"🌐 Веб-интерфейс (видео+настройки): http://{ip}:{ui_port}/")
        print(f"   (команды обрабатываются на порту {self.port})")

    def _serve(self):
        while self._running:
            srv = None
            try:
                srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                srv.bind(("0.0.0.0", self.port))
                srv.listen(3)
                srv.settimeout(1.0)
                while self._running:
                    try:
                        conn, _ = srv.accept()
                        t = threading.Thread(target=self._handle, args=(conn,), daemon=True)
                        t.start()
                    except socket.timeout:
                        continue
                    except Exception:
                        break
            except Exception as e:
                print(f"⚠️ WebServer упал, перезапуск: {e}")
            finally:
                if srv:
                    try: srv.close()
                    except Exception: pass
            time.sleep_ms(500)

    def _handle(self, conn):
        try:
            conn.settimeout(5.0)
            raw = conn.recv(1024).decode("utf-8", errors="replace")
            line = raw.split("\r\n")[0]
            parts = line.split(" ")
            if len(parts) < 2:
                return
            path = parts[1]

            if path.startswith("/state"):
                body = json.dumps(_state_cache)
                conn.sendall(("HTTP/1.1 200 OK\r\nAccess-Control-Allow-Origin: *\r\nContent-Type: application/json\r\n\r\n" + body).encode("utf-8"))
                return

            qs = path.split("?", 1)[1] if "?" in path else ""
            params = {}
            for p in qs.split("&"):
                if "=" in p:
                    k, v = p.split("=", 1)
                    params[k] = v
            if "/set" in path:
                _cmd_queue.put_nowait(params)
            conn.sendall(b"HTTP/1.1 200 OK\r\nAccess-Control-Allow-Origin: *\r\nContent-Type: text/plain\r\n\r\nOK")
        except Exception:
            pass
        finally:
            conn.close()

# =========================
# HTTP JPEG streamer (трансляция экрана на телефон по Wi-Fi)
# =========================
html_page = """<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>AOG Trapez</title>
<style>
  body{margin:0;background:#111;color:#eee;font-family:sans-serif;font-size:14px}
  img{width:100%;display:block}
  .panel{padding:8px 10px}
  .row{display:flex;align-items:center;margin:5px 0;gap:8px}
  .row label{width:130px;flex-shrink:0}
  .row input[type=range]{flex:1}
  .row span{width:38px;text-align:right}
  .btns{display:flex;gap:8px;margin:8px 0}
  button{flex:1;padding:10px;font-size:14px;font-weight:bold;border:none;border-radius:6px}
  #bdet{background:#0a0;color:#fff}
  #bsave{background:#f90;color:#000}
  #brec{background:#c00;color:#fff}
  #brec.on{background:#f00;animation:pulse 1s infinite}
  @keyframes pulse{0%,100%{opacity:1}50%{opacity:0.5}}
  h3{margin:6px 0 2px;color:#aaa;font-size:12px;text-transform:uppercase}
  #recinfo{font-size:12px;color:#f88;text-align:center;min-height:16px}
</style>
</head>
<body>
<img src="/stream">
<div class="panel">
  <h3>Main zone</h3>
  <div class="row"><label>Bottom Y</label><input type="range" id="z_yA" min="50" max="99" oninput="send()"><span id="z_yA_v"></span></div>
  <div class="row"><label>Top Y</label><input type="range" id="z_yB" min="1" max="50" oninput="send()"><span id="z_yB_v"></span></div>
  <div class="row"><label>Bottom width</label><input type="range" id="z_na" min="6" max="49" oninput="send()"><span id="z_na_v"></span></div>
  <div class="row"><label>Top width</label><input type="range" id="z_fa" min="4" max="48" oninput="send()"><span id="z_fa_v"></span></div>
  <h3>Side kink (L/R)</h3>
  <div class="row"><label>Kink Y pos</label><input type="range" id="z_ly" min="6" max="93" oninput="send()"><span id="z_ly_v"></span></div>
  <div class="row"><label>Kink bow</label><input type="range" id="z_lh" min="6" max="64" oninput="send()"><span id="z_lh_v"></span></div>
  <h3>Hood zone</h3>
  <div class="row"><label>Top Y</label><input type="range" id="e_yB" min="10" max="90" oninput="send()"><span id="e_yB_v"></span></div>
  <div class="row"><label>Bottom width</label><input type="range" id="e_na" min="6" max="45" oninput="send()"><span id="e_na_v"></span></div>
  <div class="row"><label>Top width</label><input type="range" id="e_fa" min="4" max="44" oninput="send()"><span id="e_fa_v"></span></div>
  <div class="btns">
    <button id="bdet" onclick="toggleDet()">DETECT ON</button>
    <button id="bsave" onclick="saveCfg()">SAVE</button>
  </div>
  <div class="btns">
    <button id="brec" onclick="toggleRec()">REC</button>
  </div>
  <div id="recinfo"></div>
  <div id="status" style="font-size:11px;color:#888;text-align:center"></div>
</div>
<script>
const PORT = 8765;
const ids = ['z_yA','z_yB','z_na','z_fa','z_ly','z_lh','e_yB','e_na','e_fa'];
const defs = {z_yA:95,z_yB:4,z_na:47,z_fa:8,z_ly:50,z_lh:28,e_yB:65,e_na:22,e_fa:12};
ids.forEach(id=>{
  const s=document.getElementById(id);
  s.value=defs[id];
  document.getElementById(id+'_v').textContent=defs[id]+'%';
  s.oninput=()=>{document.getElementById(id+'_v').textContent=s.value+'%';send();};
});
// Загружаем реальные значения с камеры
fetch('http://'+location.hostname+':'+PORT+'/state')
  .then(r=>r.json())
  .then(data=>{
    ids.forEach(id=>{
      if(data[id]!==undefined){
        const s=document.getElementById(id);
        s.value=data[id];
        document.getElementById(id+'_v').textContent=data[id]+'%';
      }
    });
    if(data.det!==undefined){
      const b=document.getElementById('bdet');
      b.textContent=data.det?'DETECT ON':'DETECT OFF';
    }
    updateRec(data);
    document.getElementById('status').textContent='OK';
  })
  .catch(()=>{document.getElementById('status').textContent='defaults';});
function cmd(params){
  fetch('http://'+location.hostname+':'+PORT+'/set?'+params)
    .then(()=>{document.getElementById('status').textContent=new Date().toLocaleTimeString();})
    .catch(()=>{document.getElementById('status').textContent='ERR';});
}
function send(){
  const p=ids.map(id=>id+'='+document.getElementById(id).value).join('&');
  cmd(p);
}
function toggleDet(){cmd('toggle_det=1');}
function saveCfg(){cmd('save=1');}
function toggleRec(){cmd('toggle_rec=1');}
function updateRec(data){
  if(data.rec===undefined) return;
  const b=document.getElementById('brec');
  const info=document.getElementById('recinfo');
  if(data.rec){
    b.textContent='STOP REC';
    b.classList.add('on');
    info.textContent='REC: '+data.rec_count+' frames saved';
  } else {
    b.textContent='REC';
    b.classList.remove('on');
    info.textContent=data.rec_count>0?'Saved: '+data.rec_count+' frames':'';
  }
}
// опрашиваем статус записи каждые 2 секунды
setInterval(()=>{
  fetch('http://'+location.hostname+':'+PORT+'/state')
    .then(r=>r.json()).then(updateRec).catch(()=>{});
}, 2000);
</script>
</body>
</html>"""

try:
    stream = http.JpegStreamer()
    stream.set_html(html_page)
    ret = stream.start()           # start() возвращает err::Err (0 == ERR_NONE)
    if ret != 0:
        raise Exception(f"start err={ret}")
    print(f"📺 Видеопоток: http://{stream.host()}:{stream.port()}")
except Exception as e:
    print(f"❌ Не удалось запустить JPEG-стример: {e}")
    stream = None

class_names = {
    0: "Person", 1: "Bicycle", 2: "Car", 3: "Motorbike",
    7: "Truck", 17: "Horse", 18: "Sheep", 19: "Cow"
}

# =========================
# Wi-Fi
# =========================
wifi_manager = WiFiManager()
SSID = "AOG4"
PASSWORD = "12345678"
wifi_connected = wifi_manager.connect(SSID, PASSWORD)
wifi_manager.start_auto_reconnect()   # если сеть не поднялась — подключится в фоне без перезапуска
web_server = WebControlServer(port=8765)
web_server.start()

# =========================
# Angle receiver
# =========================
angle_receiver = AngleReceiver()

# =========================
# Zone + touch calibrator
# =========================
zone_config = ZoneConfig(cam.width(), cam.height())
exclusion_zone = ExclusionZone(cam.width(), cam.height())
load_config(zone_config, exclusion_zone)
touch_calibrator = TouchCalibrator(cam.width(), cam.height())

print(f"📱 Разрешение: {cam.width()}x{cam.height()}")
print("✅ Запуск: детекция + симметричная трапеция + две точки слева")

stabilizer = ObstacleStabilizer(confirm_frames=3, clear_frames=5)

touch_count = 0
last_obstacle_print = 0
last_angle_print = 0
last_stream_time = 0
gc_counter = 0
steering_angle = 0.0
STREAM_INTERVAL_MS = 100  # 10 кадров в секунду
stream_fail_count = 0
STREAM_FAIL_MAX = 30       # после 30 ошибок подряд — перезапуск стрима
last_mem_log = 0
last_state_update = 0

# запись датасета
recording = False
rec_frame_count = 0
rec_total = 0
REC_EVERY = 5              # сохранять каждый 5-й кадр
REC_DIR = "/root/dataset"
try:
    os.makedirs(REC_DIR)
except Exception:
    pass

# =========================
# Main loop
# =========================
while not app.need_exit():
    img = cam.read()

    # актуальное состояние Wi-Fi (фоновый поток может переподключить его в любой момент)
    wifi_connected = wifi_manager.connected

    # сохранение чистого кадра для датасета — до любой отрисовки
    if recording:
        rec_frame_count += 1
        if rec_frame_count >= REC_EVERY:
            rec_frame_count = 0
            try:
                path = f"{REC_DIR}/{rec_total:06d}.jpg"
                img.save(path)
                # сохраняем угол руля рядом с кадром
                with open(f"{REC_DIR}/{rec_total:06d}.txt", "w") as f:
                    f.write(f"{steering_angle:.2f}")
                rec_total += 1
            except Exception as e:
                print(f"❌ Ошибка сохранения кадра: {e}")

    objs = detector.detect(img, conf_th=0.5, iou_th=0.45)
    target_objects = [o for o in objs if o.class_id in class_names]

    # web commands
    while not _cmd_queue.empty():
        try:
            p = _cmd_queue.get_nowait()
            if "z_yA" in p: zone_config.yA_ratio        = clamp(int(p["z_yA"]), 50, 99) / 100.0
            if "z_yB" in p: zone_config.yB_ratio        = clamp(int(p["z_yB"]),  1, 50) / 100.0
            if "z_na" in p: zone_config.near_half_ratio = clamp(int(p["z_na"]),   6, 49) / 100.0
            if "z_fa" in p: zone_config.far_half_ratio  = clamp(int(p["z_fa"]),   4, 48) / 100.0
            if "z_ly" in p: zone_config.lmid_y_ratio    = clamp(int(p["z_ly"]),   6, 93) / 100.0
            if "z_lh" in p: zone_config.lmid_half_ratio = clamp(int(p["z_lh"]),   6, 64) / 100.0
            if "e_yB" in p: exclusion_zone.yB_ratio     = clamp(int(p["e_yB"]), 10, 90) / 100.0
            if "e_na" in p: exclusion_zone.near_half_ratio = clamp(int(p["e_na"]), 6, 45) / 100.0
            if "e_fa" in p: exclusion_zone.far_half_ratio  = clamp(int(p["e_fa"]), 4, 44) / 100.0
            if p.get("toggle_det") == "1": zone_config.toggle_obstacle_detection()
            if p.get("save") == "1": save_config(zone_config, exclusion_zone)
            if p.get("toggle_rec") == "1":
                recording = not recording
                if recording:
                    # считаем уже существующие файлы чтобы не перезаписывать
                    try:
                        existing = [f for f in os.listdir(REC_DIR) if f.endswith(".jpg")]
                        rec_total = len(existing)
                    except Exception:
                        rec_total = 0
                    print(f"🔴 Запись начата → {REC_DIR} (старт с {rec_total})")
                else:
                    print(f"⏹ Запись остановлена, сохранено: {rec_total} кадров")
        except Exception:
            pass

    # обновляем кэш состояния для web-панели раз в секунду
    if time.ticks_ms() - last_state_update > 1000:
        last_state_update = time.ticks_ms()
        _state_cache.update({
            "z_yA": int(zone_config.yA_ratio        * 100),
            "z_yB": int(zone_config.yB_ratio         * 100),
            "z_na": int(zone_config.near_half_ratio  * 100),
            "z_fa": int(zone_config.far_half_ratio   * 100),
            "e_yB": int(exclusion_zone.yB_ratio      * 100),
            "e_na": int(exclusion_zone.near_half_ratio * 100),
            "e_fa": int(exclusion_zone.far_half_ratio  * 100),
            "z_ly": int(zone_config.lmid_y_ratio    * 100),
            "z_lh": int(zone_config.lmid_half_ratio * 100),
            "det":       1 if zone_config.obstacle_detection_enabled else 0,
            "rec":       1 if recording else 0,
            "rec_count": rec_total,
        })

    # angle
    if angle_receiver.receive_angle():
        steering_angle = angle_receiver.current_angle
        now = time.ticks_ms()
        if now - last_angle_print > 1000:
            print(f"📥 Угол от ESP32: {steering_angle:.1f}°")
            last_angle_print = now

    # touch
    if touchscreen and touchscreen.available():
        try:
            td = touchscreen.read()
            if td and len(td) >= 3:
                raw_x, raw_y, pressed = td
                touch_count += 1
                x, y = touch_calibrator.transform_coordinates(raw_x, raw_y)

                if touch_count <= 6:
                    print(f"👆 Touch#{touch_count}: raw({raw_x},{raw_y}) -> ({x},{y}) pressed={pressed}")

                # кнопки всегда обрабатываются первыми
                btn_w, btn_h = 120, 32
                btn_y_touch = 0
                in_button = (btn_y_touch <= y <= btn_y_touch + btn_h and
                             (x <= btn_w or x >= zone_config.width - btn_w))

                if in_button or not zone_config.edit_mode:
                    zone_config.handle_touch(x, y, pressed, steering_angle)
                elif zone_config.edit_mode:
                    if pressed == 1:
                        # выбираем ближайшую ручку из обеих зон
                        all_handles = {}
                        for k, pt in zone_config.get_left_handles(steering_angle).items():
                            all_handles[('zone', k)] = pt
                        for k, pt in zone_config.get_side_handles(steering_angle).items():
                            all_handles[('zone', k)] = pt
                        for k, pt in exclusion_zone.get_right_handles().items():
                            all_handles[('excl', k)] = pt
                        best, best_d = None, 1e9
                        for key, (hx, hy) in all_handles.items():
                            d = ((x-hx)**2 + (y-hy)**2)**0.5
                            if d < best_d:
                                best_d = d; best = key
                        if best and best_d < zone_config.touch_threshold:
                            if best[0] == 'zone':
                                zone_config.selected = best[1]
                                exclusion_zone.selected = None
                            else:
                                exclusion_zone.selected = best[1]
                                zone_config.selected = None
                        else:
                            zone_config.selected = None
                            exclusion_zone.selected = None

                    if zone_config.selected:
                        zone_config.handle_touch(x, y, pressed, steering_angle)
                    elif exclusion_zone.selected:
                        exclusion_zone.apply_drag(x, y)
        except Exception as e:
            print(f"❌ Ошибка TouchScreen: {e}")

    hex_poly, (A, Rv, B, C, Lv, D) = zone_config.get_quad_for_tests(steering_angle)

    # objects in zone (исключаем попавших в зону капота)
    objects_in_zone = []
    for o in target_objects:
        cx = o.x + o.w // 2
        cy = o.y + o.h // 2
        if point_in_convex_polygon(cx, cy, hex_poly) and not exclusion_zone.contains(cx, cy):
            objects_in_zone.append(o)

    raw_obstacle = (len(objects_in_zone) > 0) and zone_config.obstacle_detection_enabled
    has_obstacle = stabilizer.update(raw_obstacle)

    # send UDP
    if wifi_connected and zone_config.obstacle_detection_enabled:
        wifi_manager.send_obstacle_data(has_obstacle, len(objects_in_zone), steering_angle)

    # print throttled
    now = time.ticks_ms()
    if has_obstacle and now - last_obstacle_print > 2000:
        detected = []
        for cid in class_names:
            cnt = 0
            for o in objects_in_zone:
                if o.class_id == cid:
                    cnt += 1
            if cnt > 0:
                detected.append(f"{class_names[cid]}:{cnt}")
        status = "📡 UDP OK" if wifi_connected else "❌ Wi-Fi OFF"
        print(f"🚨 ПРЕПЯТСТВИЕ: {', '.join(detected)} | angle={steering_angle:.1f}° | {status}")
        last_obstacle_print = now

    # draw detections
    for o in target_objects:
        if o.class_id == 0:
            color = image.COLOR_GREEN
        elif o.class_id in [1, 2, 3, 7]:
            color = image.COLOR_BLUE
        else:
            color = image.COLOR_YELLOW
        img.draw_rect(o.x, o.y, o.w, o.h, color=color, thickness=2)
        img.draw_string(o.x, max(0, o.y - 15), f"{class_names[o.class_id]}:{o.score:.2f}", color=color, scale=1.2)

    # zone color
    if zone_config.obstacle_detection_enabled:
        zone_color = image.COLOR_RED if has_obstacle else image.COLOR_GREEN
        if zone_config.edit_mode:
            zone_color = image.COLOR_GREEN
    else:
        zone_color = image.COLOR_GRAY

    # draw hexagon edges: right lower, right upper, top, left upper, left lower, bottom
    img.draw_line(A[0],  A[1],  Rv[0], Rv[1], color=zone_color, thickness=3)
    img.draw_line(Rv[0], Rv[1], B[0],  B[1],  color=zone_color, thickness=3)
    img.draw_line(B[0],  B[1],  C[0],  C[1],  color=zone_color, thickness=3)
    img.draw_line(C[0],  C[1],  Lv[0], Lv[1], color=zone_color, thickness=3)
    img.draw_line(Lv[0], Lv[1], D[0],  D[1],  color=zone_color, thickness=3)
    img.draw_line(D[0],  D[1],  A[0],  A[1],  color=zone_color, thickness=3)

    # draw exclusion zone (капот)
    if exclusion_zone.enabled:
        eA, eB, eC, eD = exclusion_zone.get_trapezoid()
        exc_color = image.COLOR_RED if not zone_config.edit_mode else image.COLOR_YELLOW
        img.draw_line(eA[0], eA[1], eB[0], eB[1], color=exc_color, thickness=2)
        img.draw_line(eB[0], eB[1], eC[0], eC[1], color=exc_color, thickness=2)
        img.draw_line(eC[0], eC[1], eD[0], eD[1], color=exc_color, thickness=2)
        img.draw_line(eD[0], eD[1], eA[0], eA[1], color=exc_color, thickness=2)
        if zone_config.edit_mode:
            for k, (hx, hy) in exclusion_zone.get_right_handles().items():
                is_sel = (exclusion_zone.selected == k)
                c = image.COLOR_RED if is_sel else image.COLOR_YELLOW
                img.draw_rect(hx-18, hy-18, 36, 36, color=c, thickness=2)
                img.draw_rect(hx-14, hy-14, 28, 28, color=c, thickness=-1)
                img.draw_string(hx-30, hy-18, k, color=image.COLOR_WHITE, scale=1.0)

    # draw LEFT handles in edit mode
    if zone_config.edit_mode:
        handles = zone_config.get_left_handles(steering_angle)
        for k, (hx, hy) in handles.items():
            is_sel = (zone_config.selected == k)
            c = image.COLOR_RED if is_sel else image.COLOR_YELLOW
            img.draw_rect(hx - 20, hy - 20, 40, 40, color=c, thickness=2)
            img.draw_rect(hx - 16, hy - 16, 32, 32, color=c, thickness=-1)
            img.draw_string(hx + 22, hy - 18, k, color=image.COLOR_WHITE, scale=1.1)

        # draw SIDE handles (L=left side midpoint, R=right side midpoint)
        for k, (hx, hy) in zone_config.get_side_handles(steering_angle).items():
            is_sel = (zone_config.selected == k)
            c = image.COLOR_RED if is_sel else image.Color.from_rgb(0, 220, 220)
            img.draw_rect(hx - 18, hy - 18, 36, 36, color=c, thickness=2)
            img.draw_rect(hx - 13, hy - 13, 26, 26, color=c, thickness=-1)
            lbl_x = hx - 30 if k == "L" else hx + 20
            img.draw_string(lbl_x, hy - 9, k, color=image.COLOR_WHITE, scale=1.1)

    # buttons
    btn_w, btn_h = 120, 32
    btn_y = 0

    edit_text  = "EDIT" if not zone_config.edit_mode else "SAVE"
    edit_color = image.COLOR_BLUE if not zone_config.edit_mode else image.COLOR_GREEN
    img.draw_rect(0, btn_y, btn_w, btn_h, color=edit_color, thickness=3)
    img.draw_string(12, btn_y + 9, edit_text, color=edit_color, scale=0.9)

    det_text = "DETECT ON" if zone_config.obstacle_detection_enabled else "DETECT OFF"
    det_color = image.COLOR_GREEN if zone_config.obstacle_detection_enabled else image.COLOR_RED
    det_x = zone_config.width - btn_w
    img.draw_rect(det_x, btn_y, btn_w, btn_h, color=det_color, thickness=3)
    img.draw_string(det_x + 6, btn_y + 9, det_text, color=det_color, scale=0.7)

    # stats
    wifi_status = "Wi-Fi:ON" if wifi_connected else "Wi-Fi:OFF"
    det_status = "DET:ON" if zone_config.obstacle_detection_enabled else "DET:OFF"
    stats = f"Obj:{len(target_objects)} In:{len(objects_in_zone)} Ang:{steering_angle:.1f} {wifi_status} {det_status}"
    y_pos = zone_config.height - 14
    img.draw_rect(0, y_pos - 2, len(stats) * 6 + 14, 18, color=image.COLOR_BLACK, thickness=-1)
    img.draw_string(4, y_pos, stats, color=image.COLOR_WHITE, scale=0.7)

    # obstacle banner
    if zone_config.obstacle_detection_enabled and has_obstacle:
        img.draw_rect(cam.width() // 2 - 110, 45, 220, 28, color=image.COLOR_RED, thickness=-1)
        img.draw_string(cam.width() // 2 - 100, 52, "OBSTACLE!", color=image.COLOR_WHITE, scale=0.9)
        send_txt = "SENT" if wifi_connected else "Wi-Fi ERR"
        send_color = image.COLOR_GREEN if wifi_connected else image.COLOR_RED
        img.draw_string(cam.width() // 2 - 35, 76, send_txt, color=send_color, scale=0.8)

    if not zone_config.obstacle_detection_enabled:
        img.draw_rect(cam.width() // 2 - 150, 45, 300, 28, color=image.COLOR_GRAY, thickness=-1)
        img.draw_string(cam.width() // 2 - 140, 52, "DETECTION DISABLED", color=image.COLOR_WHITE, scale=0.8)

    disp.show(img.resize(disp.width(), disp.height(), image.Fit.FIT_FILL))

    # трансляция кадра на телефон через HTTP
    if stream is not None:
        now_ms = time.ticks_ms()
        if now_ms - last_stream_time >= STREAM_INTERVAL_MS:
            last_stream_time = now_ms
            try:
                ret = stream.write(img)
                if ret == 0:  # err.ERR_NONE
                    stream_fail_count = 0
                else:
                    raise Exception(f"write err={ret}")
            except Exception as e:
                stream_fail_count += 1
                if stream_fail_count >= STREAM_FAIL_MAX:
                    print(f"⚠️ Стрим упал {stream_fail_count} раз, перезапуск...")
                    try:
                        stream.stop()
                    except Exception:
                        pass
                    try:
                        gc.collect()
                        stream = http.JpegStreamer()
                        stream.set_html(html_page)
                        ret = stream.start()
                        if ret != 0:
                            raise Exception(f"start err={ret}")
                        stream_fail_count = 0
                        print(f"✅ Стрим перезапущен: http://{stream.host()}:{stream.port()}")
                    except Exception as e2:
                        print(f"❌ Перезапуск не удался: {e2}")
                        stream = None

    # каждые 50 итераций чистим память (чаще для долгой работы)
    gc_counter += 1
    if gc_counter >= 50:
        gc_counter = 0
        gc.collect()

    # лог памяти каждые 5 минут
    now_ms = time.ticks_ms()
    if now_ms - last_mem_log > 300000:
        last_mem_log = now_ms
        try:
            # ВАЖНО: sys здесь — это maix.sys (импортирован сверху), а не stdlib sys.
            # На MaixCAM2 memory_info() -> dict без ключей 'available'/'free'.
            # Свободная user-память = total - used; отдельно показываем CMM.
            mi = sys.memory_info()
            mb = 1024 * 1024
            total = mi.get("total", 0)
            used = mi.get("used", 0)
            free = (total - used) if total else -1
            cmm_free = mi.get("cmm_total", 0) - mi.get("cmm_used", 0)
            print(f"🧠 Память: user free {free // mb} MB / CMM free {cmm_free // mb} MB")
        except Exception as e:
            print(f"🧠 Память: не удалось прочитать ({e})")