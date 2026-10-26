"""Mô phỏng 3D cánh tay bám theo góc từ ESP32 (hoặc từ 5 cảm biến AS5600).

Chạy với firmware ESP32 của bạn :  python arm_live.py --port COM5
Chạy với 5 cảm biến AS5600      :  python arm_live.py --port COM5 --mode sensors
Chạy thử không cần ESP32        :  python arm_live.py --demo
Phím tắt trong cửa sổ 3D:  Z = đặt góc zero (chuẩn góc),  S = lưu offset.
Có thể gõ lệnh ngay trong terminal (SET 0 90, STATUS, ...) rồi Enter, thay cho Serial Monitor.
"""
import os, sys, math, json, time, argparse, threading, atexit
import pybullet as p
import pybullet_data

# ======================= CẤU HÌNH =======================
BAUD = 115200
# Chế độ "esp"     : đọc góc lệnh từ firmware ESP32 của bạn, thứ tự kênh CH0..CH4 =
#                    [servo vai (độ cao), servo khuỷu (góc tuyệt đối thanh trước), đế, cổ tay, kẹp]
# Chế độ "sensors" : đọc 5 AS5600, thứ tự = [đế, vai, khuỷu, cổ tay, kẹp]
SIGN_ESP = [1, 1, -1, 1, -1]      # đổi dấu (1 <-> -1) nếu khớp nào quay ngược chiều với thật (kẹp đã đảo)
SIGN_SENSORS = [1, 1, 1, 1, 1]
SCALE = [1, 1, 1, 1, 1]           # hệ số: góc mô hình = góc đọc x SCALE (kẹp, cổ tay có thể khác 1)
# Góc (độ) của kênh ở lúc bấm Z. Mặc định 0 = đúng tư thế trong Fusion.
CAL_POSE_DEG = [0, 0, 0, 0, 0]
SMOOTH = 0.4                      # 0..1, càng nhỏ càng mượt nhưng càng trễ
GRIP_RANGE_DEG = (-20, 40)        # giới hạn góc kẹp trong mô phỏng
# Phạm vi lệnh của 5 kênh ESP32 (độ). Đặt GIỐNG minAngle/maxAngle trong firmware của bạn.
SERVO_MIN = [0, 0, 0, 0, 0]
SERVO_MAX = [180, 180, 180, 180, 180]
# Tự đặt giới hạn của 5 khớp mô hình theo độ: (min, max) hoặc None để dùng giá trị tính tự động.
# Thứ tự: đế, vai, khuỷu, cổ tay, kẹp. Ví dụ: [None, (-20, 80), (-90, 60), None, None]
LIMIT_OVERRIDE = [None, None, None, None, None]
LIMIT_MARGIN = 5                  # báo "gần giới hạn" khi còn cách giới hạn dưới số độ này
# Góc nghiêng của bàn tay so với phương ngang (độ, âm = chúc xuống). None = dùng giá trị sẵn trong arm5.urdf
# (khoảng -32.5°, bàn tay thẳng hàng với 2 thanh cẳng tay). Muốn thẳng hơn/ngả hơn thì thử -25, -35, 0...
HAND_TILT_DEG = None
# ========================================================

here = os.path.dirname(os.path.abspath(__file__))
OFFSET_FILE = os.path.join(here, "offsets.json")
URDF = os.path.join(here, "..", "arm5.urdf")
if not os.path.exists(URDF):
    URDF = os.path.join(here, "arm5.urdf")


class Display:
    """In một bảng cập nhật tại chỗ trong terminal; thông báo khác được chèn lên trên mà không làm vỡ bảng."""
    def __init__(self):
        self.lock = threading.Lock()
        self.h = 0

    def invalidate(self):             # gọi khi người dùng gõ Enter: bảng cũ nằm yên, bảng mới in bên dưới
        with self.lock:
            self.h = 0

    def message(self, text):
        with self.lock:
            if self.h:
                sys.stdout.write(f"\033[{self.h}A\033[J")
                self.h = 0
            sys.stdout.write(text + "\n")
            sys.stdout.flush()

    def block(self, lines):
        with self.lock:
            if self.h:
                sys.stdout.write(f"\033[{self.h}A")
            for ln in lines:
                sys.stdout.write("\r\033[K" + ln + "\n")
            sys.stdout.flush()
            self.h = len(lines)


display = Display()


def urdf_path():
    """Trả về file URDF; nếu HAND_TILT_DEG khác None thì tạo bản tạm với góc nghiêng bàn tay mới."""
    if HAND_TILT_DEG is None:
        return URDF
    import re
    txt = open(URDF, encoding="utf-8").read()
    pat = r'(<joint name="j4_wrist"[^>]*>\s*<parent[^>]*/>\s*<child[^>]*/>\s*<origin xyz="[^"]*") rpy="[^"]*"'
    new, n = re.subn(pat, lambda m: f'{m.group(1)} rpy="{math.radians(HAND_TILT_DEG):.5f} 0 0"', txt)
    if n != 1:
        print("Cảnh báo: không đổi được góc nghiêng bàn tay trong URDF, dùng góc mặc định.")
        return URDF
    out = os.path.join(os.path.dirname(URDF), "_arm5_run.urdf")
    open(out, "w", encoding="utf-8").write(new)
    return out


def wrap180(a):
    """Đưa hiệu góc về (-180, 180] để xử lý chỗ nhảy 359 -> 0."""
    return (a + 180.0) % 360.0 - 180.0


def parse_line(line):
    """Nhận 'ANG,a0,a1,a2,a3,a4' hoặc 'a0,a1,a2,a3,a4'. Trả về list 5 số (có thể có nan) hoặc None."""
    line = line.strip()
    if line.startswith("ANG,"):
        line = line[4:]
    try:
        vals = [float(x) for x in line.split(",")]
    except ValueError:
        return None
    return vals if len(vals) == 5 else None


class Sensors:
    def __init__(self, port, demo):
        self.raw = [float("nan")] * 5
        self.demo = demo
        self.lock = threading.Lock()
        self.last_rx = 0.0
        if not demo:
            import serial
            # dtr/rts = False trước khi mở để KHÔNG làm ESP32 khởi động lại (giữ nguyên trạng thái đã SET)
            self.ser = serial.Serial()
            self.ser.port, self.ser.baudrate, self.ser.timeout = port, BAUD, 0.2
            self.ser.dtr = False
            self.ser.rts = False
            self.ser.open()
            threading.Thread(target=self._read_loop, daemon=True).start()
            threading.Thread(target=self._stdin_loop, daemon=True).start()
            atexit.register(self._bye)

    def _send(self, text):
        try:
            self.ser.write(text.encode())
        except Exception:
            pass

    def _stdin_loop(self):
        """Gõ lệnh vào terminal (SET 0 90, MOVE5 ..., STATUS ...) -> gửi tới ESP32 (thay Serial Monitor)."""
        while True:
            line = sys.stdin.readline()
            if not line:
                return
            display.invalidate()
            line = line.strip()
            if line:
                self._send(line + "\n")

    def _bye(self):
        self._send("STREAM OFF\n")

    def _read_loop(self):
        last_cmd = 0.0
        while True:
            now = time.time()
            # firmware mặc định tắt STREAM; nếu chưa thấy dữ liệu thì bật lại mỗi 3 giây
            if now - self.last_rx > 3 and now - last_cmd > 3:
                self._send("STREAM ON\n")
                last_cmd = now
            try:
                text = self.ser.readline().decode(errors="ignore").strip()
            except OSError:
                time.sleep(0.1)
                continue
            vals = parse_line(text)
            if vals is None and text and not text.startswith(("RX: STREAM", "STREAM")):
                display.message("ESP32: " + text)          # thông báo của firmware (thay Serial Monitor)
            if vals is not None:
                with self.lock:
                    self.raw = vals
                self.last_rx = time.time()

    def get(self):
        if self.demo:
            t = time.time()
            return [10 + 40 * math.sin(t * 0.5), 20 * math.sin(t * 0.7),
                    30 * math.sin(t * 0.9), 25 * math.sin(t * 1.1),
                    10 + 15 * math.sin(t * 1.5)]
        with self.lock:
            return list(self.raw)


def to_joints(d, mode):
    """d = lệch góc 5 kênh so với zero (độ). Trả về [đế, vai, khuỷu, cổ tay, kẹp] của mô hình."""
    if mode == "esp":
        # CH0 = góc tay vai so với phương ngang, CH1 = góc tuyệt đối của thanh trước (cơ cấu bình hành),
        # nên khớp khuỷu thật = CH1 - CH0. CH2 = đế.
        return [d[2], d[0], d[1] - d[0], d[3], d[4]]
    return list(d)


JOINT_NAMES = ["Đế", "Vai", "Khuỷu", "Cổ tay", "Kẹp"]


def joint_limits(offsets, sign, mode):
    """Giới hạn góc của 5 khớp mô hình (độ). Với chế độ esp: suy ra từ phạm vi lệnh servo và góc zero đã chuẩn."""
    if mode != "esp":
        return list(LIMIT_OVERRIDE)                  # chế độ sensors: chỉ dùng giá trị bạn tự đặt
    d = []
    for i in range(5):
        a = sign[i] * SCALE[i] * (SERVO_MIN[i] - offsets[i])
        b = sign[i] * SCALE[i] * (SERVO_MAX[i] - offsets[i])
        d.append((min(a, b), max(a, b)))
    # khuỷu = CH1 - CH0 nên giới hạn là hiệu hai khoảng
    lim = [d[2], d[0], (d[1][0] - d[0][1], d[1][1] - d[0][0]), d[3], d[4]]
    lim[4] = (max(lim[4][0], GRIP_RANGE_DEG[0]), min(lim[4][1], GRIP_RANGE_DEG[1]))
    for i, o in enumerate(LIMIT_OVERRIDE):
        if o is not None:
            lim[i] = o
    return lim


def limit_status(a, lim):
    if lim is None:
        return ""
    lo, hi = lim
    if a < lo or a > hi:
        return "VƯỢT GIỚI HẠN!"
    if a - lo < LIMIT_MARGIN or hi - a < LIMIT_MARGIN:
        return "gần giới hạn"
    return "OK"


def table_lines(raw, joints, speeds, limits, xyz_mm):
    lines = ["Khớp      Góc       Tốc độ        Giới hạn        Trạng thái"]
    for i, n in enumerate(JOINT_NAMES):
        lim = limits[i]
        rng = "     --      " if lim is None else f"{lim[0]:+5.0f}..{lim[1]:+4.0f}°"
        lines.append(f"{n:<8}{joints[i]:+7.1f}° {speeds[i]:+8.1f}°/s  {rng:<14}  {limit_status(joints[i], lim)}")
    ch = " ".join("--" if math.isnan(v) else f"{v:.0f}" for v in raw)
    lines.append(f"Đầu kẹp: x {xyz_mm[0]:+4.0f}  y {xyz_mm[1]:+4.0f}  z {xyz_mm[2]:4.0f} mm | góc lệnh CH0..CH4: {ch}")
    return lines


def main():
    os.system("")        # bật mã điều khiển terminal (ANSI) trên Windows
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM5")
    ap.add_argument("--demo", action="store_true")
    ap.add_argument("--mode", choices=["esp", "sensors"], default="esp")
    ap.add_argument("--no-info", action="store_true", help="tắt dòng số liệu trong terminal")
    args = ap.parse_args()

    sensors = Sensors(args.port, args.demo)
    SIGN = SIGN_ESP if args.mode == "esp" else SIGN_SENSORS
    offsets = [0.0] * 5
    if os.path.exists(OFFSET_FILE):
        offsets = json.load(open(OFFSET_FILE))
        print("Đã nạp offset:", offsets)

    p.connect(p.GUI)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.loadURDF("plane.urdf")
    p.resetDebugVisualizerCamera(0.9, 50, -20, [0, 0.15, 0.1])
    robot = p.loadURDF(urdf_path(), useFixedBase=True)
    idx = {p.getJointInfo(robot, j)[1].decode(): j for j in range(p.getNumJoints(robot))}
    names = ["j1_base", "j2_shoulder", "j3_elbow", "j4_wrist"]

    smooth = [0.0] * 5          # góc đã lọc (độ)
    last_info = time.time()
    prev_joints = [0.0] * 5
    bx, by = p.getJointInfo(robot, idx["j1_base"])[14][:2]   # trục quay của đế (m)
    TOOL = (0.0, 0.11, 0.017)                                 # điểm kẹp so với gốc bàn tay (m)
    print("Bấm Z trong cửa sổ 3D để đặt zero (đặt cánh tay thật đúng tư thế chuẩn trước), S để lưu.")
    if not args.demo:
        print("Có thể gõ lệnh ngay trong terminal này (SET 0 90, STATUS, ...) rồi Enter; không cần Serial Monitor.")

    while p.isConnected():
        raw = sensors.get()

        keys = p.getKeyboardEvents()
        if keys.get(ord("z"), 0) & p.KEY_WAS_TRIGGERED:
            for i in range(5):
                if not math.isnan(raw[i]):
                    offsets[i] = raw[i] - SIGN[i] * CAL_POSE_DEG[i] / SCALE[i]
            display.message("Đã đặt zero: " + str([round(o, 1) for o in offsets]))
        if keys.get(ord("s"), 0) & p.KEY_WAS_TRIGGERED:
            json.dump(offsets, open(OFFSET_FILE, "w"))
            display.message("Đã lưu " + OFFSET_FILE)

        for i in range(5):
            if math.isnan(raw[i]):
                continue                      # kênh chưa SET hoặc lỗi: giữ góc cũ
            target = SIGN[i] * SCALE[i] * wrap180(raw[i] - offsets[i])
            smooth[i] += SMOOTH * (target - smooth[i])

        joints = to_joints(smooth, args.mode)    # [đế, vai, khuỷu, cổ tay, kẹp]
        for i, n in enumerate(names):
            p.resetJointState(robot, idx[n], math.radians(joints[i]))
        g = max(GRIP_RANGE_DEG[0], min(GRIP_RANGE_DEG[1], joints[4]))
        p.resetJointState(robot, idx["j5_jaw_up"], math.radians(g))
        p.resetJointState(robot, idx["j5_jaw_low"], -math.radians(g))

        now = time.time()
        if not args.no_info and now - last_info > 0.2:                # cập nhật ~5 lần/giây
            dt = now - last_info
            speeds = [(joints[i] - prev_joints[i]) / dt for i in range(5)]
            prev_joints, last_info = list(joints), now
            pos, orn = p.getLinkState(robot, idx["j4_wrist"], computeForwardKinematics=True)[:2]
            R = p.getMatrixFromQuaternion(orn)                     # 9 số, theo hàng
            tx = pos[0] + R[1] * TOOL[1] + R[2] * TOOL[2]
            ty = pos[1] + R[4] * TOOL[1] + R[5] * TOOL[2]
            tz = pos[2] + R[7] * TOOL[1] + R[8] * TOOL[2]
            # x = tới trước, y = sang trái, z = lên (mm), gốc ở tâm trục đế trên mặt sàn
            xyz = ((ty - by) * 1000, -(tx - bx) * 1000, tz * 1000)
            limits = joint_limits(offsets, SIGN, args.mode)
            display.block(table_lines(raw, joints, speeds, limits, xyz))

        p.stepSimulation()
        time.sleep(1 / 240)


if __name__ == "__main__":
    main()
