"""Mô phỏng 3D cánh tay bám theo góc từ ESP32 (hoặc từ 5 cảm biến AS5600).

Chạy với firmware ESP32 của bạn :  python arm_live.py --port COM5
Chạy với 5 cảm biến AS5600      :  python arm_live.py --port COM5 --mode sensors
Chạy thử không cần ESP32        :  python arm_live.py --demo
Phím tắt trong cửa sổ 3D:  Z = đặt góc zero (chuẩn góc),  S = lưu offset,  V = nói lệnh bằng giọng nói.
Có thể gõ lệnh ngay trong terminal (SET 0 90, STATUS, ...) rồi Enter, thay cho Serial Monitor.
"""
import os, sys, re, math, json, time, queue, argparse, threading, atexit, unicodedata, socket
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
# Lệnh tiếng Việt: góc kênh CH4 khi mở/đóng kẹp (độ). Mặc định lấy theo giá trị AUTOCH4 trên trang web của bạn.
GRIP_OPEN_CH4 = 70
GRIP_CLOSE_CH4 = 110
BASE_STEP_DEFAULT = 15            # "xoay đế trái" không nói số độ -> dùng giá trị này
PREVIEW_SEC = 1.5                 # thời gian chạy xem trước trên mô hình
CONFIRM_TIMEOUT = 30              # quá số giây này mà chưa xác nhận thì tự huỷ
# --- Giọng nói (nhấn V trong cửa sổ 3D để nói) ---
VOICE_LANG = "vi-VN"
VOICE_ONLINE_TIMEOUT = 2.0      # thoi gian kiem tra duong Internet (giay)
VOICE_GOOGLE_TIMEOUT = 5.0      # thoi gian toi da cho phan hoi Google (giay)
# NHẬN DIỆN OFFLINE: giải nén model Vosk tiếng Việt vào live/vosk-model-small-vn-0.4
VOICE_MODEL_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "vosk-model-small-vn-0.4")
VOICE_DEVICE = None               # None = micro mặc định; xem danh sách bằng: arm_live.py --list-mics
VOICE_MAX_SEC = 6                 # nghe tối đa bấy nhiêu giây sau khi bắt đầu nói
VOICE_WAIT_START_SEC = 6          # chờ tối đa bấy nhiêu giây để bạn bắt đầu nói
VOICE_END_SILENCE_SEC = 1.2       # im lặng bấy nhiêu giây thì coi như nói xong
VOICE_MIN_LEVEL = 200             # ngưỡng âm lượng tối thiểu (thang int16); micro yếu thì hạ xuống 200
VOICE_AUTO_CONFIRM = True         # sau lệnh nói, tự nghe tiếp để bạn nói "đồng ý" hoặc "huỷ"
VOICE_DICTIONARY_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "robot_voice_commands.json")
# ========================================================

here = os.path.dirname(os.path.abspath(__file__))
OFFSET_FILE = os.path.join(here, "offsets.json")
POSES_FILE = os.path.join(here, "poses.json")
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


def normalize(text):
    """Chữ thường, bỏ dấu tiếng Việt: 'Mở kẹp' -> 'mo kep'."""
    t = unicodedata.normalize("NFD", text.lower())
    t = "".join(c for c in t if unicodedata.category(c) != "Mn").replace("đ", "d")
    return re.sub(r"\s+", " ", t).strip()


def load_robot_voice_dictionary():
    """Doc tu dien de dung chung cho Vosk grammar va kiem tra lenh da nhan dang."""
    with open(VOICE_DICTIONARY_FILE, "r", encoding="utf-8") as fp:
        dictionary = json.load(fp)
    phrase_to_intent = {}
    grammar = []
    for item in dictionary["intents"]:
        for phrase in item["phrases"]:
            key = normalize(phrase)
            if key in phrase_to_intent and phrase_to_intent[key] != item["id"]:
                raise ValueError("Trung cum tu khac y nghia: " + phrase)
            phrase_to_intent[key] = item["id"]
            if phrase not in grammar:
                grammar.append(phrase)
    return grammar, phrase_to_intent


def resolve_voice_phrase(text, phrase_to_intent):
    """Chi nhan dung cum tu da khai bao; khong doan gan dung cho robot that."""
    normalized = normalize(text)
    intent = phrase_to_intent.get(normalized)
    if not intent:
        return None
    canonical = {
        "GRIP_OPEN": "mở kẹp", "GRIP_CLOSE": "đóng kẹp",
        "BASE_LEFT": "xoay đế trái", "BASE_RIGHT": "xoay đế phải",
        "GO_POSE_1": "về vị trí một", "GO_POSE_2": "về vị trí hai",
        "GO_POSE_3": "về vị trí ba", "STOP": "dừng",
        "CONFIRM": "đồng ý", "CANCEL": "hủy"
    }
    return canonical.get(intent)


YES = {"y", "yes", "co", "ok", "dong y", "thuc hien", "chay", "ok luon", "duoc", "xac nhan", "vang"}
NO = {"n", "no", "khong", "huy", "huy bo", "thoi", "bo qua", "dung lai di", "khong can"}

_UNITS = {"khong": 0, "mot": 1, "hai": 2, "ba": 3, "bon": 4, "tu": 4, "nam": 5, "lam": 5,
          "sau": 6, "bay": 7, "tam": 8, "chin": 9}


def parse_vn_number(tokens):
    """['ba','muoi','lam'] -> 35. Hỗ trợ 0..199. Trả về None nếu không phải số."""
    t = list(tokens)
    if not t:
        return None

    def u(k):
        return _UNITS.get(t[k]) if k < len(t) else None
    i, total = 0, 0
    if u(0) is not None and len(t) > 1 and t[1] == "tram":
        total, i = u(0) * 100, 2
        if i < len(t) and t[i] in ("linh", "le"):
            i += 1
            if u(i) is None:
                return None
            total += u(i)
            i += 1
            return total if i == len(t) else None
    elif t[0] == "tram":
        total, i = 100, 1
    if i < len(t) and t[i] == "muoi":
        total += 10
        i += 1
        if i < len(t):
            if u(i) is None:
                return None
            total += u(i)
            i += 1
    elif i + 1 < len(t) and u(i) is not None and t[i + 1] == "muoi":
        total += u(i) * 10
        i += 2
        if i < len(t):
            if u(i) is None:
                return None
            total += u(i)
            i += 1
    elif i == 0 and len(t) == 1 and u(0) is not None:
        total, i = u(0), 1
    elif i > 0 and i == len(t) - 1 and u(i) is not None:
        total += u(i)
        i += 1
    return total if i == len(t) else None


def _tail_number(tail):
    """'30' | '12,5' | 'ba muoi' -> số, hoặc None."""
    tail = tail.strip()
    if re.fullmatch(r"\d+(?:[.,]\d+)?", tail):
        return float(tail.replace(",", "."))
    return parse_vn_number(tail.split())

HELP_TEXT = """Lệnh tiếng Việt (có thể gõ không dấu):
  mở kẹp | đóng kẹp
  nâng CH0 đến 120 độ | hạ CH0 đến 110 độ
  duỗi CH1 đến 130 độ | gập CH1 đến 100 độ
  quay đế đến 90 độ | xoay CH3 đến 80 độ
  mở kẹp đến 70 độ | đóng kẹp đến 110 độ
  xoay đế trái [n độ] | xoay đế phải [n độ]
  về vị trí N          (chỉ khi Python đã thấy bạn SAVE vị trí N)
  dừng                 (gửi STOP ngay, không cần xác nhận)
  v                    (nghe giọng nói; hoặc nhấn phím V trong cửa sổ 3D). Số có thể nói bằng chữ: "ba mươi độ"
Sau mỗi lệnh mô hình 3D chạy thử; gõ y để gửi xuống robot, n để huỷ.
Lệnh khác (SET, MOVE5, STATUS...) được gửi thẳng tới ESP32 như bình thường."""


# CH0 nang/ha; CH1 duoi/gap; CH2 quay de; CH3 xoay co tay; CH4 mo/dong kep.
# Chi ho tro GOC TUYET DOI; khong tu suy ra huong/goc hien tai.
_VOICE_JOINTS = {
    0: ("ch0", "vai phai", "khop khong", "khop 0"),
    1: ("ch1", "vai trai", "khop mot", "khop 1"),
    2: ("ch2", "de", "khop hai", "khop 2"),
    3: ("ch3", "co tay", "khop ba", "khop 3"),
    4: ("ch4", "kep", "tay kep", "khop bon", "khop 4"),
}
_VOICE_VERBS = {
    0: ("nang", "ha"),
    1: ("duoi", "gap"),
    2: ("quay", "xoay"),
    3: ("xoay", "quay"),
    4: ("mo", "dong"),
}

def parse_absolute_joint_command(text):
    """Dich 'nang ch0 den 120 do' / 'xoay co tay den 90 do' => SET.
    Khong phan tich chuoi gan dung, khong nhan lenh tuong doi bang cach doan goc.
    """
    text = re.sub(r"^(?:robot|ro bot|hey robot)\s+", "", text)
    text = re.sub(r"^(?:hay|lam on|giup toi)\s+", "", text)
    text = re.sub(r"^(?:dua|dieu khien)\s+", "", text)
    # Goc 'den' la goc tuyet doi, khong phai tang/giam delta.
    m = re.fullmatch(r"(.+?)\s+(?:den|toi|ve)\s+(.+?)\s+do", text)
    if not m: return None
    subject, angle_text = m.group(1).strip(), m.group(2).strip()
    angle = _tail_number(angle_text)
    if angle is None or not (0 <= angle <= 180): return None
    words = subject.split()
    if not words: return None
    verb = words[0]
    name = " ".join(words[1:]).strip()
    for channel, names in _VOICE_JOINTS.items():
        if verb in _VOICE_VERBS[channel] and name in names:
            return {"type": "joint_absolute", "ch": channel, "angle": float(angle), "verb": verb}
    # 'ch0 nang den 120 do' dang nguoc lai
    for channel, names in _VOICE_JOINTS.items():
        if words[-1] in _VOICE_VERBS[channel] and " ".join(words[:-1]) in names:
            return {"type": "joint_absolute", "ch": channel, "angle": float(angle), "verb": words[-1]}
    return None

def parse_command(text):
    """text đã normalize. Trả về dict mô tả lệnh, hoặc None nếu không phải lệnh tiếng Việt."""
    text = re.sub(r"^(?:robot|ro bot|hey robot)\s+", "", text)                      # từ gọi robot
    text = re.sub(r"(?:\s+(?:nhe|di|nao|giup toi|cho toi))+$", "", text)           # từ đệm cuối câu
    absolute = parse_absolute_joint_command(text)
    if absolute is not None: return absolute
    if text in ("help", "tro giup", "?"):
        return {"type": "help"}
    if text in ("dung", "stop", "dung lai"):
        return {"type": "stop"}
    if re.fullmatch(r"(?:mo|nha)(?: cai)? kep(?: ra)?", text):
        return {"type": "grip", "open": True}
    if re.fullmatch(r"(?:dong|kep)(?: cai)? kep(?: lai)?|kep lai", text):
        return {"type": "grip", "open": False}
    m = re.match(r"^xoay de(?: sang)? (trai|phai)(?: (.+?))?(?: do)?$", text)
    if m:
        tail = m.group(2)
        deg = BASE_STEP_DEFAULT if tail in (None, "do") else _tail_number(tail)
        if deg is None:
            return None
        return {"type": "base", "left": m.group(1) == "trai", "deg": deg}
    m = re.match(r"^(?:ve|di toi|di den|toi|den) vi tri(?: so)?(?: (.+))?$", text)
    if m and m.group(1):
        n = _tail_number(m.group(1))
        if n is None or n != int(n):
            return None
        return {"type": "go", "n": int(n)}
    return None


def build_plan(cmd, raw_cur, sign, poses):
    """Từ lệnh -> (góc đích 5 kênh, danh sách lệnh gửi ESP32, mô tả). Ném ValueError nếu không hợp lệ."""
    # Không yêu cầu SETALL. Với kênh chưa nhận trạng thái, dùng NaN để giữ nguyên
    # và chỉ mô phỏng những khớp đã có dữ liệu. Không tự suy ra góc vật lý.
    target = list(raw_cur)
    if cmd["type"] == "joint_absolute":
        ch, angle = cmd["ch"], cmd["angle"]
        target[ch] = angle
        cmds = [f"SET {ch} {angle:g}"]
        desc = f"{cmd['verb']} CH{ch} đến {angle:g}° (góc servo tuyệt đối)"
    elif cmd["type"] == "grip":
        target[4] = GRIP_OPEN_CH4 if cmd["open"] else GRIP_CLOSE_CH4
        cmds = [f"SET 4 {target[4]:.1f}"]
        desc = "mở kẹp" if cmd["open"] else "đóng kẹp"
    elif cmd["type"] == "base":
        if not 0 < cmd["deg"] <= 180:
            raise ValueError("số độ phải trong khoảng 0..180")
        dyaw = cmd["deg"] if cmd["left"] else -cmd["deg"]          # trái = chiều dương của khớp đế mô hình
        if math.isnan(raw_cur[2]):
            raise ValueError("Lệnh xoay TƯƠNG ĐỐI cần biết góc CH2. Hãy đồng bộ góc CH2 trước; không đoán vị trí hiện tại.")
        target[2] = raw_cur[2] + dyaw / (sign[2] * SCALE[2])
        cmds = [f"SET 2 {target[2]:.1f}"]
        desc = f"xoay đế sang {'trái' if cmd['left'] else 'phải'} {cmd['deg']:g}°"
    elif cmd["type"] == "go":
        if cmd["n"] not in poses:
            raise ValueError(f"chưa biết góc của vị trí {cmd['n']} (Python chỉ biết vị trí bạn SAVE khi nó đang chạy)")
        target = list(poses[cmd["n"]])
        cmds = [f"GO {cmd['n']}"]
        desc = f"về vị trí đã lưu số {cmd['n']}"
    else:
        raise ValueError("lệnh không hỗ trợ")
    for i, v in enumerate(target):
        if math.isnan(v):
            continue
        if not math.isnan(raw_cur[i]) and abs(v - raw_cur[i]) < 1e-9:
            continue
        if not SERVO_MIN[i] <= v <= SERVO_MAX[i]:
            raise ValueError(f"CH{i} = {v:.1f} vượt phạm vi {SERVO_MIN[i]}..{SERVO_MAX[i]}")
    return target, cmds, desc


class Commander:
    """Nhận dòng chữ người dùng gõ: lệnh tiếng Việt -> xem trước trên mô hình -> xác nhận -> gửi ESP32."""
    def __init__(self, send, sign):
        self.send, self.sign = send, sign
        self.listen_cb = None                              # hàm bắt đầu nghe giọng nói
        self.plan = None
        self.poses = {}
        self._saving, self._buf = None, {}
        if os.path.exists(POSES_FILE):
            try:
                self.poses = {int(k): v for k, v in json.load(open(POSES_FILE)).items()}
            except (ValueError, OSError):
                pass

    def feed_text(self, text):
        """Đọc thông báo SAVE của firmware để biết góc các vị trí đã lưu (kể cả lưu từ trang web)."""
        m = re.match(r"DA LUU THANH CONG: SAVE (\d+)", text)
        if m:
            self._saving, self._buf = int(m.group(1)), {}
            return
        m = re.match(r"CH(\d) = (-?[\d.]+) do", text)
        if m and self._saving is not None:
            self._buf[int(m.group(1))] = float(m.group(2))
            if len(self._buf) == 5:
                self.poses[self._saving] = [self._buf[i] for i in range(5)]
                json.dump(self.poses, open(POSES_FILE, "w"))
                display.message(f"Đã ghi nhớ góc vị trí {self._saving}: {self.poses[self._saving]}")
                self._saving = None

    def handle(self, line, raw_live, now, from_voice=False):
        text = normalize(line)
        if text in ("v", "nghe", "mic", "noi") and self.listen_cb:
            self.listen_cb()
            return
        if self.plan is not None:                         # đang chờ xác nhận
            if text in YES:
                for c in self.plan["cmds"]:
                    self.send(c)
                display.message("Đã gửi tới robot: " + ", ".join(self.plan["cmds"]))
                self.plan = None
            elif text in NO:
                display.message("Đã huỷ.")
                self.plan = None
            else:
                display.message("Đang chờ xác nhận: gõ y để thực hiện hoặc n để huỷ.")
            return
        cmd = parse_command(text)
        if cmd is None:
            if from_voice:
                display.message("TỪ CHỐI: không hiểu lệnh giọng nói thuộc danh sách cho phép; không gửi xuống ESP32.")
            elif line.strip():
                self.send(line.strip())                   # chỉ lệnh người dùng GÕ trực tiếp mới gửi thô
            return
        if cmd["type"] == "help":
            display.message(HELP_TEXT)
            return
        if cmd["type"] == "stop":
            self.send("STOP")
            display.message("Đã gửi STOP.")
            return
        try:
            target, cmds, desc = build_plan(cmd, raw_live, self.sign, self.poses)
        except ValueError as e:
            display.message("Từ chối: " + str(e))
            return
        # Góc kênh chưa biết chỉ dùng cho hình xem trước tham khảo (giữ tư thế đang vẽ).
        # Không lấy các giá trị giả định này làm góc của robot thật.
        start = [v if not math.isnan(v) else float("nan") for v in raw_live]
        self.plan = {"start": start, "target": target, "t0": now, "dur": PREVIEW_SEC, "cmds": cmds}
        if any(math.isnan(v) for v in raw_live):
            display.message("CẢNH BÁO: chưa có phản hồi góc đầy đủ. Mô phỏng chỉ xem trước các khớp có dữ liệu; "
                            "không kiểm chứng được va chạm thực tế.")
        if cmd["type"] == "joint_absolute" and cmd["ch"] in (0, 1):
            display.message("LƯU Ý: CH0/CH1 có liên kết cơ khí; chỉ xác nhận góc đã thử nghiệm an toàn.")
        display.message(f"XEM TRƯỚC trên mô hình: {desc}\n  Lệnh sẽ gửi: {', '.join(cmds)}\n"
                        "  Robot thật CHƯA chuyển động. Gõ y để thực hiện, n để huỷ.")

    def sim_raw(self, raw_live, now):
        """Góc dùng để vẽ mô hình: dữ liệu thật, hoặc chuyển động xem trước nếu đang chờ xác nhận."""
        if self.plan is None:
            return raw_live
        p_ = self.plan
        if now - p_["t0"] > CONFIRM_TIMEOUT:
            display.message("Quá thời gian chờ xác nhận, đã huỷ.")
            self.plan = None
            return raw_live
        t = min(1.0, (now - p_["t0"]) / p_["dur"])
        k = t * t * (3 - 2 * t)
        return [a + (b - a) * k if not (math.isnan(a) or math.isnan(b)) else float("nan")
                for a, b in zip(p_["start"], p_["target"])]


class Voice:
    """Nhấn V: thu một lần, thử Google online, tự chuyển Vosk khi offline/lỗi."""
    def __init__(self, out_queue):
        self.q = out_queue
        self.busy = False
        self.last_finished = 0.0
        self.missing = ""
        self.model = None
        self.voice_grammar = []
        self.phrase_to_intent = {}
        try:
            self.voice_grammar, self.phrase_to_intent = load_robot_voice_dictionary()
            import numpy, sounddevice, vosk   # noqa: F401
            if os.path.isdir(VOICE_MODEL_DIR):
                vosk.SetLogLevel(-1)
                self.model = vosk.Model(VOICE_MODEL_DIR)
            else:
                display.message("Lưu ý: không có model Vosk; Google vẫn có thể nhận diện khi online.")
        except Exception as e:
            self.missing = f"{type(e).__name__}: {e}"

    @property
    def ok(self):
        return not self.missing

    def listen_async(self):
        if not self.ok:
            display.message("Chưa cài thư viện giọng nói (" + self.missing + ").\n"
                            "  Cài: py -m pip install vosk sounddevice numpy; xem HUONG_DAN_OFFLINE.md")
            return
        if self.busy:
            return
        self.busy = True
        threading.Thread(target=self._run, daemon=True).start()

    def _record(self):
        import numpy as np
        import sounddevice as sd
        rate, block_s = 16000, 0.05
        block = int(rate * block_s)
        pre, frames, started, silence = [], [], False, 0
        display.message("Đang nghe... hãy nói lệnh.")
        with sd.InputStream(samplerate=rate, channels=1, dtype="int16", blocksize=block, device=VOICE_DEVICE) as st:
            noise = []
            for _ in range(4):                                  # đo nhiễu nền ~0.2 giây
                data, _ = st.read(block)
                noise.append(float(np.sqrt(np.mean(data.astype(np.float32) ** 2))))
            thr = max(VOICE_MIN_LEVEL, 3 * float(np.mean(noise)))
            display.message(f"MIC: nhiễu nền={float(np.mean(noise)):.0f}, ngưỡng kích hoạt={thr:.0f}. Hãy nói khi đã thấy Đang nghe.")
            t0 = time.time()
            t_speech = None
            while True:
                data, _ = st.read(block)
                level = float(np.sqrt(np.mean(data.astype(np.float32) ** 2)))
                if level > thr:
                    if not started:
                        started, t_speech, frames = True, time.time(), list(pre)
                    silence = 0
                elif started:
                    silence += 1
                if started:
                    frames.append(data.copy())
                    if silence * block_s > VOICE_END_SILENCE_SEC or time.time() - t_speech > VOICE_MAX_SEC:
                        break
                else:
                    pre = (pre + [data.copy()])[-8:]
                    if time.time() - t0 > VOICE_WAIT_START_SEC:
                        display.message("MIC: Không phát hiện âm thanh vượt ngưỡng. Kiểm tra micro mặc định/VOICE_DEVICE.")
                        return None
        captured = np.concatenate(frames).reshape(-1)
        peak = int(np.max(np.abs(captured.astype(np.int32))))
        rms = float(np.sqrt(np.mean(captured.astype(np.float32)**2)))
        display.message(f"MIC: thu {len(captured)/rate:.2f}s, RMS={rms:.0f}, peak={peak} (thang 32767)")
        return captured

    def _google_available(self):
        """Kiểm tra truy cập Internet thực tế, không chỉ trạng thái Wi-Fi."""
        try:
            with socket.create_connection(("www.google.com", 443), timeout=VOICE_ONLINE_TIMEOUT):
                return True
        except (OSError, TimeoutError):
            return False

    def _recognize_vosk(self, pcm):
        """Nhận dạng offline bằng bộ từ điển hạn chế."""
        if self.model is None:
            display.message("VOSK: chưa có mô hình tiếng Việt tại " + VOICE_MODEL_DIR)
            return ""
        import vosk
        grammar = list(self.voice_grammar)
        # Vosk offline: bo sung nhung goc thuong dung (5, 10, 15, 20...180).
        # Cac goc khac van co the nhan qua Google; Vosk khong bao dam moi so.
        numbers = [0, 5, 10, 15, 20, 30, 40, 45, 50, 60, 70, 75, 80, 90, 100, 110, 120, 130, 135, 140, 150, 160, 170, 180]
        for ch, names in _VOICE_JOINTS.items():
            for verb in _VOICE_VERBS[ch]:
                for a in numbers:
                    grammar.append(f"{verb} {names[0]} đến {a} độ")
        grammar.append("[unk]")
        rec = vosk.KaldiRecognizer(self.model, 16000, json.dumps(grammar, ensure_ascii=False))
        data = pcm.tobytes()
        for i in range(0, len(data), 8000):
            rec.AcceptWaveform(data[i:i + 8000])
        raw = json.loads(rec.FinalResult()).get("text", "").strip()
        display.message("VOSK (offline): " + repr(raw))
        canonical = resolve_voice_phrase(raw, self.phrase_to_intent)
        if raw and not canonical:
            if parse_command(normalize(raw)) is not None:
                canonical = raw
            else:
                display.message("Câu Vosk không thuộc danh sách lệnh hợp lệ, không gửi ESP32.")
        return canonical or ""

    def _recognize_google(self, pcm):
        """Dùng cùng đoạn âm thanh đã thu; không mở micro lần thứ hai."""
        import speech_recognition as sr
        rec = sr.Recognizer()
        rec.operation_timeout = VOICE_GOOGLE_TIMEOUT
        audio = sr.AudioData(pcm.tobytes(), 16000, 2)  # mono PCM16, 16 kHz
        raw = rec.recognize_google(audio, language=VOICE_LANG).strip()
        display.message("GOOGLE (online): " + repr(raw))
        canonical = resolve_voice_phrase(raw, self.phrase_to_intent)
        if raw and not canonical:
            if parse_command(normalize(raw)) is not None:
                canonical = raw  # Lenh goc co tham so, chi cho phep khi parser kiem tra dung.
            else:
                display.message("Google nhận ra câu ngoài danh sách lệnh hợp lệ, không gửi ESP32.")
        return canonical or ""

    def _recognize(self, pcm):
        """Google nếu kết nối được; tự fallback Vosk khi mất mạng/lỗi/không khớp."""
        if self._google_available():
            try:
                display.message("NHẬN DIỆN: Google (online)")
                recognized = self._recognize_google(pcm)
                if recognized:
                    return recognized
                display.message("Google chưa nhận được lệnh hợp lệ → thử Vosk offline.")
            except ImportError:
                display.message("Chưa cài SpeechRecognition → dùng Vosk offline.")
            except Exception as exc:
                display.message(f"Google không khả dụng ({type(exc).__name__}: {exc}) → chuyển Vosk offline.")
        else:
            display.message("Không truy cập được Internet → sử dụng Vosk offline.")
        return self._recognize_vosk(pcm)

    def _run(self):
        text = ""
        try:
            pcm = self._record()
            if pcm is None:
                display.message("Không nghe thấy giọng nói. Nhấn V để thử lại.")
            else:
                display.message("Đang nhận dạng...")
                text = self._recognize(pcm)
                if not text:
                    display.message("Không hiểu được. Nhấn V để nói lại.")
        except Exception as e:
            display.message(f"Lỗi giọng nói: {e}")
        self.busy = False
        self.last_finished = time.time()                        # tránh nghe lại liên tục tức thì
        if text:
            display.message(f"Nghe được: \"{text}\"")
            self.q.put((text, True))


class Sensors:
    def __init__(self, port, demo):
        self.raw = [float("nan")] * 5
        self.demo = demo
        self.lock = threading.Lock()
        self.last_rx = 0.0
        self.cmd_queue = queue.Queue()      # các dòng người dùng gõ (hoặc từ giọng nói sau này)
        self.on_text = None                 # hàm nhận thông báo văn bản của firmware
        self.voice_events = queue.Queue()  # sự kiện VOICE ON/OFF từ SW3
        threading.Thread(target=self._stdin_loop, daemon=True).start()
        if not demo:
            import serial
            # dtr/rts = False trước khi mở để KHÔNG làm ESP32 khởi động lại (giữ nguyên trạng thái đã SET)
            self.ser = serial.Serial()
            self.ser.port, self.ser.baudrate, self.ser.timeout = port, BAUD, 0.2
            self.ser.dtr = False
            self.ser.rts = False
            self.ser.open()
            threading.Thread(target=self._read_loop, daemon=True).start()
            atexit.register(self._bye)

    def _send(self, text):
        try:
            self.ser.write(text.encode())
        except Exception:
            pass

    def send_line(self, line):
        if self.demo:
            display.message(f"(demo) gửi ESP32: {line}")
        else:
            self._send(line + "\n")

    def _stdin_loop(self):
        """Đọc dòng gõ vào terminal, đưa vào hàng đợi cho vòng lặp chính xử lý."""
        while True:
            line = sys.stdin.readline()
            if not line:
                return
            display.invalidate()
            if line.strip():
                self.cmd_queue.put((line, False))

    def _bye(self):
        # Không gửi STREAM OFF vì firmware hiện tại có thể không hỗ trợ.
        pass

    def _read_loop(self):
        while True:
            # Không tự gửi STREAM ON định kỳ. Chỉ nhận dữ liệu ESP32 chủ động xuất.
            try:
                text = self.ser.readline().decode(errors="ignore").strip()
            except OSError:
                time.sleep(0.1)
                continue
            vals = parse_line(text)
            if text in ("VOICE ON", "VOICE OFF"):
                self.voice_events.put(text == "VOICE ON")
            if vals is None and text and not text.startswith(("RX: STREAM", "STREAM")):
                display.message("ESP32: " + text)          # thông báo của firmware (thay Serial Monitor)
                if self.on_text:
                    self.on_text(text)
            if vals is not None:
                with self.lock:
                    self.raw = vals
                self.last_rx = time.time()

    def get(self):
        if self.demo:
            t = time.time()
            return [90 + 20 * math.sin(t * 0.7), 90 + 30 * math.sin(t * 0.9),
                    90 + 40 * math.sin(t * 0.5), 90 + 25 * math.sin(t * 1.1),
                    90 + 15 * math.sin(t * 1.5)]
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


def table_lines(raw, joints, speeds, limits, xyz_mm, preview=False):
    lines = ["Khớp      Góc       Tốc độ        Giới hạn        Trạng thái" + ("   [XEM TRƯỚC - robot thật chưa chạy]" if preview else "")]
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
    ap.add_argument("--list-mics", action="store_true", help="liệt kê micro rồi thoát")
    args = ap.parse_args()

    if args.list_mics:
        import sounddevice as sd
        print(sd.query_devices())
        print("Chọn micro bằng cách đặt VOICE_DEVICE = <số> ở đầu arm_live.py")
        return
    sensors = Sensors(args.port, args.demo)
    SIGN = SIGN_ESP if args.mode == "esp" else SIGN_SENSORS
    commander = Commander(sensors.send_line, SIGN)
    sensors.on_text = commander.feed_text
    voice = Voice(sensors.cmd_queue)
    commander.listen_cb = voice.listen_async
    voice_sw3_on = False  # không tự bật micro khi mới kết nối; phải nhấn SW3
    offsets = [90.0] * 5 if args.demo else [0.0] * 5     # demo: giữa dải servo (90) là zero
    if os.path.exists(OFFSET_FILE) and not args.demo:
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
    print("Gõ 'help' trong terminal để xem lệnh tiếng Việt; lệnh firmware (SET, STATUS...) gõ như bình thường.")
    if voice.ok:
        print("Giọng nói: Google khi online, Vosk khi offline. SW3(GPIO27) bật/tắt nghe liên tục; V nghe một lần.")
    else:
        print("Giọng nói offline chưa sẵn sàng: " + voice.missing + " (xem HUONG_DAN_OFFLINE.md)")

    while p.isConnected():
        # SW3 joystick 3 (GPIO27) gửi VOICE ON/OFF qua Serial.
        while True:
            try:
                voice_sw3_on = sensors.voice_events.get_nowait()
            except queue.Empty:
                break
            if not voice_sw3_on:
                commander.plan = None
                display.message("VOICE OFF: tắt chu kỳ nghe mới, huỷ lệnh chờ xác nhận.")
            else:
                display.message("VOICE ON: tự nghe các câu lệnh bằng micro. Nhấn SW3 lần nữa để tắt.")
        if voice_sw3_on and voice.ok and not voice.busy and time.time() - voice.last_finished > 0.8 \
                and sensors.cmd_queue.empty():
            voice.listen_async()
        raw_live = sensors.get()
        now_t = time.time()
        while True:                                   # xử lý các dòng người dùng vừa gõ
            try:
                line, from_voice = sensors.cmd_queue.get_nowait()
            except queue.Empty:
                break
            had_plan = commander.plan is not None
            commander.handle(line, raw_live, now_t, from_voice=from_voice)
            if from_voice and VOICE_AUTO_CONFIRM and commander.plan is not None and not had_plan:
                display.message("Hãy nói \"đồng ý\" để thực hiện hoặc \"huỷ\"...")
                if not voice_sw3_on:
                    voice.listen_async()                  # chế độ V thủ công: tự nghe xác nhận
        raw = commander.sim_raw(raw_live, now_t)      # = raw_live, hoặc chuyển động xem trước

        keys = p.getKeyboardEvents()
        if keys.get(ord("v"), 0) & p.KEY_WAS_TRIGGERED:
            voice.listen_async()
        if keys.get(ord("z"), 0) & p.KEY_WAS_TRIGGERED:
            for i in range(5):
                if not math.isnan(raw_live[i]):
                    offsets[i] = raw_live[i] - SIGN[i] * CAL_POSE_DEG[i] / SCALE[i]
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
            display.block(table_lines(raw, joints, speeds, limits, xyz, commander.plan is not None))

        p.stepSimulation()
        time.sleep(1 / 240)


if __name__ == "__main__":
    main()
