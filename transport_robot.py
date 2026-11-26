"""Webots controller - autonomous parts-transport robot (e-puck + ground sensors).
Keys: L = load parts / start mission | 1,2,3 = operator takes piece at P1,P2,P3
Everything is edited in the CONFIG block."""
from controller import Robot, Keyboard
import os, threading, urllib.request, urllib.parse
from collections import deque

# ---------------- CONFIG ----------------
SPEED        = 4.0                    # rad/s (e-puck max 6.28)
WHEEL_R      = 0.0205                 # m
LAP_LENGTH   = 4.342                   # m (matches the world)
POS          = {"P1": 0.75, "P2": 1.70, "P3": 2.70}   # m from START
STORAGE_POS  = 3.60                   # m
PICKUP_WAIT  = 10.0                   # s. Real scenario = 120.0 (10 for a quick demo)
DARK         = 600                    # ground sensor value below = black line
TG_TOKEN     = os.getenv("TG_TOKEN", "")   # Telegram bot token (optional)
TG_CHAT      = os.getenv("TG_CHAT", "")    # operator 1 chat id (optional)
# ----------------------------------------

robot = Robot()
TS = int(robot.getBasicTimeStep())
kb = robot.getKeyboard(); kb.enable(TS)

lm, rm = robot.getDevice("left wheel motor"), robot.getDevice("right wheel motor")
for m in (lm, rm):
    m.setPosition(float("inf")); m.setVelocity(0.0)
ls, rs = robot.getDevice("left wheel sensor"), robot.getDevice("right wheel sensor")
ls.enable(TS); rs.enable(TS)

gs = []
for n in ("gs0", "gs1", "gs2"):
    d = robot.getDevice(n)
    if d: d.enable(TS); gs.append(d)
tilt = robot.getDevice("tilt motor")   # optional servo of the tilting platform

leds = [d for d in (robot.getDevice(f"led{i}") for i in range(8)) if d]
def red(v):
    for l in leds: l.set(1 if v else 0)

def odo(): return (ls.getValue() + rs.getValue()) / 2 * WHEEL_R

def drive(l, r): lm.setVelocity(l); rm.setVelocity(r)

def follow_line():
    if len(gs) < 3:                     # no ground sensors -> straight (demo only)
        drive(SPEED, SPEED); return
    l, c, r = [g.getValue() < DARK for g in gs]
    if c and not (l or r): drive(SPEED, SPEED)
    elif l and not r:      drive(SPEED * 0.3, SPEED)
    elif r and not l:      drive(SPEED, SPEED * 0.3)
    else:                  drive(SPEED * 0.6, SPEED * 0.6)

# ---- Telegram with offline queue (never blocks the robot) ----
outbox = deque()
def tg_worker():
    import time
    while True:
        if outbox and TG_TOKEN:
            try:
                data = urllib.parse.urlencode({"chat_id": TG_CHAT, "text": outbox[0]}).encode()
                urllib.request.urlopen(f"https://api.telegram.org/bot{TG_TOKEN}/sendMessage", data, timeout=5)
                outbox.popleft()
            except Exception:
                time.sleep(5)            # Wi-Fi down -> retry later
        else:
            time.sleep(1)
threading.Thread(target=tg_worker, daemon=True).start()

def notify(msg):
    print("[TELEGRAM]", msg); outbox.append(msg)

def screen(msg): print(f"[ECRAN] {msg}")
journal = []
def log(msg):
    line = f"{robot.getTime():7.1f}s  {msg}"; journal.append(line); print("[LOG]", line)

# ---- helpers ----
def read_keys():
    keys = set()
    k = kb.getKey()
    while k != -1:
        keys.add(chr(k) if 0 < k < 256 else ""); k = kb.getKey()
    return keys

def wait(seconds, until=None):
    """Wait with robot stopped; returns True if `until()` became true."""
    drive(0, 0); t0 = robot.getTime()
    while robot.step(TS) != -1:
        if until and until(): return True
        if robot.getTime() - t0 >= seconds: return False
    return False

# ---- mission ----
pieces = {s: True for s in POS}          # True = still in the compartment
pressed = set()

screen("Waiting for loading... press L")
while robot.step(TS) != -1:
    if "L" in read_keys(): break
screen("Mission commencée"); log("Mission start"); print("ground sensors:", [round(g.getValue()) for g in gs], "(tune DARK if the robot loses the line)")

lap, origin = 1, odo()
visited, stored = set(), False
state = "DRIVE"

while robot.step(TS) != -1:
    pressed.update(read_keys())
    s = odo() - origin
    pending = [p for p in POS if pieces[p]]

    # 1) station stop
    for st, pos in POS.items():
        targeted = (lap == 1) or (lap == 2 and pieces[st])
        if targeted and (lap, st) not in visited and s >= pos:
            visited.add((lap, st)); drive(0, 0)
            red(1)
            print(f"[{st}] arrived (lap {lap}) -> red light + vibration")
            ok = wait(PICKUP_WAIT, lambda st=st: (pressed.update(read_keys()) or True) and st[1] in pressed)
            if ok and pieces[st]:
                pieces[st] = False; pressed.discard(st[1])
                screen("Merci"); log(f"{st} delivered (lap {lap})")
            else:
                log(f"{st} pending (lap {lap})")
            red(0); s = odo() - origin
    pending = [p for p in POS if pieces[p]]

    # 2) storage zone after lap 2
    if lap == 2 and pending and not stored and s >= STORAGE_POS:
        drive(0, 0); log("Storage zone reached")
        if tilt: tilt.setPosition(0.8)
        wait(2.0)
        left = list(pending)
        for p in left: pieces[p] = False
        if tilt: tilt.setPosition(0.0)
        wait(1.0)
        assert not any(pieces.values()), "compartments not empty"
        for p in left:
            notify(f"Pièce de la station {p} non récupérée, déposée en zone de stockage")
            log(f"{p} stored")
        stored = True; continue

    # 3) end of loop
    if s >= LAP_LENGTH:
        if lap == 1 and pending:
            lap, origin = 2, origin + LAP_LENGTH
            print("-> lap 2 (pending stations only)"); continue
        drive(0, 0)
        screen("Mission terminée, pièces en stockage" if stored else "Mission terminée avec succès")
        print(f"lap length measured: {s:.2f} m"); break

    follow_line()

drive(0, 0)
print("\n--- Mission log ---"); print("\n".join(journal))
while robot.step(TS) != -1: pass
