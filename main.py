from machine import Pin
import neopixel, time, urandom

# =========================
# Hardware & tuning
# =========================
W = H = 16
N = W * H
DATA_PIN = 18
BRIGHTNESS = 0.05
FRAME_MS = 70                  # ~14 FPS

# Encoders (VCC=3V3, GND=GND)
LEFT_CLK,  LEFT_DT  = 2, 3
RIGHT_CLK, RIGHT_DT = 4, 5
LEFT_DIR, RIGHT_DIR = -1, -1   # flip to +1 if needed

PADDLE_H   = 4
WIN_SCORE  = 3

# --- Ball speed (CONSTANT magnitude) ---
SPEED         = 0.33           # pixels per frame (set your desired constant speed)
MIN_VX_FRAC   = 0.35           # keep at least 35% of SPEED horizontally
MAX_VY_FRAC   = 0.85           # cap vertical component to 85% of SPEED
SPIN_FRAC     = 0.25           # how much spin adds to vy (fraction of SPEED)

# Colors
RED   = (255,  0,  0)
BLUE  = (  0,  0,255)
GOLD  = (255,215,  0)
PH_G  = ( 80,160, 80)          # placeholder pips (light, dim green)

# =========================
# LED helpers (serpentine)
# =========================
np = neopixel.NeoPixel(Pin(DATA_PIN, Pin.OUT), N)

def put(x, y, rgb):
    if 0 <= x < W and 0 <= y < H:
        i = y*W + (x if (y % 2 == 0) else (W-1-x))
        r,g,b = rgb
        np[i] = (int(r*BRIGHTNESS), int(g*BRIGHTNESS), int(b*BRIGHTNESS))

def clear(): np.fill((0,0,0))

def draw_paddle(x, cy, h, color):
    half = h // 2
    for dy in range(-half, half + (h % 2 == 1)):
        yy = cy + dy
        if 0 <= yy < H: put(x, yy, color)

def draw_ball(x, y, color=GOLD): put(x, y, color)

def draw_score(sl, sr):
    for i in range(3):
        put(1 + i*2,   0, RED  if i < sl else PH_G)
        put(W-2 - i*2, 0, BLUE if i < sr else PH_G)

# =========================
# Rotary (IRQ + debounce)
# =========================
class RotaryIRQ:
    def _init_(self, clk_pin, dt_pin, dir=+1, min_ms=2):
        self.clk = Pin(clk_pin, Pin.IN, Pin.PULL_UP)
        self.dt  = Pin(dt_pin,  Pin.IN, Pin.PULL_UP)
        self.dir = 1 if dir >= 0 else -1
        self.pos = 0
        self._last_clk = self.clk.value()
        self._last_ms  = time.ticks_ms()
        self._min_ms   = min_ms
        self.clk.irq(handler=self._cb, trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING)
    def _cb(self, pin):
        now = time.ticks_ms()
        if time.ticks_diff(now, self._last_ms) < self._min_ms:
            return
        c = self.clk.value()
        if c != self._last_clk:
            self._last_clk = c
            self.pos += self.dir if (self.dt.value() != c) else -self.dir
            self._last_ms = now
    def read(self): return self.pos

left_enc  = RotaryIRQ(LEFT_CLK,  LEFT_DT,  dir=LEFT_DIR,  min_ms=2)
right_enc = RotaryIRQ(RIGHT_CLK, RIGHT_DT, dir=RIGHT_DIR, min_ms=2)

# =========================
# Helpers for constant speed
# =========================
def clamp(v, lo, hi): 
    return lo if v < lo else hi if v > hi else v

def enforce_speed(vx, vy):
    """Normalize (vx,vy) to fixed SPEED, keep vx sign, cap vy, enforce min |vx|."""
    sgnx = 1 if vx >= 0 else -1
    sgny = 1 if vy >= 0 else -1

    # cap vertical component first
    max_vy = SPEED * MAX_VY_FRAC
    if abs(vy) > max_vy:
        vy = sgny * max_vy

    # normalize to SPEED
    mag = (vx*vx + vy*vy) ** 0.5
    if mag == 0:
        return sgnx * SPEED, 0.0
    scale = SPEED / mag
    vx *= scale
    vy *= scale

    # ensure minimum horizontal component
    min_vx = SPEED * MIN_VX_FRAC
    if abs(vx) < min_vx:
        vx = sgnx * min_vx
        # adjust vy to keep total speed exactly SPEED
        vy = (SPEED**2 - vx*vx) ** 0.5 * (1 if vy >= 0 else -1)

    return vx, vy

def rand_sign(): return -1 if (urandom.getrandbits(1)) else 1

def reset_point(to_left_start=False):
    bx = (W-1)/2
    by = (H-1)/2
    # start mostly horizontal with a small random vertical component
    vx = SPEED * (-1 if to_left_start else 1)
    vy = SPEED * 0.25 * rand_sign()
    vx, vy = enforce_speed(vx, vy)          # <-- assign first
    return bx, by, vx, vy                    # <-- then return

# =========================
# Game state
# =========================
lp_y = float(H//2)
rp_y = float(H//2)
lp_prev = left_enc.read()
rp_prev = right_enc.read()
SMOOTH = 0.6

score_l = score_r = 0
ball_fx, ball_fy, vx, vy = reset_point(to_left_start=False)

# =========================
# Main loop
# =========================
next_tick = time.ticks_add(time.ticks_ms(), FRAME_MS)

while True:
    # --- paddles from encoders (smooth) ---
    lp_now = left_enc.read()
    rp_now = right_enc.read()
    lp_target = lp_y + (lp_now - lp_prev)
    rp_target = rp_y + (rp_now - rp_prev)
    lp_prev, rp_prev = lp_now, rp_now

    lp_y = lp_y * SMOOTH + lp_target * (1.0 - SMOOTH)
    rp_y = rp_y * SMOOTH + rp_target * (1.0 - SMOOTH)

    lo, hi = PADDLE_H/2, (H-1) - PADDLE_H/2
    lp_y = clamp(lp_y, lo, hi)
    rp_y = clamp(rp_y, lo, hi)

    # --- move ball ---
    ball_fx += vx
    ball_fy += vy

    # top/bottom bounce
    if ball_fy < 0:
        ball_fy = 0; vy = abs(vy)
        vx, vy = enforce_speed(vx, vy)
    elif ball_fy > H-1:
        ball_fy = H-1; vy = -abs(vy)
        vx, vy = enforce_speed(vx, vy)

    # left paddle (x=0)
    if vx < 0 and ball_fx <= 0:
        if abs(ball_fy - lp_y) <= PADDLE_H/2:
            ball_fx = 0; vx = abs(vx)      # reflect to the right
            # add spin proportional to impact offset
            off = (ball_fy - lp_y) / (PADDLE_H/2)   # -1..+1
            vy += off * (SPEED * SPIN_FRAC)
            vx, vy = enforce_speed(vx, vy)
        else:
            score_r += 1
            ball_fx, ball_fy, vx, vy = reset_point(to_left_start=False)

    # right paddle (x=W-1)
    if vx > 0 and ball_fx >= W-1:
        if abs(ball_fy - rp_y) <= PADDLE_H/2:
            ball_fx = W-1; vx = -abs(vx)   # reflect to the left
            off = (ball_fy - rp_y) / (PADDLE_H/2)
            vy += off * (SPEED * SPIN_FRAC)
            vx, vy = enforce_speed(vx, vy)
        else:
            score_l += 1
            ball_fx, ball_fy, vx, vy = reset_point(to_left_start=True)

    # --- draw (single write; no blinking) ---
    clear()
    draw_paddle(0,     int(round(lp_y)), PADDLE_H, RED)
    draw_paddle(W-1,   int(round(rp_y)), PADDLE_H, BLUE)
    draw_ball(int(round(ball_fx)), int(round(ball_fy)), GOLD)
    draw_score(score_l, score_r)
    np.write()

    # --- win condition (solid screen, no flashing) ---
    if score_l >= WIN_SCORE or score_r >= WIN_SCORE:
        winner = RED if score_l > score_r else BLUE
        np.fill(tuple(int(c*BRIGHTNESS) for c in winner)); np.write()
        time.sleep(2.0)
        # restart
        score_l = score_r = 0
        lp_y = rp_y = float(H//2)
        left_enc.pos = right_enc.pos = 0
        lp_prev = rp_prev = 0
        ball_fx, ball_fy, vx, vy = reset_point(to_left_start=(winner is BLUE))

    # --- fixed timestep ---
    now = time.ticks_ms()
    sleep_ms = time.ticks_diff(next_tick, now)
    if sleep_ms > 0:
        time.sleep_ms(sleep_ms)
    next_tick = time.ticks_add(next_tick, FRAME_MS)