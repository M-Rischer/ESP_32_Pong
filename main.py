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
LEFT_DIR, RIGHT_DIR = -1, -1   # pon +1 si quieres invertir sentido

# Botones de los encoders (SW) -> activo en LOW
LEFT_SW, RIGHT_SW = 6, 7

PADDLE_H   = 4
WIN_SCORE  = 3

# --- Velocidad de la pelota (magnitud CONSTANTE) ---
SPEED         = 0.33
MIN_VX_FRAC   = 0.35
MAX_VY_FRAC   = 0.85
SPIN_FRAC     = 0.25

# Colores
RED   = (255,  0,  0)
BLUE  = (  0,  0,255)
GOLD  = (255,215,  0)
PH_G  = ( 80,160, 80)          # puntos “marcadores” apagados (verde tenue)

# --- Límites del campo (excluye fila de marcadores y=0) ---
TOP_WALL = 1            # primera fila jugable
BOT_WALL = H - 1        # última fila (15 en 16x16)

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

# Botones (activo LOW)
btnL = Pin(LEFT_SW,  Pin.IN, Pin.PULL_UP)
btnR = Pin(RIGHT_SW, Pin.IN, Pin.PULL_UP)
def button_pressed():
    if not btnL.value() or not btnR.value():
        time.sleep_ms(20)  # debounce
        return (not btnL.value()) or (not btnR.value())
    return False

# =========================
# Física / utilidades
# =========================
def clamp(v, lo, hi): 
    return lo if v < lo else hi if v > hi else v

def enforce_speed(vx, vy):
    sgnx = 1 if vx >= 0 else -1
    sgny = 1 if vy >= 0 else -1
    # limitar componente vertical
    max_vy = SPEED * MAX_VY_FRAC
    if abs(vy) > max_vy: vy = sgny * max_vy
    # normalizar a SPEED
    mag = (vx*vx + vy*vy) ** 0.5
    if mag == 0: return sgnx * SPEED, 0.0
    scale = SPEED / mag
    vx *= scale; vy *= scale
    # asegurar componente horizontal mínima
    min_vx = SPEED * MIN_VX_FRAC
    if abs(vx) < min_vx:
        vx = sgnx * min_vx
        vy = (SPEED**2 - vx*vx) ** 0.5 * (1 if vy >= 0 else -1)
    return vx, vy

def rand_sign(): return -1 if (urandom.getrandbits(1)) else 1

def reset_point(to_left_start=False):
    bx = (W-1)/2
    by = (H-1)/2
    vx = SPEED * (-1 if to_left_start else 1)
    vy = SPEED * 0.25 * rand_sign()
    vx, vy = enforce_speed(vx, vy)
    return bx, by, vx, vy

# =========================
# Estado del juego
# =========================
lp_y = float(H//2)
rp_y = float(H//2)
lp_prev = left_enc.read()
rp_prev = right_enc.read()
SMOOTH = 0.6

score_l = score_r = 0
ball_fx, ball_fy, vx, vy = reset_point(to_left_start=False)

STATE_READY     = 0
STATE_PLAYING   = 1
STATE_GAME_OVER = 2

state = STATE_READY
winner_color = None
next_serve_to_left = False  # quién saca después (la pelota sale hacia la izquierda si True)

def render_ready():
    clear()
    draw_paddle(0,     int(round(lp_y)), PADDLE_H, RED)
    draw_paddle(W-1,   int(round(rp_y)), PADDLE_H, BLUE)
    draw_ball(W//2, H//2, GOLD)   # pelota centrada, estática
    draw_score(0, 0)
    np.write()

# =========================
# Bucle principal
# =========================
next_tick = time.ticks_add(time.ticks_ms(), FRAME_MS)
gameover_t0 = None

while True:
    # ---------- READY: espera botón ----------
    if state == STATE_READY:
        lp_y = rp_y = float(H//2)
        left_enc.pos = right_enc.pos = 0
        lp_prev = rp_prev = 0
        score_l = score_r = 0
        render_ready()
        while not button_pressed():
            time.sleep_ms(30)
        ball_fx, ball_fy, vx, vy = reset_point(to_left_start=next_serve_to_left)
        state = STATE_PLAYING
        next_tick = time.ticks_add(time.ticks_ms(), FRAME_MS)
        continue

    # ---------- GAME OVER: color del ganador; tras 10s pasa a READY ----------
    if state == STATE_GAME_OVER:
        np.fill(tuple(int(c*BRIGHTNESS) for c in winner_color)); np.write()
        if gameover_t0 is None:
            gameover_t0 = time.ticks_ms()
        while True:
            if button_pressed():
                state = STATE_READY; gameover_t0 = None
                break
            if time.ticks_diff(time.ticks_ms(), gameover_t0) >= 10_000:
                state = STATE_READY; gameover_t0 = None
                break
            time.sleep_ms(30)
        continue

    # ---------- PLAYING ----------
    # palas (suavizado)
    lp_now = left_enc.read()
    rp_now = right_enc.read()
    lp_target = lp_y + (lp_now - lp_prev)
    rp_target = rp_y + (rp_now - rp_prev)
    lp_prev, rp_prev = lp_now, rp_now

    lp_y = lp_y * SMOOTH + lp_target * (1.0 - SMOOTH)
    rp_y = rp_y * SMOOTH + rp_target * (1.0 - SMOOTH)

    # límites de palas: respetar fila de marcadores y permitir llegar a la última
    half = PADDLE_H // 2
    edge_fix = 1 if (PADDLE_H % 2 == 0) else 0
    lo = TOP_WALL + half                 # mínima cy para no invadir y=0
    hi = (H - 1) - half + edge_fix       # tocar fondo incluso con altura par
    lp_y = clamp(lp_y, lo, hi)
    rp_y = clamp(rp_y, lo, hi)

    # mover pelota
    ball_fx += vx
    ball_fy += vy

    # rebote arriba/abajo (la pelota NUNCA entra en y=0)
    if ball_fy < TOP_WALL:
        ball_fy = TOP_WALL
        vy = abs(vy); vx, vy = enforce_speed(vx, vy)
    elif ball_fy > BOT_WALL:
        ball_fy = BOT_WALL
        vy = -abs(vy); vx, vy = enforce_speed(vx, vy)

    # pala izquierda (x=0)
    if vx < 0 and ball_fx <= 0:
        if abs(ball_fy - lp_y) <= PADDLE_H/2:
            ball_fx = 0; vx = abs(vx)
            off = (ball_fy - lp_y) / (PADDLE_H/2)
            vy += off * (SPEED * SPIN_FRAC)
            vx, vy = enforce_speed(vx, vy)
        else:
            score_r += 1
            ball_fx, ball_fy, vx, vy = reset_point(to_left_start=False)

    # pala derecha (x=W-1)
    if vx > 0 and ball_fx >= W-1:
        if abs(ball_fy - rp_y) <= PADDLE_H/2:
            ball_fx = W-1; vx = -abs(vx)
            off = (ball_fy - rp_y) / (PADDLE_H/2)
            vy += off * (SPEED * SPIN_FRAC)
            vx, vy = enforce_speed(vx, vy)
        else:
            score_l += 1
            ball_fx, ball_fy, vx, vy = reset_point(to_left_start=True)

    # dibujar
    clear()
    draw_paddle(0,     int(round(lp_y)), PADDLE_H, RED)
    draw_paddle(W-1,   int(round(rp_y)), PADDLE_H, BLUE)
    draw_ball(int(round(ball_fx)), int(round(ball_fy)), GOLD)
    draw_score(score_l, score_r)
    np.write()

    # condición de victoria → GAME_OVER; el que perdió saca después
    if score_l >= WIN_SCORE or score_r >= WIN_SCORE:
        winner_color = RED if score_l > score_r else BLUE
        next_serve_to_left = (winner_color is BLUE)  # saca el perdedor
        state = STATE_GAME_OVER
        continue

    # timestep fijo
    now = time.ticks_ms()
    sleep_ms = time.ticks_diff(next_tick, now)
    if sleep_ms > 0:
        time.sleep_ms(sleep_ms)
    next_tick = time.ticks_add(next_tick, FRAME_MS)