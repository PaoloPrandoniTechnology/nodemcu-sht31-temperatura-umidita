import os
from PIL import Image, ImageDraw, ImageFont

OUT = os.path.join(os.path.dirname(__file__), "..", "docs", "images")
os.makedirs(OUT, exist_ok=True)

# Try to load a font
def font(size, bold=False):
    candidates = [
        "C:/Windows/Fonts/consola.ttf",
        "C:/Windows/Fonts/segoeui.ttf",
        "C:/Windows/Fonts/arial.ttf",
    ]
    if bold:
        candidates = ["C:/Windows/Fonts/segoeuib.ttf", "C:/Windows/Fonts/arialbd.ttf"] + candidates
    for c in candidates:
        if os.path.exists(c):
            try:
                return ImageFont.truetype(c, size)
            except Exception:
                pass
    return ImageFont.load_default()

# ---------- Wiring diagram ----------
W, H = 900, 520
img = Image.new("RGB", (W, H), (14, 15, 17))
d = ImageDraw.Draw(img)

# Title
d.text((W//2, 24), "Schema di collegamento - Wemos D1 Mini + SHT31-D", font=font(20, True), fill=(230,232,236), anchor="mt")

# Wemos board
bx, by, bw, bh = 120, 110, 260, 320
d.rounded_rectangle((bx, by, bx+bw, by+bh), 12, fill=(28,31,37), outline=(66,165,245), width=2)
d.text((bx+bw//2, by+14), "Wemos D1 Mini (ESP8266)", font=font(14, True), fill=(230,232,236), anchor="mt")
# pins left
pins_left = [("3V3", 60), ("GND", 110), ("D1 (GPIO5) SCL", 160), ("D2 (GPIO4) SDA", 210)]
for name, off in pins_left:
    y = by + off
    d.rectangle((bx-6, y-6, bx+2, y+6), fill=(66,165,245))
    d.text((bx+10, y), name, font=font(12), fill=(230,232,236), anchor="lm")
# pins right (unused)
d.text((bx+bw-10, by+bh-20), "USB", font=font(11), fill=(123,130,140), anchor="rm")

# SHT31 board
sx, sy, sw, sh = 540, 130, 240, 260
d.rounded_rectangle((sx, sy, sx+sw, sy+sh), 12, fill=(28,31,37), outline=(102,187,106), width=2)
d.text((sx+sw//2, sy+14), "SHT31-D (I2C)", font=font(14, True), fill=(230,232,236), anchor="mt")
spins = [("VCC", 50), ("GND", 90), ("SDA", 130), ("SCL", 170), ("ADDR", 210)]
for name, off in spins:
    y = sy + off
    d.rectangle((sx+sw-2, y-6, sx+sw+6, y+6), fill=(102,187,106))
    d.text((sx+sw-10, y), name, font=font(12), fill=(230,232,236), anchor="rm")

# Wires
def wire(y1, y2, color, label, midx=None):
    # from wemos right? Actually pins on right side of wemos box left edge; draw from box right? We'll draw from wemos pin dot at bx to sht pin at sx
    pass

# Map connections:
# Wemos 3V3 -> SHT VCC
# Wemos GND -> SHT GND (and ADDR)
# Wemos D2 (SDA) -> SHT SDA
# Wemos D1 (SCL) -> SHT SCL

conn = [
    ("3V3", 60, "VCC", 50, (255, 180, 80)),
    ("GND", 110, "GND", 90, (180, 180, 180)),
    ("D2 (GPIO4) SDA", 210, "SDA", 130, (66, 165, 245)),
    ("D1 (GPIO5) SCL", 160, "SCL", 170, (239, 83, 80)),
    ("GND", 110, "ADDR", 210, (180, 180, 180)),
]

for wname,woff,sname,soff,color in conn:
    y1 = by + woff
    y2 = sy + soff
    x1 = bx + bw
    x2 = sx
    # route with mid
    mx = (x1 + x2)//2
    d.line([(x1, y1), (mx, y1), (mx, y2), (x2, y2)], fill=color, width=3)
    # dots
    d.ellipse((x1-4, y1-4, x1+4, y1+4), fill=color)
    d.ellipse((x2-4, y2-4, x2+4, y2+4), fill=color)

# Legend
ly = by + bh + 20
d.text((W//2, ly), "Legenda colori: arancio = 3.3V   grigio = GND   blu = SDA (D2/GPIO4)   rosso = SCL (D1/GPIO5)", font=font(12), fill=(167,173,182), anchor="mt")
d.text((W//2, ly+22), "ADDR collegato a GND => indirizzo I2C 0x44. Alimentazione MAX 3.3V (NON 5V).", font=font(12, True), fill=(255,179,0), anchor="mt")

# Note box
ny = 40
d.rounded_rectangle((bx, 70, bx+bw, 95), 6, fill=(20,22,26), outline=(60,60,70))
d.text((bx+bw//2, 82), "Pin: Wire.begin(SDA, SCL) => SDA prima!", font=font(11), fill=(255,179,0), anchor="mm")

img.save(os.path.join(OUT, "wiring.png"))

# ---------- Dashboard mockup ----------
W, H = 980, 620
img = Image.new("RGB", (W, H), (14, 15, 17))
d = ImageDraw.Draw(img)
d.text((24, 18), "Sensori Temperatura · Umidità · Soggiorno", font=font(17, True), fill=(233,236,239))
d.text((24, 44), "SHT31-D I²C (D1=SCL · D2=SDA) · HIGH accuracy 1Hz", font=font(12), fill=(123,130,140))
# status dot
d.ellipse((W-120, 22, W-104, 38), fill=(102,187,106))
d.text((W-96, 30), "connesso", font=font(12), fill=(167,173,182), anchor="lm")

# tabs
tabs = ["Dashboard", "Configurazione", "Firmware"]
x = 24
for t in tabs:
    col = (30,136,229) if t=="Dashboard" else (21,23,27)
    tw = 120
    d.rounded_rectangle((x, 70, x+tw, 96), 8, fill=col, outline=(42,46,54))
    d.text((x+tw//2, 83), t, font=font(13, True), fill=(255,255,255), anchor="mm")
    x += tw + 6

# table
tx, ty = 24, 110
tw, th = W-48, 440
d.rounded_rectangle((tx, ty, tx+tw, ty+th), 10, fill=(21,23,27), outline=(42,46,54))
# header
d.rectangle((tx, ty, tx+tw, ty+34), fill=(28,31,37))
for i,(name,w) in enumerate([("Parametro",0.38),("Valore",0.34),("Note / Unità",0.28)]):
    pass
# header texts
d.text((tx+14, ty+10), "PARAMETRO", font=font(11, True), fill=(221,226,232))
d.text((tx+int(tw*0.38)+int(tw*0.17), ty+10), "VALORE", font=font(11, True), fill=(221,226,232), anchor="mm")
d.text((tx+int(tw*0.72), ty+10), "NOTE / UNITÀ", font=font(11, True), fill=(221,226,232))

# category row
cy = ty + 34
d.rectangle((tx, cy, tx+tw, cy+26), fill=(36,40,48))
d.text((tx+14, cy+6), "SHT31-D · AMBIENTE (I²C @0x44 · HIGH ACCURACY)", font=font(12, True), fill=(244,246,249))

rows = [
    ("Temperatura", "23.45", "°C", "ok"),
    ("Umidità Relativa", "48.2", "%RH", "ok"),
    ("Punto di Rugiada", "11.8", "°C", ""),
    ("Indice di Calore", "23.1", "°C", ""),
    ("Umidità Assoluta", "10.0", "g/m³", ""),
    ("Heater SHT31", "OFF", "", ""),
    ("Stato sensori (bitmask)", "0x3F", "6 bit", "ok"),
    ("RSSI WiFi", "-52", "dBm", "ok"),
    ("Free Heap", "38420", "bytes", ""),
    ("Stack usato (WM)", "1820", "bytes", ""),
    ("Uptime", "02:14:33", "hh:mm:ss", ""),
]
y = cy + 26
for k,v,u,state in rows:
    d.line([(tx, y+28), (tx+tw, y+28)], fill=(30,33,40))
    d.text((tx+14, y+6), k, font=font(13), fill=(167,173,182))
    col = (102,187,106) if state=="ok" else (246,248,250)
    d.text((tx+int(tw*0.55), y+6), v, font=font(14, True), fill=col, anchor="mm")
    d.text((tx+int(tw*0.72), y+8), u, font=font(11), fill=(123,130,140))
    y += 30

# footer
fy = ty + th + 8
d.text((tx, fy), "host: 192.168.0.42  ·  ws://...:81", font=font(11), fill=(123,130,140))
d.text((tx+tw, fy), "ultimo aggiornamento: 12:34:56", font=font(11), fill=(123,130,140), anchor="ra")

img.save(os.path.join(OUT, "dashboard.png"))

# ---------- Home Assistant mockup ----------
W, H = 900, 520
img = Image.new("RGB", (W, H), (14, 15, 17))
d = ImageDraw.Draw(img)
d.text((24, 18), "Home Assistant - Scheda Stanze (esempio)", font=font(17, True), fill=(233,236,239))
# cards
cards = [("Soggiorno", "23.4 °C", "48 %"), ("Cantina", "18.1 °C", "62 %"), ("Camera", "21.0 °C", "55 %")]
x = 24
for name,t,rh in cards:
    d.rounded_rectangle((x, 70, x+270, 260), 12, fill=(21,23,27), outline=(42,46,54))
    d.text((x+14, 86), name, font=font(14, True), fill=(233,236,239))
    d.text((x+14, 120), t, font=font(28, True), fill=(66,165,245))
    d.text((x+14, 170), "Umidità: "+rh, font=font(13), fill=(167,173,182))
    x += 290
d.text((24, 300), "Entità MQTT auto-discovery: sensor.<device>_temperature, _humidity, _dewpoint, ...", font=font(12), fill=(123,130,140))
img.save(os.path.join(OUT, "homeassistant.png"))

print("done")