# =============================================================================
# ae3_skener_test.py - WRO FE track scan test for OpenMV AE3 (12 lines)
# -----------------------------------------------------------------------------
# The camera sits on a servo tower; the servo is turned from the laptop via the
# ESP32. The robot stands in the parking lot in the CCW or CW start position.
#
# Modes (NACIN):
#   "LIVE"        - live image of one view (SMJER + KUT)
#   "PROVJERA"    - shows angles 0, 1, 2 for PROVJERA_S seconds each, measures
#                   each angle, then prints the 12-line map for the ESP32
#   "KALIBRACIJA" - one view: prints ROI rows for ROI[SMJER][KUT]
#   "SKEN"        - repeated CCW + CW scans and comparison of the maps
#
# Line numbering (each direction counts in its own driving direction):
#   line = 3 * section + L, section 0 = start section, L = 0, 1, 2
#   At most one pillar per line, so only the line colour is needed.
#   Same physical line in the other direction: 3*((4-S) % 4) + (2-L)
#
# Works on OpenMV firmware v4.8.1 and v5.0.0.
# Take ROI coordinates ONLY from the KALIBRACIJA printout, never from the IDE preview.
# =============================================================================

import csi
import time
from machine import Pin
try:
    from machine import LED                  # optional RGB LED
except ImportError:
    LED = None


# ============================ 1. SETTINGS ====================================

NACIN = "LIVE"                           # "LIVE", "PROVJERA", "KALIBRACIJA" or "SKEN"
SMJER = "CCW"                                 # "CCW" or "CW" (LIVE, PROVJERA, KALIBRACIJA)
KUT = 2                                      # angle index 0, 1, 2 (LIVE, KALIBRACIJA)

KUTOVI = [35, 80, 125]                       # tower angles in degrees (CCW = left, CW = right)

PROVJERA_S = 10                              # seconds per angle in PROVJERA
TRAZI_MRLJE = True                           # draw detected blobs in LIVE / PROVJERA

# --- SKEN ---
SMJEROVI_SKENA = ["CCW", "CW"]
BROJ_SKENOVA = 10
ODBROJAVANJE_S = 0                           # 0 = wait for SW button, >0 = countdown in seconds
PRIKAZ_REZULTATA_MS = 2000                   # how long a measured image stays on screen

# --- Exposure and colours ---
SMIRIVANJE_MS = 800                          # settle time before a measurement snapshot
ZAKLJUCAJ_BALANS = True                      # lock white balance after start
ZAKLJUCAJ_EKSPOZICIJU = False                # lock exposure after start

# LAB thresholds (L_min, L_max, A_min, A_max, B_min, B_max); OpenMV swaps reversed min/max
PRAG_CRVENA  = (11, 28, 55, 5, -5, 41)   # red pillar
PRAG_ZELENA  = (15, 85, -70, -15, -10, 60)   # green pillar
PRAG_MAGENTA = (20, 85,  25,  90, -60,  5)   # magenta parking walls

MIN_PIKSELA = 15                             # smaller blobs are treated as noise
ROI_RUB = 12                                 # KALIBRACIJA: margin added around each blob

SPREMI_SLIKE = False                         # save JPEGs to the camera drive

BROJ_LINIJA = 12

# --- ROIs: (line, x, y, w, h), line 0-11 in that direction's numbering ---
# A line may have several rectangles (inner + outer seat); the one with more colour pixels wins.
#25112026
ROI = {                                      # rječnik: smjer -> {indeks kuta -> popis ROI-ja}
    "CCW": {                                 # CCW: brojevi linija u CCW numeraciji
        0: [
            (3, 64,335, 135, 90),
            (3, 242, 245, 157,144)
        ],                                   # kraj kuta 0
        1: [
            (2,641, 412, 174,257),
            (4, 777, 242, 128,41),
            (4, 939, 176,86,71),
            (5, 703, 152,78, 79),
            (5, 800, 150, 112, 41),
            (6, 470,141, 68,47),
            (7, 219,150,71,39)

        ],                                   # kraj kuta 1
        2: [
            (8, 866, 103, 90, 39),
            (9, 541, 120, 144,69),
            (10, 371, 142,144, 79),
            (11, 90, 178, 223, 69),
            (0, 89, 288, 74, 104),
            (1, 498, 324,165, 232)

        ],                                   # kraj kuta 2
    },                                       # kraj CCW
    "CW": {
        0: [
            (1, 841, 335, 186, 227),
            (2, 230, 287, 341, 157),
            (3, 493, 182, 172, 82),
            (4, 731, 150, 165 ,92),
            (5, 911, 129 , 174, 68)
        ],                                   # kraj kuta 0
        1: [
            (6, 397, 129, 111, 47),
            (7, 622, 127, 96, 32),
            (8, 851, 112, 101, 43),
            (0, 972,446,272, 260)
        ],                                   # kraj kuta 1
        2: [
            (9, 259, 135, 92, 90),
            (9, 363, 135, 92, 28),
            (10, 365, 195, 96, 77),
            (10, 495, 152, 67, 71),
            (11, 547, 238, 112, 109),
            (11, 708, 174, 109, 143)


        ],                                   # kraj kuta 2
    },                                       # kraj CW
}

# Expected layout for SKEN: 12 chars R/G/- in CCW numbering (spaces allowed), None = no check
OCEKIVANO = None                             # e.g. "-R- G-- -R- --G"


# ============================ 2. HELPERS =====================================

csi0 = csi.CSI()
try:
    tipka = Pin("SW", Pin.IN)                # user button: released = 1, pressed = 0
except Exception:
    tipka = None                             # no button -> SKEN uses a countdown

ZUTA = (255, 255, 0)
SIVA = (128, 128, 128)
BIJELA = (255, 255, 255)
BOJE = {"R": (255, 0, 0), "G": (0, 255, 0), "M": (255, 0, 255), "-": BIJELA}

if OCEKIVANO:
    OCEKIVANO = OCEKIVANO.replace(" ", "")
    if len(OCEKIVANO) != BROJ_LINIJA:
        raise ValueError("OCEKIVANO mora imati 12 znakova bez razmaka")


def led(ime, ukljuci):                       # switch an LED colour on/off; errors are ignored
    try:
        if ukljuci:
            LED(ime).on()
        else:
            LED(ime).off()
    except Exception:
        pass


def opis_linije(linija):                     # 7 -> "S2 L1"
    return "S%d L%d" % (linija // 3, linija % 3)


def druga_linija(linija):                    # same physical line in the other direction (CCW <-> CW)
    s, l = linija // 3, linija % 3
    return 3 * ((4 - s) % 4) + (2 - l)


def gdje_kamera(smjer, i):                   # e.g. "35 st. desno"
    return "%d st. %s" % (KUTOVI[i], "lijevo" if smjer == "CCW" else "desno")


# --- firmware compatibility: blob properties are methods in v4, attributes in v5 ---
def svojstvo(b, ime):
    v = getattr(b, ime)
    return v() if callable(v) else v


def bx(b): return svojstvo(b, "x")
def by(b): return svojstvo(b, "y")
def bw(b): return svojstvo(b, "w")
def bh(b): return svojstvo(b, "h")
def bp(b): return svojstvo(b, "pixels")


# --- firmware compatibility: v5 needs tuples, v4 falls back to separate arguments ---
def crtaj_liniju(img, x0, y0, x1, y1, boja):
    try:
        img.draw_line((x0, y0, x1, y1), color=boja)
    except (TypeError, ValueError):
        img.draw_line(x0, y0, x1, y1, color=boja)


def crtaj_okvir(img, x, y, w, h, boja, debljina=2):
    try:
        img.draw_rectangle((x, y, w, h), color=boja, thickness=debljina)
    except (TypeError, ValueError):
        img.draw_rectangle(x, y, w, h, color=boja, thickness=debljina)


def crtaj_tekst(img, x, y, tekst, boja, velicina=2):
    try:
        img.draw_string((x, max(0, y)), tekst, color=boja, scale=velicina)
    except (TypeError, ValueError):          # v4 expects (x, y, text) -> ValueError
        img.draw_string(x, max(0, y), tekst, color=boja, scale=velicina)


def odrezi(x, y, w, h, sirina, visina):      # clip a rectangle to the image
    x = max(0, min(x, sirina - 1))
    y = max(0, min(y, visina - 1))
    w = max(1, min(w, sirina - x))
    h = max(1, min(h, visina - y))
    return x, y, w, h


def osvjezi_ide(img):                        # push an annotated image to the IDE Frame Buffer
    try:
        img.flush()
    except Exception:
        pass


def init_kamera():                           # RGB565, HD 1280x800; returns width, height
    csi0.reset()
    csi0.pixformat(csi.RGB565)
    csi0.framesize(csi.HD)
    csi0.snapshot(time=2000)                 # skip frames for 2 s (returns None)
    img = csi0.snapshot()
    print("Rezolucija: %d x %d" % (img.width(), img.height()))
    return img.width(), img.height()


def zakljucaj():                             # lock white balance (and optionally exposure)
    if ZAKLJUCAJ_BALANS:
        try:
            g = csi0.rgb_gain_db()
            csi0.auto_whitebal(False, rgb_gain_db=g)
            print("Balans bijele zakljucan:", g)
        except Exception as e:
            print("UPOZORENJE: balans se ne moze zakljucati:", e)
    if ZAKLJUCAJ_EKSPOZICIJU:
        try:
            gdb, exp = csi0.gain_db(), csi0.exposure_us()
            csi0.auto_gain(False, gain_db=gdb)
            csi0.auto_exposure(False, exposure_us=exp)
            print("Ekspozicija zakljucana: %d us, %.1f dB" % (exp, gdb))
        except Exception as e:
            print("UPOZORENJE: ekspozicija se ne moze zakljucati:", e)


def info_ekspozicije():                      # short exposure text for the CSV
    try:
        return "exp=%dus gain=%.1fdB" % (csi0.exposure_us(), csi0.gain_db())
    except Exception:
        return "exp=?"


def snimi():                                 # wait SMIRIVANJE_MS, then return one image for analysis
    led("LED_GREEN", True)
    csi0.snapshot(time=SMIRIVANJE_MS)        # skip frames (returns None)
    img = csi0.snapshot()
    led("LED_GREEN", False)
    return img


def nadji_mrlje(img, roi=None):              # red, green and magenta blobs -> list of (colour, blob)
    rezultat = []
    for ime, prag in (("R", PRAG_CRVENA), ("G", PRAG_ZELENA), ("M", PRAG_MAGENTA)):
        if roi:
            mrlje = img.find_blobs([prag], roi=roi, pixels_threshold=MIN_PIKSELA, area_threshold=MIN_PIKSELA, merge=True, margin=3)
        else:
            mrlje = img.find_blobs([prag], pixels_threshold=MIN_PIKSELA, area_threshold=MIN_PIKSELA, merge=True, margin=3)
        for b in mrlje:
            rezultat.append((ime, b))
    return rezultat


def mjeri_roi(img, roi):                     # colour inside one ROI: the colour with more pixels wins
    zbroj = {"R": 0, "G": 0, "M": 0}
    najveca = {}
    for ime, b in nadji_mrlje(img, roi):
        zbroj[ime] += bp(b)
        if ime not in najveca or bp(b) > bp(najveca[ime]):
            najveca[ime] = b
    ukupno = zbroj["R"] + zbroj["G"]
    if ukupno == 0:
        return ("-", 0, 0, 0, 0.0, zbroj["M"], None)
    boja = "R" if zbroj["R"] >= zbroj["G"] else "G"
    b = najveca[boja]
    return (boja, zbroj[boja], bw(b), bh(b), zbroj[boja] / ukupno, zbroj["M"], (bx(b), by(b), bw(b), bh(b)))
    # result: 0 colour, 1 pixels, 2 width, 3 height, 4 confidence, 5 magenta pixels, 6 pillar box


def nacrtaj_oznake(img, smjer, i, mrlje, natpis):   # crosshair, ROIs, blobs and caption
    w, h = img.width(), img.height()
    crtaj_liniju(img, w // 2, 0, w // 2, h - 1, SIVA)
    crtaj_liniju(img, 0, h // 2, w - 1, h // 2, SIVA)
    if smjer is not None:
        for (linija, x, y, rw, rh) in ROI[smjer][i]:
            crtaj_okvir(img, x, y, rw, rh, ZUTA)
            crtaj_tekst(img, x, y - 18, "L%d" % linija, ZUTA)
    for ime, b in mrlje:
        crtaj_okvir(img, bx(b), by(b), bw(b), bh(b), BOJE[ime])
        crtaj_tekst(img, bx(b), by(b) + bh(b) + 2, "%s h%d" % (ime, bh(b)), BOJE[ime])
    crtaj_tekst(img, 10, 10, natpis, BIJELA, 3)


def cekaj(poruka, smjer=None, i=0):          # SKEN: wait for SW or a countdown while showing live images
    print(">>> " + poruka)
    led("LED_BLUE", True)
    sekunde = ODBROJAVANJE_S if (ODBROJAVANJE_S > 0 or tipka is None) else 0
    if tipka is None and sekunde == 0:
        sekunde = 8                          # no button and no countdown -> 8 s
    kraj = time.ticks_add(time.ticks_ms(), sekunde * 1000)
    while True:
        img = csi0.snapshot()                # live image keeps the Frame Buffer updating
        ostalo = time.ticks_diff(kraj, time.ticks_ms()) // 1000 + 1
        nacrtaj_oznake(img, smjer, i, [], "%s  %s" % (poruka[:30], ("%ds" % ostalo) if sekunde else "SW"))
        if sekunde and ostalo <= 0:
            break
        if not sekunde and tipka.value() == 0:
            while tipka.value() == 0:        # wait for release
                time.sleep_ms(20)
            break
    led("LED_BLUE", False)


def izmjeri_kut(img, smjer, i, sirina, visina):   # measure every ROI of one angle -> list of (line, result)
    rezultat = []
    for (linija, x, y, w, h) in ROI[smjer][i]:
        roi = odrezi(x, y, w, h, sirina, visina)
        m = mjeri_roi(img, roi)
        rezultat.append((linija, m))
        crtaj_okvir(img, roi[0], roi[1], roi[2], roi[3], ZUTA)
        crtaj_tekst(img, roi[0], roi[1] - 20, "L%d:%s" % (linija, m[0]), BOJE[m[0]])
    return rezultat


def slozi_mapu(smjer, ocitanja):             # combine all readings into a 12-line map
    mapa = ["?"] * BROJ_LINIJA               # '?' = line without any ROI
    odluka = {}                              # line -> (angle, result) that decided the colour
    min_h = None
    for linija, lista in ocitanja.items():
        if len(set(m[0] for (i, m) in lista if m[0] != "-")) > 1:   # both R and G on one line: ROI too large
            print("   UPOZORENJE: %s linija %d vidi R i G: %s - ROI vjerojatno hvata susjednu liniju" % (smjer, linija, ", ".join("kut %d=%s" % (i, m[0]) for (i, m) in lista)))
        kut, najbolje = max(lista, key=lambda p: p[1][1])   # reading with the most colour pixels
        mapa[linija] = najbolje[0]
        odluka[linija] = (kut, najbolje)
        if najbolje[0] != "-" and (min_h is None or najbolje[3] < min_h):
            min_h = najbolje[3]
    return mapa, min_h, odluka


def ispisi_rezultat(smjer, mapa, odluka):    # per-line printout + the line that would go to the ESP32
    print("\n=== REZULTAT %s (numeracija smjera %s, linija 0 = prva koju robot prelazi) ===" % (smjer, smjer))
    for k in range(BROJ_LINIJA):
        if mapa[k] == "?":
            print("linija %2d (%s): ?  nema ROI-ja - robot o njoj ne bi znao nista" % (k, opis_linije(k)))
        elif mapa[k] == "-":
            print("linija %2d (%s): -  prazno" % (k, opis_linije(k)))
        else:
            kut, m = odluka[k]
            print("linija %2d (%s): %s  px=%d sirina=%d visina=%d sigurnost=%.2f (kut %d)" % (k, opis_linije(k), m[0], m[1], m[2], m[3], m[4], kut))
    print("Po sekcijama: %s" % grupiraj(mapa))
    print("ESP32 <- MAPA;%s;%s" % (smjer, "".join(mapa)))


# ============================ 3. MODES =======================================

def nacin_live():                            # LIVE: one view, endless loop
    print("LIVE: %s kut %d = kamera %s. Zaustavljanje: crvena tipka u IDE-u." % (SMJER, KUT, gdje_kamera(SMJER, KUT)))
    while True:
        img = csi0.snapshot()
        mrlje = nadji_mrlje(img) if TRAZI_MRLJE else []
        nacrtaj_oznake(img, SMJER, KUT, mrlje, "LIVE %s kut %d (%s)" % (SMJER, KUT, gdje_kamera(SMJER, KUT)))


def nacin_provjera(sirina, visina):          # PROVJERA: live view per angle, then measure, then print the map
    print("PROVJERA %s: svaki kut %d s. Servo okrenite na kut cim se pojavi njegov natpis." % (SMJER, PROVJERA_S))
    print("Na kraju svakog kuta slijedi MJERENJE - tada ne dirajte servo ni robota.")
    ocitanja = {}                            # line -> list of (angle, result)
    for i in range(len(KUTOVI)):
        print(">>> %s kut %d: kamera %s - %d s" % (SMJER, i, gdje_kamera(SMJER, i), PROVJERA_S))
        kraj = time.ticks_add(time.ticks_ms(), PROVJERA_S * 1000)
        while time.ticks_diff(kraj, time.ticks_ms()) > 0:   # live view while the servo is turned
            img = csi0.snapshot()
            mrlje = nadji_mrlje(img) if TRAZI_MRLJE else []
            ostalo = time.ticks_diff(kraj, time.ticks_ms()) // 1000 + 1
            nacrtaj_oznake(img, SMJER, i, mrlje, "%s kut %d (%s)  %ds" % (SMJER, i, gdje_kamera(SMJER, i), ostalo))
        img = snimi()                        # measurement of this angle
        mjerenja = izmjeri_kut(img, SMJER, i, sirina, visina)
        for (linija, m) in mjerenja:
            ocitanja.setdefault(linija, []).append((i, m))
        crtaj_tekst(img, 10, 10, "MJERENJE %s kut %d" % (SMJER, i), BIJELA, 3)
        osvjezi_ide(img)
        print("    kut %d izmjeren: %s" % (i, " ".join("L%d=%s" % (l, m[0]) for (l, m) in mjerenja) or "nema ROI-ja"))
        time.sleep_ms(PRIKAZ_REZULTATA_MS)
    mapa, min_h, odluka = slozi_mapu(SMJER, ocitanja)
    ispisi_rezultat(SMJER, mapa, odluka)
    print("PROVJERA gotova.")


def nacin_kalibracija(sirina, visina):       # KALIBRACIJA: one view, prints ROI rows to paste into ROI
    print("KALIBRACIJA %s kut %d (kamera %s). Stupovi na mjestima, ne dirajte robota." % (SMJER, KUT, gdje_kamera(SMJER, KUT)))
    img = snimi()
    mrlje = sorted(nadji_mrlje(img), key=lambda p: bx(p[1]))   # left to right
    print("\n# Retci za ROI[\"%s\"][%d] - zamijenite -1 brojem LINIJE (0-11) u %s numeraciji, obrisite retke koji nisu stup:" % (SMJER, KUT, SMJER))
    print("        %d: [                                 # kut %d (%s)" % (KUT, KUT, gdje_kamera(SMJER, KUT)))
    for k, (ime, b) in enumerate(mrlje):
        x, y, w, h = odrezi(bx(b) - ROI_RUB, by(b) - ROI_RUB, bw(b) + 2 * ROI_RUB, bh(b) + 2 * ROI_RUB, sirina, visina)
        opis = "#%d %s px=%d w=%d h=%d" % (k, ime, bp(b), bw(b), bh(b))
        if ime == "M":                       # magenta is not a pillar -> printed as a comment
            print("            # (-1, %d, %d, %d, %d),  %s <- magenta" % (x, y, w, h, opis))
        else:
            print("            (-1, %d, %d, %d, %d),  # %s" % (x, y, w, h, opis))
        crtaj_okvir(img, x, y, w, h, BOJE[ime])
        crtaj_tekst(img, x, y - 20, "#%d" % k, BOJE[ime])
    print("        ],                                   # kraj kuta %d" % KUT)
    crtaj_tekst(img, 10, 10, "KALIBRACIJA %s kut %d" % (SMJER, KUT), BIJELA, 3)
    if SPREMI_SLIKE:
        img.save("kal_%s_kut%d.jpg" % (SMJER, KUT))
    print("\nSlika s oznakama #k ostaje na ekranu. Zaustavite skriptu (crvena tipka) kad prepisete brojeve.")
    while True:                              # keep the annotated image on screen
        osvjezi_ide(img)
        time.sleep_ms(500)


def provjeri_roi():                          # before SKEN: ROIs present and line numbers 0-11
    ok = True
    for smjer in SMJEROVI_SKENA:
        if not any(len(v) for v in ROI[smjer].values()):
            print("GRESKA: ROI['%s'] je prazan." % smjer)
            ok = False
        for i, redci in ROI[smjer].items():
            for r in redci:
                if not 0 <= r[0] < BROJ_LINIJA:
                    print("GRESKA: %s kut %d: neispravan broj linije %d" % (smjer, i, r[0]))
                    ok = False
        pokrivene = set(r[0] for v in ROI[smjer].values() for r in v)
        print("ROI %s: pokriveno %d linija, nedostaju: %s" % (smjer, len(pokrivene), [l for l in range(BROJ_LINIJA) if l not in pokrivene]))
    return ok


def u_ccw(mapa, smjer):                      # convert a map to CCW numbering for comparison
    return mapa if smjer == "CCW" else [mapa[druga_linija(k)] for k in range(BROJ_LINIJA)]


def sken_smjera(n, smjer, sirina, visina):   # one scan in one direction: 3 angles -> map
    ocitanja = {}
    for i in range(len(KUTOVI)):
        cekaj("Sken %d %s kut %d: kamera %s" % (n, smjer, i, gdje_kamera(smjer, i)), smjer, i)
        img = snimi()
        eksp = info_ekspozicije()
        for (linija, m) in izmjeri_kut(img, smjer, i, sirina, visina):
            ocitanja.setdefault(linija, []).append((i, m))
            print("CSV;%d;%s;%d;%s;%d;%d;%s;%d;%d;%.2f;%d;%s" % (n, smjer, linija, opis_linije(linija),
                  linija if smjer == "CCW" else druga_linija(linija), i, m[0], m[1], m[3], m[4], m[5], eksp))
        osvjezi_ide(img)
        time.sleep_ms(PRIKAZ_REZULTATA_MS)
        if SPREMI_SLIKE:
            img.save("sken%02d_%s_kut%d.jpg" % (n, smjer, i))
    mapa, min_h, odluka = slozi_mapu(smjer, ocitanja)
    return mapa, min_h


def grupiraj(niz):                           # 12 chars -> 4 groups of 3 (one per section)
    return " ".join("".join(niz[s * 3:(s + 1) * 3]) for s in range(4))


def nacin_sken(sirina, visina):              # SKEN: BROJ_SKENOVA x all directions, compared in CCW numbering
    if not provjeri_roi():
        return
    print("CSV;sken;smjer;linija;opis;ccw_broj;kut;boja;px;h;sigurnost;magenta_px;ekspozicija")
    povijest = {s: [] for s in SMJEROVI_SKENA}
    slaganja = 0                             # scans where CCW and CW agree
    for n in range(1, BROJ_SKENOVA + 1):
        for smjer in SMJEROVI_SKENA:
            cekaj("Sken %d: postavi robota u parking u %s polozaju" % (n, smjer))
            mapa, min_h = sken_smjera(n, smjer, sirina, visina)
            ccw = u_ccw(mapa, smjer)
            povijest[smjer].append(ccw)
            greske = ("  pogresaka: %d" % sum(1 for k in range(BROJ_LINIJA) if ccw[k] != "?" and ccw[k] != OCEKIVANO[k])) if OCEKIVANO else ""
            print("SKEN %2d %-3s: %s  (u CCW brojevima: %s)  min h=%s%s" % (n, smjer, grupiraj(mapa), grupiraj(ccw), min_h, greske))
        if len(SMJEROVI_SKENA) == 2:
            a, b = povijest["CCW"][-1], povijest["CW"][-1]
            razlike = [k for k in range(BROJ_LINIJA) if a[k] != "?" and b[k] != "?" and a[k] != b[k]]
            print("   CCW i CW: %s" % (("razlike na CCW linijama %s" % razlike) if razlike else "iste"))
            slaganja += 0 if razlike else 1
    print("\n=== SAZETAK (CCW numeracija) ===")
    for k in range(BROJ_LINIJA):
        dijelovi = ["%s %s" % (s, "".join(m[k] for m in povijest[s])) for s in SMJEROVI_SKENA]
        ocek = ("  ocekivano %s" % OCEKIVANO[k]) if OCEKIVANO else ""
        print("linija %2d (%s, CW broj %2d): %s%s" % (k, opis_linije(k), druga_linija(k), " | ".join(dijelovi), ocek))
    if len(SMJEROVI_SKENA) == 2:
        print("CCW i CW iste u %d od %d skenova" % (slaganja, BROJ_SKENOVA))


# ============================ 4. START =======================================

print("=== Pokrenuto: NACIN=%s, SMJER=%s, KUT=%d ===" % (NACIN, SMJER, KUT))
sirina, visina = init_kamera()
zakljucaj()
if NACIN == "LIVE":
    nacin_live()
elif NACIN == "PROVJERA":
    nacin_provjera(sirina, visina)
elif NACIN == "KALIBRACIJA":
    nacin_kalibracija(sirina, visina)
elif NACIN == "SKEN":
    nacin_sken(sirina, visina)
else:
    print("Nepoznat NACIN:", NACIN)
print("Kraj.")
