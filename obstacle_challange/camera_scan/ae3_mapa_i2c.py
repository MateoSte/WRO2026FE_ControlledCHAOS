# =============================================================================
# ae3_mapa_i2c.py  -  verzija 1.0: skener staze NA ROBOTU (kamera = I2C target za ESP32-S3)
# -----------------------------------------------------------------------------
# NAMJENA
#   Isto mjerenje kao ae3_skener_test v6 (ROI-ji, pragovi, zbroj piksela po
#   boji), ali kut tornja više ne okrećete vi: ESP32-S3 okreće servo, javi
#   kameri "sad si na kutu i", kamera izmjeri ROI-je tog kuta i javi "gotovo,
#   može sljedeći". Nakon zadnjeg kuta kamera složi mapu 12 linija i stavi je
#   u registre, odakle je ESP32 pročita.
#
# POKRETANJE
#   Test s IDE-om: OpenMV IDE -> otvori ovu datoteku -> Start. Robot (ESP32)
#     tada radi sken, a vi u Serial Terminalu IDE-a vidite isto što i prije.
#   Natjecanje: Tools -> Save open script to OpenMV Cam (as main.py), pa
#     kamera skriptu pokrene sama kad dobije napajanje.
#   ROI-je i pragove i dalje dobivate testnom skriptom (KALIBRACIJA / PROVJERA)
#   i samo ih prepišete u blok POSTAVKE ispod.
#
# PROTOKOL (I2C adresa 0x12, kanal 5 multipleksera, 64 bajta "memorije")
#   ESP32 piše: [adresa registra, podatak, podatak...]
#   ESP32 čita: [adresa registra] pa ponovljeni start i N bajtova
#
#   Reg    Tko piše  Značenje
#   0      kamera    MAGIC = 0xC5  (skripta radi)
#   1      kamera    VERZIJA protokola = 1
#   2      kamera    STATUS: 0 čeka, 1 radi, 2 kut gotov, 3 mapa gotova, 4 greška
#   3      kamera    ACK = SEQ zadnje dovršene naredbe (kamera ga piše ZADNJEG)
#   4      kamera    broj ROI-ja izmjerenih u zadnjem kutu
#   5      kamera    zastavice mape: bit0 = pravilo trojki je nešto promijenilo,
#                                    bit1 = u mapi ima '?'
#   16     ESP32     SMJER: 'C' = CCW, 'W' = CW
#   17     ESP32     KUT: 0, 1 ili 2 (indeks u KUTOVI)
#   18     ESP32     NAREDBA: 1 = snimi kut, 2 = poništi sken, 3 = složi mapu sada
#   19     ESP32     SEQ: brojač naredbi 1-255 (ESP32 ga piše ZADNJEG, u istoj poruci)
#   32-43  kamera    SIROVA mapa: 12 znakova R / G / - / ?  (kako je kamera vidi)
#   48-59  kamera    MAPA nakon pravila trojki (ovu koristi robot)
#
#   Redoslijed je važan: ESP32 smjer, kut i naredbu šalje u ISTOJ poruci sa
#   SEQ-om na kraju, a kamera ACK piše tek kad su mapa i STATUS već upisani.
#   Zato ESP32 nikad ne pročita pola stare, pola nove mape.
#
# NUMERACIJA LINIJA - ista kao u testnoj skripti v6
#   linija = 3 * sekcija + L, u smjeru vožnje koji je ESP32 poslao.
#   PRAVILO TROJKI: u svakoj trojci (0-2, 3-5, 6-8, 9-11) je ILI jedan stup
#   na srednjoj liniji ("-R-") ILI dva stupa na vanjskim linijama ("R-G").
#   Ako kamera vidi nešto treće (npr. "RG-"), skripta izabere objašnjenje koje
#   traži najmanje "grešaka" kamere, a boju koju nije vidjela označi s '?'.
# =============================================================================

import csi                                   # kamera (senzor slike) na AE3
import time                                  # sleep_ms, ticks_ms
try:                                         # LED lampica nije nužna
    from machine import LED                  # RGB lampica na kameri
except ImportError:                          # firmware bez LED-a...
    LED = None                               # ...radimo bez nje
try:                                         # I2C target (slave) - isti kao u main.py iz Tests_new_components
    from machine import I2CTarget            # memorija koju ESP32 čita i piše
except ImportError:                          # stari firmware...
    I2CTarget = None                         # ...javit ćemo grešku na startu


# ============================ 1. POSTAVKE ====================================
# Prepišite iz testne skripte (ae3_skener_test) nakon kalibracije.

KUTOVI = [35, 80, 125]                       # kutovi tornja od "ravno naprijed" (CCW ulijevo, CW udesno) - samo za ispis

SMIRIVANJE_MS = 800                          # ms čekanja prije snimke (ESP32 već čeka servo, ovo smiruje sliku i ekspoziciju)
ZAKLJUCAJ_BALANS = True                      # True = nakon pokretanja zaključaj balans bijele
ZAKLJUCAJ_EKSPOZICIJU = False                # True = zaključaj i ekspoziciju

PRAG_CRVENA  = (11, 28, 55, 5, -5, 41)       # crveni stup (LAB)
PRAG_ZELENA  = (15, 85, -70, -15, -10, 60)   # zeleni stup
PRAG_MAGENTA = (20, 85,  25,  90, -60,  5)   # magenta zidovi parkinga

MIN_PIKSELA = 15                             # mrlja s manje piksela = šum
BROJ_LINIJA = 12                             # 4 sekcije x 3 poprečne linije
PRIKAZ_DOK_CEKA = True                       # True = dok čeka ESP32, šalje živu sliku u IDE (ne smeta I2C-u)

# --- ROI-ji: redak (linija, x, y, w, h); linija 0-11 u numeraciji TOG smjera ---
# Kopija iz ae3_skener_test_v1_0 (verzija 6), 24. 11. 2026.
ROI = {                                      # rječnik: smjer -> {indeks kuta -> popis ROI-ja}
    "CCW": {                                 # CCW: brojevi linija u CCW numeraciji
        0: [
            (3, 64,335, 135, 90),
            (3, 242, 245, 157,144)
        ],                                   # kraj kuta 0
        1: [                                
            (2,641, 412, 174,257)
            (4, 787,225, 112,49),
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
            (0, 58, 274, 150, 133),
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

# --- I2C ---
I2C_SABIRNICA = 1                            # sabirnica 1 = Qwiic konektor (isto kao main.py)
I2C_ADRESA = 0x12                            # adresa kamere (ADDR_AE3 u includes.ino)
VELICINA_MEM = 64                            # bajtova zajedničke memorije

R_MAGIC, R_VERZIJA, R_STATUS, R_ACK, R_BROJ_ROI, R_ZASTAVICE = 0, 1, 2, 3, 4, 5   # registri koje piše kamera
R_SMJER, R_KUT, R_NAREDBA, R_SEQ = 16, 17, 18, 19                              # registri koje piše ESP32
R_SIROVA, R_MAPA = 32, 48                                                      # početak dviju mapa (12 bajtova)
MAGIC, VERZIJA = 0xC5, 1                                                       # "skripta radi", verzija protokola
ST_CEKA, ST_RADI, ST_KUT_GOTOV, ST_MAPA, ST_GRESKA = 0, 1, 2, 3, 4             # vrijednosti STATUS-a
N_SNIMI, N_PONISTI, N_SLOZI = 1, 2, 3                                          # vrijednosti NAREDBE


# ========================= 2. POMOĆNE FUNKCIJE ===============================
# (iste kao u ae3_skener_test v6 - provjereno na v4.8.1 i v5.0.0)

csi0 = csi.CSI()                             # objekt kamere

ZUTA = (255, 255, 0)                         # boja ROI pravokutnika
SIVA = (128, 128, 128)                       # boja križa
BIJELA = (255, 255, 255)                     # boja natpisa
BOJE = {"R": (255, 0, 0), "G": (0, 255, 0), "M": (255, 0, 255), "-": BIJELA, "?": BIJELA}


def led(ime, ukljuci):                       # upali / ugasi jednu boju LED-a (ako postoji)
    try:
        if ukljuci:
            LED(ime).on()
        else:
            LED(ime).off()
    except Exception:
        pass


def opis_linije(linija):                     # 7 -> "S2 L1"
    return "S%d L%d" % (linija // 3, linija % 3)


def gdje_kamera(smjer, i):                   # "35 st. desno"
    return "%d st. %s" % (KUTOVI[i], "lijevo" if smjer == "CCW" else "desno")


def svojstvo(b, ime):                        # svojstvo mrlje na oba firmwarea (v4: b.x(), v5: b.x)
    v = getattr(b, ime)
    return v() if callable(v) else v


def bx(b): return svojstvo(b, "x")
def by(b): return svojstvo(b, "y")
def bw(b): return svojstvo(b, "w")
def bh(b): return svojstvo(b, "h")
def bp(b): return svojstvo(b, "pixels")


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
    except (TypeError, ValueError):
        img.draw_string(x, max(0, y), tekst, color=boja, scale=velicina)


def odrezi(x, y, w, h, sirina, visina):      # pravokutnik unutar slike
    x = max(0, min(x, sirina - 1))
    y = max(0, min(y, visina - 1))
    w = max(1, min(w, sirina - x))
    h = max(1, min(h, visina - y))
    return x, y, w, h


def osvjezi_ide(img):                        # pošalji nacrtanu sliku u Frame Buffer IDE-a
    try:
        img.flush()
    except Exception:
        pass


def init_kamera():                           # postavi kameru; vraća širinu i visinu
    csi0.reset()
    csi0.pixformat(csi.RGB565)
    csi0.framesize(csi.HD)                   # 1280 x 800 - ROI-ji su izmjereni u toj rezoluciji
    csi0.snapshot(time=2000)                 # 2 s da se ekspozicija smiri
    img = csi0.snapshot()
    print("Rezolucija: %d x %d" % (img.width(), img.height()))
    return img.width(), img.height()


def zakljucaj():                             # zaključaj balans bijele (i po želji ekspoziciju)
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


def snimi():                                 # pričekaj smirivanje i vrati jednu sliku za analizu
    led("LED_GREEN", True)                   # zelena = snimanje
    csi0.snapshot(time=SMIRIVANJE_MS)
    img = csi0.snapshot()
    led("LED_GREEN", False)
    return img


def nadji_mrlje(img, roi=None):              # crvene, zelene i magenta mrlje; popis (boja, mrlja)
    rezultat = []
    for ime, prag in (("R", PRAG_CRVENA), ("G", PRAG_ZELENA), ("M", PRAG_MAGENTA)):
        if roi:
            mrlje = img.find_blobs([prag], roi=roi, pixels_threshold=MIN_PIKSELA, area_threshold=MIN_PIKSELA, merge=True, margin=3)
        else:
            mrlje = img.find_blobs([prag], pixels_threshold=MIN_PIKSELA, area_threshold=MIN_PIKSELA, merge=True, margin=3)
        for b in mrlje:
            rezultat.append((ime, b))
    return rezultat


def mjeri_roi(img, roi):                     # boja u ROI-ju: pobjeđuje boja s više piksela
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
    # indeksi: 0 boja, 1 piksela, 2 širina, 3 visina, 4 sigurnost, 5 magenta, 6 okvir stupa


def nacrtaj_oznake(img, smjer, i, natpis):   # križ, ROI-ji pogleda (smjer, i) i natpis
    w, h = img.width(), img.height()
    crtaj_liniju(img, w // 2, 0, w // 2, h - 1, SIVA)
    crtaj_liniju(img, 0, h // 2, w - 1, h // 2, SIVA)
    if smjer is not None:
        for (linija, x, y, rw, rh) in ROI[smjer][i]:
            crtaj_okvir(img, x, y, rw, rh, ZUTA)
            crtaj_tekst(img, x, y - 18, "L%d" % linija, ZUTA)
    crtaj_tekst(img, 10, 10, natpis, BIJELA, 3)


def izmjeri_kut(img, smjer, i, sirina, visina):   # svi ROI-ji jednog kuta; popis (linija, mjerenje)
    rezultat = []
    for (linija, x, y, w, h) in ROI[smjer][i]:
        roi = odrezi(x, y, w, h, sirina, visina)
        m = mjeri_roi(img, roi)
        rezultat.append((linija, m))
    for (linija, m), (_, x, y, w, h) in zip(rezultat, ROI[smjer][i]):   # crtamo tek nakon mjerenja (crte ne kvare boje)
        roi = odrezi(x, y, w, h, sirina, visina)
        crtaj_okvir(img, roi[0], roi[1], roi[2], roi[3], ZUTA)
        crtaj_tekst(img, roi[0], roi[1] - 20, "L%d:%s" % (linija, m[0]), BOJE[m[0]])
    return rezultat


def slozi_mapu(smjer, ocitanja):             # očitanja svih kutova -> mapa 12 linija (kao u v6)
    mapa = ["?"] * BROJ_LINIJA               # '?' = linija bez ROI-ja
    odluka = {}
    for linija, lista in ocitanja.items():
        if len(set(m[0] for (i, m) in lista if m[0] != "-")) > 1:
            print("   UPOZORENJE: %s linija %d vidi R i G: %s" % (smjer, linija, ", ".join("kut %d=%s" % (i, m[0]) for (i, m) in lista)))
        kut, najbolje = max(lista, key=lambda p: p[1][1])   # očitanje s najviše piksela boje
        mapa[linija] = najbolje[0]
        odluka[linija] = (kut, najbolje)
    return mapa, odluka


# ------------------------- PRAVILO TROJKI ------------------------------------
# U trojci (L0, L1, L2) moguće su samo dvije "slike": SREDINA (stup samo na L1)
# ili VANJSKE (stupovi na L0 i L2). Za svaku sliku zbrojimo koliko se slaže s
# kamerom: viđen stup gdje ga slika traži = +1, prazno gdje ga traži = -1,
# viđen stup gdje ga slika NE traži = -1, prazno gdje ga ne traži = +0.5.
# Linija bez ROI-ja ('?') ne broji. Pobjeđuje slika s više bodova; boju stupa
# koji slika traži, a kamera ga nije vidjela, označimo s '?'.

def dokaz(linija, mapa, ocitanja):           # što je kamera vidjela na liniji: None, ('-', 0) ili (boja, piksela)
    if mapa[linija] == "?":
        return None
    if mapa[linija] == "-":
        return ("-", 0)
    return (mapa[linija], max(m[1] for (i, m) in ocitanja[linija] if m[0] == mapa[linija]))


def bodovi(dokazi, zauzete):                 # koliko se "slika" (zauzete linije) slaže s dokazima
    b = 0.0
    for L in range(3):
        d = dokazi[L]
        if d is None:
            continue
        ima = d[0] != "-"
        jacina = 1.0 + min(d[1], 1000) / 10000.0   # veća mrlja = malo jači dokaz (samo za neriješeno)
        if L in zauzete:
            b += jacina if ima else -1.0
        else:
            b += -jacina if ima else 0.5
    return b


def popravi_trojke(mapa, ocitanja):          # vraća (nova mapa, popis napomena)
    nova = list(mapa)
    napomene = []
    for s in range(BROJ_LINIJA // 3):
        lin = (3 * s, 3 * s + 1, 3 * s + 2)
        dokazi = [dokaz(l, mapa, ocitanja) for l in lin]
        if all(d is None for d in dokazi):   # nijedna linija trojke nema ROI
            for l in lin:
                nova[l] = "?"
            napomene.append("S%d: nema ROI-ja ni za jednu liniju" % s)
            continue
        b_sred = bodovi(dokazi, (1,))
        b_vanj = bodovi(dokazi, (0, 2))
        zauzete = (1,) if b_sred >= b_vanj else (0, 2)
        for L in range(3):
            if L in zauzete:
                d = dokazi[L]
                nova[lin[L]] = d[0] if (d is not None and d[0] != "-") else "?"
            else:
                nova[lin[L]] = "-"
        staro = "".join(mapa[l] for l in lin)
        novo = "".join(nova[l] for l in lin)
        if staro != novo:
            napomene.append("S%d: kamera %s -> pravilo %s (sredina %.1f, vanjske %.1f)" % (s, staro, novo, b_sred, b_vanj))
    return nova, napomene


def grupiraj(niz):                           # 12 znakova -> "-R- G-- ..."
    return " ".join("".join(niz[s * 3:(s + 1) * 3]) for s in range(4))


def ispisi_rezultat(smjer, mapa, odluka, nova, napomene):   # ispis u Serial Terminal IDE-a
    print("\n=== MAPA %s (linija 0 = prva linija startne sekcije u smjeru %s) ===" % (smjer, smjer))
    for k in range(BROJ_LINIJA):
        if mapa[k] == "?":
            print("linija %2d (%s): ?  nema ROI-ja" % (k, opis_linije(k)))
        elif mapa[k] == "-":
            print("linija %2d (%s): -" % (k, opis_linije(k)))
        else:
            kut, m = odluka[k]
            print("linija %2d (%s): %s  px=%d visina=%d sigurnost=%.2f (kut %d)" % (k, opis_linije(k), m[0], m[1], m[3], m[4], kut))
    for n in napomene:
        print("PRAVILO TROJKI  " + n)
    print("SIROVA  %s   (%s)" % ("".join(mapa), grupiraj(mapa)))
    print("MAPA    %s   (%s)" % ("".join(nova), grupiraj(nova)))


# ============================ 3. I2C RAD =====================================

def upisi_mapu(mem, pocetak, mapa):          # 12 znakova u registre
    for k in range(BROJ_LINIJA):
        mem[pocetak + k] = ord(mapa[k])


def slozi_i_objavi(mem, smjer, ocitanja):    # složi mapu, ispiši je i upiši u registre; vraća STATUS
    mapa, odluka = slozi_mapu(smjer, ocitanja)
    nova, napomene = popravi_trojke(mapa, ocitanja)
    ispisi_rezultat(smjer, mapa, odluka, nova, napomene)
    upisi_mapu(mem, R_SIROVA, mapa)
    upisi_mapu(mem, R_MAPA, nova)
    mem[R_ZASTAVICE] = (1 if nova != mapa else 0) | (2 if "?" in nova else 0)
    return ST_MAPA


def obradi_naredbu(mem, seq, ocitanja, sirina, visina):   # jedna naredba ESP32; vraća (smjer, kut) za prikaz
    smjer = {ord("C"): "CCW", ord("W"): "CW"}.get(mem[R_SMJER])
    kut = mem[R_KUT]
    naredba = mem[R_NAREDBA]
    mem[R_STATUS] = ST_RADI                  # ESP32 vidi "radi" (ACK je još stari)
    status = ST_GRESKA
    try:
        if smjer is None:
            raise ValueError("nepoznat smjer %d" % mem[R_SMJER])
        if naredba == N_PONISTI:             # nov sken od početka
            ocitanja.clear()
            print(">>> ESP32: ponisti sken")
            status = ST_CEKA
        elif naredba == N_SNIMI:
            if kut >= len(KUTOVI):
                raise ValueError("nepoznat kut %d" % kut)
            if kut == 0:                     # kut 0 = početak novog skena
                ocitanja.clear()
            print(">>> ESP32: %s kut %d (kamera %s), seq %d" % (smjer, kut, gdje_kamera(smjer, kut), seq))
            img = snimi()
            mjerenja = izmjeri_kut(img, smjer, kut, sirina, visina)
            for (linija, m) in mjerenja:
                ocitanja.setdefault(linija, []).append((kut, m))
            crtaj_tekst(img, 10, 10, "MJERENJE %s kut %d" % (smjer, kut), BIJELA, 3)
            osvjezi_ide(img)
            print("    kut %d: %s" % (kut, " ".join("L%d=%s" % (l, m[0]) for (l, m) in mjerenja) or "nema ROI-ja"))
            mem[R_BROJ_ROI] = len(mjerenja)
            if kut == len(KUTOVI) - 1:       # zadnji kut -> mapa
                status = slozi_i_objavi(mem, smjer, ocitanja)
            else:
                status = ST_KUT_GOTOV
        elif naredba == N_SLOZI:
            status = slozi_i_objavi(mem, smjer, ocitanja)
        else:
            raise ValueError("nepoznata naredba %d" % naredba)
    except Exception as e:                   # nikad ne ostavi ESP32 da čeka
        print("GRESKA u naredbi:", e)
        status = ST_GRESKA
    mem[R_STATUS] = status                   # prvo STATUS...
    mem[R_ACK] = seq                         # ...pa ACK (ESP32 gleda ACK)
    return smjer, (kut if kut < len(KUTOVI) else 0)


def glavna_petlja(sirina, visina):           # čekaj naredbe ESP32 zauvijek
    mem = bytearray(VELICINA_MEM)            # zajednička memorija s ESP32
    mem[R_MAGIC] = MAGIC
    mem[R_VERZIJA] = VERZIJA
    upisi_mapu(mem, R_SIROVA, ["?"] * BROJ_LINIJA)
    upisi_mapu(mem, R_MAPA, ["?"] * BROJ_LINIJA)
    i2c = I2CTarget(I2C_SABIRNICA, addr=I2C_ADRESA, mem=mem)   # od sada ESP32 vidi registre
    print("I2C target 0x%02X spreman, cekam ESP32..." % I2C_ADRESA)
    zadnji_seq = mem[R_SEQ]                  # 0 nakon pokretanja
    ocitanja = {}                            # linija -> popis (kut, mjerenje)
    pregled = (None, 0)                      # koji pogled crtati dok čekamo
    led("LED_BLUE", True)                    # plava = čeka ESP32
    while True:
        seq = mem[R_SEQ]
        if seq != zadnji_seq:                # nova naredba
            zadnji_seq = seq
            led("LED_BLUE", False)
            pregled = obradi_naredbu(mem, seq, ocitanja, sirina, visina)
            led("LED_BLUE", True)
        elif PRIKAZ_DOK_CEKA:
            img = csi0.snapshot()
            nacrtaj_oznake(img, pregled[0], pregled[1], "CEKAM ESP32")
        else:
            time.sleep_ms(5)
    return i2c                               # (nikad) - drži referencu na I2C objekt


# ============================== 4. START =====================================

import sys                                   # samo da znamo radimo li na kameri

if sys.implementation.name == "micropython": # na kameri uvijek; na računalu (testovi funkcija) ne
    print("=== ae3_mapa_i2c v1.0 ===")
    sirina, visina = init_kamera()
    zakljucaj()
    if I2CTarget is None:
        print("GRESKA: firmware nema machine.I2CTarget - nadogradite firmware u OpenMV IDE-u")
        led("LED_RED", True)
        while True:
            time.sleep_ms(1000)
    glavna_petlja(sirina, visina)
