# =============================================================================
# ae3_skener_test.py  -  verzija 4: oba smjera vožnje u jednom testu
#                        (svaki red objašnjen komentarom)
# -----------------------------------------------------------------------------
# NAMJENA
#   Ručni test skeniranja WRO Future Engineers staze kamerom OpenMV AE3, BEZ
#   robota. Kamera stoji na nosaču na istom mjestu kao na robotu: na L-stupu,
#   na SREDINI duljine i širine vozila, leća na 27-28 cm. Kad je vozilo
#   centrirano u parkingu, kamera je u CCW i u CW položaju na ISTOJ točki.
#   Kameru okrećete RUKOM; tipka SW glumi poruku ESP32 "servo je na kutu".
#
# KUTOVI I POGLEDI
#   Robot okreće kameru na KUTOVI = 35, 80, 125 st. prema unutrašnjosti staze:
#   u CCW vožnji ULIJEVO, u CW vožnji UDESNO. Robot je u CW okrenut naopako,
#   pa CW kutovi fizički gledaju u druge smjerove nego CCW kutovi:
#     CCW kut 0, 1, 2 -> 35, 80, 125 st. lijevo od CCW "naprijed"
#     CW  kut 0, 1, 2 -> 145, 100, 55 st. lijevo od CCW "naprijed"
#   Zato postoji 6 različitih slika ("pogleda"). KALIBRACIJA ih snima sve
#   dok nosač stoji u CCW položaju, pa jedna kalibracija daje ROI-je za oba
#   smjera. SKEN zatim stvarno okreće nosač i provjerava daju li CCW i CW
#   ISTU mapu.
#
# NUMERACIJA MJESTA - jedna, "fizička" = CCW shema iz dokumenta
#   mjesto = 6 * sekcija + 2 * linija + strana   (sve brojeno kao u CCW vožnji)
#   sekcija : 0 = startna (s parkingom), 1, 2, 3 = redom u CCW smjeru
#   linija  : 0, 1, 2 = poprečna linija redom u CCW smjeru
#   strana  : 0 = UNUTARNJA, 1 = VANJSKA
#   Isti fizički stup ima isti broj u oba smjera, pa vrijedi jedan OCEKIVANO.
#   Za CW vožnju robot koristi svoj broj (sekcija 4-S mod 4, linija 2-L);
#   skripta ga ispisuje u sažetku kao "CW broj".
#
# NAČINI RADA (varijabla NACIN)
#   "LIVE"        - živa slika: nagib, pragovi boja, provjera ROI-ja
#   "KALIBRACIJA" - 6 pogleda iz CCW položaja; na kraju ispiše gotov ROI = {...}
#   "SKEN"        - pravi test: N skenova, svaki u CCW i CW položaju, usporedba
#
# POKRETANJE
#   OpenMV IDE -> otvori ovu datoteku -> postavi NACIN -> zelena tipka Start
#   (dolje lijevo). Svi rezultati stižu u Serial Terminal (dno prozora IDE-a).
#   Provjereno na: OpenMV v4.8.1, MicroPython v1.26.0, OpenMV-AE3 (verzija 3).
# =============================================================================

import csi                                   # modul za upravljanje kamerom (senzorom slike) na AE3
import time                                  # modul za čekanje (sleep_ms) i mjerenje vremena (ticks_ms)
from machine import Pin                      # Pin = pristup pinovima (ovdje samo tipka SW)
try:                                         # LED lampica nije nužna, pa je učitavamo "ako postoji"
    from machine import LED                  # LED = RGB lampica na kameri
except ImportError:                          # ako je ovaj firmware nema...
    LED = None                               # ...radimo bez lampice
# Servo ova skripta NE koristi: na robotu ga vodi ESP32-S3 (OpenMV 4.8.1 na AE3 ionako nema machine.PWM).


# ============================ 1. POSTAVKE ====================================
# Sve što se mijenja tijekom testa nalazi se u ovom bloku. Ispod njega ne
# morate ništa dirati.

NACIN = "LIVE"                               # odabir načina rada: "LIVE", "KALIBRACIJA" ili "SKEN"

# Kutovi tornja u stupnjevima od smjera "ravno naprijed" robota, prema unutrašnjosti staze
# (u CCW vožnji ulijevo, u CW vožnji udesno). Isti brojevi koje šalje ESP32.
KUTOVI = [35, 80, 125]                       # 3 kuta; uz vidno polje ~64 st. CCW pokriva ~3-157 st., CW ~23-177 st.

# --- LIVE ---
LIVE_SMJER = "CCW"                           # čije ROI-je LIVE crta: "CCW" ili "CW" (pogled = smjer + indeks kuta)
LIVE_KUT_INDEKS = 0                          # indeks kuta (0, 1 ili 2) čije ROI-je LIVE crta
LIVE_TRAZI_MRLJE = True                      # False = LIVE samo prikazuje sliku (brže; za namještanje nagiba), True = i traži mrlje

# --- SKEN ---
SMJEROVI_SKENA = ["CCW", "CW"]               # smjerovi koje svaki sken obuhvaća, redom; ["CCW"] ili ["CW"] = samo jedan
BROJ_SKENOVA = 10                            # koliko puta ponavljamo cijeli sken (svaki put nosač ponovno postavljate)
PRIKAZ_REZULTATA_MS = 2000                   # koliko ms slika s rezultatom kuta ostaje u Frame Bufferu prije žive slike sljedećeg kuta

# --- Ručno okretanje ---
ODBROJAVANJE_S = 0                           # 0 = skripta čeka pritisak tipke SW; npr. 5 = odbrojava 5 s bez tipke

# --- Ekspozicija i boje ---
SMIRIVANJE_MS = 800                          # ms čekanja nakon okretanja: smiri se nosač i prilagodi ekspozicija
ZAKLJUCAJ_BALANS = True                      # True = nakon pokretanja zaključaj balans bijele (boje se ne mijenjaju između kutova)
ZAKLJUCAJ_EKSPOZICIJU = False                # True = zaključaj i ekspoziciju (za usporedbu kad je jedan kut presvijetao)

# Pragovi boja u LAB prostoru, redom: (L_min, L_max, A_min, A_max, B_min, B_max)
#   L = svjetlina (0-100), A = zeleno(-) / crveno(+), B = plavo(-) / žuto(+)
# Ovo su samo POČETNE vrijednosti - u fazi 2 ih podesite Threshold Editorom.
PRAG_CRVENA  = (20, 75,  25,  90,  10, 70)   # crveni stup: visok A (crveno), pozitivan B (malo prema žutoj)
PRAG_ZELENA  = (15, 85, -70, -15, -10, 60)   # zeleni stup: negativan A (zeleno)
PRAG_MAGENTA = (20, 85,  25,  90, -60,  5)   # magenta zidovi parkinga: visok A, ali negativan B (prema plavoj)

MIN_PIKSELA = 15                             # mrlja s manje piksela od ovoga smatra se šumom i ignorira se
ROI_RUB = 12                                 # koliko piksela KALIBRACIJA dodaje sa svake strane pronađene mrlje

SPREMI_SLIKE = False                         # True = sprema JPEG svake snimke s oznakama na disk kamere (za kasniju analizu)
# PRAVILO za SPREMI_SLIKE: dok skripta radi, NA RAČUNALU ništa ne otvarajte, ne mijenjajte i ne spremajte na disk kamere.
# Nove slike na računalu vidite tek nakon: Stop u IDE-u -> izbaci (eject) disk kamere -> iskopčaj i ponovo ukopčaj USB.

BROJ_MJESTA = 24                             # 4 sekcije x 3 poprečne linije x 2 strane

# --- ROI-ji (KALIBRACIJA ispiše cijeli ovaj blok; vi samo upišete brojeve mjesta) ---
# Ključ "CCW"/"CW" = smjer vožnje, ključ 0/1/2 = indeks kuta u tom smjeru.
# Redak (mjesto, x, y, w, h):
#   mjesto = FIZIČKI broj mjesta 0-23 (CCW shema), isti u oba smjera
#   x, y   = gornji lijevi kut pravokutnika u pikselima slike
#   w, h   = širina i visina pravokutnika u pikselima
# Ako se mjesto vidi na dva kuta istog smjera, smijete ga upisati na oba:
# SKEN tada uzme očitanje s više piksela i upozori ako se boje ne slažu.
ROI = {                                      # rječnik: smjer -> {indeks kuta -> popis ROI-ja}
    "CCW": {0: [], 1: [], 2: []},            # ROI-ji za tri CCW kuta; prazno dok ne kalibrirate
    "CW":  {0: [], 1: [], 2: []},            # ROI-ji za tri CW kuta; prazno dok ne kalibrirate
}                                            # kraj rječnika ROI

# Stvarni raspored stupova za provjeru točnosti: 24 znaka 'R', 'G' ili '-' u FIZIČKOJ (CCW) numeraciji.
# Isti niz vrijedi za CCW i za CW položaj, jer se stupovi između njih ne miču.
# Razmaci su dopušteni radi preglednosti - skripta ih briše. None = ne provjeravaj točnost.
OCEKIVANO = None                             # npr. "--R--- G----- -R--G- ----R-"


# ========================= 2. KAMERA I TIPKA =================================

csi0 = csi.CSI()                             # objekt koji predstavlja kameru (senzor PAG7936 na AE3)
try:                                         # pokušaj otvoriti tipku (u nekim firmwareima pin se drukčije zove)
    tipka = Pin("SW", Pin.IN)                # korisnička tipka na kameri kao ulaz; pušteno = 1, pritisnuto = 0
except Exception as e:                       # ako pin "SW" ne postoji...
    tipka = None                             # ...radimo bez tipke
    print("UPOZORENJE: tipka SW nije dostupna (%s) - koristim odbrojavanje 5 s" % e)   # ...i javimo zašto

BOJE_CRTANJA = {                             # boje (R, G, B) kojima crtamo oznake na slici
    "R": (255, 0, 0),                        # crveni stup -> crveni okvir
    "G": (0, 255, 0),                        # zeleni stup -> zeleni okvir
    "M": (255, 0, 255),                      # magenta -> ljubičasti okvir
    "-": (255, 255, 255),                    # ništa pronađeno -> bijeli natpis
}                                            # kraj rječnika boja za crtanje
ZUTA = (255, 255, 0)                         # žuta boja za crtanje ROI pravokutnika
SIVA = (128, 128, 128)                       # siva boja za križ u sredini slike

if OCEKIVANO:                                # ako je upisan očekivani raspored...
    OCEKIVANO = OCEKIVANO.replace(" ", "")   # ...obriši razmake koji služe samo za preglednost
    if len(OCEKIVANO) != BROJ_MJESTA:        # provjeri da ima točno 24 znaka
        raise ValueError("OCEKIVANO mora imati 24 znaka bez razmaka")   # zaustavi skriptu s jasnom porukom


def led(ime, ukljuci):                       # pomoćna funkcija: upali ili ugasi jednu boju LED lampice
    try:                                     # pokušaj (ako LED ne postoji, skripta ne smije pasti)
        if ukljuci:                          # ako treba upaliti...
            LED(ime).on()                    # ...upali LED zadane boje ("LED_BLUE", "LED_GREEN"...)
        else:                                # inače...
            LED(ime).off()                   # ...ugasi je
    except Exception:                        # ako LED ne postoji ili javi grešku...
        pass                                 # ...ignoriraj, LED je samo pomoć operateru


def opis_mjesta(mjesto):                     # pretvara fizički broj mjesta u opis po CCW shemi, npr. "S2 L1 V"
    sekcija = mjesto // 6                    # cjelobrojno dijeljenje sa 6 daje sekciju (0-3)
    linija = (mjesto % 6) // 2               # ostatak pri dijeljenju sa 6, podijeljen s 2, daje liniju (0-2)
    strana = "U" if mjesto % 2 == 0 else "V" # paran broj = Unutarnja strana, neparan = Vanjska strana
    return "S%d L%d %s" % (sekcija, linija, strana)   # vrati tekst npr. "S2 L1 V"


def cw_broj(mjesto):                         # fizički (CCW) broj -> broj istog mjesta u numeraciji CW vožnje
    sekcija = mjesto // 6                    # CCW sekcija (0-3)
    linija = (mjesto % 6) // 2               # CCW linija (0-2)
    strana = mjesto % 2                      # strana (0 = unutarnja, 1 = vanjska) - ista u oba smjera
    return 6 * ((4 - sekcija) % 4) + 2 * (2 - linija) + strana   # CW obilazi sekcije i linije obrnutim redom


def fizicki_smjer(smjer, i):                 # kut pogleda mjeren LIJEVO od CCW "naprijed" (zajednički sustav za oba smjera)
    a = KUTOVI[i]                            # kut tornja u sustavu robota
    return a if smjer == "CCW" else 180 - a  # CCW gleda a st. lijevo; CW (robot okrenut naopako) gleda a st. desno = 180-a lijevo


def uputa_kuta(smjer, i, polozaj):           # tekst kamo okrenuti kameru za pogled (smjer, i) dok nosač stoji u 'polozaj'
    p = fizicki_smjer(smjer, i)              # fizički smjer pogleda
    if polozaj == "CCW":                     # nosač u CCW položaju: kut mjerimo od njegovog "naprijed" ulijevo
        return "%d st. lijevo" % p           # npr. "145 st. lijevo" za CW kut 0
    return "%d st. desno" % (180 - p)        # nosač u CW položaju: isti fizički smjer je 180-p st. udesno


def ime_pogleda(smjer, i):                   # kratko ime pogleda za ispis, npr. "CW kut 0 (35 st.)"
    return "%s kut %d (%d st.)" % (smjer, i, KUTOVI[i])   # smjer vožnje, indeks i kut tornja u sustavu robota


def init_kamera():                           # postavlja kameru i vraća širinu i visinu slike
    csi0.reset()                             # vrati kameru u početno stanje
    csi0.pixformat(csi.RGB565)               # slika u boji, 16 bita po pikselu (potrebno za prepoznavanje boja)
    csi0.framesize(csi.HD)                   # najveća rezolucija; na AE3 očekujemo 1280 x 800
    csi0.snapshot(time=2000)                 # snimaj i odbacuj slike 2 s da se ekspozicija i balans smire (s time= vraća None, ne sliku!)
    img = csi0.snapshot()                    # tek sada snimi jednu pravu sliku
    print("Rezolucija: %d x %d" % (img.width(), img.height()))   # ispiši stvarnu rezoluciju (zapišite je!)
    return img.width(), img.height()         # vrati dimenzije slike ostatku skripte


def zakljucaj():                             # zaključava automatske postavke kamere prema postavkama gore
    if ZAKLJUCAJ_BALANS:                     # ako treba zaključati balans bijele...
        try:                                 # pokušaj (neki firmware ne podržava)
            g = csi0.rgb_gain_db()           # pročitaj trenutna pojačanja crvenog, zelenog i plavog kanala
            csi0.auto_whitebal(False, rgb_gain_db=g)   # isključi automatski balans i zadrži ta pojačanja
            print("Balans bijele zakljucan:", g)       # javi da je uspjelo i koje su vrijednosti
        except Exception as e:               # ako ne uspije...
            print("UPOZORENJE: balans bijele se ne moze zakljucati:", e)   # ...javi razlog, ali nastavi
    if ZAKLJUCAJ_EKSPOZICIJU:                # ako treba zaključati i ekspoziciju...
        try:                                 # pokušaj
            gdb = csi0.gain_db()             # pročitaj trenutno pojačanje senzora (dB)
            exp = csi0.exposure_us()         # pročitaj trenutno vrijeme ekspozicije (mikrosekunde)
            csi0.auto_gain(False, gain_db=gdb)          # isključi automatsko pojačanje, zadrži trenutno
            csi0.auto_exposure(False, exposure_us=exp)  # isključi automatsku ekspoziciju, zadrži trenutnu
            print("Ekspozicija zakljucana: %d us, pojacanje %.1f dB" % (exp, gdb))   # javi vrijednosti
        except Exception as e:               # ako ne uspije...
            print("UPOZORENJE: ekspozicija se ne moze zakljucati:", e)   # ...javi razlog, ali nastavi


def info_ekspozicije():                      # vraća kratak tekst s trenutnom ekspozicijom (za CSV)
    try:                                     # pokušaj pročitati vrijednosti
        return "exp=%dus gain=%.1fdB" % (csi0.exposure_us(), csi0.gain_db())   # npr. "exp=8200us gain=6.0dB"
    except Exception:                        # ako firmware to ne podržava...
        return "exp=?"                       # ...vrati upitnik umjesto da skripta padne


def pokazi_zivo(pogled):                     # snimi jednu sliku s pomoćnim oznakama, da Frame Buffer ne stoji dok čekamo
    img = csi0.snapshot()                    # snimi sliku; IDE prikazuje sliku pri svakom snimanju
    w = img.width()                          # širina slike u pikselima
    h = img.height()                         # visina slike u pikselima
    img.draw_line(w // 2, 0, w // 2, h - 1, color=SIVA)   # okomita crta kroz sredinu (pomaže pri namještanju kuta)
    img.draw_line(0, h // 2, w - 1, h // 2, color=SIVA)   # vodoravna crta kroz sredinu
    if pogled is not None:                   # ako znamo na koji pogled okrećete kameru...
        smjer, i = pogled                    # ...rastavi pogled na smjer vožnje i indeks kuta
        for (mjesto, x, y, rw, rh) in ROI[smjer][i]:   # ...nacrtaj već kalibrirane ROI-je tog pogleda
            img.draw_rectangle(x, y, rw, rh, color=ZUTA, thickness=2)   # žuti okvir: stup treba biti u njemu
            img.draw_string(x, y - 18, str(mjesto), color=ZUTA, scale=2)   # broj mjesta iznad okvira


def cekaj_operatera(poruka, pogled=None, zivo=True):   # ispiše uputu i čeka potvrdu (tipka SW ili odbrojavanje)
    # zivo=True: dok čeka, kamera stalno snima pa vidite kamo gleda; zivo=False: zadnja slika ostaje (npr. rezultat kalibracije)
    print(">>> " + poruka)                   # ispiši uputu u Serial Terminal
    led("LED_BLUE", True)                    # plava LED = kamera čeka vas
    sekunde = ODBROJAVANJE_S                 # koliko sekundi odbrojavati (0 = čekaj tipku)
    if tipka is None and sekunde == 0:       # ako tipka ne radi, a odbrojavanje nije zadano...
        sekunde = 5                          # ...automatski odbrojavaj 5 s
    if sekunde > 0:                          # način s odbrojavanjem
        print("    odbrojavanje %d s..." % sekunde)   # javi koliko imate vremena
        kraj = time.ticks_add(time.ticks_ms(), sekunde * 1000)   # trenutak u kojem odbrojavanje završava
        while time.ticks_diff(kraj, time.ticks_ms()) > 0:   # dok taj trenutak nije došao...
            if zivo:                         # ...ako treba pokazivati živu sliku...
                pokazi_zivo(pogled)          # ...snimi i pokaži jednu sliku
            else:                            # ...inače...
                time.sleep_ms(50)            # ...samo čekaj 50 ms
    else:                                    # način s tipkom
        print("    (pritisni i pusti tipku SW)")   # podsjetnik u terminalu
        while tipka.value() == 1:            # dok tipka NIJE pritisnuta (1 = puštena)...
            if zivo:                         # ...ako treba pokazivati živu sliku...
                pokazi_zivo(pogled)          # ...snimi i pokaži jednu sliku (Frame Buffer se osvježava)
            else:                            # ...inače...
                time.sleep_ms(20)            # ...čekaj 20 ms i provjeri tipku ponovo
        while tipka.value() == 0:            # dok je tipka pritisnuta (0 = pritisnuta)...
            time.sleep_ms(20)                # ...čekaj; snimka kreće tek kad maknete prst
    led("LED_BLUE", False)                   # ugasi plavu LED: čekanje je gotovo


def postavi_kut(smjer, i, polozaj):          # traži od operatera da kameru okrene na pogled (smjer, i) dok nosač stoji u 'polozaj'
    cekaj_operatera("%s: okreni kameru na %s" % (ime_pogleda(smjer, i), uputa_kuta(smjer, i, polozaj)),   # uputa, npr. "CW kut 0 (35 st.): okreni kameru na 145 st. lijevo"
                    pogled=(smjer, i))       # dok čeka: živa slika s ROI-jima tog pogleda


def postavi_nosac(polozaj, sken):            # traži od operatera da nosač postavi u parking u zadanom položaju
    cekaj_operatera("Sken %d: postavi nosac u parking u %s polozaju (prednji kraj u smjeru %s voznje), uz postavljac, pa makni postavljac"   # uputa
                    % (sken, polozaj, polozaj), pogled=None)   # živa slika bez ROI-ja dok premještate nosač


def odrezi(x, y, w, h, sirina, visina):      # drži pravokutnik unutar slike (find_blobs ne prihvaća ROI izvan slike)
    x = max(0, min(x, sirina - 1))           # x ne smije biti manji od 0 ni veći od zadnjeg stupca
    y = max(0, min(y, visina - 1))           # y ne smije biti manji od 0 ni veći od zadnjeg retka
    w = max(1, min(w, sirina - x))           # širina barem 1 i ne smije prijeći desni rub slike
    h = max(1, min(h, visina - y))           # visina barem 1 i ne smije prijeći donji rub slike
    return x, y, w, h                        # vrati ispravljeni pravokutnik


def osvjezi_ide(img):                        # šalje nacrtanu sliku u Frame Buffer OpenMV IDE-a
    try:                                     # pokušaj (starije verzije firmwarea nemaju flush)
        img.flush()                          # pošalji sliku s oznakama u prozor Frame Buffer
    except Exception:                        # ako ne postoji...
        pass                                 # ...ignoriraj; tada koristite SPREMI_SLIKE = True


def snimi():                                 # čeka da se nosač smiri i vraća jednu sliku za analizu
    led("LED_GREEN", True)                   # zelena LED = snimanje u tijeku, ne dirajte nosač
    csi0.snapshot(time=SMIRIVANJE_MS)        # pričekaj SMIRIVANJE_MS ms odbacujući slike (vraća None)
    img = csi0.snapshot()                    # zatim snimi sliku koju analiziramo
    led("LED_GREEN", False)                  # ugasi zelenu LED
    return img                               # vrati sliku


# ============================ 3. MJERENJE ====================================

def nadji_mrlje(img, roi=None):              # traži crvene, zelene i magenta mrlje; vraća popis (boja, mrlja)
    rezultat = []                            # ovdje skupljamo sve pronađene mrlje
    for ime, prag in (("R", PRAG_CRVENA), ("G", PRAG_ZELENA), ("M", PRAG_MAGENTA)):   # svaku boju tražimo ZASEBNO
        if roi:                              # ako je zadan pravokutnik...
            mrlje = img.find_blobs([prag], roi=roi, pixels_threshold=MIN_PIKSELA,   # ...traži mrlje ove boje samo unutar ROI-ja, najmanje MIN_PIKSELA piksela
                                   area_threshold=MIN_PIKSELA, merge=True, margin=3)   # okvir mrlje barem MIN_PIKSELA, spajanje bliskih dijelova (do 3 px)
        else:                                # inače...
            mrlje = img.find_blobs([prag], pixels_threshold=MIN_PIKSELA,   # ...traži mrlje ove boje po cijeloj slici, najmanje MIN_PIKSELA piksela
                                   area_threshold=MIN_PIKSELA, merge=True, margin=3)   # isti uvjeti kao gore
        # merge=True spaja dijelove ISTE boje udaljene do 3 px (margin) u jednu mrlju (npr. stup presječen sjenom)
        for b in mrlje:                      # za svaku pronađenu mrlju ove boje...
            rezultat.append((ime, b))        # ...zapamti oznaku boje i samu mrlju
    return rezultat                          # vrati sve mrlje svih boja


def mjeri_roi(img, roi):                     # određuje boju jednog mjesta unutar njegovog ROI-ja
    zbroj = {"R": 0, "G": 0, "M": 0}         # ukupan broj piksela po boji unutar ROI-ja
    najveca = {}                             # najveća mrlja po boji (za visinu i širinu stupa)
    for ime, b in nadji_mrlje(img, roi):     # prođi kroz sve mrlje pronađene u ROI-ju
        zbroj[ime] += b.pixels()             # dodaj broj piksela mrlje zbroju njezine boje
        if ime not in najveca or b.pixels() > najveca[ime].pixels():   # ako je ovo najveća mrlja te boje do sada...
            najveca[ime] = b                 # ...zapamti je
    ukupno = zbroj["R"] + zbroj["G"]         # ukupno crvenih i zelenih piksela (magenta ne ulazi u odluku)
    if ukupno == 0:                          # ako nema ni crvene ni zelene...
        return ("-", 0, 0, 0, 0.0, zbroj["M"], None)   # ...mjesto je prazno
    boja = "R" if zbroj["R"] >= zbroj["G"] else "G"    # pobjeđuje boja s više piksela
    b = najveca[boja]                        # najveća mrlja pobjedničke boje = sam stup
    sigurnost = zbroj[boja] / ukupno         # udio pobjedničke boje: 1.00 = čisto, 0.60 = ROI hvata i drugu boju
    return (boja, zbroj[boja], b.w(), b.h(), sigurnost, zbroj["M"], b.rect())   # vrati rezultat mjerenja (indeksi 0-6, vidi sljedeći red)
    # vraća redom: boja, broj piksela, širina mrlje, VISINA mrlje, sigurnost, piksela magente, okvir mrlje


# ============================= 4. NAČINI RADA ================================

def nacin_live(sirina, visina):              # LIVE: neprekidna slika s oznakama, za nagib, pragove i provjeru ROI-ja
    print("LIVE radi. Trazenje mrlja: %s. Zaustavljanje: crvena tipka u IDE-u." % LIVE_TRAZI_MRLJE)   # potvrda da je petlja krenula
    print("ROI-ji za %s. Kamera: nosac u CCW polozaju -> %s, nosac u CW polozaju -> %s"   # kamo okrenuti kameru za ovaj pogled...
          % (ime_pogleda(LIVE_SMJER, LIVE_KUT_INDEKS), uputa_kuta(LIVE_SMJER, LIVE_KUT_INDEKS, "CCW"),   # ...ako nosač stoji u CCW položaju
             uputa_kuta(LIVE_SMJER, LIVE_KUT_INDEKS, "CW")))   # ...ili ako stoji u CW položaju
    t0 = time.ticks_ms()                     # vrijeme početka mjerenja FPS-a (milisekunde)
    n = 0                                    # brojač snimljenih slika
    while True:                              # ponavljaj zauvijek (zaustavljate crvenom tipkom u IDE-u)
        img = csi0.snapshot()                # snimi jednu sliku
        mrlje = nadji_mrlje(img) if LIVE_TRAZI_MRLJE else []   # traži mrlje samo ako je uključeno (bez toga slika teče brže)
        img.draw_line(sirina // 2, 0, sirina // 2, visina - 1, color=SIVA)   # okomita crta kroz sredinu slike
        img.draw_line(0, visina // 2, sirina - 1, visina // 2, color=SIVA)   # vodoravna crta kroz sredinu slike
        for (mjesto, x, y, w, h) in ROI[LIVE_SMJER][LIVE_KUT_INDEKS]:   # za svaki kalibrirani ROI ovog pogleda...
            img.draw_rectangle(x, y, w, h, color=ZUTA, thickness=2)   # ...nacrtaj žuti pravokutnik
            img.draw_string(x, y - 18, str(mjesto), color=ZUTA, scale=2)   # ...i iznad njega broj mjesta
        for ime, b in mrlje:                 # za svaku pronađenu mrlju...
            img.draw_rectangle(b.x(), b.y(), b.w(), b.h(), color=BOJE_CRTANJA[ime], thickness=2)   # ...okvir u boji mrlje
            img.draw_string(b.x(), b.y() + b.h() + 2, "%s h%d p%d" % (ime, b.h(), b.pixels()),   # ...tekst ispod okvira mrlje
                            color=BOJE_CRTANJA[ime], scale=2)   # ...ispod okvira: boja, visina (h) i broj piksela (p)
        n += 1                               # povećaj brojač slika
        if n % 10 == 0:                      # svakih 10 slika...
            dt = max(1, time.ticks_diff(time.ticks_ms(), t0))   # ...koliko je ms trajalo zadnjih 10 slika
            print("FPS %.1f  %s  mrlja: %d" % (10000 / dt, info_ekspozicije(), len(mrlje)))   # ...ispiši FPS, ekspoziciju i broj mrlja
            t0 = time.ticks_ms()             # ...i kreni mjeriti iznova


def nacin_kalibracija(sirina, visina):       # KALIBRACIJA: 6 pogleda iz CCW položaja, na kraju gotov ROI blok
    pogledi = [(s, i) for s in ("CCW", "CW") for i in range(len(KUTOVI))]   # svih 6 pogleda: 3 CCW i 3 CW kuta
    pogledi = sorted(pogledi, key=lambda p: fizicki_smjer(p[0], p[1]))   # redom po fizičkom smjeru (35, 55, 80, 100, 125, 145): najmanje okretanja
    print("\n=== KALIBRACIJA: 6 pogleda, nosac cijelo vrijeme u CCW polozaju ===")   # naslov u terminalu
    print("Stupovi moraju stajati na svim mjestima koja kalibrirate. Brojevi mjesta = CCW shema.\n")   # podsjetnik
    blokovi = {"CCW": {}, "CW": {}}          # ovdje skupljamo ispisane retke za svaki pogled (za završni ROI blok)
    for (smjer, i) in pogledi:               # za svaki pogled redom...
        postavi_kut(smjer, i, "CCW")         # ...traži ručno okretanje (nosač stoji u CCW položaju) i čekaj tipku SW
        img = snimi()                        # ...pričekaj smirivanje i snimi sliku
        mrlje = nadji_mrlje(img)             # pronađi sve mrlje na slici
        mrlje = sorted(mrlje, key=lambda par: par[1].x())   # poredaj mrlje slijeva nadesno (po x koordinati)
        print("\n# --- %s = %s iz CCW polozaja, %s ---" % (ime_pogleda(smjer, i), uputa_kuta(smjer, i, "CCW"), info_ekspozicije()))   # zaglavlje
        redci = []                           # retci ovog pogleda
        for k, (ime, b) in enumerate(mrlje): # za svaku mrlju, uz redni broj k (0, 1, 2...)...
            x, y, w, h = odrezi(b.x() - ROI_RUB, b.y() - ROI_RUB,   # ...lijevi gornji kut ROI-ja = kut mrlje pomaknut za ROI_RUB gore-lijevo
                                b.w() + 2 * ROI_RUB, b.h() + 2 * ROI_RUB, sirina, visina)   # ...ROI = mrlja + rub sa svih strana
            komentar = "#%d %s px=%d w=%d h=%d" % (k, ime, b.pixels(), b.w(), b.h())   # opis mrlje za lakše prepoznavanje
            if ime == "M":                   # magenta nije stup...
                redak = "            # (-1, %d, %d, %d, %d),  %s  <- magenta, nije stup" % (x, y, w, h, komentar)   # ...redak kao komentar
            else:                            # crvena ili zelena mrlja = kandidat za stup
                redak = "            (-1, %d, %d, %d, %d),  # %s" % (x, y, w, h, komentar)   # redak spreman za ROI
            redci.append(redak)              # zapamti redak za završni blok
            print(redak)                     # i odmah ga ispiši, da ga vidite uz sliku
            img.draw_rectangle(x, y, w, h, color=BOJE_CRTANJA[ime], thickness=2)   # nacrtaj predloženi ROI na slici
            img.draw_string(x, y - 20, "#%d" % k, color=BOJE_CRTANJA[ime], scale=2)   # i njegov redni broj #k
        blokovi[smjer][i] = redci            # spremi retke ovog pogleda pod njegov smjer i kut
        osvjezi_ide(img)                     # pokaži sliku s brojevima u Frame Bufferu
        if SPREMI_SLIKE:                     # ako treba spremati slike...
            img.save("kal_%s_kut%d.jpg" % (smjer, i))   # ...spremi je na disk kamere, npr. "kal_CW_kut0.jpg"
        cekaj_operatera("Zapisi koje je mjesto svaki #k (CCW shema), pa nastavi", zivo=False)   # čekaj; slika s brojevima #k ostaje
    print("\n# ===== KALIBRACIJA gotova. Kopirajte cijeli blok ispod u skriptu umjesto starog ROI = {...} =====")   # uputa
    print("# U svakom retku zamijenite -1 fizickim brojem mjesta (0-23, CCW shema); retke koji nisu stup obrisite.")   # uputa
    print("ROI = {")                         # početak bloka
    for smjer in ("CCW", "CW"):              # za oba smjera vožnje...
        print('    "%s": {' % smjer)         # ...otvori rječnik tog smjera
        for i in range(len(KUTOVI)):         # ...za svaki kut...
            print("        %d: [   # %s; nosac CCW: %s" % (i, ime_pogleda(smjer, i), uputa_kuta(smjer, i, "CCW")))   # ...otvori popis s opisom pogleda
            for redak in blokovi[smjer].get(i, []):   # ...ispiši sve retke tog pogleda
                print(redak)                 # ...jedan po jedan
            print("        ],")              # ...zatvori popis
        print("    },")                      # ...zatvori rječnik smjera
    print("}")                               # kraj bloka


def provjeri_roi():                          # provjerava ROI prije skena; vraća False ako ne valja
    ok = True                                # pretpostavka: sve je u redu
    for smjer in SMJEROVI_SKENA:             # za svaki smjer koji ćemo skenirati...
        if not any(len(v) for v in ROI[smjer].values()):   # ...ako za njega nema nijednog ROI-ja...
            print("GRESKA: ROI['%s'] je prazan. Prvo napravite KALIBRACIJA." % smjer)   # ...javi grešku
            ok = False                       # ...i zapamti da ne valja
        for i, redci in ROI[smjer].items():  # za svaki kut tog smjera...
            brojevi = [r[0] for r in redci]  # ...brojevi mjesta upisani na tom kutu
            for m in brojevi:                # ...za svaki broj...
                if brojevi.count(m) > 1:     # ...ako se na ISTOM kutu pojavljuje dvaput...
                    print("GRESKA: %s: mjesto %d upisano dvaput na istom kutu" % (ime_pogleda(smjer, i), m))   # ...to je sigurno greška
                    ok = False               # ...zapamti da ne valja
                    break                    # ...dovoljno je javiti jednom po kutu
        pokrivena = set(r[0] for v in ROI[smjer].values() for r in v)   # sva mjesta pokrivena u tom smjeru
        print("ROI %s: pokriveno %d mjesta" % (smjer, len([m for m in pokrivena if 0 <= m < BROJ_MJESTA])))   # potvrda
    return ok                                # vrati rezultat provjere


def jedan_sken_smjera(broj, smjer, sirina, visina):   # jedan sken u jednom položaju: 3 kuta, svi ROI-ji, spajanje po mjestu
    ocitanja = {}                            # mjesto -> popis (kut, mjerenje) sa svih kutova koji ga vide
    for i in range(len(KUTOVI)):             # za svaki kut tornja...
        postavi_kut(smjer, i, smjer)         # ...traži ručno okretanje (nosač stoji u položaju 'smjer') i čekaj tipku SW
        img = snimi()                        # ...pričekaj smirivanje i snimi sliku
        eksp = info_ekspozicije()            # zapamti ekspoziciju ove snimke
        mjerenja = []                        # mjerenja ove snimke (za crtanje nakon mjerenja)
        for (mjesto, x, y, w, h) in ROI[smjer][i]:   # za svaki ROI kalibriran na ovom kutu...
            if mjesto < 0 or mjesto >= BROJ_MJESTA:   # ako je ostao -1 ili broj izvan 0-23...
                print("UPOZORENJE: ROI s mjestom %d preskocen" % mjesto)   # ...upozori
                continue                     # ...i preskoči taj ROI
            roi = odrezi(x, y, w, h, sirina, visina)   # osiguraj da je ROI unutar slike
            m = mjeri_roi(img, roi)          # izmjeri boju u tom ROI-ju
            mjerenja.append((mjesto, roi, m))   # zapamti za crtanje
            ocitanja.setdefault(mjesto, []).append((i, m))   # dodaj očitanje tom mjestu
            print("CSV;%d;%s;%d;%s;%d;%d;%s;%d;%d;%d;%.2f;%d;%s" % (   # ispiši jedan CSV redak (format stupaca)...
                broj, smjer, mjesto, opis_mjesta(mjesto), cw_broj(mjesto), i, m[0], m[1], m[2], m[3], m[4], m[5], eksp))   # ...vrijednosti
            # CSV: sken; smjer; mjesto; opis; CW broj; kut; boja; piksela; širina; visina; sigurnost; magenta; ekspozicija
        for (mjesto, roi, m) in mjerenja:    # crtamo TEK NAKON mjerenja, da nacrtane crte ne pokvare boje
            img.draw_rectangle(roi[0], roi[1], roi[2], roi[3], color=ZUTA, thickness=2)   # žuti okvir ROI-ja (x, y, širina, visina)
            img.draw_string(roi[0], roi[1] - 20, "%d:%s" % (mjesto, m[0]),   # tekst 20 px iznad okvira ROI-ja
                            color=BOJE_CRTANJA[m[0]], scale=2)   # iznad okvira: "broj mjesta:boja"
            if m[6]:                         # ako je pronađen stup...
                o = m[6]                     # ...okvir stupa (x, y, širina, visina)
                img.draw_rectangle(o[0], o[1], o[2], o[3], color=BOJE_CRTANJA[m[0]], thickness=1)   # ...tanki okvir oko samog stupa
        osvjezi_ide(img)                     # pokaži rezultat u Frame Bufferu (žuti ROI, "mjesto:boja", okvir stupa)
        time.sleep_ms(PRIKAZ_REZULTATA_MS)   # zadrži rezultat na ekranu, inače bi ga odmah zamijenila živa slika sljedećeg kuta
        if SPREMI_SLIKE:                     # ako treba spremati slike...
            img.save("sken%02d_%s_kut%d.jpg" % (broj, smjer, i))   # ...spremi npr. "sken03_CW_kut2.jpg"
    rezultat = ["?"] * BROJ_MJESTA           # '?' = mjesto nije pokriveno nijednim ROI-jem u ovom smjeru
    najbolje = [None] * BROJ_MJESTA          # najbolje očitanje (kut, mjerenje) po mjestu
    for mjesto, lista in ocitanja.items():   # za svako mjesto i sva njegova očitanja...
        boje = set(m[0] for (i, m) in lista if m[0] != "-")   # ...različite boje koje su kutovi vidjeli (bez praznih)
        if len(boje) > 1:                    # ...ako su dva kuta vidjela RAZLIČITE boje...
            print("   UPOZORENJE: %s mjesto %d - kutovi se ne slazu: %s" % (smjer, mjesto,   # ...upozori...
                  ", ".join("kut %d=%s" % (i, m[0]) for (i, m) in lista)))   # ...i ispiši što je koji kut vidio
        najbolje[mjesto] = max(lista, key=lambda par: par[1][1])   # ...uzmi očitanje s najviše piksela boje
        rezultat[mjesto] = najbolje[mjesto][1][0]   # ...i njegovu boju kao rezultat za to mjesto
    return rezultat, najbolje                # vrati boje svih mjesta i najbolja očitanja


def grupiraj(niz):                           # 24 znaka -> 4 grupe po 6 (jedna grupa = jedna sekcija)
    return " ".join("".join(niz[s * 6:(s + 1) * 6]) for s in range(4))   # npr. "--R--- G----- -R--G- ----R-"


def broj_gresaka(rezultat):                  # koliko se mjesta razlikuje od OCEKIVANO (samo pokrivena mjesta)
    return sum(1 for k in range(BROJ_MJESTA) if rezultat[k] != "?" and rezultat[k] != OCEKIVANO[k])   # broji razlike


def nacin_sken(sirina, visina):              # SKEN: BROJ_SKENOVA ponavljanja, svako u svim SMJEROVI_SKENA, i sažetak
    if not provjeri_roi():                   # ako ROI ne valja...
        return                               # ...prekini (greška je već ispisana)
    print("\n=== SKEN: %d skenova, smjerovi %s ===" % (BROJ_SKENOVA, ", ".join(SMJEROVI_SKENA)))   # naslov
    print("CSV;sken;smjer;mjesto;opis;cw_broj;kut;boja;px;w;h;sigurnost;magenta_px;ekspozicija")   # zaglavlje CSV stupaca
    svi = {s: [] for s in SMJEROVI_SKENA}    # rezultati svih skenova po smjeru
    min_h = {s: None for s in SMJEROVI_SKENA}   # najmanja izmjerena visina stupa po smjeru
    slaganja = 0                             # broj skenova u kojima su CCW i CW mapa iste
    for n in range(1, BROJ_SKENOVA + 1):     # za svaki sken od 1 do BROJ_SKENOVA...
        mape = {}                            # mape ovog skena po smjeru
        for smjer in SMJEROVI_SKENA:         # ...za svaki smjer...
            postavi_nosac(smjer, n)          # ...traži postavljanje nosača u taj položaj (testira i ponovljivost postavljanja)
            rezultat, najbolje = jedan_sken_smjera(n, smjer, sirina, visina)   # ...napravi sken u tom položaju
            mape[smjer] = rezultat           # ...zapamti mapu
            svi[smjer].append(rezultat)      # ...i dodaj je u povijest
            greske = ("  pogresaka: %d" % broj_gresaka(rezultat)) if OCEKIVANO else ""   # tekst o pogreškama
            print("SKEN %2d %-3s: %s%s" % (n, smjer, grupiraj(rezultat), greske))   # npr. "SKEN  3 CW : --R--- G----- ...  pogresaka: 0"
            for d in najbolje:               # za najbolje očitanje svakog mjesta...
                if d and d[1][0] in ("R", "G"):   # ...ako je pronađen stup...
                    if min_h[smjer] is None or d[1][3] < min_h[smjer]:   # ...i njegova visina je najmanja do sada...
                        min_h[smjer] = d[1][3]   # ...zapamti je
                if d and d[1][0] == "R" and d[1][5] > d[1][1] // 2:   # ako uz crvenu ima magente više od pola piksela crvene...
                    print("   UPOZORENJE: %s kut %d puno magente uz crvenu" % (smjer, d[0]))   # ...upozori na moguću zamjenu
        if "CCW" in mape and "CW" in mape:   # ako su u ovom skenu snimljena oba smjera...
            razlike = [k for k in range(BROJ_MJESTA)   # ...nađi mjesta...
                       if mape["CCW"][k] != "?" and mape["CW"][k] != "?" and mape["CCW"][k] != mape["CW"][k]]   # ...gdje se mape razlikuju
            if razlike:                      # ...ako ih ima...
                print("   CCW i CW se NE slazu na mjestima: %s" % ", ".join(str(k) for k in razlike))   # ...ispiši ih
            else:                            # ...inače...
                slaganja += 1                # ...zabilježi slaganje
                print("   CCW i CW mapa su iste")   # ...i javi ga

    print("\n=== SAZETAK (mjesta u CCW numeraciji; 'CW broj' = broj istog mjesta u CW voznji) ===")   # naslov sažetka
    for k in range(BROJ_MJESTA):             # za svako mjesto 0-23...
        dijelovi = []                        # dijelovi retka za ovo mjesto
        for smjer in SMJEROVI_SKENA:         # ...za svaki smjer...
            vidjeno = "".join(s[k] for s in svi[smjer])   # ...niz boja kroz sve skenove, npr. "RRRRRRRRRR"
            if set(vidjeno) == {"?"}:        # ako mjesto ni u jednom skenu nije bilo pokriveno ROI-jem...
                stabilno = "nije pokriveno"  # ...to nije stabilnost nego rupa u kalibraciji
            elif len(set(vidjeno)) == 1:     # ...isti znak u svim skenovima...
                stabilno = "stabilno"        # ...= stabilno
            else:                            # ...različiti znakovi...
                stabilno = "NESTABILNO"      # ...= nestabilno
            dijelovi.append("%s %s %s" % (smjer, vidjeno, stabilno))   # npr. "CW RRRRRRRRRR stabilno"
        ocek = ("  ocekivano %s" % OCEKIVANO[k]) if OCEKIVANO else ""   # dodaj očekivanu boju ako je poznata
        print("mjesto %2d (%s, CW broj %2d): %s%s" % (k, opis_mjesta(k), cw_broj(k), " | ".join(dijelovi), ocek))   # jedan redak po mjestu
    for smjer in SMJEROVI_SKENA:             # za svaki smjer...
        if OCEKIVANO:                        # ...ako znamo stvarni raspored...
            tocno = sum(1 for s in svi[smjer] for k in range(BROJ_MJESTA) if s[k] != "?" and s[k] == OCEKIVANO[k])   # ...točna očitanja
            moguce = sum(1 for s in svi[smjer] for k in range(BROJ_MJESTA) if s[k] != "?")   # ...sva očitanja pokrivena ROI-jem
            print("Tocnost %s: %d / %d" % (smjer, tocno, moguce))   # npr. "Tocnost CW: 220 / 220"
        print("Najmanja visina stupa %s: %s px" % (smjer, min_h[smjer]))   # ključni broj za odluku (cilj >= 6 px)
    if "CCW" in SMJEROVI_SKENA and "CW" in SMJEROVI_SKENA:   # ako su skenirana oba smjera...
        print("CCW i CW mapa iste u %d od %d skenova" % (slaganja, BROJ_SKENOVA))   # ...glavni rezultat: vrijedi li jedna mapa za oba smjera


# ============================== 5. START =====================================

print("=== Pokrenuto: NACIN=%s, KUTOVI=%s ===" % (NACIN, KUTOVI))   # prvi redak u terminalu: potvrda da je skripta krenula
sirina, visina = init_kamera()               # postavi kameru i doznaj dimenzije slike
zakljucaj()                                  # zaključaj balans (i po želji ekspoziciju) u SVIM načinima, da pragovi iz LIVE vrijede i u SKEN-u
if NACIN == "LIVE":                          # ako je odabran LIVE...
    nacin_live(sirina, visina)               # ...pokreni živu sliku (radi dok je ne zaustavite)
elif NACIN == "KALIBRACIJA":                 # ako je odabrana KALIBRACIJA...
    nacin_kalibracija(sirina, visina)        # ...pokreni kalibraciju
elif NACIN == "SKEN":                        # ako je odabran SKEN...
    nacin_sken(sirina, visina)               # ...pokreni test skeniranja
else:                                        # ako je upisano nešto drugo...
    print("Nepoznat NACIN:", NACIN)          # ...javi grešku
print("Kraj.")                               # oznaka da je skripta završila
