# =============================================================================
# ae3_skener_test.py  -  verzija 2 (svaki red objašnjen komentarom)
# -----------------------------------------------------------------------------
# NAMJENA
#   Ručni test skeniranja WRO Future Engineers staze kamerom OpenMV AE3, BEZ
#   robota. Kamera stoji na nosaču na istom mjestu na kojem će biti na robotu
#   (sredina širine robota, što bliže izlazu iz parkinga, leća na 27-28 cm).
#   Kamera se okreće na nekoliko kutova, na svakom snimi jednu sliku i u
#   unaprijed zadanim pravokutnicima (ROI) odredi boju prepreke.
#
# NUMERACIJA MJESTA (24 mjesta, ista za oba smjera vožnje)
#   mjesto = 6 * sekcija + 2 * linija(vanjska i unutarnja) + strana
#   sekcija : 0 = startna sekcija (ona s parkingom), 1, 2, 3 = sljedeće sekcije
#             redom kojim ih robot prolazi u odabranom smjeru vožnje
#   linija  : 0, 1, 2 = poprečna linija na podlozi, redom kojim ih robot
#             prelazi dok vozi tom sekcijom (0 = prva koju dosegne)
#   strana  : 0 = pozicija bliže UNUTARNJEM zidu, 1 = bliže VANJSKOM zidu
#   Primjer: mjesto 15 = 6*2 + 2*1 + 1 = sekcija 2, srednja linija, vanjska strana.
#
# NAČINI RADA (varijabla NACIN)
#   "LIVE"        - živa slika: namještanje nagiba i pragova boja
#   "KALIBRACIJA" - ispisuje prijedloge ROI-ja koje kopirate u ROI = {...}
#   "SKEN"        - pravi test: N skenova, boja svakog mjesta, CSV i sažetak
#
# POKRETANJE
#   OpenMV IDE -> otvori ovu datoteku -> postavi NACIN -> zelena tipka Start
#   (dolje lijevo). Svi rezultati stižu u Serial Terminal (dno prozora IDE-a).
# =============================================================================

import csi                                   # modul za upravljanje kamerom (senzorom slike) na AE3
import time                                  # modul za čekanje (sleep_ms) i mjerenje FPS-a (clock)
from machine import Pin, PWM, LED            # Pin = tipka SW, PWM = impulsi za servo, LED = RGB lampica na kameri


# ============================ 1. POSTAVKE ====================================
# Sve što se mijenja tijekom testa nalazi se u ovom bloku. Ispod njega ne
# morate ništa dirati.

NACIN = "LIVE"                               # odabir načina rada: "LIVE", "KALIBRACIJA" ili "SKEN"
SMJER = "CCW"                                # smjer vožnje koji testiramo: "CCW" (suprotno satu) ili "CW" (u smjeru sata)

# Kutovi tornja u stupnjevima, mjereno od smjera "ravno naprijed" (0 st.).
# Za CCW se kamera okreće ULIJEVO, za CW skripta iste brojeve okreće UDESNO,
# jer je unutrašnjost staze uvijek na strani prema kojoj robot skreće.
KUTOVI = [30, 70, 110, 150]                  # 4 kuta po 40 st. razmaka; uz vidno polje ~64 st. pokrivaju ~-2 do ~182 st.
LIVE_KUT_INDEKS = 0                          # u LIVE načinu: indeks kuta (0-3) čije ROI-je crtamo na slici

# --- Servo (opcionalno) ---
SERVO_PIN = None                             # None = kameru okrećete rukom; npr. "P0" = servo spojen na pin P0
SERVO_MIN_US = 500                           # širina impulsa (mikrosekunde) za položaj servoa 0 st.
SERVO_MAX_US = 2500                          # širina impulsa za položaj servoa SERVO_RASPON st.
SERVO_RASPON = 270                           # ukupni mehanički hod servoa u stupnjevima
SERVO_NAPRIJED = 135                         # položaj servoa (st.) u kojem kamera gleda ravno naprijed
SERVO_SMJER = 1                              # 1 ili -1: promijenite u -1 ako se servo okreće na krivu stranu

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

# --- SKEN ---
BROJ_SKENOVA = 10                            # koliko puta ponavljamo cijeli sken (između njih premještate nosač)
SPREMI_SLIKE = False                         # True = sprema JPEG svake snimke na disk kamere (sporije, korisno za analizu)

BROJ_MJESTA = 24                             # 4 sekcije x 3 poprečne linije x 2 strane

# --- ROI-ji (popunjavate nakon faze KALIBRACIJA) ---
# Za svaki smjer i za svaki indeks kuta: popis redaka (mjesto, x, y, w, h)
#   mjesto = broj mjesta 0-23 (vidi NUMERACIJU gore)
#   x, y   = gornji lijevi kut pravokutnika u pikselima slike
#   w, h   = širina i visina pravokutnika u pikselima
# Svako mjesto upišite SAMO JEDNOM (na kutu na kojem se najbolje vidi).
ROI = {                                      # rječnik: smjer -> {indeks kuta -> popis ROI-ja}
    "CCW": {0: [], 1: [], 2: [], 3: []},     # ROI-ji za vožnju suprotno satu; prazno dok ne kalibrirate
    "CW":  {0: [], 1: [], 2: [], 3: []},     # ROI-ji za vožnju u smjeru sata; prazno dok ne kalibrirate
}                                            # kraj rječnika ROI

# Stvarni raspored stupova za provjeru točnosti: 24 znaka 'R', 'G' ili '-'
# (prazno mjesto). Razmaci su dopušteni radi preglednosti - skripta ih briše.
# Redoslijed je redoslijed mjesta 0-23; jedna grupa od 6 znakova = jedna sekcija.
# None = ne provjeravaj točnost.
OCEKIVANO = None                             # npr. "--R--- G----- -R--G- ----R-"


# ======================= 2. KAMERA, SERVO, TIPKA =============================

csi0 = csi.CSI()                             # objekt koji predstavlja kameru (senzor PAG7936 na AE3)
tipka = Pin("SW", Pin.IN)                    # korisnička tipka na kameri kao ulaz; pušteno = 1, pritisnuto = 0
servo = None                                 # pretpostavka: servo nije spojen
if SERVO_PIN:                                # ako je u postavkama upisan pin za servo...
    servo = PWM(Pin(SERVO_PIN), freq=50, duty_u16=0)   # ...napravi PWM izlaz 50 Hz (standard za servo), za sada bez impulsa

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


def opis_mjesta(mjesto):                     # pretvara broj mjesta u čitljiv opis, npr. "S2 L1 V"
    sekcija = mjesto // 6                    # cjelobrojno dijeljenje sa 6 daje sekciju (0-3)
    linija = (mjesto % 6) // 2               # ostatak pri dijeljenju sa 6, podijeljen s 2, daje liniju (0-2)
    strana = "U" if mjesto % 2 == 0 else "V" # paran broj = Unutarnja strana, neparan = Vanjska strana
    return "S%d L%d %s" % (sekcija, linija, strana)   # vrati tekst npr. "S2 L1 V"


def init_kamera():                           # postavlja kameru i vraća širinu i visinu slike
    csi0.reset()                             # vrati kameru u početno stanje
    csi0.pixformat(csi.RGB565)               # slika u boji, 16 bita po pikselu (potrebno za prepoznavanje boja)
    csi0.framesize(csi.HD)                   # najveća rezolucija; na AE3 očekujemo 1280 x 800
    img = csi0.snapshot(time=2000)           # snimaj i odbacuj slike 2 s da se ekspozicija i balans bijele smire
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


def cekaj_operatera(poruka):                 # ispiše uputu i čeka da operater potvrdi (tipka SW ili odbrojavanje)
    print(">>> " + poruka)                   # ispiši uputu u Serial Terminal
    led("LED_BLUE", True)                    # plava LED = kamera čeka vas
    if ODBROJAVANJE_S > 0:                   # ako je zadano odbrojavanje...
        for s in range(ODBROJAVANJE_S, 0, -1):   # ...broji od zadanog broja prema 1
            print("    %d..." % s)           # ispiši preostale sekunde
            time.sleep_ms(1000)              # čekaj 1 sekundu
    else:                                    # inače čekamo tipku
        print("    (pritisni i pusti tipku SW)")   # podsjetnik u terminalu
        while tipka.value() == 1:            # dok tipka NIJE pritisnuta (1 = puštena)...
            time.sleep_ms(20)                # ...čekaj 20 ms i provjeri ponovo
        while tipka.value() == 0:            # dok je tipka pritisnuta (0 = pritisnuta)...
            time.sleep_ms(20)                # ...čekaj; snimka kreće tek kad maknete prst
    led("LED_BLUE", False)                   # ugasi plavu LED: čekanje je gotovo


def stvarni_kut(indeks):                     # vraća kut s predznakom: + = lijevo, - = desno
    kut = KUTOVI[indeks]                     # uzmi kut iz popisa KUTOVI
    return kut if SMJER == "CCW" else -kut   # za CCW ostaje lijevo (+), za CW se zrcali udesno (-)


def postavi_kut(indeks, cekaj=True):         # okreće kameru na kut s indeksom 'indeks' (servo ili ručno)
    kut = stvarni_kut(indeks)                # kut s predznakom
    strana = "lijevo" if kut > 0 else ("desno" if kut < 0 else "ravno")   # tekstualni opis smjera
    if servo:                                # ako je spojen servo...
        s = SERVO_NAPRIJED + SERVO_SMJER * kut   # ...izračunaj položaj servoa: položaj "naprijed" + traženi kut
        s = max(0, min(SERVO_RASPON, s))     # ograniči na mehanički hod servoa (ne smije izvan 0..270)
        us = SERVO_MIN_US + (SERVO_MAX_US - SERVO_MIN_US) * s / SERVO_RASPON   # pretvori stupnjeve u širinu impulsa (linearno)
        servo.duty_u16(int(us * 65535 / 20000))   # pošalji impuls: udio od perioda 20000 us (50 Hz), skaliran na 0-65535
        print("Kut %d: %d st. %s (servo %d st., %d us)" % (indeks, abs(kut), strana, s, us))   # javi što je postavljeno
    elif cekaj:                              # ako nema servoa i treba čekati operatera...
        cekaj_operatera("Kut %d: okreni kameru na %d st. %s" % (indeks, abs(kut), strana))   # ...traži ručno okretanje


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

def nacin_live(sirina, visina):              # LIVE: neprekidna slika s oznakama, za nagib i pragove
    if servo:                                # ako je spojen servo...
        postavi_kut(LIVE_KUT_INDEKS, cekaj=False)   # ...okreni kameru na odabrani kut (bez čekanja tipke)
    sat = time.clock()                       # sat za mjerenje broja slika u sekundi (FPS)
    n = 0                                    # brojač snimljenih slika
    while True:                              # ponavljaj zauvijek (zaustavljate crvenom tipkom u IDE-u)
        sat.tick()                           # označi početak nove slike za mjerenje FPS-a
        img = csi0.snapshot()                # snimi jednu sliku
        mrlje = nadji_mrlje(img)             # pronađi sve obojene mrlje na cijeloj slici
        img.draw_line(sirina // 2, 0, sirina // 2, visina - 1, color=SIVA)   # okomita crta kroz sredinu slike
        img.draw_line(0, visina // 2, sirina - 1, visina // 2, color=SIVA)   # vodoravna crta kroz sredinu slike
        for (mjesto, x, y, w, h) in ROI[SMJER][LIVE_KUT_INDEKS]:   # za svaki kalibrirani ROI na ovom kutu...
            img.draw_rectangle(x, y, w, h, color=ZUTA, thickness=2)   # ...nacrtaj žuti pravokutnik
            img.draw_string(x, y - 18, str(mjesto), color=ZUTA, scale=2)   # ...i iznad njega broj mjesta
        for ime, b in mrlje:                 # za svaku pronađenu mrlju...
            img.draw_rectangle(b.rect(), color=BOJE_CRTANJA[ime], thickness=2)   # ...okvir u boji mrlje
            img.draw_string(b.x(), b.y() + b.h() + 2, "%s h%d p%d" % (ime, b.h(), b.pixels()),   # ...tekst ispod okvira mrlje
                            color=BOJE_CRTANJA[ime], scale=2)   # ...ispod okvira: boja, visina (h) i broj piksela (p)
        n += 1                               # povećaj brojač slika
        if n % 30 == 0:                      # svakih 30 slika...
            print("FPS %.1f  %s  mrlja: %d" % (sat.fps(), info_ekspozicije(), len(mrlje)))   # ...ispiši FPS, ekspoziciju i broj mrlja


def nacin_kalibracija(sirina, visina):       # KALIBRACIJA: za svaki kut ispisuje ROI za svaku pronađenu mrlju
    print("\n=== KALIBRACIJA, smjer %s ===" % SMJER)   # naslov u terminalu
    print("Stupovi moraju stajati na svim mjestima koja kalibrirate.\n")   # podsjetnik
    for i in range(len(KUTOVI)):             # za svaki kut tornja redom (0, 1, 2, 3)...
        postavi_kut(i)                       # ...okreni kameru (servo) ili traži ručno okretanje i čekaj SW
        img = csi0.snapshot(time=SMIRIVANJE_MS)   # pričekaj SMIRIVANJE_MS ms pa snimi sliku
        mrlje = nadji_mrlje(img)             # pronađi sve mrlje na slici
        mrlje = sorted(mrlje, key=lambda par: par[1].x())   # poredaj mrlje slijeva nadesno (po x koordinati)
        print("\n# --- kut %d (%d st.), %s ---" % (i, stvarni_kut(i), info_ekspozicije()))   # zaglavlje za ovaj kut
        print("%d: [   # zamijenite -1 brojem mjesta (0-23)" % i)   # početak bloka koji kopirate u ROI
        for k, (ime, b) in enumerate(mrlje): # za svaku mrlju, uz redni broj k (0, 1, 2...)...
            x, y, w, h = odrezi(b.x() - ROI_RUB, b.y() - ROI_RUB,   # ...lijevi gornji kut ROI-ja = kut mrlje pomaknut za ROI_RUB gore-lijevo
                                b.w() + 2 * ROI_RUB, b.h() + 2 * ROI_RUB, sirina, visina)   # ...ROI = mrlja + rub sa svih strana
            komentar = "#%d %s px=%d w=%d h=%d" % (k, ime, b.pixels(), b.w(), b.h())   # opis mrlje za lakše prepoznavanje
            if ime == "M":                   # magenta nije stup...
                print("    # (-1, %d, %d, %d, %d),  %s  <- magenta, nije stup" % (x, y, w, h, komentar))   # ...ispiši kao komentar
            else:                            # crvena ili zelena mrlja = kandidat za stup
                print("    (-1, %d, %d, %d, %d),  %s" % (x, y, w, h, komentar))   # redak spreman za kopiranje u ROI
            img.draw_rectangle(x, y, w, h, color=BOJE_CRTANJA[ime], thickness=2)   # nacrtaj predloženi ROI na slici
            img.draw_string(x, y - 20, "#%d" % k, color=BOJE_CRTANJA[ime], scale=2)   # i njegov redni broj #k
        print("],")                          # kraj bloka za ovaj kut
        osvjezi_ide(img)                     # pokaži sliku s brojevima u Frame Bufferu
        if SPREMI_SLIKE:                     # ako treba spremati slike...
            img.save("kal_kut%d.jpg" % i)    # ...spremi je na disk kamere
        cekaj_operatera("Usporedi brojeve #k sa stazom, pa nastavi")   # čekaj da pregledate sliku
    print("\nKALIBRACIJA gotova. Kopirajte blokove u ROI['%s'] i upisite brojeve mjesta." % SMJER)   # završna uputa


def jedan_sken(broj, sirina, visina):        # jedan cijeli sken: svi kutovi, svi ROI-ji
    rezultat = ["?"] * BROJ_MJESTA           # '?' = mjesto nije pokriveno nijednim ROI-jem (nije kalibrirano)
    detalji = [None] * BROJ_MJESTA           # ovdje pamtimo (kut, mjerenje) za svako mjesto
    for i in range(len(KUTOVI)):             # za svaki kut tornja...
        postavi_kut(i)                       # ...okreni kameru
        led("LED_GREEN", True)               # zelena LED = snimanje u tijeku, ne dirajte nosač
        img = csi0.snapshot(time=SMIRIVANJE_MS)   # pričekaj smirivanje pa snimi
        led("LED_GREEN", False)              # ugasi zelenu LED
        eksp = info_ekspozicije()            # zapamti ekspoziciju ove snimke
        mjerenja = []                        # mjerenja ove snimke (za crtanje nakon mjerenja)
        for (mjesto, x, y, w, h) in ROI[SMJER][i]:   # za svaki ROI kalibriran na ovom kutu...
            if mjesto < 0 or mjesto >= BROJ_MJESTA:   # ako je ostao -1 ili broj izvan 0-23...
                print("UPOZORENJE: ROI s mjestom %d preskocen" % mjesto)   # ...upozori
                continue                     # ...i preskoči taj ROI
            roi = odrezi(x, y, w, h, sirina, visina)   # osiguraj da je ROI unutar slike
            m = mjeri_roi(img, roi)          # izmjeri boju u tom ROI-ju
            mjerenja.append((mjesto, roi, m))   # zapamti za crtanje
            rezultat[mjesto] = m[0]          # upiši boju mjesta ('R', 'G' ili '-')
            detalji[mjesto] = (i, m)         # upiši kut i cijelo mjerenje
            print("CSV;%d;%d;%s;%d;%s;%d;%d;%d;%.2f;%d;%s" % (   # ispiši jedan CSV redak (format stupaca)...
                broj, mjesto, opis_mjesta(mjesto), i, m[0], m[1], m[2], m[3], m[4], m[5], eksp))   # ...vrijednosti koje idu u stupce
            # CSV redak: sken; mjesto; opis; kut; boja; piksela; širina; visina; sigurnost; magenta; ekspozicija
        for (mjesto, roi, m) in mjerenja:    # crtamo TEK NAKON mjerenja, da nacrtane crte ne pokvare boje
            img.draw_rectangle(roi, color=ZUTA, thickness=2)   # žuti okvir ROI-ja
            img.draw_string(roi[0], roi[1] - 20, "%d:%s" % (mjesto, m[0]),   # tekst 20 px iznad okvira ROI-ja
                            color=BOJE_CRTANJA[m[0]], scale=2)   # iznad okvira: "broj mjesta:boja"
            if m[6]:                         # ako je pronađen stup...
                img.draw_rectangle(m[6], color=BOJE_CRTANJA[m[0]], thickness=1)   # ...tanki okvir oko samog stupa
        osvjezi_ide(img)                     # pokaži rezultat u Frame Bufferu
        if SPREMI_SLIKE:                     # ako treba spremati slike...
            img.save("sken%02d_kut%d.jpg" % (broj, i))   # ...spremi npr. "sken03_kut2.jpg"
    return rezultat, detalji                 # vrati boje svih mjesta i detalje mjerenja


def grupiraj(niz):                           # 24 znaka -> 4 grupe po 6 (jedna grupa = jedna sekcija)
    return " ".join("".join(niz[s * 6:(s + 1) * 6]) for s in range(4))   # npr. "--R--- G----- -R--G- ----R-"


def nacin_sken(sirina, visina):              # SKEN: BROJ_SKENOVA ponavljanja i sažetak
    if not any(len(v) for v in ROI[SMJER].values()):   # ako za ovaj smjer nema nijednog ROI-ja...
        print("GRESKA: ROI['%s'] je prazan. Prvo napravite KALIBRACIJA." % SMJER)   # ...javi grešku
        return                               # ...i prekini
    print("\n=== SKEN, smjer %s, %d skenova ===" % (SMJER, BROJ_SKENOVA))   # naslov
    print("CSV;sken;mjesto;opis;kut;boja;px;w;h;sigurnost;magenta_px;ekspozicija")   # zaglavlje CSV stupaca
    svi = []                                 # popis rezultata svih skenova
    min_h = None                             # najmanja izmjerena visina stupa kroz sve skenove
    for n in range(1, BROJ_SKENOVA + 1):     # za svaki sken od 1 do BROJ_SKENOVA...
        if n > 1:                            # ...prije svakog osim prvog...
            if servo:                        # ...ako je servo, vrati kameru na prvi kut da ne smeta pri premještanju
                postavi_kut(0, cekaj=False)  # (bez čekanja tipke)
            cekaj_operatera("Sken %d: izvadi nosac iz starta, vrati ga uz postavljac, okreni kameru na kut 0" % n)   # uputa operateru
            # ...traži premještanje nosača: tako testiramo ponovljivost postavljanja
        rezultat, detalji = jedan_sken(n, sirina, visina)   # napravi jedan cijeli sken
        svi.append(rezultat)                 # zapamti njegov rezultat
        greske = ""                          # tekst o pogreškama (prazan ako nema OCEKIVANO)
        if OCEKIVANO:                        # ako znamo stvarni raspored...
            g = sum(1 for k in range(BROJ_MJESTA)   # ...za svako mjesto 0-23 broji 1...
                    if rezultat[k] != "?" and rezultat[k] != OCEKIVANO[k])   # ...prebroji mjesta gdje se razlikuje
            greske = "  pogresaka: %d" % g   # tekst za ispis
        print("SKEN %2d: %s%s" % (n, grupiraj(rezultat), greske))   # npr. "SKEN  3: --R--- G----- ...  pogresaka: 0"
        for d in detalji:                    # za detalje svakog mjesta...
            if d and d[1][0] in ("R", "G"):  # ...ako je pronađen stup...
                if min_h is None or d[1][3] < min_h:   # ...i njegova visina je najmanja do sada...
                    min_h = d[1][3]          # ...zapamti je
            if d and d[1][0] == "R" and d[1][5] > d[1][1] // 2:   # ako uz crvenu ima magente više od pola piksela crvene...
                print("   UPOZORENJE: na kutu %d puno magente uz crvenu" % d[0])   # ...upozori na moguću zamjenu

    print("\n=== SAZETAK ===")              # naslov sažetka
    for k in range(BROJ_MJESTA):             # za svako mjesto 0-23...
        vidjeno = "".join(s[k] for s in svi) # ...niz boja kroz sve skenove, npr. "RRRRRRRRRR"
        stabilno = "stabilno" if len(set(vidjeno)) == 1 else "NESTABILNO"   # isti znak u svim skenovima = stabilno
        ocek = (" ocekivano %s" % OCEKIVANO[k]) if OCEKIVANO else ""   # dodaj očekivanu boju ako je poznata
        print("mjesto %2d (%s): %s  %s%s" % (k, opis_mjesta(k), vidjeno, stabilno, ocek))   # jedan redak po mjestu
    if OCEKIVANO:                            # ako znamo stvarni raspored...
        tocno = sum(1 for s in svi for k in range(BROJ_MJESTA)   # ...za svaki sken i svako mjesto broji 1...
                    if s[k] != "?" and s[k] == OCEKIVANO[k])   # ...prebroji točna očitanja
        moguce = sum(1 for s in svi for k in range(BROJ_MJESTA) if s[k] != "?")   # ...i sva očitanja pokrivena ROI-jem
        print("Tocnost: %d / %d" % (tocno, moguce))   # npr. "Tocnost: 220 / 220"
    print("Najmanja visina pronadenog stupa: %s px" % min_h)   # ključni broj za odluku (cilj >= 6 px)


# ============================== 5. START =====================================

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
if servo:                                    # na kraju, ako je spojen servo...
    postavi_kut(0, cekaj=False)              # ...vrati kameru na prvi kut
print("Kraj.")                               # oznaka da je skripta završila
