"""
KALIBRACIJA KAMERE - testni program za podesavanje RED_HUE / GREEN_HUE
========================================================================
Pokreni ovo PRIJE vlastite voznje, spojen na laptop (USB/BT), s Pybricks
Code konzolom otvorenom da vidis print() ispis uzivo.

Najbolje rezultate dobijes ako ovo pokrenes na samom mjestu natjecanja,
pod istim svjetlom koje ce biti za vrijeme voznje - hue se mijenja s
osvjetljenjem, pa kalibracija od kuce ne mora vrijediti u dvorani.

Postupak:
1. Postavi CRVENU prepreku ispred kamere telefona, na otprilike istoj
   udaljenosti/kutu kako je robot stvarno vidi.
2. Pritisni CENTER gumb na hubu - program skenira raspon hue vrijednosti
   oko pocetne pretpostavke i ispisuje povrsinu (w*h) detekcije za
   svaku. Najveca povrsina = najbolje poklapanje.
3. Nakon skeniranja, 5 sekundi ispisuje live ocitanja na pronadjenoj
   najboljoj hue vrijednosti - pomakni prepreku malo (kut, udaljenost)
   da provjeris koliko je stabilno.
4. Zamijeni prepreku ZELENOM, pritisni CENTER ponovno za drugi krug.
5. Na kraju ispisuje dvije linije koje samo kopiras u glavni program:
       RED_HUE   = bytes([...])
       GREEN_HUE = bytes([...])
"""

from pybricks.hubs import PrimeHub
from pybricks.tools import AppData, wait
from pybricks.parameters import Button
from struct import unpack

hub = PrimeHub()
app = AppData([(0, 4)])

# Pocetne pretpostavke za skeniranje (u stupnjevima, 0-360).
# Stavi ovdje svoje zadnje poznate vrijednosti * 2 (byte je pola hue-a).
POCETNI_HUE_CRVENA = 354   # npr. ako je zadnji dobri byte bio 177
POCETNI_HUE_ZELENA = 110   # npr. ako je zadnji dobri byte bio 55


def cekaj_centar():
    """Ceka da se CENTER gumb pritisne i otpusti (s debounceom)."""
    while Button.LEFT not in hub.buttons.pressed():
        wait(20)
    while Button.LEFT in hub.buttons.pressed():
        wait(20)
    wait(200)


def ocitaj(hue_deg, cekanje=300):
    """Postavi hue (0-360 stupnjeva) i vrati (x, y, w, h)."""
    hue_byte = (hue_deg % 360) // 2
    app.configure(0, 0, bytes([hue_byte]))
    wait(cekanje)  # ako ocitanja djeluju "zakasnjelo", povecaj na 500
    return unpack("BBBB", app.get_bytes(0))


def skeniraj_boju(naziv, pocetni_hue, raspon=40, korak=2, uzoraka=3):
    """
    Skenira hue vrijednosti oko pocetnog hue i ispisuje povrsinu (w*h)
    detekcije za svaku. Vraca (hue u stupnjevima, odgovarajuci byte)
    za vrijednost koja je dala najvecu povrsinu.
    """
    print(f"\n--- Skeniram: {naziv} ---")
    najbolji_hue = pocetni_hue
    najbolja_pov = -1
    for h in range(pocetni_hue - raspon, pocetni_hue + raspon + 1, korak):
        h = h % 360
        povrsine = []
        zadnje = (0, 0, 0, 0)
        for _ in range(uzoraka):
            zadnje = ocitaj(h)
            x, y, w, hh = zadnje
            povrsine.append(w * hh)
        prosjek = sum(povrsine) / len(povrsine)
        print(f"hue={h:>3}  byte={h // 2:>3}  w*h(avg)={prosjek:>6.1f}  zadnji={zadnje}")
        if prosjek > najbolja_pov:
            najbolja_pov = prosjek
            najbolji_hue = h
    print(f"--> NAJBOLJI hue za {naziv} = {najbolji_hue} (byte={najbolji_hue // 2}), povrsina={najbolja_pov:.1f}")
    return najbolji_hue, najbolji_hue // 2


def provjeri_uzivo(hue_deg, trajanje_ms=5000):
    """Ispisuje live ocitanja na zadanom hue tijekom trajanje_ms milisekundi."""
    print(f"Live provjera na hue={hue_deg} - pomakni prepreku da testiras stabilnost...")
    t = 0
    while t < trajanje_ms:
        x, y, w, h = ocitaj(hue_deg, cekanje=200)
        print(f"   x={x:>3} y={y:>3} w={w:>3} h={h:>3}")
        t += 200


# --- glavni tok programa ---

hub.speaker.beep()
print("===========================================")
print(" Postavi CRVENU prepreku, pritisni CENTER")
print(" za pocetak skeniranja.")
print("===========================================")
cekaj_centar()
crvena_hue, crvena_byte = skeniraj_boju("CRVENA", POCETNI_HUE_CRVENA)
provjeri_uzivo(crvena_hue)

hub.speaker.beep()
print("\n===========================================")
print(" Zamijeni s ZELENOM preprekom, pritisni CENTER")
print(" za pocetak skeniranja.")
print("===========================================")
cekaj_centar()
zelena_hue, zelena_byte = skeniraj_boju("ZELENA", POCETNI_HUE_ZELENA)
provjeri_uzivo(zelena_hue)

hub.speaker.beep()
print("\n===========================================")
print(" REZULTAT - kopiraj ove dvije linije u glavni program:")
print(f" RED_HUE   = bytes([{crvena_byte}])   # hue={crvena_hue}")
print(f" GREEN_HUE = bytes([{zelena_byte}])   # hue={zelena_hue}")
print("===========================================")