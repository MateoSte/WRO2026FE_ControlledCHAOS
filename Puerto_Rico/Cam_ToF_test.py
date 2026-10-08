import tof
import time

tof.init()

previous_avg = None

while True:

    depth, depth_min, depth_max = tof.read_depth()

    # Uzimamo svih 64 mjerenja
    values = []

    for value in depth:
        if value > 0:
            values.append(value)

    if len(values) == 0:
        print("Nema valjanih mjerenja")
        time.sleep_ms(100)
        continue

    # Sortiranje
    values.sort()

    # -------------------------------------------------
    # 1. Prvi AVG - ako još nemamo prethodni
    # -------------------------------------------------

    if previous_avg is None:

        average = sum(values) / len(values)

    else:

        # -------------------------------------------------
        # 2. Provjera koliko se novo mjerenje razlikuje
        #    od prethodnog AVG-a
        # -------------------------------------------------

        difference = abs(sum(values) / len(values) - previous_avg)
        difference_percent = difference / previous_avg * 100

        # -------------------------------------------------
        # 3. Ako nema velike promjene:
        #    odbacujemo krajnje vrijednosti
        # -------------------------------------------------

        if difference_percent <= 20:

            # Uzimamo srednjih 70% podataka.
            # Time uklanjamo najniže i najviše vrijednosti.
            count = len(values)

            start = int(count * 0.15)
            end = int(count * 0.85)

            filtered = values[start:end]

            average = sum(filtered) / len(filtered)

        # -------------------------------------------------
        # 4. Velika promjena:
        #    moguće je da je robot stvarno ušao u zavoj
        # -------------------------------------------------

        else:

            # Provjeravamo jesu li nova mjerenja međusobno
            # dovoljno blizu jedno drugome.

            new_average = sum(values) / len(values)

            tolerance = new_average * 0.20

            consistent_values = []

            for value in values:

                if abs(value - new_average) <= tolerance:
                    consistent_values.append(value)

            if len(consistent_values) > len(values) * 0.5:

                # Većina senzora vidi novu udaljenost.
                # Prihvaćamo novu situaciju.
                average = sum(consistent_values) / len(consistent_values)

            else:

                # Velika promjena, ali mjerenja nisu
                # međusobno konzistentna.
                # Vjerojatno imamo outliere.
                average = previous_avg

    # Spremamo rezultat za sljedeći ciklus
    previous_avg = average

    print("AVG:", round(average), "mm")
    print("Broj mjerenja:", len(values))
    print("--------------------")

    time.sleep_ms(100)
