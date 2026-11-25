// =====================================================================
//  obstacle_plan.h  -  Controlled C.H.A.O.S. v2  -  plan voznje iz mape stupova
//
//  Cisti C++ bez Arduino funkcija: isti kod se prevodi i na racunalu
//  (obstacle_challange/plan_sim) gdje se provjerava za SVE rasporede.
//
//  MAPA: 12 znakova 'R', 'G', '-', '?' u numeraciji smjera voznje:
//        linija = 3 * sekcija + L, sekcija 0 = startna (s parkingom),
//        L = 0, 1, 2 poprecna linija redom kojim je robot prelazi.
//
//  TRAKE: IN  = uz unutarnji zid (LANE_WALL_MM od zida),
//         OUT = uz vanjski zid  (LANE_WALL_MM, a u sekciji 0 PARK_LANE_WALL_MM
//               jer su magenta zidovi parkinga duboki 200 mm).
//         Srednja traka se ne koristi.
//
//  BOJA -> TRAKA: crveni = zaobidji s DESNE, zeleni = s LIJEVE strane.
//         CCW (unutarnji zid lijevo): R -> OUT, G -> IN
//         CW  (unutarnji zid desno):  R -> IN,  G -> OUT
//         Strana stupa (unutarnje ili vanjsko sjedalo) nije bitna.
//
//  OBILAZAK = 13 "posjeta" sekcija: S0 (od parkinga), S1, S2, S3, S0, ... S3,
//             S0 (do mjesta starta) = 3 puna kruga.
//  U jednoj posjeti robot ima traku na ulazu i traku na izlazu. Ako su
//  razlicite (trojka "R-G" kojoj boje traze razlicite trake), mijenja traku
//  oko srednje linije L1 (po pravilu trojki tamo nema stupa).
//  Promjena trake izmedju sekcija radi se u zavoju (biranjem trenutka zavoja).
//
//  TRIPLET RULE (WRO 2026 rules, Figure 8c - the 36 sign cards):
//  a section holds EITHER one pillar on any of its lines ("X--", "-X-", "--X")
//  OR two pillars on the two outer lines ("X-Y"). It is never empty, and a
//  middle pillar is always alone. Consequences used by the planner:
//    - a lane change is only ever needed in an "X-Y" section, and there the
//      middle line L1 is guaranteed to be free, so the change is centred on L1;
//    - a single pillar (on any line) fixes one lane for the whole section,
//      so no lane change happens inside that section.
// =====================================================================
#pragma once
#include <stdint.h>
#include <string.h>
#include <stdio.h>

namespace route {

enum : int8_t { NONE = -1, IN = 0, OUT = 1 };

const int LINES  = 12;
const int VISITS = 13;

// ---------------------------------------------------------------------
//  GEOMETRIJA (mm) - sve sto ovisi o robotu treba IZMJERITI na stazi
// ---------------------------------------------------------------------
struct Geo {
  float corridor     = 1000.0f;  // razmak zidova u Obstacle Challengeu
  float laneWall     = 150.0f;   // trake IN i OUT: lice bocnog ToF-a -> zid
  float parkLaneWall = 300.0f;   // traka OUT u sekciji 0 (parking 200 mm + rezerva)
  float sideToCenter = 70.0f;    // lice bocnog ToF-a -> sredina robota (pola sirine)
  float frontToCenter= 80.0f;    // lice prednjeg ToF-a -> sredina robota
  // Zaokret 90 st. na punom zakretu: koliko se SREDINA robota pomakne
  //   turnFwd  = u starom smjeru voznje,
  //   turnSide = bocno, prema strani skretanja.
  // Mjerenje: oznaci sredinu robota na podu, napravi zaokret 'q' (vidi .ino),
  // izmjeri oba pomaka. Teorija (R straznje osovine 173, sredina 53 mm
  // ispred straznje osovine): turnFwd = R - 53 = 120, turnSide = R + 53 = 226.
  float turnFwd      = 120.0f;
  float turnSide     = 226.0f;
  float lineStep     = 500.0f;   // razmak poprecnih linija (L0 = 0, L1 = 500, L2 = 1000)
};

struct Visit {
  int8_t section;     // 0..3
  int8_t firstLine;   // prva linija (0..2) o kojoj ova posjeta brine
  int8_t lastLine;    // zadnja linija (0..2) o kojoj ova posjeta brine
  int8_t entry;       // traka na ulazu u sekciju
  int8_t exit;        // traka na izlazu iz sekcije
  bool   midSwitch;   // promjena trake oko L1
  bool   freeLane;    // nijedan stup nije odredio traku (uzeta traka sljedece posjete)
  bool   forcedOut;   // start: zbog stupa iza robota prvo OUT (vidi startBlockLine)
};

struct Plan {
  int   dir;          // +1 = CCW, -1 = CW
  char  map[LINES + 1];
  Visit v[VISITS];
  int   unknown;      // broj linija s '?'
  int   badTriplets;  // sections that break the triplet rule ("X--", "-X-", "--X" or "X-Y")
};

inline const char *laneName(int8_t lane) {
  return lane == IN ? "IN" : lane == OUT ? "OUT" : "--";
}

inline bool isPillar(char c) { return c == 'R' || c == 'G'; }

// Traka koju trazi stup boje c u smjeru dir (NONE ako boja nije poznata).
inline int8_t laneFor(char c, int dir) {
  if (!isPillar(c)) return NONE;
  bool red = (c == 'R');
  if (dir > 0) return red ? OUT : IN;     // CCW: unutarnji zid lijevo
  return red ? IN : OUT;                  // CW:  unutarnji zid desno
}

// Triplet rule check for section s. Allowed layouts (Figure 8c of the rules):
//   one pillar on any line  -> "X--", "-X-", "--X"
//   two pillars             -> "X-Y" (always on the two outer lines)
// '?' (unknown) never passes, so the robot prints a warning for it.
inline bool tripletOk(const char *m, int s) {
  char a = m[3 * s], b = m[3 * s + 1], c = m[3 * s + 2];
  int n = isPillar(a) + isPillar(b) + isPillar(c);           // pillars in this section
  int e = (a == '-') + (b == '-') + (c == '-');               // lines seen empty
  if (n == 1 && e == 2) return true;                          // one pillar, on any line
  if (n == 2 && isPillar(a) && b == '-' && isPillar(c)) return true;   // two pillars on the outer lines
  return false;
}

// ---------------------------------------------------------------------
//  IZGRADNJA PLANA
//    startFirst  = prva linija sekcije 0 ISPRED robota u parkingu
//    finishLast  = zadnja linija sekcije 0 PRIJE mjesta zaustavljanja
//    startBlock  = linija sekcije 0 cijim bi stupom robot zapeo kad bi nakon
//                  izlaska iz parkinga presao preko cijele staze u IN
//                  (-1 = nema takve). Ako je ta linija zauzeta, robot prvo
//                  ide u OUT, a u IN prelazi kod L1.
// ---------------------------------------------------------------------
inline void build(Plan &p, const char *map12, int dir, int startFirst, int finishLast, int startBlock) {
  p.dir = dir;
  memcpy(p.map, map12, LINES);
  p.map[LINES] = 0;
  p.unknown = 0;
  p.badTriplets = 0;
  for (int k = 0; k < LINES; k++) if (p.map[k] == '?') p.unknown++;
  for (int s = 0; s < 4; s++) if (!tripletOk(p.map, s)) p.badTriplets++;

  // 1) trake koje traze stupovi
  for (int i = 0; i < VISITS; i++) {
    Visit &v = p.v[i];
    v.section   = i % 4;
    v.firstLine = (i == 0) ? startFirst : 0;
    v.lastLine  = (i == VISITS - 1) ? finishLast : 2;
    v.entry = v.exit = NONE;
    v.forcedOut = false;
    for (int L = v.firstLine; L <= v.lastLine; L++) {
      int8_t lane = laneFor(p.map[3 * v.section + L], dir);
      if (lane == NONE) continue;
      if (v.entry == NONE) v.entry = lane;  // prvi stup odredjuje ulaz
      v.exit = lane;                         // zadnji stup odredjuje izlaz
    }
    v.freeLane = (v.entry == NONE);
  }

  // 2) start: stup iza robota na putu preko staze -> prvo OUT
  if (startBlock >= 0 && p.map[startBlock] != '-') {
    p.v[0].forcedOut = true;
    p.v[0].entry = OUT;
    // No pillar ahead in section 0 (e.g. CW "X--": the only pillar is behind
    // the robot): stay in OUT for the whole visit; the corner turn moves the
    // robot into the lane of section 1, so no extra lane change is needed.
    if (p.v[0].freeLane) p.v[0].exit = OUT;
    p.v[0].freeLane = false;
  }

  // 3) slobodne posjete preuzimaju traku sljedece (nema suvisnih manevara)
  int8_t carry = NONE;
  for (int i = VISITS - 1; i >= 0; i--) {
    Visit &v = p.v[i];
    if (v.freeLane && carry != NONE) v.entry = v.exit = carry;
    if (v.exit == NONE && carry != NONE) v.exit = carry;   // forcedOut bez stupa
    carry = v.entry;
  }
  int8_t prev = IN;                       // ako bas nista nije poznato: IN
  for (int i = 0; i < VISITS; i++) {
    Visit &v = p.v[i];
    if (v.entry == NONE) v.entry = prev;
    if (v.exit == NONE) v.exit = v.entry;
    v.midSwitch = (v.entry != v.exit);
    prev = v.exit;
  }
}

// ---------------------------------------------------------------------
//  GEOMETRIJA MANEVARA (sredina robota; x = uzduz sekcije od linije L0,
//  w = udaljenost sredine robota od UNUTARNJEG zida)
// ---------------------------------------------------------------------
inline float laneW(const Geo &g, int section, int8_t lane) {
  if (lane == IN) return g.laneWall + g.sideToCenter;
  float wall = (section == 0) ? g.parkLaneWall : g.laneWall;
  return g.corridor - wall - g.sideToCenter;
}

// Udaljenost sredine robota od zida prema kojem ide traka (IN: unutarnji, OUT: vanjski)
inline float laneFromOwnWall(const Geo &g, int section, int8_t lane) {
  return lane == IN ? laneW(g, section, lane) : g.corridor - laneW(g, section, lane);
}

// Zavoj: prednji ToF (lice) na kojem pocinje zaokret, da robot zavrsi u
// traci 'lane' sekcije 'nextSection'.
inline float cornerFront(const Geo &g, int nextSection, int8_t lane) {
  return g.corridor - laneW(g, nextSection, lane) + g.turnFwd - g.frontToCenter;
}

// Zavoj: x u novoj sekciji nakon zaokreta (negativno = prije linije L0).
inline float xAfterCorner(const Geo &g, int prevSection, int8_t prevLane) {
  return g.turnSide - laneW(g, prevSection, prevLane);
}

// Promjena trake: x na kojem pocinje prvi zaokret (manevar centriran na L1).
inline float switchStartX(const Geo &g) {
  return g.lineStep - (g.turnFwd + g.turnSide) / 2.0f;
}

// Promjena trake / start: prednji ToF (lice) na kojem prestaje voznja preko
// staze, da drugi zaokret zavrsi tocno u traci 'lane'.
inline float acrossFront(const Geo &g, int section, int8_t lane) {
  return laneFromOwnWall(g, section, lane) + g.turnFwd - g.frontToCenter;
}

// Start: robot gleda prema unutarnjem zidu; prednji ToF na kojem staje.
inline float startFront(const Geo &g, int8_t lane) {
  return laneW(g, 0, lane) + g.turnFwd - g.frontToCenter;
}

// Plan kao tekst, npr. "S0> OUT | S1 IN->OUT | S2 OUT* | ..."
//   *  = traku nije odredio stup ove sekcije (preuzeta iz sljedece)
//   !  = start: prvo OUT zbog stupa iza robota
inline int toText(const Plan &p, char *buf, int n) {
  int k = 0;
  for (int i = 0; i < VISITS && k < n; i++) {
    const Visit &v = p.v[i];
    const char *tag = (i == 0) ? ">" : (i == VISITS - 1) ? "|" : "";
    if (v.midSwitch)
      k += snprintf(buf + k, n - k, "%sS%d%s %s->%s%s", i ? " | " : "", v.section, tag,
                    laneName(v.entry), laneName(v.exit), v.forcedOut ? "!" : "");
    else
      k += snprintf(buf + k, n - k, "%sS%d%s %s%s%s", i ? " | " : "", v.section, tag,
                    laneName(v.entry), v.freeLane ? "*" : "", v.forcedOut ? "!" : "");
  }
  return k;
}

}  // namespace route
