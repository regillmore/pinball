// House-ish pinball loop (64 bars = 2 minutes @ 128 BPM)
// Paste into https://strudel.cc and hit Ctrl/Cmd+Enter

setcpm(128/4) // 128 BPM in 4/4 (4 beats per cycle)

// ---------- DRUMS (909-ish) ----------
const KICK = s("bd*4").gain(1.05)
const CLAP = s("~ cp ~ cp").gain(0.85)
const HHC  = s("hh*8").gain(0.33)
const HHO  = s("~ 808oh ~ 808oh").gain(0.22).clip(0.20)
const PERC = s("<~ [mt lt] ~ [mt lt]>")
  .gain(0.18)
  .degradeBy(0.25)

// ---------- BASS (offbeat saw) ----------
const BASS = note("<a1 ~ a1 ~ a1 ~ g1 ~>")
  .s("sawtooth")
  .clip(0.18)
  .lpf(220)
  .lpq(2)
  .gain(0.70)
  .orbit(2)
  .delay(0.20).delaytime(0.125).delayfeedback(0.25)
  .room(0.18).roomsize(2.5)

// ---------- CHORD STABS (minor-7 vibe) ----------
const CHORDS = cat(
  "a3,c4,e4,g4",   // Am7
  "f3,a3,c4,e4",   // Fmaj7-ish
  "g3,b3,d4,f4",   // G7
  "e3,g3,b3,d4"    // Em7
).note()
  .s("sawtooth")
  .clip(0.22)
  .lpf(1200)
  .lpq(1.5)
  .gain(0.45)
  .struct("~ x ~ x") // offbeat stabs
  .orbit(2)
  .delay(0.25).delaytime(0.25).delayfeedback(0.35)
  .room(0.22).roomsize(3)

// ---------- LEAD / ARP (subtle sparkle) ----------
const LEAD = n("<0 2 4 7 9 7 4 2>")
  .scale("A:minor")
  .s("square")
  .clip(0.12)
  .lpf(1800)
  .gain(0.18)
  .every(4, x => x.add(12)) // occasional octave pop
  .orbit(2)
  .delay(0.30).delaytime(0.125).delayfeedback(0.30)
  .room(0.25).roomsize(3.5)

// ---------- 64 BAR FORM ----------
const kickForm = arrange(
  [32, KICK],
  [8,  silence], // breakdown
  [24, KICK]
)

const drumsForm = stack(
  kickForm,
  arrange([8, HHC], [56, stack(HHC, PERC)]),
  arrange([8, silence], [56, CLAP]),
  arrange([16, silence], [48, HHO])
).room(0.08).roomsize(1.2).delay(0.05).orbit(1)

// sidechain-ish pump: silent kick that ducks orbit 2
const DUCKER = s("bd*4")
  .postgain(0)
  .duckorbit(2)
  .duckattack(0.18)
  .duckdepth(0.85)

const song = stack(
  drumsForm,
  arrange([8,  silence], [24, BASS], [8, silence], [24, BASS]),
  arrange([16, silence], [16, CHORDS], [8, CHORDS.lpf(600)], [24, CHORDS]),
  arrange([24, silence], [24, LEAD], [16, silence]),
  DUCKER
).ribbon(0, 64) // loops exactly 64 cycles/bars

$:
song
