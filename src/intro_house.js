// Pinball House Intro Loop (16 bars, loops forever)
// Play this until the ball launches, then switch to your main loop.
// Paste into https://strudel.cc

setcpm(128 / 4)

// ----- Intro drum bed: hats + soft clap + occasional kick -----
const introKick = s("<bd ~ bd ~>*2")
    .gain(0.85)
    .lpf(140)      // thumpy, not punchy
    .clip(0.25)

const introClap = s("~ cp ~ cp")
    .gain(0.45)
    .lpf(1400)
    .room(0.12).roomsize(1.6)

const introHats = s("hh*8")
    .gain(0.28)
    .lpf(2200)
    .degradeBy(0.15)

const introOpenHat = s("~ ~ 808oh ~")
    .gain(0.16)
    .lpf(1800)
    .clip(0.20)

// ----- Teaser chord stabs (same vibe as your main loop) -----
const introChords = cat(
    "a3,c4,e4,g4",   // Am7
    "f3,a3,c4,e4",   // Fmaj7-ish
    "g3,b3,d4,f4",   // G7
    "e3,g3,b3,d4"    // Em7
).note()
    .s("sawtooth")
    .clip(0.18)
    .gain(0.32)
    .lpf(650)         // keep it filtered in the intro
    .struct("~ x ~ x") // offbeat stabs
    .orbit(2)
    .delay(0.25).delaytime(0.25).delayfeedback(0.35)
    .room(0.25).roomsize(3)

// ----- Noise sweep / riser (adds “ready… set…” energy) -----
const sweep = s("noise")
    .clip(0.08)
    .gain(0.10)
    .lpf(300).hpf(200)
    .every(2, x =>
        x
            .slow(2)        // sweep over 2 bars
            .lpf("<500 800 1200 2000>")
            .gain("<0.02 0.05 0.08 0.12>")
    )
    .room(0.35).roomsize(5)
    .orbit(3)

// ----- Form: 16 bars total (loops) -----
// - Bars 1–4: hats only
// - Bars 5–8: add clap + soft kick
// - Bars 9–12: add chord teases + sweep
// - Bars 13–16: slightly brighter, like “launch imminent”
const intro = stack(
    arrange(
        [4, introHats],
        [4, stack(introHats, introClap)],
        [4, stack(introHats, introClap, introKick)],
        [4, stack(introHats.lpf(3000), introClap, introKick)]
    ),

    arrange(
        [8, silence],
        [4, introOpenHat],
        [4, introOpenHat.lpf(2400)]
    ),

    arrange(
        [8, silence],
        [4, introChords],
        [4, introChords.lpf(900)] // opens a bit near the end
    ),

    arrange(
        [8, silence],
        [8, sweep]
    )
).ribbon(0, 16) // exact 16-bar loop

$:
intro
