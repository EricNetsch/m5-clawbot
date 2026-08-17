# Liveliness: gated speaking pulse (independent, NOT waveform) + servo motion

**Goal:** Make the StackChan feel alive during a turn WITHOUT reintroducing the
I2C-bus/AXP2101 false-shutdown bug. Two safe additions:
1. A slow, INDEPENDENT (time-based, not audio-waveform) LED pulse while SPEAKING.
2. Servo head motion at speak start/end (and subtle idle life).

## HARD SAFETY CONTEXT (read memory/2026-08-15.md first)
- The 12x WS2812C ring is driven via the PY32 I2C expander on **I2C_NUM_0**
  (SDA12/SCL11) — the SAME bus as the AW88298 codec + AXP2101 poller. Hammering
  I2C while the speaker codec is active caused a false "power key -> shutdown".
- Therefore the pulse MUST be: (a) SLOW/rate-limited, (b) serialized on the
  audio owner task, (c) wrapped in the SAME AXP2101-poller-pause guard already
  used around LED writes (see commit 8598940 / stackchan_voice.cpp), (d) NEVER
  per-audio-sample. It's a gentle breathing effect, not a VU meter.
- Servos are Feetech SCSCL on UART1 (GPIO6/7) — a SEPARATE peripheral, NOT on
  I2C. Safe. Only note: VM_EN servo rail must be up (already is at runtime).

## Part 1 — Independent speaking pulse (LED ring)
- Trigger: on state -> SPEAKING, start a slow breathing pulse; on state leaving
  SPEAKING (-> IDLE/LISTENING/DEGRADED), stop and settle the ring.
- Rate: ~1-2 Hz sine/triangle brightness envelope (e.g. update every 150-250ms,
  <= ~5 Hz HARD MAX). This bounds I2C traffic far below the storm threshold.
- Color: a "speaking" hue (e.g. green/teal or the existing speaking color).
  Pulse BRIGHTNESS only, not per-LED chase, to keep writes minimal (one ring
  write per update).
- Implementation: drive it from the audio owner task loop (the single hardware
  owner) so it's serialized with codec begin/end; reuse the existing
  AXP-poller-pause-around-LED-write helper. Do NOT spawn a new task that writes
  the ring concurrently with the codec.
- MUST degrade safe: if a ring write ever errors, back off (skip pulses) rather
  than retry-hammer. Confirm no I2C error storm / no false shutdown during a full
  streamtest + real TTS turn.

## Part 2 — Servo motion (UART, safe)
Use stackchan_servo.{h,c} (scs_write_pos with move_ms easing). Keep motions
small + smooth. Suggested:
- SPEAKING start: a small "wake" gesture — slight upward pitch + tiny yaw
  turn (e.g. pitch +8-10deg, yaw +/-8deg), move_ms ~300-400 for a smooth ease.
- SPEAKING end: return toward center (home) with move_ms ~400-500.
- LISTENING start (optional, subtle): a tiny attentive tilt.
- Idle life (optional, low priority): occasional slow micro look-around when
  IDLE for a while (there may already be idle look-around from
  stackchan_servo_start — extend, don't duplicate).
- Do NOT move servos during graceful shutdown path except the existing recenter.
- Keep amplitudes conservative so it doesn't look twitchy or strain the servos.

## Wiring
- Hook both effects off the SAME state transitions the owner already drives
  (IDLE/LISTENING/THINKING/SPEAKING) so they stay in sync with audio + the
  ready dot. Best place = the owner task's state-change points in
  stackchan_voice.cpp (SPEAKING enter/exit), calling into servo + pulse helpers.
- Reuse stackchan_ready state if useful (e.g. don't do liveliness before READY).

## Constraints (unchanged)
- No STT/TTS logic change in stackchan_voice_net.c.
- stackchan_voice.h public API stays frozen (add internal helpers / use existing
  servo + rgb modules; don't change the seam).
- Commit LOCALLY only, NEW commits, NEVER push.
- Never leave idf.py monitor holding /dev/cu.usbmodem2101.
- Build: cd ~/esp-openclaw-node/examples/esp32-node && . $HOME/esp-idf/export.sh
  >/dev/null 2>&1 && idf.py build && idf.py -p /dev/cu.usbmodem2101 flash
- Serial python: /Users/eric/.espressif/python_env/idf5.3_py3.13_env/bin/python

## Verify on device (CRITICAL — this is the regression-prone one)
- Run a full real TTS turn AND `voice streamtest 24000`: the ring breathes
  slowly during SPEAKING, settles after. NO tlsf/panic/guru/reboot, NO
  "i2c ... failed" storm, NO false shutdown. Run barge-in x4 — still clean.
- Servos: smooth wake at speak start, smooth return at end, no twitch, no
  brownout/reset (servo current spike shouldn't reset the board — if it does,
  reduce simultaneous motion / add small stagger).
- Confirm the ready dot + gateway turn still work (no regression).

## Definition of done
Independent pulse + servo motion implemented, on-device verified with NO bus
instability/false shutdown, voice turn + barge-in still clean, committed locally
(never pushed), net.c untouched. Checkpoint to memory/2026-08-15.md.

## COMMIT INCREMENTALLY (timeout safety, ~30m cap)
Commit after: (a) servo motion works (safe, do this FIRST — lower risk),
(b) pulse compiles, (c) pulse verified no-bus-storm on device. If capped, resume
from last commit. Because the pulse is the risky part, land servos first so at
least that ships if time runs out.
