# Power button: single-press -> "goodnight" -> servos home -> power off

**Eric's ask:** ONE press of the power button triggers a shutdown sequence:
servos go to 0 (dead center), the screen says "goodnight", THEN the unit powers
off. DO NOT GUESS — verify against the OOTB M5Unified driver + on-device.

## What ALREADY exists (verified in code — build on this, don't rewrite blind)
- `main/esp_openclaw_node_stackchan_rgb_cmd.c`:
  - `power_button_task` polls AXP2101 PKEY IRQ; on SHORT press (INTSTS1 bit5
    0x20 = `AXP2101_IRQ_PKEY_SHORT`) calls `stackchan_shutdown_gracefully()`.
  - `stackchan_shutdown_gracefully()` = `stackchan_servo_shutdown()` (glide to
    DEAD CENTER + release torque) then `stackchan_power_off()` (AXP2101 soft
    power-off, cuts all rails, no return).
  - Monitor IS started: app_main.c:254 `stackchan_power_button_monitor_start()`.
  - `POWER_BTN_DIAG` is 0 (diag logging off).
  - Poller cooperatively pauses via `g_stackchan_power_poll_pause` during codec
    begin/end to avoid the false-shutdown I2C storm (this session's fix).
- `stackchan_servo_shutdown()` already glides to dead center = "servos to 0". Good.

## What's MISSING / TO FIX
1. **"Goodnight" screen** — the shutdown path never draws it. Add: on shutdown,
   before cutting power, show a goodnight screen (stop the avatar loop first via
   `stackchan_avatar_set(false)` / `stackchan_display_clear` +
   `stackchan_display_text("Goodnight")`, or a goodnight JPEG if one exists in
   spiffs). Order must be: freeze avatar -> draw goodnight -> recenter servos ->
   short hold so it's readable (~1-1.5s) -> power off.
2. **Confirm single-short-press ACTUALLY triggers on-device.** Comments admit
   tap-to-shutdown historically "did nothing" (IRQs were masked; now INTEN1/2/3
   set 0xFF). Eric says "the power button needs fixed" => treat as NOT working
   until proven. Verify a real single press fires the sequence exactly once.
3. **Make sure it can't false-fire** (the poller-pause guard must still let a
   REAL press through — real presses latch in INTSTS W1C and are caught on the
   next unpaused poll). Verify no accidental shutdown during a voice turn / LED
   pulse.

## MANDATORY RESEARCH (read before changing — no guessing)
- OOTB driver (authoritative): `components/M5Unified/src/utility/power/AXP2101_Class.cpp`
  and `components/M5Unified/src/utility/Power_Class.cpp` — confirm:
  - the exact INTSTS register + bit for a SHORT press on THIS AXP2101 (our code
    assumes SHORT latches in INTSTS1 0x20; the header comment even notes INTSTS2
    0x49 per datasheet but "confirmed on bench" INTSTS1 — RESOLVE this against
    M5Unified's own reads so we key off the correct register/bit).
  - how M5Unified debounces / distinguishes short vs long press, and whether it
    expects to clear (W1C) the same bits.
  - the correct soft-power-off sequence (REG 0x10 bit0 vs M5Unified's method).
- Cross-check the ESP-IDF docs if needed, but M5Unified is the ground truth for
  the CoreS3 AXP2101 wiring.
- Read `stackchan_servo.c` `stackchan_servo_shutdown()` + `handle_servo_home` to
  confirm "0" == dead center is what Eric means (it is: BSP center raw pos).

## Constraints (unchanged hard rules)
- No STT/TTS logic change in stackchan_voice_net.c; keep stackchan_voice.h frozen.
- Screen draw = SPI (safe). The AXP2101/servo path: keep the poller-pause guard;
  don't add new I2C hammering.
- Commit LOCALLY only, NEW commits, NEVER push.
- Never leave idf.py monitor holding /dev/cu.usbmodem2101 — open only for a test.
- Build: cd ~/esp-openclaw-node/examples/esp32-node && . $HOME/esp-idf/export.sh
  >/dev/null 2>&1 && idf.py build && idf.py -p /dev/cu.usbmodem2101 flash
- Serial python: /Users/eric/.espressif/python_env/idf5.3_py3.13_env/bin/python

## Verify on device (REQUIRED — this is a "fix it" task, prove it works)
- Boot, wait past the 3s boot-grace (so the power-on press is cleared).
- Single SHORT press the power key ONCE: goodnight screen shows -> servos glide
  to center -> unit powers off. Exactly once, no double-trigger.
- Confirm it does NOT fire during a normal voice turn / TTS / LED pulse.
- Long press still hard-cuts via AXP hardware (leave that as the fallback).
- Because the board powers OFF, capture serial log UP TO the power-off to prove
  the sequence ran (goodnight drawn, servo recenter logged, "AXP2101 soft
  power-off" logged) — then it dies as expected.

## Definition of done
Single short press => goodnight screen + servos to center + clean power off,
verified on device; no false-fire during a turn; net.c untouched; committed
locally (never pushed). Checkpoint to memory/2026-08-15.md.

## COMMIT INCREMENTALLY (~30m cap)
Commit after: (a) goodnight screen wired into shutdown path + builds,
(b) on-device single-press verified. If capped, resume from last commit.
