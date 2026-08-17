# System Ready State + Gateway "green dot" indicator

**Goal:** A reusable system-readiness state (boot + wifi + tailscale + gateway
legs all up) AND a tiny status dot in the upper corner of the LCD: appears green
when the OpenClaw gateway is established and ready to accept commands. Other code
can query/subscribe to the ready state for future use.

## Two deliverables
### 1. Reusable ready-state module (`main/stackchan_ready.{h,c}`)
Central place that tracks readiness and exposes it. Do NOT scatter booleans.
- Track sub-states, each set from its real source:
  - `WIFI` — from `esp_openclaw_node_wifi_is_connected()` / wifi events.
  - `TAILSCALE` — from stackchan_tailscale (link up / has address). If a
    trustworthy signal isn't exposed, add a minimal getter in
    stackchan_tailscale rather than guessing.
  - `GATEWAY_OPERATOR` — set true on ESP_OPENCLAW_NODE_EVENT_CONNECTED for the
    operator leg (user_ctx == &s_saved_session_reconnect), false on DISCONNECTED.
  - `GATEWAY_NODE` — same for the node/ctrl leg (user_ctx == &s_ctrl_reconnect).
    (If the node leg isn't configured in a given build, treat it as
    not-required so a single-leg build can still be READY.)
- Expose:
  - `typedef enum { SYS_BOOTING, SYS_CONNECTING, SYS_READY, SYS_DEGRADED } stackchan_ready_state_t;`
  - `stackchan_ready_state_t stackchan_ready_get(void);`
  - `bool stackchan_ready_is_ready(void);`  (READY = wifi + tailscale + required
    gateway legs all up)
  - `void stackchan_ready_set_wifi(bool)/set_tailscale(bool)/set_gateway(leg,bool)`
    (called from app_main event handler + boot).
  - Optional: a registerable callback `stackchan_ready_on_change(cb, ctx)` so
    other subsystems can react to READY transitions (this is the "ready state we
    can use in other places" Eric asked for).
- Thread-safe (a mutex or atomics); called from wifi/event/timer contexts.

### 2. LCD status dot (upper corner)
- The avatar task (stackchan_boot.cpp) redraws full 320x240 JPEG frames at 12fps.
  Draw the dot as an OVERLAY at the END of each frame draw so it isn't clobbered.
  Add a small `draw_status_dot(lcd)` called right after `lcd.drawJpg(...)` in the
  avatar loop (and after the orb/other full-screen draws if they run).
- Placement: top-right corner, small (r=5-6px), with a subtle dark outline so
  it's visible over any avatar frame. Upper-right recommended (top-left often has
  status text); confirm nothing else draws there.
- Colors by `stackchan_ready_get()`:
  - SYS_BOOTING → dim grey (or none)
  - SYS_CONNECTING → amber/yellow
  - SYS_READY → green
  - SYS_DEGRADED (was ready, a required leg dropped) → red
- LCD is SPI (M5.Display), OFF the I2C bus — safe to draw anytime, does NOT
  touch the codec/AXP2101 I2C bus. Do NOT add any I2C traffic for this.
- Keep it cheap: only fillCircle a few px each frame; no extra tasks needed
  (piggyback the existing avatar loop). If a screen is showing static content
  (avatar paused), still refresh the dot periodically or on ready-state change.

## Wiring (app_main.c)
- In the node event handler (~line 131), on CONNECTED/DISCONNECTED, call
  `stackchan_ready_set_gateway(leg, connected)` deriving `leg` from user_ctx
  (operator vs ctrl reconnect struct pointer).
- After wifi connect and after tailscale start, set those sub-states.
- Init the ready module early in app_main before the legs start.

## Constraints (unchanged hard rules)
- Do NOT change STT/TTS logic in stackchan_voice_net.c.
- Keep stackchan_voice.h public API frozen (this feature shouldn't need to touch
  it — new module + boot/app_main only).
- A2 scope; commit LOCALLY only, NEW commits, NEVER push.
- Never leave idf.py monitor holding /dev/cu.usbmodem2101 — open only for a test.
- Build: cd ~/esp-openclaw-node/examples/esp32-node && . $HOME/esp-idf/export.sh
  >/dev/null 2>&1 && idf.py build && idf.py -p /dev/cu.usbmodem2101 flash
- Serial python: /Users/eric/.espressif/python_env/idf5.3_py3.13_env/bin/python

## Verify on device
- Boot with gateway reachable: dot goes grey → amber → GREEN when both legs
  connect. Log `SYS -> READY`.
- Pull the gateway (or wifi) after ready: dot goes red (DEGRADED), log the
  transition; restore → back to green.
- Confirm a normal voice turn still works (no regression) and no I2C/codec
  errors, no false shutdown.

## Definition of done
Ready module + dot implemented, verified on device (grey→amber→green + degraded
red), voice turn still clean, committed locally (never pushed), net.c STT/TTS
untouched. Checkpoint to ~/.openclaw/workspace/memory/2026-08-15.md.

## COMMIT INCREMENTALLY (timeout safety)
Commit after: (a) ready module compiles, (b) app_main wiring builds, (c) dot
draws, (d) on-device verified. A runtime cap of ~30m applies — never do a giant
uncommitted rewrite. If capped, the next run resumes from your last commit.
