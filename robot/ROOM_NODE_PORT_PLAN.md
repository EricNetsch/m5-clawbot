# StackChan CoreS3 → Room-Node Control Pattern Port (Path A2)

**Goal id:** a1c19973-1ce3-4036-825b-a2066ee8d6a9
**Owner:** opus subagent (this doc is your spec — execute against it, do not re-derive)
**Board:** M5Stack CoreS3 (ESP32-S3) + StackChan. Serial: `/dev/cu.usbmodem2101`.
**Repo:** `~/esp-openclaw-node`, example `examples/esp32-node`.

---

## 0. THE ONE RULE (read first)

**Do NOT touch `stackchan_voice_net.c` / `.h`.** That is the OpenClaw ElevenLabs
STT/TTS transport (`talk.session.create`, `talk.session.appendAudio` = STT,
`talk.speak`, `voice.transcript`, `chat.subscribe`). It works perfectly. Eric
will be furious if the STT/TTS pipeline changes. You are ONLY rewriting the
audio/hardware layer.

**The seam:** `stackchan_voice.cpp` and `stackchan_voice_net.c` communicate
through the public API in `stackchan_voice.h`. That API is FROZEN. If every
function in `stackchan_voice.h` keeps its exact signature and observable
behavior, `stackchan_voice_net.c` and `app_main` need zero changes and STT/TTS
is guaranteed untouched.

### Frozen public API (stackchan_voice.h) — must all keep working identically
- `esp_err_t stackchan_voice_start(void)`
- `bool stackchan_voice_toggle_listening(void)`
- `bool stackchan_voice_is_listening(void)`
- `esp_err_t stackchan_voice_speaker_selftest(void)`
- `esp_err_t stackchan_voice_register_command(void)`  (console: on/off/toggle/test/status/streamtest/talk)
- `void stackchan_voice_set_speaking(bool)`
- `esp_err_t stackchan_voice_play_pcm(const int16_t*, size_t, uint32_t)`
- `esp_err_t stackchan_voice_stream_begin(uint32_t, uint8_t)`
- `esp_err_t stackchan_voice_stream_feed(const int16_t*, size_t)`
- `void stackchan_voice_stream_end(void)`
- `esp_err_t stackchan_voice_stream_abort(void)`  (barge-in)
- `bool stackchan_voice_is_streaming(void)`
- `void stackchan_voice_set_mic_chunk_cb(cb, ctx)`  (mic PCM sink → net appendAudio)
- `void stackchan_voice_set_ptt_cb(cb, ctx)`        (PTT edge → net open/close turn)

---

## 1. WHY (the bug class we are killing)

Current `stackchan_voice.cpp` mutates a pile of `volatile` flags
(`s_listening`, `s_capture_active`, `s_stream_active/done/abort`, `s_thinking`)
from 5+ contexts (mic capture task core1, stream feeder core0, gateway RPC/HTTP
task, console REPL, espnow/BtnB, thinking-LED task). `M5.Mic/Speaker.begin/end`
and buffer free() are scattered across all of them. Every bug so far is the same
disease: shared hardware mutated by many tasks with ad-hoc flags.
- stuck I2S port, double-init, "i2c_master_transmit failed" codec init
- heap double-free (`tlsf_free: block already marked as free`) on barge-in race
  (stream_abort from BtnB + stream_end from RPC both free the ring)

Already patched with mutexes as a stopgap (commit 33c717c). This port removes
the disease instead of patching symptoms.

## 2. THE PATTERN WE LIFT (from esp-openclaw-room-node)

Reference files (READ, mirror the structure, do not copy WebRTC bits):
- `~/esp-openclaw-node/components/esp-openclaw-room-node/room_ui_controller.h`
  → the state enum: IDLE / LISTENING / (CONNECTING/THINKING) / SPEAKING / ERROR.
- `.../capture_os.h` → recursive-mutex + event-group + sema wrapper idioms.
- `.../msg_q.h` → command-queue concept (you may use a plain FreeRTOS
  `xQueue` instead of lifting esp_capture's msg_q; the pattern is what matters).
- `.../esp_openclaw_room_node.c` → single-owner-task + posted-command structure.

### Target architecture for stackchan_voice.cpp
1. **ONE audio-owner task** ("voice_owner"). It is the ONLY code that calls
   `M5.Mic.begin/end`, `M5.Speaker.begin/end`, frees stream buffers, or touches
   the shared I2S/I2C. Nothing else. Ever.
2. **Command queue** (FreeRTOS `xQueue`). Every entry point in the public API
   that changes hardware state POSTS a command and (where the API is
   synchronous, e.g. stream_end waits for drain) blocks on a per-command
   completion sema/event. Commands:
   `CMD_START_LISTEN, CMD_STOP_LISTEN, CMD_PLAY_PCM, CMD_STREAM_BEGIN,
    CMD_STREAM_FEED (or a stream buffer), CMD_STREAM_END, CMD_ABORT,
    CMD_SELFTEST, CMD_SET_SPEAKING`.
3. **Explicit state machine**: `VOICE_IDLE → VOICE_LISTENING → VOICE_THINKING
   → VOICE_SPEAKING → VOICE_IDLE`. Transitions happen ONLY in the owner task.
   - Barge-in (CMD_ABORT / a PTT press during SPEAKING) is just a
     SPEAKING→(IDLE|LISTENING) transition — a serialized teardown, not a
     concurrent free. Double-free becomes structurally impossible.
4. **One bus mutex** guarding all I2S + shared I2C touches. NOTE: the pink
   "thinking" LED ring shares `M5.In_I2C` with the AW88298 speaker codec — the
   owner must hold the bus mutex (or stop the LED task) around Speaker.begin,
   same as commit 33c717c did. Fold that into the owner.
5. **Timeouts on every wait** (drain, teardown) so nothing hangs.

### Mic PCM + PTT still flow to net exactly as today
- Owner task captures mic PCM (16 kHz mono int16) and invokes the registered
  `mic_chunk_cb` (which net.c set → it base64s + `talk.session.appendAudio`).
  Keep chunk size/format identical (CLEAN raw, no gain, pcm_16000).
- On PTT edges the owner invokes `ptt_cb(listening)` so net opens/closes the
  Talk turn in lockstep. Keep the edge semantics identical.

## 3. SCOPE — A2 (DEFAULT, do this)

Keep the **working M5Unified** `M5.Mic` / `M5.Speaker` (half-duplex). Wrap all
of it in the owner-task/queue/state-machine. This preserves 100% of current
behavior (including PTT barge-in) and kills the crash/race class. Do NOT attempt
full-duplex raw-I2S or AEC in this project.

### A1 (NOT NOW — phase 2, only if Eric explicitly says so)
Full-duplex raw i2s TX/RX pair + `esp_codec_dev` + AW88298 codec_if (port M5's
register sequence at M5Unified.cpp ~L440 `aw88298_write_reg` /
`_speaker_enabled_cb_cores3`) + AEC for voice-barge-in-during-speech. This is a
real rewrite and a rabbit hole. Leave it out.

## 4. IMPLEMENTATION PHASES (checkpoint after each)

Work in small, build-verified steps. After each phase: build, flash, run the
matching serial test, and append a dated checkpoint to
`~/.openclaw/workspace/memory/2026-08-15.md`.

- **P0 – scaffold:** Add the owner task, command queue, state enum, bus mutex.
  Owner loop pops commands, logs transitions. No behavior change yet (old paths
  still call through). Build + flash + confirm boots clean, no regressions on
  `voice status`.
- **P1 – route listening through owner:** `toggle_listening` / console on|off and
  the mic capture move behind `CMD_START/STOP_LISTEN`. Verify: `voice on` →
  RMS mic logs → `voice off` logs duration. mic_chunk_cb still fires (talk
  appendAudio still works if talk mode on).
- **P2 – route playback through owner:** `play_pcm` + `speaker_selftest` behind
  `CMD_PLAY_PCM/CMD_SELFTEST`. Verify `voice test` tone plays, LED goes
  speaking→idle.
- **P3 – route streaming through owner:** `stream_begin/feed/end/abort` behind
  commands; the owner owns the ring/pool lifecycle (single owner = no
  double-free). Verify `voice streamtest 24000` plays cleanly.
- **P4 – barge-in as transition:** PTT press / CMD_ABORT during SPEAKING =
  owner-serialized teardown. Run the stress test below repeatedly.
- **P5 – delete dead code:** remove the old scattered flags/mutex stopgaps now
  that the owner is the single authority. Keep the public API identical.

## 5. BUILD / FLASH / TEST (exact commands)

Build + flash (auto-reset via RTS works; no manual button needed):
```
cd ~/esp-openclaw-node/examples/esp32-node
. $HOME/esp-idf/export.sh >/dev/null 2>&1
idf.py build 2>&1 | tail -20
idf.py -p /dev/cu.usbmodem2101 flash 2>&1 | tail -3
```

Serial driver for tests (python + pyserial already available at
`/Users/eric/.espressif/python_env/idf5.3_py3.13_env/bin/python`). Use it to
send `voice ...` console commands and scrape logs. DO NOT hold the port open
across a flash — open, test, close.

**Barge-in double-free stress test (the acceptance gate for P4):**
Start `voice streamtest 24000`, then slam `voice on`/`voice off` (PTT barge-in)
while playback finishes, x4 rounds. PASS = no `tlsf`, `assert`, `abort()`,
`panic`, `guru`, `Backtrace`, `rst:`/`heap_init` (reboot) in the log. (This
exact test already passes on the stopgap; it must still pass after the port.)

## 6. GOTCHAS / HARD RULES (from Eric's MEMORY)
- **NEVER start a dev server / never `idf.py monitor` and leave it holding the
  port.** Open serial only for a test, then close.
- The thinking-LED ring shares I2C with the AW88298 codec → serialize LED vs
  Speaker.begin (bus mutex). This is why codec init failed before.
- Keep replies/commits terse. Commit locally only — **do NOT push** unless Eric
  says so. Add NEW commits (no --amend) if iterating on an open branch.
- Generated files: none relevant here, but never commit build/ sdkconfig.
- If a flash wedges (rare), you may need Eric to replug/hold BOOT — log it and
  keep going on what you can; do not spin.

## 7. CHECKPOINT / DON'T-HANG RULES (for the 5h subagent)
- After every phase, append findings + next step to
  `~/.openclaw/workspace/memory/2026-08-15.md` so progress survives.
- Never block waiting on user input. If you hit a decision the spec doesn't
  cover, pick the lowest-risk option that preserves the frozen API + STT/TTS,
  log the decision, and continue.
- If truly blocked on hardware (device unreachable after retries), write the
  blocker to memory and finish with a clear status rather than looping.
- Definition of done: P0–P5 complete, `voice status/on/off/test/streamtest`
  all work, barge-in stress test passes clean, `stackchan_voice_net.c`
  unchanged (verify with `git diff --stat`), build is clean, changes committed
  locally with a clear message. Report a concise summary.
```
