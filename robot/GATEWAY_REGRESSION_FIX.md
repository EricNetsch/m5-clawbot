# Fix: A2 port broke OpenClaw gateway mic-STT flow + confirm-gated loading state

**Goal:** Restore EXACT pre-port gateway behavior (mic STT streaming up +
TTS down + transcripts), then add: only enter the "thinking"/loading LED state
once OpenClaw has actually confirmed receipt + is responding.

**Regression window:** commit `33c717c` (WORKED end-to-end with gateway — Eric
confirmed) → `30b48bf` (the A2 port — nothing reaches gateway in Eric's test).
So the break is 100% inside `30b48bf`, in `stackchan_voice.cpp`. `net.c` is
byte-unchanged.

## PRIME HYPOTHESIS (diagnose first, then fix)
In the OLD code, `mic_chunk_cb` fired on a DEDICATED capture task, so net.c's
BLOCKING `talk.session.appendAudio` RPC didn't stall the state machine. In the
PORT, `owner_capture_step()` calls `s_mic_cb(...)` INLINE ON THE SINGLE OWNER
TASK (stackchan_voice.cpp ~line 420). If that callback does a blocking network
RPC, it stalls the owner task → command queue backs up → capture/state machine
wedge → nothing flows to the gateway. Same risk for `ptt_cb` (talk.session.create)
called inline from `toggle_listening`, and for any net RPC on the owner thread.

**Likely fix:** decouple network from the owner task. Hand mic PCM chunks to a
SEPARATE uplink task via a queue/StreamBuffer (owner just enqueues, never
blocks). Same for PTT/turn open-close if those RPCs block. The owner task must
NEVER make a blocking gateway call. This preserves the room-node single-owner
hardware guarantee while restoring the old non-blocking network threading.

## VERIFY THE REAL CAUSE ON DEVICE (don't guess-patch)
1. Build+flash current 30b48bf. Open serial, enable talk mode (`voice talk on`
   or whatever the console exposes), do `voice on` … speak … `voice off`.
2. Watch the log for the seam:
   - Do `mic chunk N rms=...` lines appear? (capture running)
   - Does net.c log `appendAudio` / `talk.session.create` / failures?
   - Does the owner task stop logging after the first chunk? (= stalled/wedged)
   - Do transcript/reply events arrive (`voice.transcript`, `talk.speak`)?
3. Compare with `git show 33c717c:examples/esp32-node/main/stackchan_voice.cpp`
   to see exactly how mic_cb/ptt_cb were invoked before (task context, ordering,
   blocking vs posted) and restore that CONTRACT.

## HARD RULES (unchanged)
- DO NOT modify `stackchan_voice_net.c` / `.h` STT/TTS LOGIC. (Adding a NEW
  public setter call for the loading-state feature below is OK — see note.)
- Keep `stackchan_voice.h` public API signatures identical (frozen seam).
- A2 scope only (M5Unified half-duplex). No full-duplex/AEC.
- Commit LOCALLY only, new commits (no --amend), never push.
- Never leave `idf.py monitor` holding the serial port; open only for a test.
- Device: CoreS3 on /dev/cu.usbmodem2101, auto-reset RTS works.
- Build: `cd ~/esp-openclaw-node/examples/esp32-node && . $HOME/esp-idf/export.sh
  >/dev/null 2>&1 && idf.py build && idf.py -p /dev/cu.usbmodem2101 flash`
- Serial test python: /Users/eric/.espressif/python_env/idf5.3_py3.13_env/bin/python

## ACCEPTANCE (gateway restored)
Full turn works end-to-end on device: PTT on → speak → PTT off → transcript
comes back from gateway → TTS audio plays → LED returns idle. mic RMS logs +
net appendAudio logs both present, owner task never wedges. Barge-in
(PTT during playback) still clean (no tlsf/panic/reboot).

---

## FEATURE: confirm-gated "thinking"/loading LED state

**Current (wrong):** `toggle_listening()` calls `stackchan_thinking_set(true)`
immediately on PTT-off — before OpenClaw has confirmed anything. So the pink
loading LEDs light even if the gateway never received/responded.

**Wanted:** enter the loading/thinking state ONLY when OpenClaw has actually
(a) confirmed it received our audio/turn AND (b) is responding.

### Research + implement
- Read `stackchan_voice_net.c` to find the earliest reliable signal that the
  gateway received the turn and is responding. Candidates, pick the earliest
  TRUSTWORTHY one:
  - `talk.session.create` success (session accepted) — earliest, but only means
    session open, not "responding".
  - first successful `appendAudio` ack — proves audio is landing.
  - `voice.transcript` (partial/interim) event — proves STT is processing our
    speech (strong "received + responding" signal).
  - `talk.speak` start / first TTS chunk — definitely responding (but that's
    late; loading state would barely show).
  Recommended: gate on the FIRST interim `voice.transcript` (or the
  appendAudio-accepted signal if transcripts are final-only) = "gateway heard us
  and is working." Document the choice.
- Implement by adding a NEW public hook on the FROZEN api surface, e.g.
  `void stackchan_voice_on_gateway_activity(void)` (or reuse an existing
  setter), which net.c calls from its event handler when that signal fires. That
  handler flips the thinking LED on. This ADDS a call in net.c's event path
  (allowed — it's not changing STT/TTS logic), keeps hardware control in
  stackchan_voice.cpp. If you can wire it WITHOUT editing net.c (e.g. net.c
  already calls a voice setter you can repurpose), prefer that.
- On PTT-off: do NOT light thinking immediately. Instead enter a neutral
  "armed/waiting" (keep idle or a subtle state), and only switch to the pink
  thinking animation when the gateway-activity hook fires. Add a timeout
  fallback (e.g. if no gateway activity in N seconds, show error/idle) so it
  can't hang lit or never light.
- Verify on device: PTT off with gateway reachable → thinking lights only after
  the confirm signal; PTT off with gateway UNREACHABLE → thinking does NOT light
  (goes to error/idle after timeout).

## CHECKPOINTS
Append dated progress to `~/.openclaw/workspace/memory/2026-08-15.md` after:
diagnosis confirmed, gateway flow restored, loading-state feature done. Never
block on user input; log decisions and continue. Report concise summary at end:
root cause, the fix, on-device turn result, loading-state behavior, and
`git diff --stat` confirming net.c STT/TTS logic untouched.
