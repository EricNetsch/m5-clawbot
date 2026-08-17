# Bug: TTS audio repeats / fires multiple times (OpenClaw streams it ONCE)

Jem: OpenClaw logs are clean, reply broadcast once. Repeat is firmware-side.

## Root cause (high confidence — verified in code)
Two WS legs both deliver gateway events into the same handler:
- app_main.c: BOTH `node_config.gateway_event_cb` (operator leg `s_node`) AND
  `ctrl_config.gateway_event_cb` (node/ctrl leg `s_node_ctrl`) =
  `handle_gateway_event` -> `stackchan_voice_net_on_gateway_event` ->
  `handle_chat_event`.
- The assistant "chat" final broadcast can arrive on BOTH legs (chat.subscribe
  rides the node leg; the operator leg may also receive it).
- The ONLY dedup is `handle_chat_event`'s `s_last_reply` strncmp
  (stackchan_voice_net.c ~L796). It is NOT lock-protected and the two legs run
  independent WS RX tasks. Both can read the old s_last_reply, both pass the
  check, both call `speak_text()` before either writes s_last_reply = TOCTOU race
  -> duplicate back-to-back playback. Matches the symptom exactly.
- Secondary: dedup keys on TEXT only (prefix, 255 bytes) not a stable reply id,
  so it's fragile (long replies sharing a prefix, or the same text legitimately
  repeated).

## Fix (do all; keep STT/TTS *decoding* logic untouched — this is dispatch/dedup)
NOTE: net.c is normally frozen, but THIS bug lives in net.c's event dispatch.
Allowed change = the chat-event de-dupe/leg-gating ONLY. Do NOT touch the
appendAudio / STT capture / PCM decode / playback DSP. Get explicit care here.

1. **Single-leg gating for chat replies.** Only handle "chat" events from ONE
   leg (the node leg, since chat.subscribe rides s_node_ctrl). Pass the source
   leg handle into on_gateway_event (or tag the callback per leg) and ignore
   chat events that arrive on the operator leg. talk.event transcripts still
   come on the operator leg — keep those. This alone removes the dual-delivery.
2. **Lock + id-based dedup.** Guard the dedup + s_last_reply write under
   s_state_lock (or a dedicated mutex). Prefer de-duping on a stable id:
   read `runId` + `seq` (verified gateway shape:
   { runId, sessionKey, seq, state:"final", message }) and skip if we've already
   spoken that (runId,seq). Fall back to text compare only if id absent.
3. **Speaking-guard.** If already SPEAKING/streaming the same utterance id,
   ignore a duplicate speak_text() request (don't queue a second playback).

## Investigate to CONFIRM before/after (add temporary telemetry)
- Log the leg + runId/seq on every chat final received:
  `chat final leg=%s runId=%s seq=%d -> speak/skip`. Reproduce and SHOW the two
  deliveries (or the race) in serial, then prove the fix collapses them to one.
- Also confirm whether the STREAMED tts path (CMD_STREAM_BEGIN/FEED/END in
  stackchan_voice.cpp) can be started twice for one reply (begin called while
  s_stream_active) — voice.cpp L605/L616 guards s_stream_active; verify a second
  begin can't double-open. If speak_text -> talk.speak (buffered) is the only
  path, the dual-leg race is the whole story.

## Constraints
- Change limited to chat-event dispatch/dedup + app_main leg wiring. Do NOT
  alter appendAudio, mic capture, PCM decode, or playback DSP.
- stackchan_voice.h public API stays frozen.
- Commit LOCALLY only, NEW commits, NEVER push.
- Never leave idf.py monitor holding /dev/cu.usbmodem2101.
- Build: cd ~/esp-openclaw-node/examples/esp32-node && . $HOME/esp-idf/export.sh
  >/dev/null 2>&1 && idf.py build && idf.py -p /dev/cu.usbmodem2101 flash
- Serial python: /Users/eric/.espressif/python_env/idf5.3_py3.13_env/bin/python

## Verify on device
- Real turn: assistant reply plays EXACTLY once. Serial shows the reply arriving
  (possibly on both legs) but only ONE speak dispatched. Run several turns +
  barge-in; never a back-to-back repeat. No regression to normal playback.

## Definition of done
Duplicate/back-to-back TTS eliminated, proven on device with before/after serial
(two deliveries -> one speak), STT/TTS decode + playback DSP untouched, committed
locally (never pushed). Checkpoint to memory/2026-08-15.md.

## COMMIT INCREMENTALLY (~30m cap)
Commit after: (a) telemetry added + repro captured, (b) leg-gate + locked
id-dedup builds, (c) on-device single-fire verified.
