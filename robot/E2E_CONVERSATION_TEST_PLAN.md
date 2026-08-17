# Autonomous End-to-End Conversation Test (gateway loop verification)

**Goal:** A single command that runs a REAL conversation turn through the live
OpenClaw gateway and asserts every state + gateway RPC works, error-free.
Runnable autonomously (no human speaking, no human watching LEDs).

**Run AFTER the gateway-regression fix (GATEWAY_REGRESSION_FIX.md) verifies.**
Do NOT run in parallel — only one process can hold /dev/cu.usbmodem2101.

## Approach — inject a canned utterance as if from the mic
Physical mic can't be driven autonomously, so bypass it at the seam:
- Store a short real spoken utterance as PCM (16 kHz mono int16) in spiffs, e.g.
  `spiffs_image/utterance.wav` ("what time is it" / "say hello"). Keep it small.
- Add a console command `voice e2e` (and/or `voice injectmic <file>`) that:
  1. Fires the PTT-on path (ptt_cb → net opens the Talk turn) WITHOUT starting
     the physical mic.
  2. Streams the canned PCM through the SAME `mic_chunk_cb` uplink path the real
     mic uses (so it hits net.c `appendAudio` exactly like live audio).
  3. Fires PTT-off (end of utterance).
  4. Lets the real gateway transcribe → reply → stream TTS back down through the
     normal playback path.
- This exercises the ENTIRE real loop (gateway STT + LLM + TTS) end to end; only
  the physical microphone is substituted.

## Console command must emit machine-scrapeable markers
So the python harness can assert without ambiguity, log tagged lines, e.g.:
- `E2E: start` / `E2E: injected N samples`
- state transitions already logged; ensure each prints a stable token:
  `STATE -> IDLE|LISTENING|THINKING|SPEAKING`
- gateway seam: net.c already logs create/appendAudio; ensure a success token
  for each (`talk.session.create ok`, `appendAudio ok`, `transcript: "..."`,
  `talk.speak ok` / first TTS chunk).
- `E2E: PASS` / `E2E: FAIL <stage>` summary line from firmware if feasible;
  otherwise the python harness computes pass/fail from the tokens.

## Python harness (I run this)
`~/esp-openclaw-node/examples/esp32-node/tools/e2e_conversation_test.py`
- Uses /Users/eric/.espressif/python_env/idf5.3_py3.13_env/bin/python + pyserial.
- Opens the port, sends `voice talk on` then `voice e2e`, captures serial for a
  bounded window, closes the port.
- ASSERTS the ordered sequence + gateway tokens:
  1. STATE -> LISTENING
  2. appendAudio ok (>=1)
  3. gateway confirm signal (first voice.transcript / appendAudio-accepted)
  4. STATE -> THINKING appears ONLY AFTER #3 (confirm-gated loading state)
  5. transcript text non-empty
  6. STATE -> SPEAKING + TTS chunks played
  7. STATE -> IDLE (clean return)
  8. NO crash markers anywhere: tlsf|assert|abort|panic|guru|Backtrace|rst:|heap_init
- Prints PASS/FAIL per stage + overall, non-zero exit on FAIL.

## Extra scenarios (same harness, flags)
- `--unreachable`: temporarily point the node at a bad gateway (or block) and
  confirm THINKING never lights and it recovers to idle/error after timeout.
- `--bargein`: streamtest + slam PTT on/off x4, assert clean (no crash markers).

## Definition of done
- `voice e2e` on device drives a full real turn through the gateway.
- `tools/e2e_conversation_test.py` prints PASS on a healthy gateway and
  correctly FAILS when a stage is broken.
- Canned utterance committed (spiffs), harness committed. Local commits only.
- net.c STT/TTS logic still untouched (git diff --stat).
- Checkpoint to ~/.openclaw/workspace/memory/2026-08-15.md.
