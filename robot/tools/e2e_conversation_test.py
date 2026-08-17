#!/usr/bin/env python3
"""
Autonomous end-to-end conversation test for the StackChan CoreS3 voice loop.

Drives the firmware `voice e2e` console command, which injects a canned PCM
utterance (spiffs_image/utterance.wav) through the SAME mic_chunk_cb uplink path
the physical mic uses -- so it flows to the REAL OpenClaw gateway as
appendAudio -> transcript -> agent reply -> TTS back down through the speaker.

It scrapes the serial log and asserts the ordered turn sequence, that the
confirm-gated loading LED (STATE -> THINKING) lights ONLY AFTER the gateway
returns a transcript, and that NO crash / false-shutdown markers appear.

Usage:
    PY=/Users/eric/.espressif/python_env/idf5.3_py3.13_env/bin/python
    $PY tools/e2e_conversation_test.py [--port /dev/cu.usbmodem2101]
                                       [--timeout 60] [--attempts 4]
                                       [--warmup 12] [--bargein]

Exit code 0 = PASS, non-zero = FAIL.
"""
import argparse
import re
import sys
import time

try:
    import serial  # pyserial
except ImportError:
    sys.stderr.write(
        "pyserial not found. Run with the idf python env:\n"
        "  /Users/eric/.espressif/python_env/idf5.3_py3.13_env/bin/python "
        "tools/e2e_conversation_test.py\n")
    sys.exit(2)

DEFAULT_PORT = "/dev/cu.usbmodem2101"
BAUD = 115200

# Crash / false-shutdown markers. Absence is required in the post-command window.
CRASH_RE = re.compile(
    r"tlsf|assert|abort|panic|guru|Backtrace|rst:|heap_init|"
    r"power key SHORT press|graceful shutdown|AXP2101 soft power-off",
    re.IGNORECASE)

# Confirm-gated loading-state token: printed ONLY inside CMD_THINKING_ON, which
# only fires after the gateway returns a transcript (turn_flush hook).
THINKING_RE = re.compile(r"STATE -> THINKING")
LISTENING_RE = re.compile(r"STATE -> LISTENING")
SPEAKING_RE = re.compile(r"STATE -> SPEAKING")
IDLE_RE = re.compile(r"STATE -> IDLE")
APPEND_OK_RE = re.compile(r"appendAudio ok")
SESSION_OPEN_RE = re.compile(r"talk transcription session open")
CONNECTED_RE = re.compile(r"OpenClaw session connected|gateway handshake complete")
# transcript confirm: either the buffered STT final or the whole-turn inject.
TRANSCRIPT_RE = re.compile(r'(?:STT final \(buffered\)|turn flush -> inject full turn):\s*"([^"]*)"')
# TTS playback evidence: the assistant reply audio landing on the speaker, via
# either the talk.speak one-shot path or the streamed playStream path.
TTS_RE = re.compile(r"talk\.speak audio:|-> speaker|stream begin @|playStream: streamed")
FAIL_TOKEN_RE = re.compile(r"E2E: FAIL (\S+)")


def open_port(port):
    """Open the serial port WITHOUT triggering a reset (preserve the live
    gateway connection). ESP32-S3 native USB-CDC ignores DTR/RTS for reset, but
    we clear them defensively anyway."""
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = BAUD
    ser.timeout = 0.2
    try:
        ser.dtr = False
        ser.rts = False
    except Exception:
        pass
    ser.open()
    return ser


def send(ser, line):
    ser.reset_input_buffer()
    ser.write((line + "\r\n").encode())
    ser.flush()


def collect(ser, seconds, sink, stop_on=None):
    """Read lines for up to `seconds`, append decoded lines to `sink`.
    Returns early if `stop_on` regex matches a line."""
    end = time.time() + seconds
    buf = b""
    while time.time() < end:
        chunk = ser.read(4096)
        if chunk:
            buf += chunk
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                line = raw.decode("utf-8", "replace").rstrip("\r")
                if line:
                    sink.append(line)
                    if stop_on is not None and stop_on.search(line):
                        return
        else:
            time.sleep(0.02)


def first_index(lines, rx):
    for i, ln in enumerate(lines):
        if rx.search(ln):
            return i
    return -1


def run_attempt(ser, timeout):
    """Run one `voice e2e` turn. Returns (lines, stages dict)."""
    lines = []
    send(ser, "voice e2e")
    # Scrape until IDLE (turn complete) or timeout.
    collect(ser, timeout, lines, stop_on=IDLE_RE)
    # brief tail read to catch anything trailing IDLE
    collect(ser, 1.5, lines)
    return lines


def evaluate(lines):
    """Compute ordered-stage pass/fail. Returns (ok, results, transcript)."""
    results = []  # (stage_name, ok, detail)

    i_listen = first_index(lines, LISTENING_RE)
    i_append = first_index(lines, APPEND_OK_RE)
    i_session = first_index(lines, SESSION_OPEN_RE)
    i_confirm = first_index(lines, TRANSCRIPT_RE)
    i_think = first_index(lines, THINKING_RE)
    i_speak = first_index(lines, SPEAKING_RE)
    i_tts = first_index(lines, TTS_RE)
    i_idle = first_index(lines, IDLE_RE)

    # firmware-side hard fail token (e.g. talk disabled / load failure)
    fw_fail = None
    for ln in lines:
        m = FAIL_TOKEN_RE.search(ln)
        if m:
            fw_fail = m.group(1)
            break

    transcript = ""
    for ln in lines:
        m = TRANSCRIPT_RE.search(ln)
        if m:
            transcript = m.group(1)
            break

    # Stage 1: LISTENING
    results.append(("1 LISTENING", i_listen >= 0,
                    "STATE -> LISTENING" if i_listen >= 0 else "missing"))

    # Stage 2: session open + appendAudio ok (>=1)
    results.append(("2 appendAudio ok", i_append >= 0,
                    ("session_open + appendAudio ok" if i_append >= 0
                     else ("session opened but no appendAudio" if i_session >= 0
                           else "no gateway session (gateway unreachable?)"))))

    # Stage 3: gateway confirm (transcript came back)
    results.append(("3 gateway confirm (transcript)", i_confirm >= 0,
                    ('"%s"' % transcript) if i_confirm >= 0 else "no transcript from gateway"))

    # Stage 4: THINKING only AFTER confirm (confirm-gated loading state)
    if i_think < 0:
        s4 = (False, "THINKING never lit")
    elif i_confirm < 0:
        s4 = (False, "THINKING lit but no confirm")
    elif i_think > i_confirm:
        s4 = (True, "THINKING after confirm (gated OK)")
    else:
        s4 = (False, "THINKING lit BEFORE confirm (gate broken!)")
    results.append(("4 THINKING confirm-gated", s4[0], s4[1]))

    # Stage 5: transcript non-empty
    results.append(("5 transcript non-empty", bool(transcript.strip()),
                    ('"%s"' % transcript) if transcript.strip() else "empty"))

    # Stage 6: SPEAKING + TTS chunks
    results.append(("6 SPEAKING + TTS", i_speak >= 0 and i_tts >= 0,
                    "SPEAKING + TTS audio" if (i_speak >= 0 and i_tts >= 0)
                    else "no playback (speak=%d tts=%d)" % (i_speak, i_tts)))

    # Stage 7: IDLE (clean return), after SPEAKING
    results.append(("7 IDLE (clean return)", i_idle >= 0 and (i_speak < 0 or i_idle > i_speak),
                    "returned to IDLE" if i_idle >= 0 else "never returned to IDLE"))

    # Stage 8: no crash markers
    crashes = [ln for ln in lines if CRASH_RE.search(ln)]
    results.append(("8 no crash/false-shutdown", len(crashes) == 0,
                    "clean" if not crashes else ("FOUND: " + " | ".join(crashes[:4]))))

    ok = all(r[1] for r in results) and fw_fail is None
    if fw_fail:
        results.insert(0, ("0 firmware", False, "E2E: FAIL " + fw_fail))
    return ok, results, transcript


def bargein_scan(ser, timeout):
    """--bargein: streamtest + slam PTT on/off x4, assert no crash markers."""
    lines = []
    print("[bargein] streamtest 24000 + rapid voice on/off x4")
    send(ser, "voice streamtest 24000")
    time.sleep(0.6)
    for _ in range(4):
        send(ser, "voice on")
        time.sleep(0.25)
        send(ser, "voice off")
        time.sleep(0.25)
    collect(ser, timeout, lines)
    crashes = [ln for ln in lines if CRASH_RE.search(ln)]
    ok = len(crashes) == 0
    print("[bargein] %s" % ("PASS (no crash markers)" if ok
                            else "FAIL: " + " | ".join(crashes[:6])))
    return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=DEFAULT_PORT)
    ap.add_argument("--timeout", type=float, default=60.0,
                    help="seconds to scrape one turn")
    ap.add_argument("--attempts", type=int, default=4,
                    help="retries to allow the gateway connection to come up")
    ap.add_argument("--warmup", type=float, default=12.0,
                    help="seconds to wait after opening the port (gateway connect)")
    ap.add_argument("--bargein", action="store_true",
                    help="run the barge-in crash-safety scenario instead")
    args = ap.parse_args()

    try:
        ser = open_port(args.port)
    except Exception as e:
        print("FAIL: cannot open %s: %s" % (args.port, e))
        return 3

    try:
        # Opening the port resets the ESP32-S3, so wait for the gateway to
        # (re)connect before driving a turn. Fall back to a fixed drain if the
        # connected token isn't seen within the window.
        print("[warmup] waiting up to %.0fs for gateway connect ..." % max(args.warmup, 50))
        drain = []
        collect(ser, max(args.warmup, 50.0), drain, stop_on=CONNECTED_RE)
        connected = any(CONNECTED_RE.search(ln) for ln in drain)
        print("[warmup] connected=%s (%d boot lines)" % (connected, len(drain)))
        time.sleep(3)  # settle after connect

        # Confirm talk mode is enabled (required for the gateway loop).
        send(ser, "voice talk-status")
        st = []
        collect(ser, 2.0, st)
        talk_on = any("talk mode enabled=1" in ln for ln in st)
        if not talk_on:
            print("FAIL: talk mode is DISABLED. Run `voice talk-enable` + reboot, "
                  "then approve the node re-pair. (saw: %s)"
                  % next((ln for ln in st if "talk mode enabled" in ln), "?"))
            return 4

        if args.bargein:
            return 0 if bargein_scan(ser, 8.0) else 1

        best = None
        for attempt in range(1, args.attempts + 1):
            print("\n=== e2e attempt %d/%d ===" % (attempt, args.attempts))
            lines = run_attempt(ser, args.timeout)
            ok, results, transcript = evaluate(lines)
            best = (ok, results, transcript, lines)
            # If the gateway session never opened, the connection likely isn't
            # up yet -> wait and retry. Any other failure is real.
            session_up = first_index(lines, SESSION_OPEN_RE) >= 0 or \
                first_index(lines, APPEND_OK_RE) >= 0
            if ok:
                break
            if not session_up and attempt < args.attempts:
                print("[retry] no gateway session yet; waiting 10s ...")
                time.sleep(10)
                continue
            break

        ok, results, transcript, lines = best
        print("\n---- E2E STAGE RESULTS ----")
        for name, good, detail in results:
            print("  [%s] %-30s %s" % ("PASS" if good else "FAIL", name, detail))
        print("---------------------------")
        print("TRANSCRIPT: %r" % transcript)
        print("OVERALL: %s" % ("PASS" if ok else "FAIL"))
        if not ok:
            print("\n--- last 40 serial lines ---")
            for ln in lines[-40:]:
                print("  " + ln)
        return 0 if ok else 1
    finally:
        try:
            ser.close()
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main())
