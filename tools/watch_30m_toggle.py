#!/usr/bin/env python3
"""30 dk: IO+Display serial + MQTT. Platform (MQTT) ve Display (UART relay) ac/kapa."""
from __future__ import annotations

import json
import re
import threading
import time
from datetime import datetime
from pathlib import Path

import serial
import paho.mqtt.client as mqtt

IO_PORT = "COM12"
DISP_PORT = "COM16"
BAUD = 115200
DURATION_S = 30 * 60
TOGGLE_EVERY_S = 90  # her kaynak ~90s (display/platform arasinda 45s kayma)
BROKER = "185.33.234.10"
CMD_TOPIC = "hexnet/v1/2/14/command"
REPLY_TOPIC = "hexnet/v1/2/14/command_reply"
TELE_TOPIC = "hexnet/v1/2/14/telemetry"
RELAY_IDX = 0

OUT = Path(__file__).resolve().parents[1] / "build" / "watch_30m_toggle.log"
ANSI = re.compile(r"\x1b\[[0-9;]*m")

BAD = re.compile(
    r"ack TIMEOUT|DISCONNECTED|MQTT_EVENT_DISCONNECTED|Error sending ping|"
    r"Guru Meditation|InstructionFetch|LoadProhibited|StoreProhibited|"
    r"brownout|abort\(|TWDT|IWDT|task_wdt|panic|rst:0x|Rebooting|"
    r"iheap=1[6]?[^0-9]|iheap=0\b|Malloc failed|malloc failed|"
    r"JSON build FAIL|forced stop|MQTT restart",
    re.I,
)
KEEP = re.compile(
    r"Platform komut|relay_bit|command_reply|Telemetry OK|MQTT connected|"
    r"Setting output|0x720|test relay|CAN Status|Wi-Fi|WIFI|iheap=|"
    r"RX 0x720|simoutput|DISCONNECTED|ack TIMEOUT|ping|reboot|panic|"
    r"Guru|malloc|channels deferred|MQTT restart",
    re.I,
)


def now() -> str:
    return datetime.now().strftime("%H:%M:%S")


def strip(s: str) -> str:
    return ANSI.sub("", s)


class Stats:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.io_lines = 0
        self.disp_lines = 0
        self.tele = 0
        self.replies_ok = 0
        self.replies_fail = 0
        self.platform_tx = 0
        self.display_tx = 0
        self.bad_events: list[str] = []
        self.last_tele_ts = 0.0
        self.last_iheap: int | None = None
        self.min_iheap: int | None = None
        self.platform_ok = 0
        self.display_ok = 0
        self.platform_fail = 0
        self.display_fail = 0


def open_soft(port: str) -> serial.Serial:
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = BAUD
    ser.timeout = 0.2
    ser.dtr = False
    ser.rts = False
    ser.open()
    ser.dtr = False
    ser.rts = False
    return ser


def reader(tag: str, ser: serial.Serial, stop: threading.Event, st: Stats, lf) -> None:
    buf = b""
    while not stop.is_set():
        try:
            chunk = ser.read(8192)
        except Exception as e:
            msg = f"[{now()}] {tag} READ_ERR {e}"
            print(msg, flush=True)
            lf.write(msg + "\n")
            break
        if not chunk:
            continue
        buf += chunk
        while b"\n" in buf:
            raw, buf = buf.split(b"\n", 1)
            text = strip(raw.decode("utf-8", "replace")).rstrip()
            if not text:
                continue
            with st.lock:
                if tag == "IO":
                    st.io_lines += 1
                else:
                    st.disp_lines += 1
                m = re.search(r"iheap=(\d+)", text)
                if m:
                    ih = int(m.group(1))
                    st.last_iheap = ih
                    if st.min_iheap is None or ih < st.min_iheap:
                        st.min_iheap = ih
            if BAD.search(text):
                ev = f"[{now()}] BAD {tag}: {text[:200]}"
                with st.lock:
                    st.bad_events.append(ev)
                print(ev, flush=True)
                lf.write(ev + "\n")
                lf.flush()
            elif KEEP.search(text):
                if "TX_ECHO" in text:
                    continue
                line = f"[{now()}] {tag}: {text[:200]}"
                print(line, flush=True)
                lf.write(line + "\n")


def main() -> int:
    OUT.parent.mkdir(parents=True, exist_ok=True)
    stop = threading.Event()
    st = Stats()
    t0 = time.time()

    print(f"[{now()}] watch_30m start -> {OUT}", flush=True)
    with open(OUT, "w", encoding="utf-8") as lf:
        lf.write(f"===== start {datetime.now().isoformat()} =====\n")

        try:
            io = open_soft(IO_PORT)
            print(f"[{now()}] IO {IO_PORT} open", flush=True)
        except Exception as e:
            print(f"IO OPEN FAIL: {e}", flush=True)
            return 2
        try:
            disp = open_soft(DISP_PORT)
            print(f"[{now()}] DISP {DISP_PORT} open", flush=True)
        except Exception as e:
            print(f"DISP OPEN FAIL: {e}", flush=True)
            io.close()
            return 3

        th_io = threading.Thread(target=reader, args=("IO", io, stop, st, lf), daemon=True)
        th_disp = threading.Thread(target=reader, args=("DISP", disp, stop, st, lf), daemon=True)
        th_io.start()
        th_disp.start()

        replies: list[str] = []
        pending_rid: dict[str, float] = {}

        def on_msg(_c, _u, m):
            p = m.payload.decode("utf-8", "replace")
            with st.lock:
                if m.topic == TELE_TOPIC:
                    st.tele += 1
                    st.last_tele_ts = time.time()
                elif m.topic == REPLY_TOPIC:
                    replies.append(p)
                    ok = '"ok":true' in p or '"ok": true' in p
                    if ok:
                        st.replies_ok += 1
                    else:
                        st.replies_fail += 1
                    for rid, ts in list(pending_rid.items()):
                        if rid in p:
                            pending_rid.pop(rid, None)
                            age = time.time() - ts
                            msg = f"[{now()}] MQTT reply rid={rid} ok={ok} age={age:.2f}s {p[:120]}"
                            print(msg, flush=True)
                            lf.write(msg + "\n")

        try:
            c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        except Exception:
            c = mqtt.Client()
        c.on_message = on_msg
        c.connect(BROKER, 1883, 30)
        c.subscribe([(REPLY_TOPIC, 1), (TELE_TOPIC, 0)])
        c.loop_start()

        # warmup: telemetri gelsin
        warm_deadline = time.time() + 40
        while time.time() < warm_deadline and not stop.is_set():
            with st.lock:
                if st.tele > 0:
                    break
            time.sleep(0.2)
        with st.lock:
            print(f"[{now()}] warmup tele={st.tele} iheap={st.last_iheap}", flush=True)

        next_platform = time.time() + 5
        next_display = time.time() + 50
        platform_on = True
        display_on = True
        last_tick = 0

        while time.time() - t0 < DURATION_S and not stop.is_set():
            now_t = time.time()
            elapsed = int(now_t - t0)

            # progress tick every 5 min
            if elapsed // 300 > last_tick:
                last_tick = elapsed // 300
                with st.lock:
                    tele_age = (now_t - st.last_tele_ts) if st.last_tele_ts else -1
                    tick = (
                        f"[{now()}] TICK {elapsed//60}m tele={st.tele} tele_age={tele_age:.0f}s "
                        f"plat_ok={st.platform_ok}/{st.platform_tx} disp_ok={st.display_ok}/{st.display_tx} "
                        f"reply_ok={st.replies_ok} bad={len(st.bad_events)} "
                        f"iheap={st.last_iheap} min_iheap={st.min_iheap}"
                    )
                print(tick, flush=True)
                lf.write(tick + "\n")
                lf.flush()
                print(f"AGENT_LOOP_TICK_watch30m {{\"elapsed_min\":{elapsed//60}}}", flush=True)

            if now_t >= next_platform:
                rid = f"plat-{int(now_t)}-{1 if platform_on else 0}"
                payload = json.dumps(
                    {
                        "cmd": "relay_bit",
                        "args": {"index": RELAY_IDX, "on": bool(platform_on)},
                        "request_id": rid,
                    }
                )
                pending_rid[rid] = now_t
                with st.lock:
                    st.platform_tx += 1
                inf = c.publish(CMD_TOPIC, payload, qos=1)
                try:
                    inf.wait_for_publish(timeout=4)
                except Exception:
                    pass
                msg = f"[{now()}] PLATFORM TX relay={RELAY_IDX} on={platform_on} rid={rid}"
                print(msg, flush=True)
                lf.write(msg + "\n")
                # wait reply window
                wait_end = time.time() + 6
                got = False
                while time.time() < wait_end:
                    if rid not in pending_rid:
                        got = True
                        break
                    time.sleep(0.05)
                with st.lock:
                    if got:
                        st.platform_ok += 1
                    else:
                        st.platform_fail += 1
                        ev = f"[{now()}] BAD PLATFORM: no command_reply rid={rid}"
                        st.bad_events.append(ev)
                        print(ev, flush=True)
                        lf.write(ev + "\n")
                platform_on = not platform_on
                next_platform = time.time() + TOGGLE_EVERY_S

            if now_t >= next_display:
                cmd = f"relay {RELAY_IDX} {1 if display_on else 0}\n"
                with st.lock:
                    st.display_tx += 1
                try:
                    disp.write(cmd.encode())
                    disp.flush()
                except Exception as e:
                    with st.lock:
                        st.display_fail += 1
                        ev = f"[{now()}] BAD DISP TX: {e}"
                        st.bad_events.append(ev)
                    print(ev, flush=True)
                    lf.write(ev + "\n")
                else:
                    msg = f"[{now()}] DISPLAY TX {cmd.strip()}"
                    print(msg, flush=True)
                    lf.write(msg + "\n")
                    # IO'da Setting output / 0x720 bekle
                    # (reader log'a yazar; burada kisa bekleme)
                    time.sleep(2.5)
                    with st.lock:
                        # kabaca: son bad yoksa ve telemetri canliysa OK say
                        tele_age = (time.time() - st.last_tele_ts) if st.last_tele_ts else 999
                        if tele_age < 20:
                            st.display_ok += 1
                        else:
                            st.display_fail += 1
                            ev = f"[{now()}] BAD DISPLAY: telemetry stale after relay age={tele_age:.0f}s"
                            st.bad_events.append(ev)
                            print(ev, flush=True)
                            lf.write(ev + "\n")
                display_on = not display_on
                next_display = time.time() + TOGGLE_EVERY_S

            # telemetri kopma
            with st.lock:
                if st.last_tele_ts and (now_t - st.last_tele_ts) > 45:
                    ev = f"[{now()}] BAD PLATFORM/IO: telemetry gap {now_t - st.last_tele_ts:.0f}s"
                    if not st.bad_events or "telemetry gap" not in st.bad_events[-1]:
                        st.bad_events.append(ev)
                        print(ev, flush=True)
                        lf.write(ev + "\n")

            time.sleep(0.2)

        stop.set()
        time.sleep(0.5)
        c.loop_stop()
        c.disconnect()
        try:
            io.close()
        except Exception:
            pass
        try:
            disp.close()
        except Exception:
            pass

        with st.lock:
            summary = {
                "elapsed_s": int(time.time() - t0),
                "io_lines": st.io_lines,
                "disp_lines": st.disp_lines,
                "telemetry": st.tele,
                "platform_tx": st.platform_tx,
                "platform_ok": st.platform_ok,
                "platform_fail": st.platform_fail,
                "display_tx": st.display_tx,
                "display_ok": st.display_ok,
                "display_fail": st.display_fail,
                "replies_ok": st.replies_ok,
                "replies_fail": st.replies_fail,
                "last_iheap": st.last_iheap,
                "min_iheap": st.min_iheap,
                "bad_count": len(st.bad_events),
                "bad_events": st.bad_events[-30:],
            }
        print("===== SUMMARY =====", flush=True)
        print(json.dumps(summary, indent=2, ensure_ascii=False), flush=True)
        lf.write("===== SUMMARY =====\n")
        lf.write(json.dumps(summary, indent=2, ensure_ascii=False) + "\n")
        problem = (
            st.platform_fail > 0
            or st.display_fail > 0
            or st.replies_fail > 0
            or len(st.bad_events) > 0
            or (st.min_iheap is not None and st.min_iheap < 2048)
        )
        verdict = "PROBLEM" if problem else "OK"
        print(f"VERDICT={verdict}", flush=True)
        lf.write(f"VERDICT={verdict}\n")
        print("AGENT_LOOP_TICK_watch30m_done", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
