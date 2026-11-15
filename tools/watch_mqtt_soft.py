#!/usr/bin/env python3
"""COM12 soft listen (try no DTR/RTS reset) + EMQX age poll. Catch MQTT drop cause."""
from __future__ import annotations

import os
import re
import sys
import time
from datetime import datetime

import serial

PORT = os.environ.get("HEXNET_IO_PORT", "COM12")
BAUD = 115200
LOG = os.path.join(os.path.dirname(__file__), "..", "build", "mqtt_soft_watch.log")
KEYS = re.compile(
    r"HEXNET_MQTT|HEXNET_WIFI|Telemetry|JSON build|disconnected|forced stop|"
    r"restart|iheap|ADIM|BLE basli|WIFI|Brownout|Guru|abort|panic|rst:",
    re.I,
)


def emqx_age() -> str:
    try:
        import paramiko

        passwd = os.environ.get("HEXNET_SSH_PASS") or "1zo11Ki"
        c = paramiko.SSHClient()
        c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        c.connect(
            "185.33.234.10",
            username="root",
            password=passwd,
            timeout=20,
            allow_agent=False,
            look_for_keys=False,
        )
        php = (
            "<?php require_once '/var/www/html/myhexnet/api/_lib/db.php';"
            "$pdo=hexnet_db();"
            "$r=$pdo->query('SELECT last_seen_at,status,TIMESTAMPDIFF(SECOND,last_seen_at,NOW()) age "
            "FROM devices WHERE id=14')->fetch(PDO::FETCH_ASSOC);"
            "echo json_encode($r);"
        )
        sftp = c.open_sftp()
        with sftp.file("/tmp/d14age.php", "w") as f:
            f.write(php)
        sftp.close()
        _, o, _ = c.exec_command("php /tmp/d14age.php; emqx ctl clients list 2>/dev/null | grep -c connected=true || true", timeout=25)
        out = o.read().decode(errors="replace").strip()
        c.close()
        return out.replace("\n", " | ")
    except Exception as e:
        return f"emqx_err={e}"


def main() -> int:
    os.makedirs(os.path.dirname(os.path.abspath(LOG)), exist_ok=True)
    print(f"soft-watch {PORT} -> {LOG}", flush=True)
    ser = serial.Serial()
    ser.port = PORT
    ser.baudrate = BAUD
    ser.timeout = 0.5
    ser.dtr = False
    ser.rts = False
    ser.open()
    # Some FTDI still pulse; note boot markers
    print("serial open (dtr=0 rts=0)", flush=True)
    last_emqx = 0.0
    boot = False
    with open(LOG, "a", encoding="utf-8") as lf:
        lf.write(f"\n===== start {datetime.now().isoformat()} =====\n")
        buf = b""
        t0 = time.time()
        while time.time() - t0 < 1200:  # 20 min
            chunk = ser.read(4096)
            if chunk:
                buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    try:
                        s = line.decode("utf-8", errors="replace").rstrip()
                    except Exception:
                        continue
                    if "rst:" in s.lower() or "boot:" in s.lower() or "app_init" in s:
                        boot = True
                        print(f"BOOT? {s}", flush=True)
                    if KEYS.search(s):
                        stamp = datetime.now().strftime("%H:%M:%S")
                        msg = f"[{stamp}] {s}"
                        print(msg, flush=True)
                        lf.write(msg + "\n")
                        lf.flush()
            now = time.time()
            if now - last_emqx >= 30:
                last_emqx = now
                age = emqx_age()
                msg = f"[{datetime.now().strftime('%H:%M:%S')}] EMQX/DB {age}"
                print(msg, flush=True)
                lf.write(msg + "\n")
                lf.flush()
            time.sleep(0.05)
    ser.close()
    print(f"done boot_seen={boot}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
