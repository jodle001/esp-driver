#!/usr/bin/env python3
"""End to end: the real operator drives the firmware core's Eshed-dialect front-end
(scorbot_core/acl) behind core_sim's pseudo-terminal, while this script plays commander
over the ICD on TCP port 61616. It checks what the operator sees on that port: ACKs, the
TELP telemetry it parses out of LISTPV replies, and the FAULT lines it forwards.

    make -C test/host e2e OPERATOR=/path/to/erv

The operator is built out of tree so reference_only stays untouched:

    cmake -S reference_only/operator -B /tmp/opbuild -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
    make -C /tmp/opbuild erv

Short paths only: the operator keeps its device path in 32 bytes and its config path in
a fixed buffer, so the config and log live under /tmp/talos-e2e.
"""
import argparse
import os
import socket
import struct
import subprocess
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
LINK = "/tmp/scorbot_core"
WORK = "/tmp/talos-e2e"

ap = argparse.ArgumentParser()
ap.add_argument("--operator", default=os.environ.get("OPERATOR", "/tmp/opbuild/bin/erv"), help="the operator's erv binary")
ap.add_argument("--core-sim", default=os.path.join(HERE, "build", "core_sim"))
ap.add_argument("--shim", default=os.path.join(HERE, "build", "ptyshim.so"))
ap.add_argument("--speed", default="2", help="core_sim time scale (homing takes 15 s of core time)")
args = ap.parse_args()
for path in (args.operator, args.core_sim, args.shim):
    if not os.path.exists(path):
        sys.exit(f"missing: {path}")
os.makedirs(WORK, exist_ok=True)
with open(f"{WORK}/op.conf", "w") as f:
    f.write(f"log_loc: {WORK}/op.log\nscorbot_dev: {LINK}\nsocket_address: 127.0.0.1\nsocket_port: 61616\n")

sim = subprocess.Popen([args.core_sim, "--acl", "--link", LINK, "--speed", args.speed, "--quiet"], stdin=subprocess.PIPE,
                       stdout=open(f"{WORK}/core_sim.out", "w"), stderr=subprocess.STDOUT, text=True)
time.sleep(0.5)
op = subprocess.Popen([args.operator, f"{WORK}/op.conf"], env=dict(os.environ, LD_PRELOAD=args.shim),
                      stdout=open(f"{WORK}/op.out", "w"), stderr=subprocess.STDOUT)
time.sleep(1.0)
if op.poll() is not None:
    sys.exit(f"the operator exited at start, see {WORK}/op.out")

sock = None
for _ in range(50):
    try:
        sock = socket.create_connection(("127.0.0.1", 61616), timeout=1)
        break
    except OSError:
        time.sleep(0.2)
if not sock:
    sys.exit("no operator socket on 61616")
sock.settimeout(0.2)
rx, rx_lock, stop = [], threading.Lock(), False


def reader():
    buf = b""
    while not stop:
        try:
            d = sock.recv(4096)
        except socket.timeout:
            continue
        except OSError:
            break
        if not d:
            break
        buf += d
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            line = line.strip(b"\r\0")
            if line:
                with rx_lock:
                    rx.append((time.time(), line.decode(errors="replace")))


threading.Thread(target=reader, daemon=True).start()
msg_id = [0]


def send(cmd, payload=b""):
    """One ICD packet: big-endian header, payload, XOR byte."""
    msg_id[0] += 1
    pkt = struct.pack(">IHHH", msg_id[0], 0, cmd, len(payload)) + payload
    crc = 0
    for b in pkt:
        crc ^= b
    sock.sendall(pkt + bytes([crc]))


def hw(sub, payload=b""):
    send(0x0008, struct.pack(">BI", sub, 0) + payload)


def lines_since(t0, prefix):
    with rx_lock:
        return [l for (t, l) in rx if t >= t0 and l.startswith(prefix)]


def last_telp(t0):
    tl = lines_since(t0, "TELP")
    return [int(x) for x in tl[-1].split()[1:6]] if tl else None


def wait_telp(t0, timeout, pred=lambda v: True):
    end = time.time() + timeout
    while time.time() < end:
        v = last_telp(t0)
        if v is not None and pred(v):
            return v
        time.sleep(0.1)
    return None


ok = True


def check(cond, what):
    global ok
    print(("  ok   " if cond else "  FAIL ") + what)
    ok = ok and cond


send(0x0000)  # handshake
time.sleep(0.5)
with rx_lock:
    acks = [l for (t, l) in rx if l == "ACK"]
check(len(acks) >= 1, "ACK to the handshake")
print("waiting for the operator's start-up HOME and its first telemetry (it settles 10 s)...")
t0 = time.time()
v = wait_telp(t0, 60)
check(v is not None, f"telemetry after the start-up home: {v}")
base0 = v[0] if v else 0

# TrackingJog: the operator enters manual mode and sends 'q' every 10 ms until the jog times out
t1 = time.time()
hw(0x06, struct.pack(">bbB", 1, 0, 10))
time.sleep(3.0)
v = wait_telp(t1 + 1.0, 10)
check(v is not None and v[0] > base0 + 10, f"base moved by a tracking jog: {base0} -> {v}")

# JointMoveRelative: HERE, SHIFT x3, clrbuf, MOVE
v0 = last_telp(0)
t2 = time.time()
hw(0x03, struct.pack(">iii", 150, -60, 0))
v = wait_telp(t2 + 0.5, 15, lambda w: abs(w[1] - (v0[1] + 150)) <= 4 and abs(w[2] - (v0[2] - 60)) <= 4)
check(v is not None, f"JointMoveRelative reached shoulder +150, elbow -60: {v0} -> {v if v else last_telp(t2)}")

hw(0x05, bytes([50]))  # SetSpeedPercent -> SPEED 50
time.sleep(0.6)
hw(0x04)  # EnableControl -> CON
time.sleep(0.6)

# the e-stop: core_sim drops the motor rail; the operator forwards the controller's fault line
t3 = time.time()
sim.stdin.write("power off\n")
sim.stdin.flush()
time.sleep(2.0)
faults = lines_since(t3, "FAULT")
check(any("CONTROL DISABLED" in f for f in faults), f"e-stop reported as FAULT with CONTROL DISABLED: {faults[:2]}")
t4 = time.time()
hw(0x04)  # CON while the chain is open: refused
time.sleep(1.5)
faults = lines_since(t4, "FAULT")
check(any("CONTROL DISABLED" in f for f in faults), f"CON refused while the power is off: {faults[:2]}")
sim.stdin.write("power on\n")
sim.stdin.flush()
time.sleep(0.5)
t5 = time.time()
hw(0x04)  # CON after MOTORS ON: accepted
time.sleep(1.5)
check(not lines_since(t5, "FAULT"), f"CON accepted after MOTORS ON: {lines_since(t5, 'FAULT')[:2]}")
v = wait_telp(t5, 10)
check(v is not None, f"telemetry back after CON: {v}")

# a jog the other way, ended by PolarPanStop
t6 = time.time()
vb = last_telp(0)
hw(0x06, struct.pack(">bbB", -1, 0, 20))
time.sleep(0.3)
hw(0x06, struct.pack(">bbB", -1, 0, 20))
time.sleep(0.3)
send(0x0004)
time.sleep(2.5)
v = wait_telp(t6 + 1.5, 10)
check(v is not None and v[0] < vb[0] - 5, f"base jogged the other way, then stopped: {vb} -> {v}")

# commander's Home over the ICD: DEFP, HOME, HERE again (a full-range search on the elbow)
t7 = time.time()
send(0x0002, struct.pack(">I", 0))
time.sleep(1.0)
v = wait_telp(t7 + 2.0, 90)
check(v is not None and all(abs(x) < 80 for x in v), f"ICD Home re-homed, positions near the home pose: {v}")

stop = True
sim.stdin.write("status\nquit\n")
sim.stdin.flush()
op.terminate()
op.wait(5)
sim.wait(5)
print("RESULT:", "PASS" if ok else "FAIL", f"(the operator's byte log: {WORK}/op.log, grep 'ACL [TR]X')")
sys.exit(0 if ok else 1)
