#!/usr/bin/env python3
"""
Clock-offset client.

Listens for the UDP broadcast packets sent by the Raspberry Pi server, each of
the form:

    b"0,{t1},{t1_server},{duration}"
        t1         : server monotonic time   (microseconds)  time.monotonic_ns()//1000
        t1_server  : server NTP/wall time     (nanoseconds)   time.time_ns()
        duration   : target period (us)        -- not needed here

For every packet it stamps its OWN monotonic + NTP clock the instant recvfrom()
returns and computes the monotonic offset between the two machines:

        mono_delta = client_mono - t1            =  O + d
        ntp_delta  = (client_ntp - t1_server)    =      d     (NTP clocks ~ equal)
        offset     = mono_delta - ntp_delta      =  O

where O is the monotonic offset (client_mono ~= server_mono + O) and d is the
one-way transit/buffering delay. The same d appears in both deltas, so it (and
most network jitter) cancels. Assumes the two NTP clocks are very close: any
residual NTP error e biases the result to (O - e).

The offset is low-pass filtered (EMA) so it changes smoothly, then published
into a triple-buffered shared-memory segment named "netbuf" that the reader
attaches to. Slot layout:

        offset 0  : "<Q"     current slot index (0/1/2)
        slots     : "<QQddd" at offsets 8, 48, 88 ->
                        server_mono_us       (uint64)  server t1 from the packet
                        client_recv_mono_us  (uint64)  client mono at receive (freshness)
                        offset_smoothed_us   (double)  EMA-smoothed offset
                        offset_raw_us        (double)  latest raw offset
                        delay_us             (double)  per-packet one-way delay
        total size: 128 bytes
"""

import socket
import struct
import sys
from multiprocessing import shared_memory

import time

# ----------------------------- configuration ------------------------------- #
UDP_PORT   = 4210            # must match the server's destination port
SHM_NAME   = "netbuf"        # the reader attaches to this name

# Smoothing: exponential moving average. Packets arrive ~25 Hz (server loop is
# 4 x 10 ms), so the EMA time constant is roughly (1/alpha) packets. Smaller
# alpha -> smoother and slower to react. 0.05 ~= 0.8 s settling.
EMA_ALPHA  = 0.05

# Spike rejection (set OUTLIER_US = None to disable). A raw sample further than
# this from the current smoothed value is treated as a glitch and not folded in
# -- unless MAX_REJECTS such samples occur in a row, in which case we assume the
# offset really has shifted (e.g. an NTP step) and re-baseline onto it.
OUTLIER_US  = 2000.0         # 2 ms
MAX_REJECTS = 15

PRINT_EVERY = True           # print each update to stdout

# --------------------------- shared-memory layout -------------------------- #
INDEX_OFF = 0
SLOT_FMT  = "<QQddd"          # server_mono, client_recv_mono, smoothed, raw, delay
SLOT_SIZE = struct.calcsize(SLOT_FMT)                 # 40
SLOTS_OFF = (8, 8 + SLOT_SIZE, 8 + 2 * SLOT_SIZE)     # (8, 48, 88)
SHM_SIZE  = SLOTS_OFF[-1] + SLOT_SIZE                 # 128


def make_shm():
    """Create the segment (recreating a stale one left by a previous run)."""
    try:
        shm = shared_memory.SharedMemory(name=SHM_NAME, create=True, size=SHM_SIZE)
    except FileExistsError:
        stale = shared_memory.SharedMemory(name=SHM_NAME)
        stale.close()
        stale.unlink()
        shm = shared_memory.SharedMemory(name=SHM_NAME, create=True, size=SHM_SIZE)
    # initialise: index -> slot 0, all slots zeroed (reader sees recv==0 = no data)
    struct.pack_into("<Q", shm.buf, INDEX_OFF, 0)
    for off in SLOTS_OFF:
        struct.pack_into(SLOT_FMT, shm.buf, off, 0, 0, 0.0, 0.0, 0.0)
    return shm


class TripleBuffer:
    """Single-writer lock-free triple buffer matching the reader's index scheme.

    The writer fills a slot that the index is NOT pointing at, then atomically
    moves the index to it. Round-robining over 3 slots means a slot is not
    reused until two further publishes, giving readers ample time to finish a
    read of the previously-published slot without tearing.
    """

    def __init__(self, buf):
        self.buf = buf
        self.published = 0          # slot the shared index currently points at

    def publish(self, server_mono, client_recv_mono, smoothed, raw, delay):
        nxt = (self.published + 1) % 3
        struct.pack_into(SLOT_FMT, self.buf, SLOTS_OFF[nxt],
                         server_mono, client_recv_mono, smoothed, raw, delay)
        struct.pack_into("<Q", self.buf, INDEX_OFF, nxt)   # publish after fill
        self.published = nxt


def compute_offset_us(t1, t1_server, t_recv_mono, t_recv_ntp):
    """Monotonic offset O (us): client_mono ~= server_mono + O, transit cancelled."""
    mono_delta = t_recv_mono - t1                     # us  = O + d
    ntp_delta  = (t_recv_ntp - t1_server) / 1000.0    # us  = d
    return mono_delta - ntp_delta                     # us  = O


def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    # bind to all interfaces so subnet-broadcast packets are delivered
    sock.bind(("", UDP_PORT))

    shm = make_shm()
    tb = TripleBuffer(shm.buf)

    ema = None          # smoothed offset (us)
    reject_streak = 0

    print(f"listening for broadcasts on UDP/{UDP_PORT}, "
          f"publishing offset to shared memory '{SHM_NAME}'")
    try:
        while True:
            data, _addr = sock.recvfrom(1024)
            # stamp our clocks immediately; monotonic first to mirror the server
            t_recv_mono = time.monotonic_ns() // 1000      # us
            t_recv_ntp  = time.time_ns()                   # ns

            # parse "0,t1,t1_server,duration" defensively
            try:
                parts = data.split(b",")
                t1        = int(parts[1])      # server monotonic, us
                t1_server = int(parts[2])      # server NTP, ns
            except (IndexError, ValueError):
                continue                       # ignore malformed packets

            raw      = compute_offset_us(t1, t1_server, t_recv_mono, t_recv_ntp)
            delay_us = (t_recv_ntp - t1_server) / 1000.0   # per-packet one-way delay

            # ---- spike rejection + EMA smoothing (offset only) ----
            if ema is None:
                ema = raw                      # first sample seeds the filter
            elif OUTLIER_US is not None and abs(raw - ema) > OUTLIER_US:
                reject_streak += 1
                if reject_streak >= MAX_REJECTS:
                    ema = raw                  # sustained shift -> re-baseline
                    reject_streak = 0
                # else: drop this sample, keep last smoothed value
            else:
                reject_streak = 0
                ema = (1.0 - EMA_ALPHA) * ema + EMA_ALPHA * raw

            tb.publish(t1, t_recv_mono, ema, raw, delay_us)

            if PRINT_EVERY:
                print(f"server_mono={t1}us  "
                      f"offset(raw)={raw:10.1f}us  "
                      f"offset(smooth)={ema:10.1f}us  "
                      f"delay~{delay_us:7.1f}us"
                      + ("  [spike]" if (OUTLIER_US is not None
                                         and abs(raw - ema) > OUTLIER_US) else ""))
    except KeyboardInterrupt:
        pass
    finally:
        sock.close()
        shm.close()
        try:
            shm.unlink()               # writer owns the segment
        except FileNotFoundError:
            pass


if __name__ == "__main__":
    sys.exit(main())
