"""
Reader for the clock-offset shared buffer published by clock_offset_client.py.

Attaches (without owning) to the triple-buffered segment "netbuf" and reads the
most recent record. Slot layout produced by the writer:

        offset 0  : "<Q"     current slot index (0/1/2)
        slots     : "<QQddd" at offsets 8, 48, 88 ->
                        server_mono_us       (uint64)  server t1 from the packet
                        client_recv_mono_us  (uint64)  client mono at receive (freshness)
                        offset_smoothed_us   (double)  EMA-smoothed monotonic offset
                        offset_raw_us        (double)  latest raw offset
                        delay_us             (double)  per-packet one-way delay

Reminder on the offset's meaning:  client_mono ~= server_mono + offset
so the server's monotonic clock, as seen right now from the client, is
    server_mono_now ~= client_mono_now - offset_smoothed
"""

import struct
import time
from multiprocessing import resource_tracker, shared_memory

SHM_NAME  = "netbuf"

INDEX_OFF = 0
SLOT_FMT  = "<QQddd"
SLOT_SIZE = struct.calcsize(SLOT_FMT)                 # 40
SLOTS_OFF = (8, 8 + SLOT_SIZE, 8 + 2 * SLOT_SIZE)     # (8, 48, 88)


def attach(name, timeout=None, poll=0.1):
    """Attach to a segment we did NOT create; never unlink it on exit."""
    deadline = None if timeout is None else time.perf_counter() + timeout
    while True:
        try:
            shm = shared_memory.SharedMemory(name=name)
            # the writer owns this segment -- stop the resource tracker from
            # unlinking it when this process exits
            try:
                resource_tracker.unregister(shm._name, "shared_memory")
            except Exception:
                pass
            return shm
        except FileNotFoundError:
            if deadline is not None and time.perf_counter() > deadline:
                raise
            time.sleep(poll)


shm = attach(SHM_NAME)          # blocks until the writer creates "netbuf"
buf = shm.buf


def read_latest():
    """Return the newest fully-written record as a tuple.

    Consistency check: the index changes on every publish, so if it is the same
    before and after reading the slot, no write happened during the read and the
    slot is intact. Otherwise retry. (At the ~40 ms publish rate vs a microsecond
    read this practically never loops.)
    """
    while True:
        idx  = struct.unpack_from("<Q", buf, INDEX_OFF)[0]
        slot = struct.unpack_from(SLOT_FMT, buf, SLOTS_OFF[idx])
        if struct.unpack_from("<Q", buf, INDEX_OFF)[0] == idx:
            return slot


def main():
    try:
        while True:
            (server_mono, client_recv,
             off_smooth, off_raw, delay) = read_latest()

            if client_recv == 0:
                print("waiting for first packet...")
                time.sleep(0.1)
                continue

            now_mono = time.monotonic_ns() // 1000          # client mono, us
            age_ms   = (now_mono - client_recv) / 1000.0    # data freshness
            # live estimate of the server's monotonic clock from our clock
            server_now_est = now_mono - off_smooth

            print(f"server_mono={server_mono:>12d}us | "
                  f"offset={off_smooth:>11.1f}us (raw {off_raw:>11.1f}) | "
                  f"delay={delay:>7.1f}us | "
                  f"age={age_ms:>6.1f}ms | "
                  f"server_now~={server_now_est:>14.0f}us")

            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        shm.close()                # close our mapping; do NOT unlink (writer owns it)


if __name__ == "__main__":
    main()
