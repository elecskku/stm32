#!/usr/bin/env python3
"""
flash_tool.py - PC side of the PC <-> STM32 <-> W25Q32 transfer demo.

  pip install pyserial

Examples (Windows: COM5, Linux: /dev/ttyACM0, macOS: /dev/tty.usbmodemXXXX):
  python flash_tool.py ports
  python flash_tool.py makebmp test.bmp                 # ~3.5 MB test BMP
  python flash_tool.py -p COM5 id
  python flash_tool.py -p COM5 write test.bmp
  python flash_tool.py -p COM5 read restored.bmp
  python flash_tool.py -p COM5 verify
  python flash_tool.py -p COM5 demo test.bmp            # full PC->STM32->Flash->STM32->PC
  python flash_tool.py tobmp photo.jpg photo.bmp        # any photo -> 3~4 MB BMP
  python flash_tool.py compare photo.bmp restored.bmp   # side-by-side + difference map
  python flash_tool.py corrupt photo.bmp broken.bmp     # damaged copy (to show a FAIL case)

  Visual compare needs:  python -m pip install pillow matplotlib
"""
import argparse
import hashlib
import os
import struct
import sys
import time
import zlib

ACK, NAK, CAN = 0x06, 0x15, 0x18
CHUNK = 4096
FLASH_DATA_MAX = 4 * 1024 * 1024 - 4096   # last 4 KB sector holds metadata
DEFAULT_BAUD = 921600


# ------------------------------------------------------------------ serial
def open_port(port, baud):
    import serial
    s = serial.Serial(port, baud, timeout=0.5, write_timeout=5)
    time.sleep(0.1)
    s.reset_input_buffer()
    return s


def read_exact(s, n, timeout, progress=None):
    buf = bytearray()
    deadline = time.time() + timeout
    while len(buf) < n:
        part = s.read(min(n - len(buf), 65536))
        if part:
            buf += part
            deadline = time.time() + timeout      # timeout = max silence
            if progress:
                progress(len(buf), n)
        elif time.time() > deadline:
            raise TimeoutError(f"timeout: got {len(buf)}/{n} bytes")
    return bytes(buf)


def read_byte(s, timeout):
    return read_exact(s, 1, timeout)[0]


_last_bar = [0.0]


def bar(done, total, t0, label):
    now = time.time()
    if done < total and now - _last_bar[0] < 0.2:
        return
    _last_bar[0] = now
    pct = done * 100 // total
    el = time.time() - t0
    speed = done / el / 1024 if el > 0 else 0
    fill = pct // 2
    sys.stdout.write(f"\r  {label} [{'#' * fill}{'.' * (50 - fill)}] {pct:3d}%  "
                     f"{done:,}/{total:,} B  {speed:6.1f} KB/s")
    sys.stdout.flush()
    if done >= total:
        sys.stdout.write("\n")


# ------------------------------------------------------------------ commands
def do_id(s):
    s.reset_input_buffer()
    s.write(b"I")
    r = read_exact(s, 5, 2)
    if r[0] != ACK:
        raise RuntimeError("no ACK for 'I'")
    jedec = (r[1] << 16) | (r[2] << 8) | r[3]
    print(f"  JEDEC ID : 0x{jedec:06X}   SR1 : 0x{r[4]:02X}")
    makers = {0xEF: "Winbond", 0x1C: "EON (cFeon)", 0xC8: "GigaDevice", 0x20: "XMC/Micron",
              0x68: "Boya", 0x85: "Puya", 0x5E: "Zbit", 0xA1: "Fudan"}
    if (jedec & 0xFF) == 0x16 and (jedec >> 16) in makers:
        print(f"  -> {makers[jedec >> 16]} 32 Mbit (4 MB) SPI flash detected  OK")
    elif jedec in (0x000000, 0xFFFFFF):
        print("  -> No response from flash. Check wiring / 3V3 / CS pin / soldering.")
    else:
        print("  -> Unexpected ID (may be a compatible clone).")
    if r[4] & 0x1C:
        print("  -> WARNING: block-protect bits set in SR1, writes may be ignored.")
    return jedec


def do_write(s, data):
    size = len(data)
    if size == 0 or size > FLASH_DATA_MAX:
        raise ValueError(f"file size {size:,} must be 1..{FLASH_DATA_MAX:,} bytes")

    s.reset_input_buffer()
    s.write(b"W" + struct.pack("<I", size))
    print(f"  erasing flash for {size:,} bytes ...", end="", flush=True)
    t = time.time()
    r = read_byte(s, 60)
    if r != ACK:
        raise RuntimeError(f"erase failed / rejected (0x{r:02X})")
    print(f" done ({time.time() - t:.1f} s)")

    nchunks = (size + CHUNK - 1) // CHUNK
    t0 = time.time()
    retries = 0
    for idx in range(nchunks):
        chunk = data[idx * CHUNK:(idx + 1) * CHUNK]
        frame = struct.pack("<IH", idx, len(chunk)) + chunk + struct.pack("<I", zlib.crc32(chunk))
        for attempt in range(8):
            s.write(frame)
            try:
                r = read_byte(s, 3)
            except TimeoutError:
                r = None
            if r == ACK:
                break
            if r == CAN:
                raise RuntimeError(f"flash write/verify FAILED at chunk {idx} (addr 0x{idx * CHUNK:06X})")
            retries += 1
            time.sleep(0.1)
            s.reset_input_buffer()
        else:
            raise RuntimeError(f"chunk {idx}: too many retries")
        bar(min((idx + 1) * CHUNK, size), size, t0, "PC->STM32->Flash")

    r = read_exact(s, 5, 5)
    if r[0] != ACK:
        raise RuntimeError("no final ACK")
    dev_crc = struct.unpack("<I", r[1:])[0]
    elapsed = time.time() - t0
    print(f"  write done in {elapsed:.1f} s, retries={retries}, device CRC32=0x{dev_crc:08X}")
    return dev_crc


def do_read(s):
    s.reset_input_buffer()
    s.write(b"R")
    r = read_byte(s, 3)
    if r == NAK:
        raise RuntimeError("no valid file stored in flash")
    if r != ACK:
        raise RuntimeError(f"unexpected response 0x{r:02X}")
    size, stored_crc = struct.unpack("<II", read_exact(s, 8, 3))
    t0 = time.time()
    data = read_exact(s, size, 3, progress=lambda d, n: bar(d, n, t0, "Flash->STM32->PC"))
    read_crc = struct.unpack("<I", read_exact(s, 4, 3))[0]
    print(f"  read done in {time.time() - t0:.1f} s")
    return data, stored_crc, read_crc


def do_verify(s):
    s.reset_input_buffer()
    s.write(b"V")
    r = read_byte(s, 30)
    if r == NAK:
        raise RuntimeError("no valid file stored in flash")
    size, stored, computed = struct.unpack("<III", read_exact(s, 12, 30))
    ok = stored == computed
    print(f"  size={size:,}  stored CRC=0x{stored:08X}  recomputed CRC=0x{computed:08X}  "
          f"-> {'OK' if ok else 'MISMATCH'}")
    return ok


def do_erase(s):
    s.reset_input_buffer()
    s.write(b"E")
    print("  chip erase (up to ~50 s) ...", flush=True)
    r = read_byte(s, 120)
    print("  done" if r == ACK else f"  failed (0x{r:02X})")


# ------------------------------------------------------------------ BMP generator
def make_bmp(path, width=1280, height=960):
    row = width * 3
    pad = (4 - row % 4) % 4
    img_size = (row + pad) * height
    header = struct.pack("<2sIHHI", b"BM", 54 + img_size, 0, 0, 54)
    dib = struct.pack("<IiiHHIIiiII", 40, width, height, 1, 24, 0, img_size, 2835, 2835, 0, 0)
    out = bytearray(header + dib)
    xs = range(width)
    for y in range(height):                       # bottom-up rows, BGR
        g = y * 255 // (height - 1)
        line = bytearray()
        for x in xs:
            b = x * 255 // (width - 1)
            r = 255 if ((x // 80) + (y // 80)) % 2 else 40
            line += bytes((b, g, r))
        out += line + b"\x00" * pad
    with open(path, "wb") as f:
        f.write(out)
    print(f"  wrote {path}: {width}x{height} 24-bit BMP, {len(out):,} bytes "
          f"({len(out) / 1024 / 1024:.2f} MB)")


# ------------------------------------------------------------------ image helpers (pip install pillow matplotlib)
BMP_TARGET_PIXELS = 1_250_000      # ~3.6 MB as 24-bit BMP (>= 3 MB, < 4 MB flash)


def to_bmp(src, out):
    """Convert any photo (jpg/png/...) to a 24-bit BMP between 3 MB and 4 MB."""
    from PIL import Image
    img = Image.open(src).convert("RGB")
    w, h = img.size
    k = (BMP_TARGET_PIXELS / (w * h)) ** 0.5
    nw, nh = max(1, round(w * k)), max(1, round(h * k))
    img = img.resize((nw, nh), Image.LANCZOS)
    img.save(out, format="BMP")
    size = len(open(out, "rb").read())
    print(f"  wrote {out}: {nw}x{nh} 24-bit BMP, {size:,} bytes ({size / 1024 / 1024:.2f} MB)")


def show_compare(orig_path, rest_path, save_png="compare_result.png", show=True):
    """Original | Restored | Difference map, side by side. Saves a PNG for slides."""
    try:
        from PIL import Image
        import numpy as np
        import matplotlib
        if not show:
            matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("  (visual compare skipped: run  python -m pip install pillow matplotlib)")
        return

    raw_a, raw_b = open(orig_path, "rb").read(), open(rest_path, "rb").read()
    a = np.asarray(Image.open(orig_path).convert("RGB")).astype(np.int16)
    try:
        b = np.asarray(Image.open(rest_path).convert("RGB")).astype(np.int16)
    except Exception:
        b = np.zeros_like(a)                     # restored file not even a valid BMP
    if b.shape != a.shape:
        bb = np.zeros_like(a)
        hh, ww = min(a.shape[0], b.shape[0]), min(a.shape[1], b.shape[1])
        bb[:hh, :ww] = b[:hh, :ww]
        b = bb

    mask = np.any(a != b, axis=2)
    nbad = int(mask.sum())
    total = mask.size
    diff = np.zeros(a.shape, dtype=np.uint8)
    diff[mask] = (255, 0, 0)                     # red = pixel differs, black = identical
    nbyte = sum(1 for x, y in zip(raw_a, raw_b) if x != y) + abs(len(raw_a) - len(raw_b))
    ok = raw_a == raw_b

    fig, ax = plt.subplots(1, 3, figsize=(16, 5.6))
    panels = [
        (a.astype(np.uint8), f"ORIGINAL (PC)\n{os.path.basename(orig_path)}  {len(raw_a):,} B"),
        (b.astype(np.uint8), f"RESTORED (from SPI Flash)\n{os.path.basename(rest_path)}  {len(raw_b):,} B"),
        (diff, f"DIFFERENCE MAP  (red = mismatch)\n{nbad:,} / {total:,} pixels differ"),
    ]
    for axis, (img, title) in zip(ax, panels):
        axis.imshow(img)
        axis.set_title(title, fontsize=11)
        axis.axis("off")
    color = "#1a7f37" if ok else "#c62828"
    fig.suptitle(
        f"{'PASS' if ok else 'FAIL'}  |  CRC32 0x{zlib.crc32(raw_a):08X} vs 0x{zlib.crc32(raw_b):08X}  |  "
        f"SHA-256 {'match' if hashlib.sha256(raw_a).digest() == hashlib.sha256(raw_b).digest() else 'MISMATCH'}  |  "
        f"different bytes: {nbyte:,}",
        fontsize=14, fontweight="bold", color=color)
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    fig.savefig(save_png, dpi=110)
    print(f"  visual compare: {nbad:,} differing pixels, {nbyte:,} differing bytes -> saved {save_png}")
    if show:
        plt.show()
    plt.close(fig)


def corrupt_copy(src, out, nbytes=65536):
    """Damaged copy: invert one 64 KB flash block in the middle (simulates a failed block)."""
    data = bytearray(open(src, "rb").read())
    start = (len(data) // 2) & ~0xFFFF
    for i in range(start, min(len(data), start + nbytes)):
        data[i] ^= 0xFF
    open(out, "wb").write(data)
    print(f"  wrote {out}: {nbytes:,} bytes inverted at flash block 0x{start:06X}")


# ------------------------------------------------------------------ main
def main():
    ap = argparse.ArgumentParser(description="STM32 + W25Q32 BMP transfer tool")
    ap.add_argument("-p", "--port", help="serial port (COM5, /dev/ttyACM0 ...)")
    ap.add_argument("-b", "--baud", type=int, default=DEFAULT_BAUD)
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("ports")
    m = sub.add_parser("makebmp"); m.add_argument("out"); m.add_argument("--w", type=int, default=1280); m.add_argument("--h", type=int, default=960)
    sub.add_parser("id")
    w = sub.add_parser("write"); w.add_argument("file")
    r = sub.add_parser("read"); r.add_argument("out")
    sub.add_parser("verify")
    sub.add_parser("erase")
    d = sub.add_parser("demo"); d.add_argument("file"); d.add_argument("--out", default="restored.bmp")
    d.add_argument("--no-show", action="store_true", help="do not open the comparison window")
    t = sub.add_parser("tobmp"); t.add_argument("src"); t.add_argument("out")
    c = sub.add_parser("compare"); c.add_argument("orig"); c.add_argument("restored")
    k = sub.add_parser("corrupt"); k.add_argument("src"); k.add_argument("out")
    a = ap.parse_args()

    if a.cmd == "ports":
        from serial.tools import list_ports
        for p in list_ports.comports():
            print(f"  {p.device:15s} {p.description}")
        return
    if a.cmd == "makebmp":
        make_bmp(a.out, a.w, a.h)
        return
    if a.cmd == "tobmp":
        to_bmp(a.src, a.out)
        return
    if a.cmd == "compare":
        show_compare(a.orig, a.restored)
        return
    if a.cmd == "corrupt":
        corrupt_copy(a.src, a.out)
        return
    if not a.port:
        ap.error("-p/--port is required")

    try:
        s = open_port(a.port, a.baud)
    except Exception as e:
        sys.exit(f"  ERROR: cannot open {a.port}: {e}")
    try:
        if a.cmd == "id":
            do_id(s)
        elif a.cmd == "write":
            data = open(a.file, "rb").read()
            dev_crc = do_write(s, data)
            print("  CRC check:", "OK" if dev_crc == zlib.crc32(data) else "MISMATCH")
        elif a.cmd == "read":
            data, stored, rcrc = do_read(s)
            open(a.out, "wb").write(data)
            pc = zlib.crc32(data)
            print(f"  saved {a.out} ({len(data):,} B)  stored=0x{stored:08X} "
                  f"flash-read=0x{rcrc:08X} pc=0x{pc:08X} -> "
                  f"{'OK' if stored == rcrc == pc else 'MISMATCH'}")
        elif a.cmd == "verify":
            do_verify(s)
        elif a.cmd == "erase":
            do_erase(s)
        elif a.cmd == "demo":
            run_demo(s, a.file, a.out, show=not a.no_show)
    except (RuntimeError, TimeoutError, ValueError, OSError) as e:
        sys.exit(f"\n  ERROR: {e}")
    finally:
        s.close()


def run_demo(s, path, out_path, show=True):
    src = open(path, "rb").read()
    print("=" * 72)
    print(f" DEMO: PC -> STM32 -> SPI Flash -> STM32 -> PC   ({path}, {len(src):,} bytes)")
    print("=" * 72)
    print("[1] Flash ID (SPI check)")
    do_id(s)
    print("[2] Write file to flash")
    t = time.time()
    dev_crc = do_write(s, src)
    t_w = time.time() - t
    print("[3] Read file back from flash")
    t = time.time()
    back, stored, read_crc = do_read(s)
    t_r = time.time() - t
    open(out_path, "wb").write(back)

    print("[4] Integrity check")
    src_crc, back_crc = zlib.crc32(src), zlib.crc32(back)
    src_sha, back_sha = hashlib.sha256(src).hexdigest(), hashlib.sha256(back).hexdigest()
    first_diff = next((i for i, (x, y) in enumerate(zip(src, back)) if x != y), None)
    same = src == back
    rows = [
        ("Original file size", f"{len(src):,} B"),
        ("Restored file size", f"{len(back):,} B"),
        ("CRC32 original (PC)", f"0x{src_crc:08X}"),
        ("CRC32 received by STM32", f"0x{dev_crc:08X}"),
        ("CRC32 stored in flash", f"0x{stored:08X}"),
        ("CRC32 read from flash", f"0x{read_crc:08X}"),
        ("CRC32 restored (PC)", f"0x{back_crc:08X}"),
        ("SHA-256 original", src_sha[:32] + "..."),
        ("SHA-256 restored", back_sha[:32] + "..."),
        ("Byte-by-byte compare", "identical" if same else f"differs at offset {first_diff}"),
        ("Write / read time", f"{t_w:.1f} s / {t_r:.1f} s"),
    ]
    for k, v in rows:
        print(f"  {k:26s}: {v}")
    ok = same and src_sha == back_sha and len({src_crc, dev_crc, stored, read_crc, back_crc}) == 1
    print("=" * 72)
    print(f" RESULT: {'PASS - file restored without any error' if ok else 'FAIL'}")
    print(f" restored file: {out_path}")
    print("=" * 72)
    print("[5] Visual compare (original | restored | difference)")
    show_compare(path, out_path, show=show)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
