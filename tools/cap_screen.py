#!/usr/bin/env python3
"""Grab a c6_sdr screen dump over the console UART and save a PNG.

The firmware emits, on 'c' input or the CAP touch button:
    @@SCR <w> <h> <len>\n  + <len> raw RGB565-LE bytes + crc16 + '\n'

Usage:
    ./cap_screen.py [PORT] [out.png] [--view SPEC|PANO|WFL|IQ]

Sends 'v' to cycle views until --view is reached, settles, sends 'c'.
"""
import re, serial, struct, sys, time, zlib

def crc16(d):
    c = 0xffff
    for b in d:
        c ^= b << 8
        for _ in range(8):
            c = ((c << 1) ^ 0x1021) & 0xffff if c & 0x8000 else (c << 1) & 0xffff
    return c

def save_png(path, raw, w, h):
    rows = bytearray()
    for y in range(h):
        rows.append(0)
        for x in range(w):
            v = raw[(y * w + x) * 2] | (raw[(y * w + x) * 2 + 1] << 8)
            rows += bytes(((v >> 8) & 0xf8, (v >> 3) & 0xfc, (v << 3) & 0xf8))
    def chunk(t, d):
        c = t + d
        return struct.pack('>I', len(d)) + c + struct.pack('>I', zlib.crc32(c))
    with open(path, 'wb') as f:
        f.write(b'\x89PNG\r\n\x1a\n'
                + chunk(b'IHDR', struct.pack('>IIBBBBB', w, h, 8, 2, 0, 0, 0))
                + chunk(b'IDAT', zlib.compress(bytes(rows)))
                + chunk(b'IEND', b''))

def main():
    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB2'
    out = sys.argv[2] if len(sys.argv) > 2 else 'screen.png'
    tgt = None
    if '--view' in sys.argv:
        tgt = sys.argv[sys.argv.index('--view') + 1]
    s = serial.Serial(port, 115200, timeout=1)
    s.reset_input_buffer()
    if tgt:
        cur, t0 = None, time.time()
        while cur != tgt and time.time() - t0 < 90:
            s.write(b'v')
            t = time.time()
            while time.time() - t < 4:
                m = re.search(rb'view: (\w+)', s.readline())
                if m:
                    cur = m.group(1).decode()
                    print('view:', cur)
        if cur != tgt:
            sys.exit('never reached view %s' % tgt)
        time.sleep(6)
    for attempt in range(3):
        s.write(b'c')
        buf, t = b'', time.time()
        while time.time() - t < 90:
            b = s.read(1)
            if not b:
                continue
            buf += b
            if b'@@SCR' in buf:
                hdr = buf[buf.index(b'@@SCR'):]
                nl = hdr.find(b'\n')
                if nl <= 0:
                    continue
                w, h, plen = map(int, hdr[:nl].split()[1:4])
                if len(hdr) < nl + 1 + plen + 2:
                    continue
                payload = hdr[nl + 1:nl + 1 + plen]
                got = hdr[nl + 1 + plen] | (hdr[nl + 2 + plen] << 8)
                if crc16(payload) == got:
                    save_png(out, payload, w, h)
                    print('saved %s (%dx%d)' % (out, w, h))
                    return
                print('crc mismatch, retrying')
                buf = b''
        print('timeout, retrying' if attempt < 2 else 'timeout — giving up')
    sys.exit(1)

if __name__ == '__main__':
    main()
