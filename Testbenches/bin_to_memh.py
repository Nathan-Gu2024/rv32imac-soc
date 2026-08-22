#!/usr/bin/env python3
# Converts a flat little-endian .bin (as produced by objcopy -O binary) into
# a $readmemh-compatible word file, one 32-bit little-endian word per line,
# starting at a given word offset within the target array (padded with
# zeros for any earlier words) - so it can be loaded directly with
# $readmemh(file, mem_array) into a flat 32-bit-wide memory.
import sys

bin_path, out_path, byte_addr_str = sys.argv[1], sys.argv[2], sys.argv[3]
byte_addr = int(byte_addr_str, 0)
assert byte_addr % 4 == 0
word_offset = byte_addr // 4

with open(bin_path, "rb") as f:
    data = f.read()

# Pad to a multiple of 4 bytes
if len(data) % 4 != 0:
    data += b"\x00" * (4 - (len(data) % 4))

with open(out_path, "w") as f:
    f.write(f"@{word_offset:08x}\n")
    for i in range(0, len(data), 4):
        word = int.from_bytes(data[i:i+4], "little")
        f.write(f"{word:08x}\n")

print(f"Wrote {len(data)//4} words starting at word offset 0x{word_offset:x} (byte addr 0x{byte_addr:x})")
