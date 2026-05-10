#!/usr/bin/env python3
"""Splice our patched SPL into Armbian's u-boot binary, preserving Armbian's ATF/main U-Boot."""
import struct, shutil, os

spl_path = '/home/rosie/u-boot/spl/sunxi-spl.bin'
armbian_path = '/usr/lib/linux-u-boot-current-orangepizero2w/u-boot-sunxi-with-spl.bin'
output_path = '/home/rosie/u-boot-patched-spl.bin'

with open(spl_path, 'rb') as f:
    our_spl = f.read()

with open(armbian_path, 'rb') as f:
    armbian = f.read()

# Parse eGON headers
def parse_egon(data, label):
    magic = data[4:12]
    spl_size = struct.unpack('<I', data[16:20])[0]
    print(f'{label}: magic={magic!r}, spl_size={spl_size} ({spl_size//1024}KB), total={len(data)} bytes')
    return spl_size

our_spl_size = parse_egon(our_spl, 'Our SPL    ')
armbian_spl_size = parse_egon(armbian, 'Armbian SPL')

# Sanity check: our SPL should fit within the Armbian SPL area
if our_spl_size > armbian_spl_size:
    print(f'ERROR: Our SPL ({our_spl_size}) is larger than Armbian SPL area ({armbian_spl_size})')
    exit(1)

# Splice: take our SPL for the SPL region, then Armbian for everything after
# The SPL is at the beginning of the binary, and the main U-Boot follows
# We need to place our SPL in the same-size slot as the Armbian SPL

# Round up our SPL to the armbian SPL size (pad with zeros)
padded_spl = our_spl + bytes(armbian_spl_size - our_spl_size)
print(f'\nPadded our SPL from {our_spl_size} to {armbian_spl_size} bytes')

# Combine: our padded SPL + Armbian's main U-Boot (after the SPL)
result = padded_spl + armbian[armbian_spl_size:]
print(f'Output binary: {len(result)} bytes ({len(result)//1024}KB)')
print(f'  SPL region: 0..{armbian_spl_size-1} (our patched SPL)')
print(f'  Main U-Boot: {armbian_spl_size}..{len(result)-1} (Armbian ATF + U-Boot 2025.04)')

with open(output_path, 'wb') as f:
    f.write(result)
print(f'\nWritten to: {output_path}')
