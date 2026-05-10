#!/usr/bin/env python3
"""
Extract /home/rosie/uboot_backup.bin from the SD card's ext4 partition
and write the original Armbian U-Boot back to the disk to recover the Pi.

Run from an Administrator PowerShell.
"""

import sys
import os
import struct
import ctypes

DISK = r'\\.\PhysicalDrive3'
PARTITION_START_SECTOR = 8192        # from fdisk output
SECTOR_SIZE = 512
PARTITION_START = PARTITION_START_SECTOR * SECTOR_SIZE  # 4MB

UBOOT_OFFSET = 8 * 1024              # 8KB from start of disk
OUTPUT_PATH = r'E:\Code\ROSie\uboot_original_from_pi.bin'
RESTORE_CONFIRM = '--restore' in sys.argv


def is_admin():
    try:
        return ctypes.windll.shell32.IsUserAnAdmin()
    except:
        return False


def read_disk(disk, offset, size):
    with open(disk, 'rb') as f:
        f.seek(offset)
        return f.read(size)


def extract_via_ext4_lib(disk_path, partition_start):
    import ext4
    print(f"  Opening {disk_path} partition at offset {partition_start} ({partition_start//1024//1024}MB)...")

    class PartitionView:
        """Wrap the raw disk file, offsetting reads to the partition start."""
        def __init__(self, path, offset):
            self._f = open(path, 'rb')
            self._offset = offset

        def read(self, offset, size):
            self._f.seek(self._offset + offset)
            return self._f.read(size)

        def seek(self, pos):
            self._f.seek(self._offset + pos)

        def close(self):
            self._f.close()

    view = PartitionView(disk_path, partition_start)

    # ext4 library needs a file-like object
    # Try different API styles depending on ext4 version
    try:
        import ext4

        class AlignedDiskAdapter:
            """
            Wraps raw disk access with sector-aligned reads (Windows requirement).
            Buffers reads to serve arbitrary sizes/offsets to the ext4 library.
            """
            SECTOR = 512

            def __init__(self, path, partition_offset):
                self._f = open(path, 'rb')
                self._part = partition_offset
                self._pos = 0
                self._cache = {}   # sector_no -> bytes

            def _read_sector(self, sector_no):
                if sector_no not in self._cache:
                    offset = self._part + sector_no * self.SECTOR
                    self._f.seek(offset)
                    data = self._f.read(self.SECTOR)
                    if len(data) < self.SECTOR:
                        data = data + bytes(self.SECTOR - len(data))
                    self._cache[sector_no] = data
                return self._cache[sector_no]

            def _read_bytes(self, abs_offset, size):
                """Read 'size' bytes at abs_offset (relative to partition start)."""
                result = bytearray()
                pos = abs_offset
                while len(result) < size:
                    sector_no = pos // self.SECTOR
                    sector_off = pos % self.SECTOR
                    sector_data = self._read_sector(sector_no)
                    chunk = sector_data[sector_off:]
                    needed = size - len(result)
                    result.extend(chunk[:needed])
                    pos += min(len(chunk), needed)
                return bytes(result)

            def read(self, size=-1):
                if size < 0:
                    raise NotImplementedError("Unbounded read not supported")
                data = self._read_bytes(self._pos, size)
                self._pos += len(data)
                return data

            def seek(self, pos, whence=0):
                if whence == 0:
                    self._pos = pos
                elif whence == 1:
                    self._pos += pos
                elif whence == 2:
                    # SEEK_END - compute partition size from superblock
                    sb = self._read_bytes(1024, 28)
                    blocks = struct.unpack('<I', sb[0:4])[0]
                    log_bs = struct.unpack('<I', sb[24:28])[0]
                    block_size = 1024 << log_bs
                    self._pos = blocks * block_size + pos
                return self._pos

            def tell(self):
                return self._pos

            def peek(self, size=1):
                return self._read_bytes(self._pos, size)

            def close(self):
                self._f.close()

        print(f"  Opening {disk_path} partition at offset {partition_start} ({partition_start//1024//1024}MB)...")
        adapter = AlignedDiskAdapter(disk_path, partition_start)
        vol = ext4.Volume(adapter)
        print("  ext4 volume opened successfully")

        # Use path-based inode lookup
        print(f"  Looking up /home/rosie/uboot_backup.bin ...")
        backup_inode = vol.inode_at('/home/rosie/uboot_backup.bin')
        print(f"  Found: size={backup_inode.size} bytes")
        data = backup_inode.open().read()
        return data

        backup_inode = current
        print(f"  Found inode for uboot_backup.bin, size={backup_inode.size} bytes")
        data = backup_inode.open().read()
        return data

    except Exception as e:
        print(f"  ext4 lib error: {e}")
        import traceback
        traceback.print_exc()
        return None
    finally:
        try:
            adapter.close()
        except:
            pass


if __name__ == '__main__':
    if not is_admin():
        print("ERROR: Must run as Administrator.")
        sys.exit(1)

    print("=== Extract uboot_backup.bin from SD card ext4 partition ===\n")

    # Quick sanity check: is this the right disk?
    try:
        header = read_disk(DISK, 0, 16)
        print(f"Disk 3 first bytes: {header[:4].hex()} (OK if non-zero)")
        uboot_area = read_disk(DISK, UBOOT_OFFSET, 12)
        magic = uboot_area[4:12]
        print(f"Current U-Boot magic at 8KB: {magic!r}")
        if magic == b'eGON.BT0':
            print("  -> SPL present (currently our broken 2021.07 SPL)")
        else:
            print("  -> WARNING: No valid SPL found!")
    except Exception as e:
        print(f"Cannot read disk: {e}")
        sys.exit(1)

    print(f"\nPartition 1 starts at byte {PARTITION_START} ({PARTITION_START//1024//1024}MB)\n")

    # Check ext4 superblock magic
    superblock = read_disk(DISK, PARTITION_START + 1024, 64)
    ext4_magic = struct.unpack('<H', superblock[56:58])[0]
    print(f"ext4 superblock magic: 0x{ext4_magic:04x} (expected 0xEF53)")
    if ext4_magic != 0xEF53:
        print("ERROR: ext4 superblock magic mismatch - wrong partition offset?")
        sys.exit(1)
    print("  -> Valid ext4 superblock confirmed\n")

    # Extract the file
    data = extract_via_ext4_lib(DISK, PARTITION_START)

    if data is None:
        print("\nFailed to extract via ext4 library.")
        sys.exit(1)

    print(f"\nExtracted {len(data)} bytes")

    # Verify it looks like a U-Boot binary
    magic = data[4:12] if len(data) > 12 else b''
    if magic == b'eGON.BT0':
        spl_size = struct.unpack('<I', data[16:20])[0]
        print(f"Valid eGON SPL: size={spl_size} bytes ({spl_size//1024}KB)")
    else:
        print(f"WARNING: Unexpected magic: {magic!r} - may not be a valid U-Boot binary")

    # Save extracted file
    with open(OUTPUT_PATH, 'wb') as f:
        f.write(data)
    print(f"Saved to: {OUTPUT_PATH}")

    if RESTORE_CONFIRM:
        print(f"\n[--restore] Writing {len(data)} bytes to {DISK} at offset {UBOOT_OFFSET}...")
        with open(DISK, 'r+b') as f:
            f.seek(UBOOT_OFFSET)
            f.write(data)
            f.flush()
            os.fsync(f.fileno())
        print("RESTORE COMPLETE. Remove SD card, reinsert into Pi, power on.")
    else:
        print(f"\nDry run complete. Review {OUTPUT_PATH} then run with --restore to flash.")
        print(f"  python scripts/extract_and_restore_uboot.py --restore")
