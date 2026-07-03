// SPDX-License-Identifier: MIT
// Copyright (c) 2026 p0rtm0lester
// =====================================================================
//  M5Cardputer Loader — custom bootloader hook ("boot-once" flag)
// ---------------------------------------------------------------------
//  Runs in the 2nd-stage bootloader, after hardware/flash init but BEFORE
//  the bootloader chooses which app to boot.
//
//  Mechanism (works for ANY app, even ones that self-validate like Bruce):
//    - The loader, just before launching a firmware, writes BOOT_MAGIC into
//      the 'bootflag' sector and points otadata at ota_0.
//    - This hook reads bootflag:
//        * magic set  -> consume it (erase bootflag), leave otadata alone,
//                        so the bootloader boots ota_0 (the chosen firmware).
//        * magic clear-> wipe otadata, so the bootloader falls back to the
//                        factory app (the loader). This overrides any
//                        esp_ota_mark_app_valid the running app may have done.
//    - Net effect: pressing RESET always returns to the loader; the loader
//      explicitly arms a single boot of the selected firmware each time.
//
//  NOT a security boundary: bootflag/otadata/factory are plain writable flash
//  (no Secure Boot / flash encryption) and this hook does not check WHO set the
//  flag. A launched firmware runs fully privileged in ota_0 and can forge
//  BOOT_MAGIC to keep booting itself, or erase factory to disable the loader.
//  The "works for ANY app" guarantee above holds only for well-behaved apps.
//
//  Offsets are hard-coded to match partitions.csv (the bootloader is built
//  for this exact layout).
// =====================================================================
#include <stdint.h>
#include "bootloader_flash_priv.h"

// Forces the linker to pull this object into the bootloader (the bootloader
// 'main' component links with `-u bootloader_hooks_include`). Defining it here
// guarantees our strong bootloader_after_init() overrides the weak default.
void bootloader_hooks_include(void) {}

#define BOOTFLAG_OFFSET 0x0000d000u
#define BOOTFLAG_SIZE   0x00001000u
#define OTADATA_OFFSET  0x0000e000u
#define OTADATA_SIZE    0x00002000u
#define BOOT_MAGIC      0xB007A001u

// True if [off, off+len) reads back as all-0xFF (erased). On any read error we
// return 0 (not-blank) so the caller performs the erase — the safe default.
static int region_is_blank(uint32_t off, uint32_t len)
{
    uint8_t b[256];
    for (uint32_t p = 0; p < len; p += sizeof(b)) {
        uint32_t n = len - p; if (n > sizeof(b)) n = sizeof(b);
        if (bootloader_flash_read(off + p, b, n, false) != 0) return 0;
        for (uint32_t i = 0; i < n; i++) if (b[i] != 0xFF) return 0;
    }
    return 1;
}

void bootloader_after_init(void)
{
    uint32_t magic = 0;
    if (bootloader_flash_read(BOOTFLAG_OFFSET, &magic, sizeof(magic), false) != 0) {
        return;  // on read failure, behave like a normal bootloader
    }

    if (magic == BOOT_MAGIC) {
        // One-shot launch request: consume it; keep otadata -> boot ota_0.
        bootloader_flash_erase_range(BOOTFLAG_OFFSET, BOOTFLAG_SIZE);
    } else {
        // No request -> force the loader: wipe otadata -> boot factory.
        // Skip the erase when otadata is already blank: this hook runs on EVERY
        // boot, and erasing 2 sectors each time is needless flash wear plus a
        // power-loss window. When it isn't blank, the erase is essential (else the
        // just-run app boots again and can mark itself valid), so check the result
        // and retry once on failure.
        if (!region_is_blank(OTADATA_OFFSET, OTADATA_SIZE)) {
            if (bootloader_flash_erase_range(OTADATA_OFFSET, OTADATA_SIZE) != 0) {
                bootloader_flash_erase_range(OTADATA_OFFSET, OTADATA_SIZE);
            }
        }
    }
}
