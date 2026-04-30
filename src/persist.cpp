#include "persist.h"
#include "controller_profile.h"

#include <cstdio>
#include <cstring>

#include "hardware/flash.h"
#include "pico/flash.h"

// Use the last 4 KB sector of flash. PICO_FLASH_SIZE_BYTES is provided by the
// board header (pico2_w => 4 MB).
#define PERSIST_FLASH_OFFSET (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)
#define PERSIST_MAGIC        0xD55E0001u  // "DS5E v1"

struct persisted {
    uint32_t magic;
    uint8_t  addr[6];
    uint8_t  profile_idx;
    uint8_t  pad;
    uint32_t crc;       // simple integrity check; survives bit-rot detection
};
static_assert(sizeof(persisted) <= FLASH_PAGE_SIZE, "persisted struct must fit in one flash page");

static uint32_t simple_crc(const persisted *p) {
    // Tiny FNV-1a — strictly correctness, not security.
    uint32_t h = 0x811C9DC5u;
    const uint8_t *b = (const uint8_t*)p;
    for (size_t i = 0; i < offsetof(persisted, crc); i++) {
        h ^= b[i];
        h *= 0x01000193u;
    }
    return h;
}

static const persisted* persist_view(void) {
    return reinterpret_cast<const persisted*>(XIP_BASE + PERSIST_FLASH_OFFSET);
}

void persist_load(void) {
    const persisted *p = persist_view();
    if (p->magic != PERSIST_MAGIC) return;
    if (simple_crc(p) != p->crc) return;
    controller_profile_set(controller_profile_for_idx(p->profile_idx));
    printf("[Persist] Loaded profile_idx=%u for %02X:%02X:%02X:%02X:%02X:%02X\n",
           p->profile_idx,
           p->addr[0], p->addr[1], p->addr[2], p->addr[3], p->addr[4], p->addr[5]);
}

namespace {
struct write_args {
    persisted record;
};

void do_flash_write(void *p) {
    write_args *a = static_cast<write_args*>(p);
    flash_range_erase(PERSIST_FLASH_OFFSET, FLASH_SECTOR_SIZE);
    // Pad to a full page; flash_range_program requires a 256-byte multiple.
    uint8_t page[FLASH_PAGE_SIZE];
    memset(page, 0xFF, sizeof(page));
    memcpy(page, &a->record, sizeof(a->record));
    flash_range_program(PERSIST_FLASH_OFFSET, page, sizeof(page));
}
}

void persist_save_if_changed(const uint8_t addr[6], uint8_t profile_idx) {
    const persisted *cur = persist_view();
    if (cur->magic == PERSIST_MAGIC &&
        cur->profile_idx == profile_idx &&
        memcmp(cur->addr, addr, 6) == 0 &&
        simple_crc(cur) == cur->crc) {
        return;  // already up to date — no flash wear
    }

    write_args args{};
    args.record.magic = PERSIST_MAGIC;
    memcpy(args.record.addr, addr, 6);
    args.record.profile_idx = profile_idx;
    args.record.pad = 0;
    args.record.crc = simple_crc(&args.record);

    int rc = flash_safe_execute(do_flash_write, &args, 500);
    if (rc != 0) {
        printf("[Persist] flash_safe_execute failed rc=%d\n", rc);
        return;
    }
    printf("[Persist] Saved profile_idx=%u\n", profile_idx);
}
