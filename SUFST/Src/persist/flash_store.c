/**
 * @file    flash_store.c
 * @brief   Append-only gate/mag-cal persistence (see flash_store.h).
 *
 * Gates are stored as ABSOLUTE lat/lon (i32 1e-7 deg) plus a per-slot
 * valid flag, so they reproduce across a power cycle even though the ENU
 * origin lands somewhere different each boot. Both sets and clears are
 * persisted: a cleared slot writes a valid=0 marker, and the "wipe
 * everything" gestures (clear-all button, or the sector wipe implied by
 * setting a new start/finish) write a KIND_GATE_CLEAR_ALL record.
 * Superseded history is reclaimed by flash_store_erase_and_compact(),
 * which rewrites only the latest still-valid gate per slot.
 */

#include "persist/flash_store.h"

#include <string.h>

static uint32_t
crc32_update(uint32_t crc, const uint8_t *data, uint32_t len)
{
    crc = ~crc;
    for (uint32_t i = 0U; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            uint32_t mask = (uint32_t) (-(int32_t) (crc & 1U));
            crc = (crc >> 1) ^ (0xEDB88320U & mask);
        }
    }
    return ~crc;
}

static uint32_t
record_crc(const flash_store_record_t *rec)
{
    return crc32_update(0U, (const uint8_t *) rec,
                         (uint32_t) (sizeof(*rec) - sizeof(rec->crc)));
}

/* Gate payload lives in the record's `payload` bytes as
 * [i32 lat_1e7][i32 lon_1e7][f32 heading] - written scalar-by-scalar
 * (not via a struct memcpy) so no struct padding leaks indeterminate
 * bytes into the CRC. */
static void
gate_payload_write(flash_store_record_t *out, int32_t lat_1e7,
                   int32_t lon_1e7, float heading_rad)
{
    uint8_t *pb = (uint8_t *) out->payload;
    memcpy(pb + 0, &lat_1e7, sizeof lat_1e7);
    memcpy(pb + 4, &lon_1e7, sizeof lon_1e7);
    memcpy(pb + 8, &heading_rad, sizeof heading_rad);
}

static void
gate_payload_read(const flash_store_record_t *rec, int32_t *lat_1e7,
                  int32_t *lon_1e7, float *heading_rad)
{
    const uint8_t *pb = (const uint8_t *) rec->payload;
    memcpy(lat_1e7, pb + 0, sizeof *lat_1e7);
    memcpy(lon_1e7, pb + 4, sizeof *lon_1e7);
    memcpy(heading_rad, pb + 8, sizeof *heading_rad);
}

void
flash_store_build_gate_record(uint8_t index, int32_t lat_1e7,
                               int32_t lon_1e7, float heading_rad,
                               bool valid, flash_store_record_t *out)
{
    memset(out, 0, sizeof(*out));
    out->kind = FLASH_STORE_KIND_GATE;
    out->key = index;
    out->gate_valid = valid ? 1U : 0U;
    gate_payload_write(out, lat_1e7, lon_1e7, heading_rad);
    out->crc = record_crc(out);
}

void
flash_store_build_gate_clear_all_record(flash_store_record_t *out)
{
    memset(out, 0, sizeof(*out));
    out->kind = FLASH_STORE_KIND_GATE_CLEAR_ALL;
    out->crc = record_crc(out);
}

void
flash_store_build_mag_cal_record(const mag_cal_result_t *cal,
                                  flash_store_record_t *out)
{
    memset(out, 0, sizeof(*out));
    out->kind = FLASH_STORE_KIND_MAG_CAL;
    out->key = 0U;
    out->payload[0] = cal->bias[0];
    out->payload[1] = cal->bias[1];
    out->payload[2] = cal->bias[2];
    out->payload[3] = cal->scale[0];
    out->payload[4] = cal->scale[1];
    out->payload[5] = cal->scale[2];
    out->crc = record_crc(out);
}

static bool
record_is_blank(const flash_store_record_t *rec)
{
    const uint8_t *bytes = (const uint8_t *) rec;
    for (uint32_t i = 0U; i < sizeof(*rec); i++) {
        if (bytes[i] != 0xFFU) {
            return false;
        }
    }
    return true;
}

bool
flash_store_record_is_valid(const flash_store_record_t *rec)
{
    if (record_is_blank(rec)) {
        return false;
    }
    return rec->crc == record_crc(rec);
}

bool
flash_store_decode_gate(const flash_store_record_t *rec, uint8_t *index,
                         int32_t *lat_1e7, int32_t *lon_1e7,
                         float *heading_rad, bool *valid)
{
    if (!flash_store_record_is_valid(rec) ||
        rec->kind != FLASH_STORE_KIND_GATE) {
        return false;
    }
    *index = rec->key;
    *valid = (rec->gate_valid != 0U);
    gate_payload_read(rec, lat_1e7, lon_1e7, heading_rad);
    return true;
}

bool
flash_store_decode_mag_cal(const flash_store_record_t *rec,
                            mag_cal_result_t *out)
{
    if (!flash_store_record_is_valid(rec) ||
        rec->kind != FLASH_STORE_KIND_MAG_CAL) {
        return false;
    }
    out->bias[0] = rec->payload[0];
    out->bias[1] = rec->payload[1];
    out->bias[2] = rec->payload[2];
    out->scale[0] = rec->payload[3];
    out->scale[1] = rec->payload[4];
    out->scale[2] = rec->payload[5];
    return true;
}

uint32_t
flash_store_find_next_slot(const uint8_t *page, uint32_t page_size)
{
    uint32_t offset = 0U;
    while (offset + FLASH_STORE_RECORD_SIZE <= page_size) {
        if (record_is_blank((const flash_store_record_t *) &page[offset])) {
            return offset;
        }
        offset += FLASH_STORE_RECORD_SIZE;
    }
    return page_size;
}

void
flash_store_restore(const uint8_t *page, uint32_t page_size,
                     flash_store_gate_t *gates_out, uint32_t gates_max,
                     mag_cal_result_t *mag_cal_out)
{
    bool have_mag_cal = false;
    mag_cal_result_t mag_cal; /* only read after have_mag_cal is set true */

    for (uint32_t i = 0U; i < gates_max; i++) {
        gates_out[i].valid = false;
    }

    uint32_t offset = 0U;
    while (offset + FLASH_STORE_RECORD_SIZE <= page_size) {
        const flash_store_record_t *rec =
            (const flash_store_record_t *) &page[offset];

        uint8_t index;
        int32_t lat_1e7, lon_1e7;
        float h;
        bool valid;
        mag_cal_result_t cal;

        if (flash_store_decode_gate(rec, &index, &lat_1e7, &lon_1e7, &h,
                                     &valid)) {
            /* Append-order = newest-wins per slot, including a clear
             * (valid=false) superseding an earlier set. */
            if (index < gates_max) {
                gates_out[index].lat_1e7 = lat_1e7;
                gates_out[index].lon_1e7 = lon_1e7;
                gates_out[index].heading_rad = h;
                gates_out[index].valid = valid;
            }
        } else if (flash_store_record_is_valid(rec) &&
                   rec->kind == FLASH_STORE_KIND_GATE_CLEAR_ALL) {
            for (uint32_t i = 0U; i < gates_max; i++) {
                gates_out[i].valid = false;
            }
        } else if (flash_store_decode_mag_cal(rec, &cal)) {
            have_mag_cal = true;
            mag_cal = cal;
        }

        offset += FLASH_STORE_RECORD_SIZE;
    }

    if (have_mag_cal) {
        *mag_cal_out = mag_cal;
    } else {
        mag_cal_identity(mag_cal_out);
    }
}

#ifndef HOST_TEST_BUILD

#include "main.h"

#ifndef GPS_FLASH_TOTAL_KB
#error "GPS_FLASH_TOTAL_KB must be supplied by the build (see CMakeLists.txt) \
so the reserved page tracks the flash size the linker script was built for."
#endif

/* flash_store_erase_page() erases one hardware flash page per call
 * (NbPages = 1U); that's only actually one flash_store page if the two
 * sizes agree. FLASH_PAGE_SIZE (from the HAL, fixed by the STM32G431's
 * silicon) and FLASH_STORE_PAGE_SIZE (this module's own record-page size,
 * flash_store.h) are independent constants today only because both
 * happen to be 2 KB - this catches the two ever silently drifting apart. */
_Static_assert(FLASH_PAGE_SIZE == FLASH_STORE_PAGE_SIZE,
               "flash_store assumes one hardware flash page == one "
               "flash_store page");

/* Two adjacent 2 KB pages of flash on the STM32G431CB (128 KB), reserved
 * by carving GPS_FLASH_TOTAL_KB*2 KB off the top of the FLASH region in
 * STM32G431XB_FLASH.ld - both derive from the single GPS_FLASH_TOTAL_KB
 * the build sets (cmake/gcc-arm-none-eabi.cmake), not hand-maintained
 * constants that could drift apart.
 *
 * A/B redundancy: exactly one page is "active" (appended to, and read at
 * boot); the other is kept erased as the compaction target. Without this
 * a brownout/reset between erasing the sole page and finishing its
 * rewrite would leave every gate and the mag-cal blank/partial with
 * nothing to fall back on - see flash_store_erase_and_compact(). */
#define FLASH_STORE_NUM_PAGES 2U
#define FLASH_STORE_REGION_BASE                       \
    (FLASH_BASE + (GPS_FLASH_TOTAL_KB * 1024U) -      \
     (FLASH_STORE_PAGE_SIZE * FLASH_STORE_NUM_PAGES))
#define FLASH_STORE_PAGE_ADDR(n) \
    (FLASH_STORE_REGION_BASE + ((uint32_t) (n) * FLASH_STORE_PAGE_SIZE))

/* Which of the two pages is currently active, set by flash_store_init()
 * and updated by flash_store_erase_and_compact() when it hands off to
 * the freshly-written page. */
static uint32_t s_active_page;

static uint32_t
flash_store_page_number(uint32_t page_idx)
{
    return (FLASH_STORE_PAGE_ADDR(page_idx) - FLASH_BASE) / FLASH_PAGE_SIZE;
}

/* True if a page holds at least one CRC-valid record, i.e. it has been
 * written to since it was last erased. Used at boot to tell an in-use
 * page apart from a blank/spare one. */
static bool
flash_store_page_in_use(const uint8_t *page)
{
    uint32_t offset = 0U;
    while (offset + FLASH_STORE_RECORD_SIZE <= FLASH_STORE_PAGE_SIZE) {
        if (flash_store_record_is_valid(
                (const flash_store_record_t *) &page[offset])) {
            return true;
        }
        offset += FLASH_STORE_RECORD_SIZE;
    }
    return false;
}

/* RAM-resident (.ccmfunc, see the linker script): but this alone does
 * NOT make flash writes/erases stall-free. HAL_FLASH_Program() and
 * HAL_FLASHEx_Erase() - called from here - are themselves linked into
 * FLASH, not CCM, so the CPU fetch unit still stalls on every instruction
 * fetch from either for the full duration of the operation (confirmed via
 * `nm`: their symbols sit at 0x0800xxxx). That stalls the scheduler and
 * every flash-resident ISR for the operation's duration - low tens of us
 * for a program, tens of ms for a page erase. Placing these three thin
 * wrappers in CCM doesn't change that; it was never going to without also
 * moving the whole HAL flash driver there, which isn't done. Accepted
 * as-is: a single flash_store_program_record() (one double-word loop,
 * sub-ms) stalls briefly regardless of vehicle speed - e.g. every live
 * gate-set during a lap - which is short enough to tolerate; the much
 * longer flash_store_erase_page() (tens of ms) is the one that actually
 * needs gating, and sys_task.c only calls flash_store_erase_and_compact()
 * (which erases) while the car's fused speed reads as stationary. */
__attribute__((section(".ccmfunc"))) static status_t
flash_store_program_record(uint32_t page_idx, uint32_t offset,
                            const flash_store_record_t *rec)
{
    if (offset + FLASH_STORE_RECORD_SIZE > FLASH_STORE_PAGE_SIZE) {
        return STATUS_FULL;
    }

    HAL_FLASH_Unlock();

    status_t result = STATUS_OK;
    const uint64_t *words = (const uint64_t *) (const void *) rec;
    uint32_t addr = FLASH_STORE_PAGE_ADDR(page_idx) + offset;

    for (uint32_t i = 0U; i < FLASH_STORE_RECORD_SIZE / 8U; i++) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                               addr + (i * 8U), words[i]) != HAL_OK) {
            result = STATUS_ERROR;
            break;
        }
    }

    HAL_FLASH_Lock();
    return result;
}

__attribute__((section(".ccmfunc"))) static status_t
flash_store_erase_page(uint32_t page_idx)
{
    HAL_FLASH_Unlock();
    FLASH_EraseInitTypeDef erase = {0};
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Banks = FLASH_BANK_1;
    erase.Page = flash_store_page_number(page_idx);
    erase.NbPages = 1U;
    uint32_t page_error = 0U;
    HAL_StatusTypeDef erase_status = HAL_FLASHEx_Erase(&erase, &page_error);
    HAL_FLASH_Lock();
    return (erase_status == HAL_OK) ? STATUS_OK : STATUS_ERROR;
}

static status_t
flash_store_append(const flash_store_record_t *rec)
{
    uint32_t offset = flash_store_find_next_slot(
        (const uint8_t *) FLASH_STORE_PAGE_ADDR(s_active_page),
        FLASH_STORE_PAGE_SIZE);
    if (offset >= FLASH_STORE_PAGE_SIZE) {
        return STATUS_FULL;
    }
    return flash_store_program_record(s_active_page, offset, rec);
}

void
flash_store_init(flash_store_gate_t *gates_out, uint32_t gates_max,
                 mag_cal_result_t *mag_cal_out)
{
    bool in_use[FLASH_STORE_NUM_PAGES];
    for (uint32_t i = 0U; i < FLASH_STORE_NUM_PAGES; i++) {
        in_use[i] =
            flash_store_page_in_use((const uint8_t *) FLASH_STORE_PAGE_ADDR(i));
    }

    if (in_use[0] && in_use[1]) {
        /* A reset landed between "compacted copy fully written to the
         * spare" and "old copy erased" in flash_store_erase_and_compact().
         * Compaction is a lossless replay, so both pages agree on the
         * latest-per-key state - arbitrarily prefer page 0 and finish the
         * interrupted cleanup so the next boot/compaction sees the normal
         * single-active-page state again. */
        s_active_page = 0U;
        (void) flash_store_erase_page(1U);
    } else if (in_use[1]) {
        s_active_page = 1U;
    } else {
        s_active_page = 0U; /* page 0 in use, or neither (first-ever boot) */
    }

    flash_store_restore((const uint8_t *) FLASH_STORE_PAGE_ADDR(s_active_page),
                         FLASH_STORE_PAGE_SIZE, gates_out, gates_max,
                         mag_cal_out);
}

status_t
flash_store_save_gate(uint8_t index, int32_t lat_1e7, int32_t lon_1e7,
                       float heading_rad)
{
    flash_store_record_t rec;
    flash_store_build_gate_record(index, lat_1e7, lon_1e7, heading_rad,
                                   true, &rec);
    return flash_store_append(&rec);
}

status_t
flash_store_save_gate_cleared(uint8_t index)
{
    flash_store_record_t rec;
    /* A cleared slot: coordinates are don't-care, the valid=false flag is
     * what restore keys on. */
    flash_store_build_gate_record(index, 0, 0, 0.0f, false, &rec);
    return flash_store_append(&rec);
}

status_t
flash_store_save_gates_cleared_all(void)
{
    flash_store_record_t rec;
    flash_store_build_gate_clear_all_record(&rec);
    return flash_store_append(&rec);
}

status_t
flash_store_save_mag_cal(const mag_cal_result_t *cal)
{
    flash_store_record_t rec;
    flash_store_build_mag_cal_record(cal, &rec);
    return flash_store_append(&rec);
}

/* Reclaims space from superseded history by rewriting only the current
 * (latest-per-key) records - but into the SPARE page, not over the active
 * one. The active page is only erased (and only then does the spare
 * become active) once the rewrite has fully landed, so a brownout/reset
 * at any point up to and including a failed program leaves the active
 * page - and every gate/mag-cal it holds - completely untouched. Without
 * this, erasing the sole page before rewriting it (as this function used
 * to) meant a crash between the erase and the rewrite finishing lost
 * every gate and the mag-cal outright, with no fallback. */
__attribute__((section(".ccmfunc"))) status_t
flash_store_erase_and_compact(void)
{
    /* Replay to the latest state first (the source page itself is never
     * modified by this function until the very last step), then rewrite
     * only the gates that are still valid - cleared/superseded slots and
     * clear-all markers just vanish, since an absent record already
     * reads back as "not set". */
    flash_store_gate_t gates[FLASH_STORE_MAX_GATES];
    uint32_t n_gates = 0U;
    flash_store_record_t mag_cal_rec;
    bool have_mag_cal = false;

    for (uint32_t i = 0U; i < FLASH_STORE_MAX_GATES; i++) {
        gates[i].valid = false;
    }

    const uint8_t *page = (const uint8_t *) FLASH_STORE_PAGE_ADDR(s_active_page);
    uint32_t offset = 0U;
    while (offset + FLASH_STORE_RECORD_SIZE <= FLASH_STORE_PAGE_SIZE) {
        const flash_store_record_t *rec =
            (const flash_store_record_t *) &page[offset];
        uint8_t index;
        int32_t lat_1e7, lon_1e7;
        float h;
        bool valid;
        mag_cal_result_t cal;

        if (flash_store_decode_gate(rec, &index, &lat_1e7, &lon_1e7, &h,
                                     &valid)) {
            if (index < FLASH_STORE_MAX_GATES) {
                gates[index].lat_1e7 = lat_1e7;
                gates[index].lon_1e7 = lon_1e7;
                gates[index].heading_rad = h;
                gates[index].valid = valid;
            }
        } else if (flash_store_record_is_valid(rec) &&
                   rec->kind == FLASH_STORE_KIND_GATE_CLEAR_ALL) {
            for (uint32_t i = 0U; i < FLASH_STORE_MAX_GATES; i++) {
                gates[i].valid = false;
            }
        } else if (flash_store_decode_mag_cal(rec, &cal)) {
            have_mag_cal = true;
            flash_store_build_mag_cal_record(&cal, &mag_cal_rec);
        }
        offset += FLASH_STORE_RECORD_SIZE;
    }

    for (uint32_t i = 0U; i < FLASH_STORE_MAX_GATES; i++) {
        if (gates[i].valid) {
            n_gates++;
        }
    }

    if ((n_gates + (have_mag_cal ? 1U : 0U)) * FLASH_STORE_RECORD_SIZE >
        FLASH_STORE_PAGE_SIZE) {
        return STATUS_ERROR; /* shouldn't happen: fewer keys than slots */
    }

    /* Erase the spare unconditionally, even if it looks blank: a prior
     * crash could have left a partial rewrite there from an attempt that
     * never reached the final "erase the old active page" step below, and
     * this guarantees a clean target regardless. The active page (this
     * function's only source of truth so far) is not touched by this. */
    uint32_t spare_page = 1U - s_active_page;
    if (flash_store_erase_page(spare_page) != STATUS_OK) {
        return STATUS_ERROR;
    }

    uint32_t write_offset = 0U;
    for (uint32_t i = 0U; i < FLASH_STORE_MAX_GATES; i++) {
        if (gates[i].valid) {
            flash_store_record_t rec;
            flash_store_build_gate_record((uint8_t) i, gates[i].lat_1e7,
                                           gates[i].lon_1e7,
                                           gates[i].heading_rad, true, &rec);
            if (flash_store_program_record(spare_page, write_offset, &rec) !=
                STATUS_OK) {
                /* The spare is left half-written, but the active page (the
                 * one actually in service) was never touched - restart
                 * from a still-fully-valid state next boot/compaction. */
                return STATUS_ERROR;
            }
            write_offset += FLASH_STORE_RECORD_SIZE;
        }
    }
    if (have_mag_cal) {
        if (flash_store_program_record(spare_page, write_offset,
                                        &mag_cal_rec) != STATUS_OK) {
            return STATUS_ERROR;
        }
    }

    /* Commit: only now that the spare fully holds the compacted state is
     * the old active page erased and the spare promoted. A reset between
     * these two lines leaves both pages holding valid (and, since
     * compaction is a lossless replay, equivalent) data - flash_store_
     * init() resolves that tie and finishes this cleanup on the next
     * boot. */
    status_t erase_status = flash_store_erase_page(s_active_page);
    s_active_page = spare_page;
    return erase_status;
}

#endif /* HOST_TEST_BUILD */
