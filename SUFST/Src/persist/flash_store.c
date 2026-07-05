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
so the reserved page tracks whichever linker script (C8/CB) was used."
#endif

/* Last 2 KB page of flash, whichever variant this was built for - this
 * must stay in lockstep with the FLASH region length in
 * STM32G431X8_FLASH.ld / STM32G431XB_FLASH.ld, which is why both derive
 * from the single GPS_FLASH_TOTAL_KB the build picks (CMakeLists.txt +
 * cmake/gcc-arm-none-eabi.cmake), not two hand-maintained constants. */
#define FLASH_STORE_PAGE_ADDR \
    (FLASH_BASE + (GPS_FLASH_TOTAL_KB * 1024U) - FLASH_STORE_PAGE_SIZE)

static uint32_t
flash_store_page_number(void)
{
    return (FLASH_STORE_PAGE_ADDR - FLASH_BASE) / FLASH_PAGE_SIZE;
}

/* RAM-resident: writing/erasing the internal flash bank while executing
 * from that same bank stalls the fetch unit on this part, so the actual
 * program/erase calls run from CCM instead. */
__attribute__((section(".ccmfunc"))) static status_t
flash_store_program_record(uint32_t offset,
                            const flash_store_record_t *rec)
{
    if (offset + FLASH_STORE_RECORD_SIZE > FLASH_STORE_PAGE_SIZE) {
        return STATUS_FULL;
    }

    HAL_FLASH_Unlock();

    status_t result = STATUS_OK;
    const uint64_t *words = (const uint64_t *) (const void *) rec;
    uint32_t addr = FLASH_STORE_PAGE_ADDR + offset;

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

static status_t
flash_store_append(const flash_store_record_t *rec)
{
    uint32_t offset =
        flash_store_find_next_slot((const uint8_t *) FLASH_STORE_PAGE_ADDR,
                                    FLASH_STORE_PAGE_SIZE);
    if (offset >= FLASH_STORE_PAGE_SIZE) {
        return STATUS_FULL;
    }
    return flash_store_program_record(offset, rec);
}

void
flash_store_init(flash_store_gate_t *gates_out, uint32_t gates_max,
                 mag_cal_result_t *mag_cal_out)
{
    flash_store_restore((const uint8_t *) FLASH_STORE_PAGE_ADDR,
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

__attribute__((section(".ccmfunc"))) status_t
flash_store_erase_and_compact(void)
{
    /* Replay to the latest state before erasing (the source page is about
     * to be wiped), then rewrite only the gates that are still valid -
     * cleared/superseded slots and clear-all markers just vanish, since an
     * absent record already reads back as "not set". */
    flash_store_gate_t gates[FLASH_STORE_MAX_GATES];
    uint32_t n_gates = 0U;
    flash_store_record_t mag_cal_rec;
    bool have_mag_cal = false;

    for (uint32_t i = 0U; i < FLASH_STORE_MAX_GATES; i++) {
        gates[i].valid = false;
    }

    const uint8_t *page = (const uint8_t *) FLASH_STORE_PAGE_ADDR;
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

    HAL_FLASH_Unlock();
    FLASH_EraseInitTypeDef erase = {0};
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Banks = FLASH_BANK_1;
    erase.Page = flash_store_page_number();
    erase.NbPages = 1U;
    uint32_t page_error = 0U;
    HAL_StatusTypeDef erase_status =
        HAL_FLASHEx_Erase(&erase, &page_error);
    HAL_FLASH_Lock();

    if (erase_status != HAL_OK) {
        return STATUS_ERROR;
    }

    uint32_t write_offset = 0U;
    for (uint32_t i = 0U; i < FLASH_STORE_MAX_GATES; i++) {
        if (gates[i].valid) {
            flash_store_record_t rec;
            flash_store_build_gate_record((uint8_t) i, gates[i].lat_1e7,
                                           gates[i].lon_1e7,
                                           gates[i].heading_rad, true, &rec);
            if (flash_store_program_record(write_offset, &rec) !=
                STATUS_OK) {
                return STATUS_ERROR;
            }
            write_offset += FLASH_STORE_RECORD_SIZE;
        }
    }
    if (have_mag_cal) {
        if (flash_store_program_record(write_offset, &mag_cal_rec) !=
            STATUS_OK) {
            return STATUS_ERROR;
        }
    }

    return STATUS_OK;
}

#endif /* HOST_TEST_BUILD */
