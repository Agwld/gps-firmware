/**
 * @file    flash_store.c
 * @brief   Append-only gate/mag-cal persistence (see flash_store.h).
 *
 * Known simplification: only gate *sets* are persisted, not clears - a
 * CAN_CMD_GATE_CLEAR takes effect immediately in RAM (gates.c) but isn't
 * written to flash, so a power cycle after clearing a gate (without
 * setting a new one over it) would restore the old one. Acceptable for
 * now since the common sequence is clear-then-set-new, which does
 * persist correctly; a dedicated CLEAR record kind is the natural fix
 * if bench use shows the gap matters.
 */

#include "persist/flash_store.h"

#include <string.h>

#include "laptimer/gates.h"

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

void
flash_store_build_gate_record(uint8_t index, float east_m, float north_m,
                               float heading_rad, flash_store_record_t *out)
{
    memset(out, 0, sizeof(*out));
    out->kind = FLASH_STORE_KIND_GATE;
    out->key = index;
    out->payload[0] = east_m;
    out->payload[1] = north_m;
    out->payload[2] = heading_rad;
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
                         float *east_m, float *north_m, float *heading_rad)
{
    if (!flash_store_record_is_valid(rec) ||
        rec->kind != FLASH_STORE_KIND_GATE) {
        return false;
    }
    *index = rec->key;
    *east_m = rec->payload[0];
    *north_m = rec->payload[1];
    *heading_rad = rec->payload[2];
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
                     mag_cal_result_t *mag_cal_out)
{
    bool have_gate[FLASH_STORE_MAX_RECORDS] = {false};
    float gate_east[FLASH_STORE_MAX_RECORDS];
    float gate_north[FLASH_STORE_MAX_RECORDS];
    float gate_heading[FLASH_STORE_MAX_RECORDS];
    bool have_mag_cal = false;
    mag_cal_result_t mag_cal; /* only read after have_mag_cal is set true */

    uint32_t offset = 0U;
    while (offset + FLASH_STORE_RECORD_SIZE <= page_size) {
        const flash_store_record_t *rec =
            (const flash_store_record_t *) &page[offset];

        uint8_t index;
        float e, n, h;
        mag_cal_result_t cal;

        if (flash_store_decode_gate(rec, &index, &e, &n, &h) &&
            index < FLASH_STORE_MAX_RECORDS) {
            have_gate[index] = true;
            gate_east[index] = e;
            gate_north[index] = n;
            gate_heading[index] = h;
        } else if (flash_store_decode_mag_cal(rec, &cal)) {
            have_mag_cal = true;
            mag_cal = cal;
        }

        offset += FLASH_STORE_RECORD_SIZE;
    }

    for (uint32_t i = 0U; i < FLASH_STORE_MAX_RECORDS; i++) {
        if (have_gate[i]) {
            gates_set((uint8_t) i, gate_east[i], gate_north[i],
                      gate_heading[i]);
        }
    }

    if (have_mag_cal) {
        *mag_cal_out = mag_cal;
    } else {
        mag_cal_identity(mag_cal_out);
    }
}

#ifndef HOST_TEST_BUILD

#include "main.h"

#define FLASH_STORE_PAGE_ADDR 0x0800F800U

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
flash_store_init(mag_cal_result_t *mag_cal_out)
{
    flash_store_restore((const uint8_t *) FLASH_STORE_PAGE_ADDR,
                         FLASH_STORE_PAGE_SIZE, mag_cal_out);
}

status_t
flash_store_save_gate(uint8_t index, float east_m, float north_m,
                       float heading_rad)
{
    flash_store_record_t rec;
    flash_store_build_gate_record(index, east_m, north_m, heading_rad,
                                   &rec);
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
    /* Snapshot the current (latest-per-key) state before erasing - the
     * source page is about to be wiped. */
    flash_store_record_t gate_recs[FLASH_STORE_MAX_RECORDS];
    bool have_gate[FLASH_STORE_MAX_RECORDS] = {false};
    uint32_t n_gates = 0U;
    flash_store_record_t mag_cal_rec;
    bool have_mag_cal = false;

    const uint8_t *page = (const uint8_t *) FLASH_STORE_PAGE_ADDR;
    uint32_t offset = 0U;
    while (offset + FLASH_STORE_RECORD_SIZE <= FLASH_STORE_PAGE_SIZE) {
        const flash_store_record_t *rec =
            (const flash_store_record_t *) &page[offset];
        uint8_t index;
        float e, n, h;
        mag_cal_result_t cal;

        if (flash_store_decode_gate(rec, &index, &e, &n, &h) &&
            index < FLASH_STORE_MAX_RECORDS) {
            if (!have_gate[index]) {
                n_gates++;
            }
            have_gate[index] = true;
            flash_store_build_gate_record(index, e, n, h,
                                           &gate_recs[index]);
        } else if (flash_store_decode_mag_cal(rec, &cal)) {
            have_mag_cal = true;
            flash_store_build_mag_cal_record(&cal, &mag_cal_rec);
        }
        offset += FLASH_STORE_RECORD_SIZE;
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
    for (uint32_t i = 0U; i < FLASH_STORE_MAX_RECORDS; i++) {
        if (have_gate[i]) {
            if (flash_store_program_record(write_offset, &gate_recs[i]) !=
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
