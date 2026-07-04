/**
 * @file    flash_store.h
 * @brief   Append-only gate/mag-cal persistence in the last flash page.
 *
 * Record layout and scan/restore logic are pure C and host-testable
 * against a plain in-memory buffer standing in for the flash page; only
 * the actual erase/program calls (flash_store_save_*(),
 * flash_store_erase_and_compact()) touch real flash and are target-only.
 */

#ifndef FLASH_STORE_H
#define FLASH_STORE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "imu/mag_cal.h"
#include "sys/status.h"

#define FLASH_STORE_RECORD_SIZE 32U
#define FLASH_STORE_PAGE_SIZE   2048U
#define FLASH_STORE_MAX_RECORDS (FLASH_STORE_PAGE_SIZE / FLASH_STORE_RECORD_SIZE)

#define FLASH_STORE_KIND_GATE    1U
#define FLASH_STORE_KIND_MAG_CAL 2U

typedef struct {
    uint8_t kind;
    uint8_t key; /* gate index for KIND_GATE; unused (0) for KIND_MAG_CAL */
    uint8_t reserved[2];
    float payload[6];
    uint32_t crc; /* CRC32 (reflected, poly 0xEDB88320) over every
                   * preceding byte of the record */
} flash_store_record_t;

void flash_store_build_gate_record(uint8_t index, float east_m,
                                    float north_m, float heading_rad,
                                    flash_store_record_t *out);
void flash_store_build_mag_cal_record(const mag_cal_result_t *cal,
                                       flash_store_record_t *out);

/** @brief CRC-valid and not a blank (all-0xFF, i.e. erased/unwritten)
 *         slot. */
bool flash_store_record_is_valid(const flash_store_record_t *rec);

bool flash_store_decode_gate(const flash_store_record_t *rec,
                              uint8_t *index, float *east_m,
                              float *north_m, float *heading_rad);
bool flash_store_decode_mag_cal(const flash_store_record_t *rec,
                                 mag_cal_result_t *out);

/**
 * @brief Byte offset of the first blank (erased) record slot in a page
 *        buffer, for the next append.
 * @return offset, or FLASH_STORE_PAGE_SIZE if the page is full.
 */
uint32_t flash_store_find_next_slot(const uint8_t *page,
                                     uint32_t page_size);

/**
 * @brief Replay every valid record in a page buffer (append-order, so a
 *        later record for the same kind+key overrides an earlier one),
 *        restoring gates.c's state via gates_set() and writing the
 *        latest mag-cal record (if any) to *mag_cal_out.
 *
 * @param mag_cal_out  [out] set to mag_cal_identity() if no valid
 *                     mag-cal record was found.
 */
void flash_store_restore(const uint8_t *page, uint32_t page_size,
                          mag_cal_result_t *mag_cal_out);

#ifndef HOST_TEST_BUILD
/** @brief Scan the real flash page and restore gates/mag-cal. Call once
 *         from sys_task at boot. */
void flash_store_init(mag_cal_result_t *mag_cal_out);

/** @brief Append a gate record. STATUS_FULL if the page has no free
 *         slot - caller should flash_store_erase_and_compact() (only
 *         when safe - e.g. car stationary) and retry. */
status_t flash_store_save_gate(uint8_t index, float east_m, float north_m,
                                float heading_rad);
status_t flash_store_save_mag_cal(const mag_cal_result_t *cal);

/**
 * @brief Erase the page and rewrite only the current (latest-per-key)
 *        records - reclaims space from superseded history. The erase
 *        itself takes tens of ms with flash reads stalled, so the
 *        caller must only invoke this when it judges it safe to do so
 *        (e.g. vehicle stationary), never from a time-critical path.
 */
status_t flash_store_erase_and_compact(void);
#endif

#ifdef __cplusplus
}
#endif

#endif /* FLASH_STORE_H */
