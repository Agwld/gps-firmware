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

/* Upper bound on gate slots this store can hold. Kept independent of
 * (and >=) laptimer's LAP_MAX_GATES so this header needn't pull in
 * board_config.h; callers size their own arrays and pass the count in. */
#define FLASH_STORE_MAX_GATES 8U

#define FLASH_STORE_KIND_GATE           1U
#define FLASH_STORE_KIND_MAG_CAL        2U
#define FLASH_STORE_KIND_GATE_CLEAR_ALL 3U

typedef struct {
    uint8_t kind;
    uint8_t key;      /* gate index for KIND_GATE; unused (0) otherwise */
    uint8_t gate_valid; /* KIND_GATE: 1 = set, 0 = a persisted "cleared"
                         * marker (so a clear survives a power cycle);
                         * unused for other kinds */
    uint8_t reserved;
    float payload[6];
    uint32_t crc; /* CRC32 (reflected, poly 0xEDB88320) over every
                   * preceding byte of the record */
} flash_store_record_t;

/* A restored gate slot, in ABSOLUTE lat/lon (not ENU) so it reproduces
 * regardless of where the ENU origin lands on the next power-up. Stored
 * as u-blox-style i32 1e-7 deg: a float32 degree only carries ~1 m of
 * precision (see geodesy.c), which is too coarse for a gate line, so the
 * absolute coordinate must be kept as an integer, not a float. */
typedef struct {
    int32_t lat_1e7;
    int32_t lon_1e7;
    float heading_rad; /* travel direction through the gate (ENU bearing;
                        * origin-independent over a track-sized area) */
    bool valid;        /* true = a gate should be placed at this index */
} flash_store_gate_t;

/** @brief Build a gate SET (valid=true) or persisted-CLEAR (valid=false)
 *         record for the given slot. */
void flash_store_build_gate_record(uint8_t index, int32_t lat_1e7,
                                    int32_t lon_1e7, float heading_rad,
                                    bool valid, flash_store_record_t *out);
void flash_store_build_gate_clear_all_record(flash_store_record_t *out);
void flash_store_build_mag_cal_record(const mag_cal_result_t *cal,
                                       flash_store_record_t *out);

/** @brief CRC-valid and not a blank (all-0xFF, i.e. erased/unwritten)
 *         slot. */
bool flash_store_record_is_valid(const flash_store_record_t *rec);

bool flash_store_decode_gate(const flash_store_record_t *rec,
                              uint8_t *index, int32_t *lat_1e7,
                              int32_t *lon_1e7, float *heading_rad,
                              bool *valid);
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
 *        producing the latest gate table and mag-cal.
 *
 * Unlike the old version this does NOT touch gates.c: at boot the ENU
 * origin isn't known yet, so gates can only be resolved to ENU once the
 * first fix sets it. The caller (imu_task) holds this absolute-lat/lon
 * table and converts each entry to ENU via gates_set() at origin time.
 *
 * @param gates_out    [out] array of at least `gates_max` entries; every
 *                     slot is initialised (valid=false) then filled from
 *                     the latest record per index. A KIND_GATE_CLEAR_ALL
 *                     record voids all slots seen so far.
 * @param gates_max    number of usable entries in gates_out.
 * @param mag_cal_out  [out] set to mag_cal_identity() if no valid
 *                     mag-cal record was found.
 */
void flash_store_restore(const uint8_t *page, uint32_t page_size,
                          flash_store_gate_t *gates_out, uint32_t gates_max,
                          mag_cal_result_t *mag_cal_out);

#ifndef HOST_TEST_BUILD
/** @brief Scan the real flash page and restore the gate table + mag-cal.
 *         Call once at boot. */
void flash_store_init(flash_store_gate_t *gates_out, uint32_t gates_max,
                      mag_cal_result_t *mag_cal_out);

/** @brief Append a gate SET record (absolute lat/lon). STATUS_FULL if the
 *         page has no free slot - caller should flash_store_erase_and_
 *         compact() (only when safe - e.g. car stationary) and retry. */
status_t flash_store_save_gate(uint8_t index, int32_t lat_1e7,
                               int32_t lon_1e7, float heading_rad);
/** @brief Append a persisted "gate cleared" marker for one slot. */
status_t flash_store_save_gate_cleared(uint8_t index);
/** @brief Append a "clear every gate" marker (steering-wheel clear-all /
 *         the sector wipe that setting a new start/finish implies). */
status_t flash_store_save_gates_cleared_all(void);
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
