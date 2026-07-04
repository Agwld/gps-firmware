/**
 * @file    test_flash_store.c
 * @brief   Host tests for persist/flash_store.c.
 */

#include "persist/flash_store.h"

#include <string.h>
#include <stdio.h>

#include "laptimer/gates.h"

static int s_failures = 0;

#define CHECK(desc, cond)                                                   \
    do {                                                                    \
        if (!(cond)) {                                                      \
            fprintf(stderr, "FAIL: %s\n", (desc));                          \
            s_failures++;                                                   \
        }                                                                    \
    } while (0)

#define CHECK_NEAR(desc, actual, expected, tol)                             \
    do {                                                                    \
        float diff = (actual) - (expected);                                 \
        if (diff < 0.0f) {                                                  \
            diff = -diff;                                                   \
        }                                                                    \
        if (diff > (tol)) {                                                 \
            fprintf(stderr, "FAIL: %s: got %f, expected %f\n", (desc),      \
                    (double) (actual), (double) (expected));                \
            s_failures++;                                                   \
        }                                                                    \
    } while (0)

static void
test_build_and_validate_roundtrip(void)
{
    flash_store_record_t rec;
    flash_store_build_gate_record(3U, 12.5f, -4.5f, 1.2f, &rec);

    CHECK("built gate record is valid", flash_store_record_is_valid(&rec));

    uint8_t index;
    float e, n, h;
    CHECK("decodes as a gate", flash_store_decode_gate(&rec, &index, &e,
                                                        &n, &h));
    CHECK("index roundtrips", index == 3U);
    CHECK_NEAR("east roundtrips", e, 12.5f, 1e-6f);
    CHECK_NEAR("north roundtrips", n, -4.5f, 1e-6f);
    CHECK_NEAR("heading roundtrips", h, 1.2f, 1e-6f);

    /* Corrupting one payload byte must invalidate the CRC. */
    ((uint8_t *) &rec)[8] ^= 0xFFU;
    CHECK("corrupted record fails validation",
          !flash_store_record_is_valid(&rec));
}

static void
test_blank_record_is_invalid(void)
{
    flash_store_record_t rec;
    memset(&rec, 0xFF, sizeof(rec));
    CHECK("all-0xFF (erased) record is not valid",
          !flash_store_record_is_valid(&rec));
}

static void
test_find_next_slot(void)
{
    uint8_t page[FLASH_STORE_PAGE_SIZE];
    memset(page, 0xFF, sizeof(page));

    CHECK("empty page: next slot is offset 0",
          flash_store_find_next_slot(page, sizeof(page)) == 0U);

    flash_store_record_t rec;
    flash_store_build_gate_record(0U, 0.0f, 0.0f, 0.0f, &rec);
    memcpy(page, &rec, sizeof(rec));

    CHECK("one record used: next slot is offset RECORD_SIZE",
          flash_store_find_next_slot(page, sizeof(page)) ==
              FLASH_STORE_RECORD_SIZE);

    memset(page, 0x00, sizeof(page)); /* fully "used" (not blank anywhere) */
    CHECK("full page: next slot is page_size (full)",
          flash_store_find_next_slot(page, sizeof(page)) ==
              FLASH_STORE_PAGE_SIZE);
}

/* Appending two records for the same gate index (simulating the gate
 * being re-set twice) must restore the LATER one - append order is the
 * source of truth, not first-write-wins. */
static void
test_restore_latest_wins(void)
{
    uint8_t page[FLASH_STORE_PAGE_SIZE];
    memset(page, 0xFF, sizeof(page));

    flash_store_record_t rec;
    flash_store_build_gate_record(1U, 1.0f, 1.0f, 0.0f, &rec);
    memcpy(&page[0], &rec, sizeof(rec));

    flash_store_build_gate_record(1U, 99.0f, 99.0f, 0.5f, &rec);
    memcpy(&page[FLASH_STORE_RECORD_SIZE], &rec, sizeof(rec));

    mag_cal_result_t cal;
    gates_init();
    flash_store_restore(page, sizeof(page), &cal);

    float e, n, h;
    CHECK("restored gate 1", gates_get(1U, &e, &n, &h) == STATUS_OK);
    CHECK_NEAR("latest east wins", e, 99.0f, 1e-6f);
    CHECK_NEAR("latest north wins", n, 99.0f, 1e-6f);

    /* No mag-cal record was written: restore must fall back to identity. */
    CHECK_NEAR("no mag-cal record -> identity bias", cal.bias[0], 0.0f,
               1e-6f);
    CHECK_NEAR("no mag-cal record -> identity scale", cal.scale[0], 1.0f,
               1e-6f);
}

static void
test_restore_mag_cal(void)
{
    uint8_t page[FLASH_STORE_PAGE_SIZE];
    memset(page, 0xFF, sizeof(page));

    mag_cal_result_t written = {{1.0f, 2.0f, 3.0f}, {1.1f, 1.2f, 1.3f}};
    flash_store_record_t rec;
    flash_store_build_mag_cal_record(&written, &rec);
    memcpy(&page[0], &rec, sizeof(rec));

    mag_cal_result_t restored;
    gates_init();
    flash_store_restore(page, sizeof(page), &restored);

    CHECK_NEAR("mag-cal bias restored", restored.bias[1], 2.0f, 1e-6f);
    CHECK_NEAR("mag-cal scale restored", restored.scale[2], 1.3f, 1e-6f);
}

int
main(void)
{
    test_build_and_validate_roundtrip();
    test_blank_record_is_invalid();
    test_find_next_slot();
    test_restore_latest_wins();
    test_restore_mag_cal();

    if (s_failures == 0) {
        printf("test_flash_store: all tests passed\n");
        return 0;
    }

    fprintf(stderr, "test_flash_store: %d failure(s)\n", s_failures);
    return 1;
}
