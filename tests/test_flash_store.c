/**
 * @file    test_flash_store.c
 * @brief   Host tests for persist/flash_store.c.
 *
 * Gates persist as absolute lat/lon (i32 1e-7) with a per-slot valid flag;
 * restore replays the append log (newest-per-slot wins, clears and a
 * clear-all marker void slots) into a caller-provided table rather than
 * touching gates.c, since the ENU origin isn't known at boot.
 */

#include "persist/flash_store.h"

#include <string.h>
#include <stdio.h>

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
    flash_store_build_gate_record(3U, 509876543, -11234567, 1.2f, true,
                                   &rec);

    CHECK("built gate record is valid", flash_store_record_is_valid(&rec));

    uint8_t index;
    int32_t lat, lon;
    float h;
    bool valid;
    CHECK("decodes as a gate",
          flash_store_decode_gate(&rec, &index, &lat, &lon, &h, &valid));
    CHECK("index roundtrips", index == 3U);
    CHECK("lat roundtrips", lat == 509876543);
    CHECK("lon roundtrips", lon == -11234567);
    CHECK("valid roundtrips", valid == true);
    CHECK_NEAR("heading roundtrips", h, 1.2f, 1e-6f);

    /* Corrupting one payload byte must invalidate the CRC. */
    ((uint8_t *) &rec)[8] ^= 0xFFU;
    CHECK("corrupted record fails validation",
          !flash_store_record_is_valid(&rec));
}

static void
test_cleared_record_roundtrip(void)
{
    flash_store_record_t rec;
    flash_store_build_gate_record(2U, 0, 0, 0.0f, false, &rec);

    uint8_t index;
    int32_t lat, lon;
    float h;
    bool valid;
    CHECK("cleared record decodes as a gate",
          flash_store_decode_gate(&rec, &index, &lat, &lon, &h, &valid));
    CHECK("cleared record has valid=false", valid == false);
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
    flash_store_build_gate_record(0U, 0, 0, 0.0f, true, &rec);
    memcpy(page, &rec, sizeof(rec));

    CHECK("one record used: next slot is offset RECORD_SIZE",
          flash_store_find_next_slot(page, sizeof(page)) ==
              FLASH_STORE_RECORD_SIZE);

    memset(page, 0x00, sizeof(page)); /* fully "used" (not blank anywhere) */
    CHECK("full page: next slot is page_size (full)",
          flash_store_find_next_slot(page, sizeof(page)) ==
              FLASH_STORE_PAGE_SIZE);
}

/* Helper: append a record at the next free slot of a page buffer. */
static void
append(uint8_t *page, const flash_store_record_t *rec)
{
    uint32_t off = flash_store_find_next_slot(page, FLASH_STORE_PAGE_SIZE);
    memcpy(&page[off], rec, sizeof(*rec));
}

/* Appending two records for the same gate index (the gate re-set twice)
 * must restore the LATER one - append order is the source of truth. */
static void
test_restore_latest_wins(void)
{
    uint8_t page[FLASH_STORE_PAGE_SIZE];
    memset(page, 0xFF, sizeof(page));

    flash_store_record_t rec;
    flash_store_build_gate_record(1U, 100, 100, 0.0f, true, &rec);
    append(page, &rec);
    flash_store_build_gate_record(1U, 999, 999, 0.5f, true, &rec);
    append(page, &rec);

    flash_store_gate_t gates[FLASH_STORE_MAX_GATES];
    mag_cal_result_t cal;
    flash_store_restore(page, sizeof(page), gates, FLASH_STORE_MAX_GATES,
                        &cal);

    CHECK("gate 1 restored valid", gates[1].valid);
    CHECK("latest lat wins", gates[1].lat_1e7 == 999);
    CHECK("latest lon wins", gates[1].lon_1e7 == 999);

    /* No mag-cal record was written: restore must fall back to identity. */
    CHECK_NEAR("no mag-cal record -> identity bias", cal.bias[0], 0.0f,
               1e-6f);
    CHECK_NEAR("no mag-cal record -> identity scale", cal.scale[0], 1.0f,
               1e-6f);
}

/* A later cleared-record for a slot must void an earlier set (a
 * clear-then-power-cycle must not resurrect the old gate). */
static void
test_restore_clear_supersedes_set(void)
{
    uint8_t page[FLASH_STORE_PAGE_SIZE];
    memset(page, 0xFF, sizeof(page));

    flash_store_record_t rec;
    flash_store_build_gate_record(4U, 500, 500, 0.0f, true, &rec);
    append(page, &rec);
    flash_store_build_gate_record(4U, 0, 0, 0.0f, false, &rec); /* cleared */
    append(page, &rec);

    flash_store_gate_t gates[FLASH_STORE_MAX_GATES];
    mag_cal_result_t cal;
    flash_store_restore(page, sizeof(page), gates, FLASH_STORE_MAX_GATES,
                        &cal);

    CHECK("cleared gate stays cleared after restore", !gates[4].valid);
}

/* A clear-all marker voids every slot set before it; sets after it survive
 * (this is exactly the "new start/finish wipes sectors, then places gate
 * 0" sequence). */
static void
test_restore_clear_all(void)
{
    uint8_t page[FLASH_STORE_PAGE_SIZE];
    memset(page, 0xFF, sizeof(page));

    flash_store_record_t rec;
    flash_store_build_gate_record(1U, 10, 10, 0.0f, true, &rec);
    append(page, &rec);
    flash_store_build_gate_record(2U, 20, 20, 0.0f, true, &rec);
    append(page, &rec);
    flash_store_build_gate_clear_all_record(&rec);
    append(page, &rec);
    flash_store_build_gate_record(0U, 30, 30, 0.0f, true, &rec);
    append(page, &rec);

    flash_store_gate_t gates[FLASH_STORE_MAX_GATES];
    mag_cal_result_t cal;
    flash_store_restore(page, sizeof(page), gates, FLASH_STORE_MAX_GATES,
                        &cal);

    CHECK("clear-all voided sector 1", !gates[1].valid);
    CHECK("clear-all voided sector 2", !gates[2].valid);
    CHECK("gate 0 set after clear-all survives", gates[0].valid);
    CHECK("gate 0 lat survives", gates[0].lat_1e7 == 30);
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

    flash_store_gate_t gates[FLASH_STORE_MAX_GATES];
    mag_cal_result_t restored;
    flash_store_restore(page, sizeof(page), gates, FLASH_STORE_MAX_GATES,
                        &restored);

    CHECK_NEAR("mag-cal bias restored", restored.bias[1], 2.0f, 1e-6f);
    CHECK_NEAR("mag-cal scale restored", restored.scale[2], 1.3f, 1e-6f);
}

/* ------------------------------------------------------------------ */
/* A/B page selection (interrupted-compaction recovery)                */
/* ------------------------------------------------------------------ */

static uint8_t s_page0[FLASH_STORE_PAGE_SIZE];
static uint8_t s_page1[FLASH_STORE_PAGE_SIZE];

static void
blank_pages(void)
{
    memset(s_page0, 0xFF, sizeof(s_page0));
    memset(s_page1, 0xFF, sizeof(s_page1));
}

static void
append_gate(uint8_t *page, uint8_t index)
{
    flash_store_record_t rec;
    flash_store_build_gate_record(index, 509000000 + index, -13000000, 0.5f,
                                   true, &rec);
    append(page, &rec);
}

static void
append_done(uint8_t *page, uint32_t generation)
{
    flash_store_record_t rec;
    flash_store_build_compact_done_record(generation, &rec);
    append(page, &rec);
}

static uint32_t
select_page(void)
{
    return flash_store_select_active_page(s_page0, s_page1,
                                           FLASH_STORE_PAGE_SIZE);
}

static void
test_free_records_and_in_use(void)
{
    blank_pages();
    CHECK("blank page: every slot free",
          flash_store_free_records(s_page0, FLASH_STORE_PAGE_SIZE) ==
              FLASH_STORE_MAX_RECORDS);
    CHECK("blank page is not in use",
          !flash_store_page_in_use(s_page0, FLASH_STORE_PAGE_SIZE));

    append_gate(s_page0, 0U);
    append_gate(s_page0, 1U);
    CHECK("two records appended: two fewer free",
          flash_store_free_records(s_page0, FLASH_STORE_PAGE_SIZE) ==
              FLASH_STORE_MAX_RECORDS - 2U);
    CHECK("page with records is in use",
          flash_store_page_in_use(s_page0, FLASH_STORE_PAGE_SIZE));

    /* Garbage that is neither blank nor CRC-valid (a half-erased page)
     * occupies slots but doesn't make the page "in use". */
    memset(s_page1, 0x5A, sizeof(s_page1));
    CHECK("garbage page: no free slots",
          flash_store_free_records(s_page1, FLASH_STORE_PAGE_SIZE) == 0U);
    CHECK("garbage page is not in use",
          !flash_store_page_in_use(s_page1, FLASH_STORE_PAGE_SIZE));
}

static void
test_compact_done_marker(void)
{
    blank_pages();
    uint32_t gen = 123U;

    append_gate(s_page0, 0U);
    CHECK("no marker: page has no generation",
          !flash_store_page_generation(s_page0, FLASH_STORE_PAGE_SIZE, &gen));
    CHECK("no marker: generation reads 0", gen == 0U);

    append_done(s_page0, 7U);
    append_gate(s_page0, 1U); /* later appends don't hide the marker */
    CHECK("marker found",
          flash_store_page_generation(s_page0, FLASH_STORE_PAGE_SIZE, &gen));
    CHECK("marker generation roundtrips", gen == 7U);

    /* The marker is inert to restore: gates around it still replay. */
    flash_store_gate_t gates[FLASH_STORE_MAX_GATES];
    mag_cal_result_t cal;
    flash_store_restore(s_page0, FLASH_STORE_PAGE_SIZE, gates,
                         FLASH_STORE_MAX_GATES, &cal);
    CHECK("gate before marker restored", gates[0].valid);
    CHECK("gate after marker restored", gates[1].valid);
}

static void
test_select_single_page(void)
{
    blank_pages();
    CHECK("first boot (both blank): page 0", select_page() == 0U);

    append_gate(s_page0, 0U);
    CHECK("only page 0 in use: page 0", select_page() == 0U);

    blank_pages();
    append_gate(s_page1, 0U);
    append_done(s_page1, 1U);
    CHECK("only page 1 in use: page 1", select_page() == 1U);
}

/* Reset mid-rewrite: the target holds some records but no completion
 * marker. The complete source must win whichever page it is - the
 * regression here is the old "both in use => page 0" rule, which picked
 * the partial copy (and erased the good one) whenever the source was
 * page 1. */
static void
test_select_interrupted_rewrite(void)
{
    /* Never-compacted source on page 0, partial target on page 1. */
    blank_pages();
    append_gate(s_page0, 0U);
    append_gate(s_page0, 1U);
    append_gate(s_page1, 0U);
    CHECK("partial target on page 1: source page 0 wins",
          select_page() == 0U);

    /* Source on page 1 (itself a completed compaction), partial target
     * on page 0. */
    blank_pages();
    append_gate(s_page1, 0U);
    append_gate(s_page1, 1U);
    append_done(s_page1, 1U);
    append_gate(s_page1, 2U);
    append_gate(s_page0, 0U);
    CHECK("partial target on page 0: source page 1 wins",
          select_page() == 1U);

    /* Same again, one compaction later: source on page 0 with a marker. */
    blank_pages();
    append_gate(s_page0, 0U);
    append_done(s_page0, 2U);
    append_gate(s_page1, 0U);
    CHECK("partial target on page 1: marked source page 0 wins",
          select_page() == 0U);
}

/* Reset after the rewrite completed but before (or during) the erase of
 * the old page: the target carries the next generation and must win,
 * because the old page may be half-erased - here it has lost its newest
 * record, so preferring it would roll the gate table back. */
static void
test_select_interrupted_erase(void)
{
    blank_pages();
    append_gate(s_page0, 0U); /* old page, its later records erased away */
    append_gate(s_page1, 0U);
    append_gate(s_page1, 1U);
    append_done(s_page1, 1U);
    CHECK("completed target (gen 1) beats unmarked old page",
          select_page() == 1U);

    blank_pages();
    append_gate(s_page1, 0U);
    append_done(s_page1, 3U); /* old page */
    append_gate(s_page0, 0U);
    append_gate(s_page0, 1U);
    append_done(s_page0, 4U); /* completed target */
    CHECK("higher generation wins (page 0)", select_page() == 0U);

    blank_pages();
    append_done(s_page0, 4U);
    append_done(s_page1, 5U);
    CHECK("higher generation wins (page 1)", select_page() == 1U);

    /* Generation counter wrap: 0 follows 0xFFFFFFFF. */
    blank_pages();
    append_done(s_page0, 0xFFFFFFFFU);
    append_done(s_page1, 0U);
    CHECK("generation comparison survives wrap", select_page() == 1U);
}

int
main(void)
{
    test_free_records_and_in_use();
    test_compact_done_marker();
    test_select_single_page();
    test_select_interrupted_rewrite();
    test_select_interrupted_erase();
    test_build_and_validate_roundtrip();
    test_cleared_record_roundtrip();
    test_blank_record_is_invalid();
    test_find_next_slot();
    test_restore_latest_wins();
    test_restore_clear_supersedes_set();
    test_restore_clear_all();
    test_restore_mag_cal();

    if (s_failures == 0) {
        printf("test_flash_store: all tests passed\n");
        return 0;
    }

    fprintf(stderr, "test_flash_store: %d failure(s)\n", s_failures);
    return 1;
}
