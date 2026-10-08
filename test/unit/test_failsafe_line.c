#include "test_runner.h"
#include "failsafe_line.h"

/* ========================================================================== */
/* Déclenchement                                                              */
/* ========================================================================== */

static void test_line_init_unlatched(void)
{
    failsafe_line_t l;
    failsafe_line_init(&l);
    TEST_ASSERT_FALSE(l.latched);
    TEST_ASSERT_EQUAL_INT(0, l.closed_polls);
}

static void test_line_first_edge_trips(void)
{
    failsafe_line_t l;
    failsafe_line_init(&l);
    TEST_ASSERT_EQUAL_INT(FAILSAFE_LINE_TRIP, failsafe_line_on_edge(&l));
    TEST_ASSERT_TRUE(l.latched);
}

static void test_line_bounce_edges_single_trip(void)
{
    failsafe_line_t l;
    failsafe_line_init(&l);
    int trips = 0;
    for (int i = 0; i < 20; i++) {
        if (failsafe_line_on_edge(&l) == FAILSAFE_LINE_TRIP) trips++;
    }
    TEST_ASSERT_EQUAL_INT(1, trips);
}

static void test_line_open_level_trips_once(void)
{
    failsafe_line_t l;
    failsafe_line_init(&l);
    TEST_ASSERT_EQUAL_INT(FAILSAFE_LINE_TRIP, failsafe_line_on_level(&l, true));
    TEST_ASSERT_EQUAL_INT(FAILSAFE_LINE_NONE, failsafe_line_on_level(&l, true));
    TEST_ASSERT_EQUAL_INT(FAILSAFE_LINE_NONE, failsafe_line_on_level(&l, true));
}

/* Ligne déjà ouverte au boot : aucun front, seul le niveau la révèle. */
static void test_line_open_at_boot_detected_by_level(void)
{
    failsafe_line_t l;
    failsafe_line_init(&l);
    TEST_ASSERT_EQUAL_INT(FAILSAFE_LINE_TRIP, failsafe_line_on_level(&l, true));
    TEST_ASSERT_TRUE(l.latched);
}

static void test_line_edge_then_level_no_second_trip(void)
{
    failsafe_line_t l;
    failsafe_line_init(&l);
    TEST_ASSERT_EQUAL_INT(FAILSAFE_LINE_TRIP, failsafe_line_on_edge(&l));
    TEST_ASSERT_EQUAL_INT(FAILSAFE_LINE_NONE, failsafe_line_on_level(&l, true));
}

static void test_line_closed_level_when_idle_does_nothing(void)
{
    failsafe_line_t l;
    failsafe_line_init(&l);
    for (int i = 0; i < 5; i++) {
        TEST_ASSERT_EQUAL_INT(FAILSAFE_LINE_NONE, failsafe_line_on_level(&l, false));
    }
    TEST_ASSERT_FALSE(l.latched);
}

/* ========================================================================== */
/* Déverrouillage                                                             */
/* ========================================================================== */

static void test_line_unlock_needs_consecutive_closed_polls(void)
{
    failsafe_line_t l;
    failsafe_line_init(&l);
    (void)failsafe_line_on_edge(&l);

    for (uint32_t i = 1; i < FAILSAFE_LINE_CLOSED_POLLS_TO_UNLOCK; i++) {
        TEST_ASSERT_EQUAL_INT(FAILSAFE_LINE_NONE, failsafe_line_on_level(&l, false));
        TEST_ASSERT_TRUE(l.latched);
    }
    TEST_ASSERT_EQUAL_INT(FAILSAFE_LINE_UNLOCK, failsafe_line_on_level(&l, false));
    TEST_ASSERT_FALSE(l.latched);
}

/* Rebond à la remise du jumper : un niveau ouvert intercalé remet le
 * compteur à zéro, sans nouvelle ALARM (déjà verrouillé). */
static void test_line_reopen_during_unlock_resets_count(void)
{
    failsafe_line_t l;
    failsafe_line_init(&l);
    (void)failsafe_line_on_edge(&l);

    (void)failsafe_line_on_level(&l, false);
    TEST_ASSERT_EQUAL_INT(FAILSAFE_LINE_NONE, failsafe_line_on_level(&l, true));
    TEST_ASSERT_EQUAL_INT(0, l.closed_polls);
    TEST_ASSERT_TRUE(l.latched);

    for (uint32_t i = 1; i < FAILSAFE_LINE_CLOSED_POLLS_TO_UNLOCK; i++) {
        TEST_ASSERT_EQUAL_INT(FAILSAFE_LINE_NONE, failsafe_line_on_level(&l, false));
    }
    TEST_ASSERT_EQUAL_INT(FAILSAFE_LINE_UNLOCK, failsafe_line_on_level(&l, false));
}

static void test_line_edge_during_unlock_no_new_trip(void)
{
    failsafe_line_t l;
    failsafe_line_init(&l);
    (void)failsafe_line_on_edge(&l);
    (void)failsafe_line_on_level(&l, false);
    TEST_ASSERT_EQUAL_INT(FAILSAFE_LINE_NONE, failsafe_line_on_edge(&l));
    TEST_ASSERT_EQUAL_INT(0, l.closed_polls);
}

static void test_line_unlock_emitted_once(void)
{
    failsafe_line_t l;
    failsafe_line_init(&l);
    (void)failsafe_line_on_edge(&l);
    int unlocks = 0;
    for (int i = 0; i < 10; i++) {
        if (failsafe_line_on_level(&l, false) == FAILSAFE_LINE_UNLOCK) unlocks++;
    }
    TEST_ASSERT_EQUAL_INT(1, unlocks);
}

/* Nouvelle ouverture après une reprise complète : nouvelle ALARM. */
static void test_line_retrip_after_unlock(void)
{
    failsafe_line_t l;
    failsafe_line_init(&l);
    (void)failsafe_line_on_edge(&l);
    for (uint32_t i = 0; i < FAILSAFE_LINE_CLOSED_POLLS_TO_UNLOCK; i++) {
        (void)failsafe_line_on_level(&l, false);
    }
    TEST_ASSERT_FALSE(l.latched);
    TEST_ASSERT_EQUAL_INT(FAILSAFE_LINE_TRIP, failsafe_line_on_edge(&l));
}

static void test_line_null_safe(void)
{
    failsafe_line_init(NULL);
    TEST_ASSERT_EQUAL_INT(FAILSAFE_LINE_NONE, failsafe_line_on_edge(NULL));
    TEST_ASSERT_EQUAL_INT(FAILSAFE_LINE_NONE, failsafe_line_on_level(NULL, true));
}

int main(void)
{
    TEST_SUITE_BEGIN("failsafe_line");

    RUN_TEST(test_line_init_unlatched);
    RUN_TEST(test_line_first_edge_trips);
    RUN_TEST(test_line_bounce_edges_single_trip);
    RUN_TEST(test_line_open_level_trips_once);
    RUN_TEST(test_line_open_at_boot_detected_by_level);
    RUN_TEST(test_line_edge_then_level_no_second_trip);
    RUN_TEST(test_line_closed_level_when_idle_does_nothing);
    RUN_TEST(test_line_unlock_needs_consecutive_closed_polls);
    RUN_TEST(test_line_reopen_during_unlock_resets_count);
    RUN_TEST(test_line_edge_during_unlock_no_new_trip);
    RUN_TEST(test_line_unlock_emitted_once);
    RUN_TEST(test_line_retrip_after_unlock);
    RUN_TEST(test_line_null_safe);

    TEST_SUITE_SUMMARY();
}
