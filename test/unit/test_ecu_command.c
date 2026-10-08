#include "test_runner.h"
#include "ecu_command.h"

/* File FIFO simulée (queue_tx côté OUTPUT) : enregistre l'ordre des trames
 * et peut simuler une queue pleine. Les fonctions d'ecu_command étant
 * exécutées sous une seule prise de mutex sur cible, une séquence d'appels
 * ici représente n'importe quel entrelacement possible entre task_control
 * et task_failsafe. */
#define FAKE_QUEUE_CAP 32

typedef struct {
    float values[FAKE_QUEUE_CAP];
    int   count;
    int   free_slots;   /* places restantes ; 0 = queue pleine */
} fake_queue_t;

static void fq_init(fake_queue_t *q, int free_slots)
{
    memset(q, 0, sizeof(*q));
    q->free_slots = free_slots;
}

static bool fq_emit(float output, void *ctx)
{
    fake_queue_t *q = (fake_queue_t *)ctx;
    if (q->free_slots <= 0 || q->count >= FAKE_QUEUE_CAP) return false;
    q->values[q->count++] = output;
    q->free_slots--;
    return true;
}

static ecu_command_t make_auto(void)
{
    ecu_command_t c;
    ecu_command_init(&c);
    (void)ecu_command_try_set_mode(&c, ECU_MODE_AUTO);
    return c;
}

/* ========================================================================== */
/* commit_output                                                              */
/* ========================================================================== */

static void test_cmd_init_off_zero_unlocked(void)
{
    ecu_command_t c;
    ecu_command_init(&c);
    TEST_ASSERT_EQUAL_INT(ECU_MODE_OFF, c.mode);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, c.output);
    TEST_ASSERT_FALSE(c.failsafe_locked);
}

static void test_cmd_commit_auto_emits_requested(void)
{
    ecu_command_t c = make_auto();
    fake_queue_t q; fq_init(&q, 8);
    TEST_ASSERT_TRUE(ecu_command_commit_output(&c, 42.0f, fq_emit, &q));
    TEST_ASSERT_EQUAL_INT(1, q.count);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 42.0f, q.values[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 42.0f, c.output);
}

static void test_cmd_commit_off_and_manual_emit_zero(void)
{
    ecu_command_t c;
    ecu_command_init(&c);
    fake_queue_t q; fq_init(&q, 8);
    TEST_ASSERT_TRUE(ecu_command_commit_output(&c, 99.0f, fq_emit, &q));
    (void)ecu_command_try_set_mode(&c, ECU_MODE_MANUAL);
    TEST_ASSERT_TRUE(ecu_command_commit_output(&c, 99.0f, fq_emit, &q));
    TEST_ASSERT_EQUAL_INT(2, q.count);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, q.values[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, q.values[1]);
}

static void test_cmd_commit_queue_full_returns_false_state_updated(void)
{
    ecu_command_t c = make_auto();
    fake_queue_t q; fq_init(&q, 0);
    TEST_ASSERT_FALSE(ecu_command_commit_output(&c, 10.0f, fq_emit, &q));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 10.0f, c.output);
    TEST_ASSERT_EQUAL_INT(0, q.count);
}

/* ========================================================================== */
/* enter_failsafe                                                             */
/* ========================================================================== */

static void test_cmd_failsafe_off_zero_and_emits_zero(void)
{
    ecu_command_t c = make_auto();
    fake_queue_t q; fq_init(&q, 8);
    (void)ecu_command_commit_output(&c, 80.0f, fq_emit, &q);
    TEST_ASSERT_TRUE(ecu_command_enter_failsafe(&c, false, fq_emit, &q));
    TEST_ASSERT_EQUAL_INT(ECU_MODE_OFF, c.mode);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, c.output);
    TEST_ASSERT_FALSE(c.failsafe_locked);
    TEST_ASSERT_EQUAL_INT(2, q.count);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, q.values[1]);
}

static void test_cmd_failsafe_lock_sets_lock(void)
{
    ecu_command_t c = make_auto();
    fake_queue_t q; fq_init(&q, 8);
    (void)ecu_command_enter_failsafe(&c, true, fq_emit, &q);
    TEST_ASSERT_TRUE(c.failsafe_locked);
}

static void test_cmd_failsafe_queue_full_still_safe_state(void)
{
    ecu_command_t c = make_auto();
    fake_queue_t q; fq_init(&q, 0);
    TEST_ASSERT_FALSE(ecu_command_enter_failsafe(&c, true, fq_emit, &q));
    TEST_ASSERT_EQUAL_INT(ECU_MODE_OFF, c.mode);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, c.output);
    TEST_ASSERT_TRUE(c.failsafe_locked);
}

/* ========================================================================== */
/* Ordonnancement des trames OUTPUT                                           */
/* ========================================================================== */

/* Entrelacement type : commits AUTO, failsafe, puis commits avec une commande
 * PID déjà calculée (requested non nul). OUTPUT(0) du failsafe suit toutes les
 * commandes non nulles et toutes les trames suivantes valent 0. */
static void test_cmd_order_zero_after_last_nonzero(void)
{
    ecu_command_t c = make_auto();
    fake_queue_t q; fq_init(&q, FAKE_QUEUE_CAP);
    (void)ecu_command_commit_output(&c, 50.0f, fq_emit, &q);
    (void)ecu_command_commit_output(&c, 60.0f, fq_emit, &q);
    (void)ecu_command_enter_failsafe(&c, true, fq_emit, &q);
    const int fs_idx = q.count - 1;
    (void)ecu_command_commit_output(&c, 70.0f, fq_emit, &q);   /* PID calculé avant le failsafe */
    (void)ecu_command_commit_output(&c, 75.0f, fq_emit, &q);

    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, q.values[fs_idx]);
    for (int i = 0; i < fs_idx; i++) {
        TEST_ASSERT_TRUE(q.values[i] > 0.0f);
    }
    for (int i = fs_idx; i < q.count; i++) {
        TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, q.values[i]);
    }
}

/* Propriété sur toutes les positions possibles du failsafe dans une suite de
 * N commits : aucune trame non nulle après la première trame du failsafe. */
static void test_cmd_order_all_failsafe_positions(void)
{
    const int N = 10;
    for (int fs_at = 0; fs_at <= N; fs_at++) {
        ecu_command_t c = make_auto();
        fake_queue_t q; fq_init(&q, FAKE_QUEUE_CAP);
        int fs_idx = -1;
        for (int k = 0; k <= N; k++) {
            if (k == fs_at) {
                (void)ecu_command_enter_failsafe(&c, true, fq_emit, &q);
                fs_idx = q.count - 1;
            }
            if (k < N) {
                (void)ecu_command_commit_output(&c, 10.0f + (float)k, fq_emit, &q);
            }
        }
        int nonzero_after = 0;
        for (int i = fs_idx; i < q.count; i++) {
            if (q.values[i] != 0.0f) nonzero_after++;
        }
        TEST_ASSERT_EQUAL_INT(0, nonzero_after);
    }
}

/* Queue pleine au failsafe : OUTPUT(0) perdu, mais le commit suivant de
 * task_control (≤ 100 ms) émet 0, jamais la commande PID déjà calculée. */
static void test_cmd_failsafe_dropped_next_commit_emits_zero(void)
{
    ecu_command_t c = make_auto();
    fake_queue_t q; fq_init(&q, 1);
    (void)ecu_command_commit_output(&c, 50.0f, fq_emit, &q);    /* remplit la queue */
    TEST_ASSERT_FALSE(ecu_command_enter_failsafe(&c, true, fq_emit, &q));
    q.free_slots = 4;                                            /* task_tx a vidé */
    TEST_ASSERT_TRUE(ecu_command_commit_output(&c, 55.0f, fq_emit, &q));
    TEST_ASSERT_EQUAL_INT(2, q.count);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, q.values[1]);
}

/* ========================================================================== */
/* Verrou et reprise                                                          */
/* ========================================================================== */

static void test_cmd_locked_refuses_auto_allows_others(void)
{
    ecu_command_t c = make_auto();
    fake_queue_t q; fq_init(&q, 8);
    (void)ecu_command_enter_failsafe(&c, true, fq_emit, &q);
    TEST_ASSERT_FALSE(ecu_command_try_set_mode(&c, ECU_MODE_AUTO));
    TEST_ASSERT_EQUAL_INT(ECU_MODE_OFF, c.mode);
    TEST_ASSERT_TRUE(ecu_command_try_set_mode(&c, ECU_MODE_MANUAL));
    TEST_ASSERT_TRUE(ecu_command_try_set_mode(&c, ECU_MODE_OFF));
}

static void test_cmd_unlock_then_auto_resumes_output(void)
{
    ecu_command_t c = make_auto();
    fake_queue_t q; fq_init(&q, 8);
    (void)ecu_command_enter_failsafe(&c, true, fq_emit, &q);
    ecu_command_set_failsafe_locked(&c, false);
    TEST_ASSERT_TRUE(ecu_command_try_set_mode(&c, ECU_MODE_AUTO));
    (void)ecu_command_commit_output(&c, 33.0f, fq_emit, &q);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 33.0f, q.values[q.count - 1]);
}

/* Failsafe UART (sans verrou) : AUTO de nouveau accepté au MODE_SET suivant. */
static void test_cmd_unlocked_failsafe_allows_auto(void)
{
    ecu_command_t c = make_auto();
    fake_queue_t q; fq_init(&q, 8);
    (void)ecu_command_enter_failsafe(&c, false, fq_emit, &q);
    TEST_ASSERT_TRUE(ecu_command_try_set_mode(&c, ECU_MODE_AUTO));
}

static void test_cmd_null_safe(void)
{
    fake_queue_t q; fq_init(&q, 8);
    ecu_command_t c;
    ecu_command_init(&c);
    ecu_command_init(NULL);
    ecu_command_set_failsafe_locked(NULL, true);
    TEST_ASSERT_FALSE(ecu_command_try_set_mode(NULL, ECU_MODE_OFF));
    TEST_ASSERT_FALSE(ecu_command_commit_output(NULL, 1.0f, fq_emit, &q));
    TEST_ASSERT_FALSE(ecu_command_enter_failsafe(NULL, true, fq_emit, &q));
    TEST_ASSERT_FALSE(ecu_command_commit_output(&c, 1.0f, NULL, &q));
    TEST_ASSERT_FALSE(ecu_command_enter_failsafe(&c, true, NULL, &q));
    TEST_ASSERT_EQUAL_INT(ECU_MODE_OFF, c.mode);   /* l'état passe quand même en OFF */
    TEST_ASSERT_EQUAL_INT(0, q.count);
}

int main(void)
{
    TEST_SUITE_BEGIN("ecu_command");

    RUN_TEST(test_cmd_init_off_zero_unlocked);
    RUN_TEST(test_cmd_commit_auto_emits_requested);
    RUN_TEST(test_cmd_commit_off_and_manual_emit_zero);
    RUN_TEST(test_cmd_commit_queue_full_returns_false_state_updated);
    RUN_TEST(test_cmd_failsafe_off_zero_and_emits_zero);
    RUN_TEST(test_cmd_failsafe_lock_sets_lock);
    RUN_TEST(test_cmd_failsafe_queue_full_still_safe_state);
    RUN_TEST(test_cmd_order_zero_after_last_nonzero);
    RUN_TEST(test_cmd_order_all_failsafe_positions);
    RUN_TEST(test_cmd_failsafe_dropped_next_commit_emits_zero);
    RUN_TEST(test_cmd_locked_refuses_auto_allows_others);
    RUN_TEST(test_cmd_unlock_then_auto_resumes_output);
    RUN_TEST(test_cmd_unlocked_failsafe_allows_auto);
    RUN_TEST(test_cmd_null_safe);

    TEST_SUITE_SUMMARY();
}
