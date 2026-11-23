// Testes da maquina de estados de voo (flight_control.c) com replay de um voo simulado (RocketPy).
//
// O arquivo testado e o REAL (main/flight_control.c): o CMake gera uma copia em build/ e troca o
// global.h pelos stubs (ver CMakeLists.txt). Nao existe mais copia manual para manter em sincronia.
#include "readers/flight_log_reader/flight_log_reader.h"
#include "flight_control_under_test.c" // = main/flight_control.c, copiado pelo CMake
#include "unity/unity.h"

// Janela de aceitacao da latencia = (instante detectado) - (instante de referencia do RocketPy).
// Negativo = detectou ANTES da referencia; positivo = detectou DEPOIS.
#define MAX_EARLY_MS 250
#define MAX_LATE_MS  500

#define N_EXPECTED_STATES 6
static const flight_state_t expected_order[N_EXPECTED_STATES] = {STATE_BOOST, STATE_COAST, STATE_DROGUE,
                                                                 STATE_MAIN,  STATE_LANDING, STATE_LANDED};

typedef struct {
    flight_state_t state;
    uint32_t       time_ms;
    float          alt_m;
} transition_t;

static flight_telemetry_log_t *telemetry_log;
static bool                    expected_events_loaded;
static float                   alt_max;
static transition_t            transitions[16];
static size_t                  n_transitions;

static const char *flight_state_name(flight_state_t state) {
    switch (state) {
    case STATE_IDLE:    return "IDLE";
    case STATE_BOOST:   return "BOOST";
    case STATE_COAST:   return "COAST";
    case STATE_DROGUE:  return "DROGUE";
    case STATE_MAIN:    return "MAIN";
    case STATE_LANDING: return "LANDING";
    case STATE_LANDED:  return "LANDED";
    case STATE_FATAL:   return "FATAL";
    default:            return "UNKNOWN";
    }
}

// Estado do firmware e dos stubs volta ao inicial antes de CADA teste (principio "Independentes" do FIRST).
static void reset_firmware_state(void) {
    atomic_store(&data_g.flight_state, STATE_IDLE);
    atomic_store(&sys_flags_g, 0);
    db_boost = db_coast = db_drogue = db_main = db_landing = db_landed = 0;

    data_g.time        = 0;
    data_g.icm.az      = 0;
    data_g.icm.accel   = 0;
    data_g.kf.x.vz     = 0;
    data_g.kf.x.h      = 0;
    data_g.kf.x.apogee = 0;

    pyro_drogue.gpio  = DROGUE_GPIO;
    pyro_drogue.label = "DROGUE";
    pyro_main.gpio    = MAIN_GPIO;
    pyro_main.label   = "MAIN";

    memset(stub_gpio_high_count, 0, sizeof(stub_gpio_high_count));
    stub_rbf_level = 1; // pino RBF retirado => rbf_check() arma o sistema no primeiro passo em IDLE

    alt_max       = 0;
    n_transitions = 0;
}

void setUp(void) {
    reset_firmware_state();
    telemetry_log          = read_flight_telemetry_from_csv();
    expected_events_loaded = read_expected_events_from_txt();
}

void tearDown(void) {
    if (telemetry_log != NULL) {
        free(telemetry_log->samples);
        free(telemetry_log);
        telemetry_log = NULL;
    }
    free_expected_events();
}

// Reproduz o que o firmware teria em data_g a cada ciclo.
static void feed_sample(const flight_telemetry_sample_t *s) {
    if (s->altitude_m > alt_max)
        alt_max = s->altitude_m; // apogeu "visto ate agora" (o estimador real tambem nao conhece o futuro)

    float az_ms2 = s->vertical_acceleration_mps2;

    data_g.time    = s->timestamp_ms;
    data_g.icm.az  = az_ms2;
    data_g.kf.x.vz = s->vertical_velocity_mps;
    data_g.kf.x.h  = s->altitude_m;
    data_g.kf.x.apogee = alt_max;

    // icm.accel no firmware = |a| / G * 10 (uint8_t, gravidade ja removida). O CSV so tem a
    // componente vertical, entao usamos |az|: nao precisa exportar nada novo do RocketPy.
    float scaled   = fabsf(az_ms2) / G * 10.0f;
    data_g.icm.accel = (uint8_t)lroundf(scaled > 255.0f ? 255.0f : scaled);
}

// O RocketPy encerra a simulacao no impacto (vz ~ -6 m/s). Para o firmware poder confirmar LANDED
// (|vz| < 0.5 m/s por mais que o debounce), completamos o log com amostras de repouso no solo.
#define REST_SAMPLES_AFTER_TOUCHDOWN 20

static void process_sample(const flight_telemetry_sample_t *s, flight_state_t *prev) {
    feed_sample(s);
    flight_control_step();

    flight_state_t now = atomic_load(&data_g.flight_state);
    if (now != *prev && n_transitions < sizeof(transitions) / sizeof(transitions[0])) {
        transitions[n_transitions].state   = now;
        transitions[n_transitions].time_ms = data_g.time;
        transitions[n_transitions].alt_m   = data_g.kf.x.h;
        n_transitions++;
        *prev = now;
    }
}

// Passa o voo inteiro pela maquina de estados e registra cada transicao.
static void replay_flight(void) {
    flight_state_t prev = STATE_IDLE;
    n_transitions       = 0;

    for (size_t i = 0; i < telemetry_log->total_samples; i++)
        process_sample(&telemetry_log->samples[i], &prev);

    const flight_telemetry_sample_t *last = &telemetry_log->samples[telemetry_log->total_samples - 1];
    uint32_t dt = last->timestamp_ms - telemetry_log->samples[telemetry_log->total_samples - 2].timestamp_ms;
    for (uint32_t k = 1; k <= REST_SAMPLES_AFTER_TOUCHDOWN; k++) {
        flight_telemetry_sample_t rest = {.timestamp_ms = last->timestamp_ms + k * dt, .altitude_m = last->altitude_m,
                                          .vertical_velocity_mps = 0.0f, .vertical_acceleration_mps2 = 0.0f};
        process_sample(&rest, &prev);
    }
}

static int32_t latency_ms(const transition_t *t) {
    return (int32_t)t->time_ms - (int32_t)get_expected_event_trigger_time(t->state);
}

// Altitude do voo simulado no instante t (amostra mais proxima).
static float sim_altitude_at_ms(uint32_t t_ms) {
    size_t best = 0;
    for (size_t i = 1; i < telemetry_log->total_samples; i++) {
        uint32_t a = telemetry_log->samples[i].timestamp_ms, b = telemetry_log->samples[best].timestamp_ms;
        if ((a > t_ms ? a - t_ms : t_ms - a) < (b > t_ms ? b - t_ms : t_ms - b))
            best = i;
    }
    return telemetry_log->samples[best].altitude_m;
}

static void print_latency_table(void) {
    printf("\n  %-8s %13s %14s %13s %9s\n", "EVENTO", "ESPERADO[ms]", "DETECTADO[ms]", "LATENCIA[ms]", "ALT[m]");
    for (size_t i = 0; i < n_transitions; i++)
        printf("  %-8s %13u %14u %+13d %9.1f\n", flight_state_name(transitions[i].state),
               (unsigned)get_expected_event_trigger_time(transitions[i].state), (unsigned)transitions[i].time_ms,
               (int)latency_ms(&transitions[i]), transitions[i].alt_m);
    printf("  * MAIN: o RocketPy abre o principal a ~%.0f m; o firmware dispara em MAIN_THRESHOLD_ALT = %d m\n"
           "    (diferenca de parametro, nao de algoritmo -> avaliado por altitude, ver test_main_deploys_at_configured_altitude)\n\n",
           sim_altitude_at_ms(get_expected_event_trigger_time(STATE_MAIN)), (int)MAIN_THRESHOLD_ALT);
}

static void require_inputs(void) {
    TEST_ASSERT_NOT_NULL_MESSAGE(telemetry_log, "nao foi possivel ler data/flight_data.csv");
    TEST_ASSERT_TRUE_MESSAGE(expected_events_loaded, "nao foi possivel ler data/event_triggers.txt");
}

/* ============================ testes ============================ */

// Caixa-preta: todas as fases sao detectadas, uma vez cada, na ordem do voo.
void test_all_phases_detected_in_order(void) {
    require_inputs();
    replay_flight();
    print_latency_table();

    TEST_ASSERT_EQUAL_UINT_MESSAGE(N_EXPECTED_STATES, n_transitions, "nem todas as transicoes de voo foram detectadas");
    for (size_t i = 0; i < N_EXPECTED_STATES; i++)
        TEST_ASSERT_EQUAL_STRING(flight_state_name(expected_order[i]), flight_state_name(transitions[i].state));
    TEST_ASSERT_EQUAL(STATE_LANDED, atomic_load(&data_g.flight_state));
}

// Cada transicao cai dentro da janela [-MAX_EARLY_MS, +MAX_LATE_MS] em relacao ao RocketPy.
void test_transition_latency_within_window(void) {
    require_inputs();
    replay_flight();
    TEST_ASSERT_EQUAL_UINT_MESSAGE(N_EXPECTED_STATES, n_transitions, "nem todas as transicoes de voo foram detectadas");

    for (size_t i = 0; i < n_transitions; i++) {
        if (transitions[i].state == STATE_MAIN)
            continue; // evento por altitude: parametro do firmware (450 m) difere do simulado -> teste proprio abaixo
        int32_t lat = latency_ms(&transitions[i]);
        char    msg[96];
        snprintf(msg, sizeof(msg), "%s: latencia %+d ms fora de [-%d, +%d]", flight_state_name(transitions[i].state), (int)lat,
                 MAX_EARLY_MS, MAX_LATE_MS);
        TEST_ASSERT_TRUE_MESSAGE(lat >= -MAX_EARLY_MS && lat <= MAX_LATE_MS, msg);
    }
}

// O principal e acionado por ALTITUDE: avalia-se a altitude em que disparou, e nao o instante.
void test_main_deploys_at_configured_altitude(void) {
    require_inputs();
    replay_flight();

    const transition_t *main_t = NULL;
    for (size_t i = 0; i < n_transitions; i++)
        if (transitions[i].state == STATE_MAIN)
            main_t = &transitions[i];

    TEST_ASSERT_NOT_NULL_MESSAGE(main_t, "MAIN nao foi detectado");
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(10.0f, (float)MAIN_THRESHOLD_ALT - 5.0f, main_t->alt_m,
                                     "main disparou longe da altitude configurada");
    TEST_ASSERT_TRUE_MESSAGE(main_t->alt_m < (float)MAIN_THRESHOLD_ALT, "main disparou acima da altitude configurada");
}

// Seguranca: com o sistema armado, drogue e main disparam exatamente uma vez cada.
void test_pyros_fire_exactly_once_when_armed(void) {
    require_inputs();
    replay_flight();

    TEST_ASSERT_EQUAL_INT_MESSAGE(1, stub_gpio_high_count[DROGUE_GPIO], "drogue deveria disparar 1 vez");
    TEST_ASSERT_EQUAL_INT_MESSAGE(1, stub_gpio_high_count[MAIN_GPIO], "main deveria disparar 1 vez");
    TEST_ASSERT_TRUE((atomic_load(&sys_flags_g) & STATUS_DROGUE_DEPLOYED) != 0);
    TEST_ASSERT_TRUE((atomic_load(&sys_flags_g) & STATUS_MAIN_DEPLOYED) != 0);
}

// Seguranca (teste de defeitos): com o RBF inserido (desarmado), NENHUM pirotecnico pode disparar.
void test_pyros_stay_off_when_not_armed(void) {
    require_inputs();
    stub_rbf_level = 0;
    replay_flight();

    TEST_ASSERT_EQUAL_INT_MESSAGE(0, stub_gpio_high_count[DROGUE_GPIO], "drogue disparou com sistema desarmado");
    TEST_ASSERT_EQUAL_INT_MESSAGE(0, stub_gpio_high_count[MAIN_GPIO], "main disparou com sistema desarmado");
    TEST_ASSERT_TRUE_MESSAGE(atomic_load(&data_g.flight_state) < STATE_DROGUE, "estado avancou alem de COAST sem armar");
}

// Defeitos: um unico pico de aceleracao (ruido) nao pode disparar BOOST por causa do debounce.
void test_single_sample_spike_does_not_trigger_boost(void) {
    flight_telemetry_sample_t spike = {.timestamp_ms = 1000, .altitude_m = 0, .vertical_velocity_mps = 5.0f,
                                       .vertical_acceleration_mps2 = 3.0f * G};
    flight_telemetry_sample_t calm  = {.timestamp_ms = 1121, .altitude_m = 0, .vertical_velocity_mps = 5.0f,
                                       .vertical_acceleration_mps2 = 0.0f};
    feed_sample(&spike);
    flight_control_step();
    feed_sample(&calm);
    flight_control_step();

    TEST_ASSERT_EQUAL(STATE_IDLE, atomic_load(&data_g.flight_state));
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_all_phases_detected_in_order);
    RUN_TEST(test_transition_latency_within_window);
    RUN_TEST(test_main_deploys_at_configured_altitude);
    RUN_TEST(test_pyros_fire_exactly_once_when_armed);
    RUN_TEST(test_pyros_stay_off_when_not_armed);
    RUN_TEST(test_single_sample_spike_does_not_trigger_boost);
    return UNITY_END();
}
