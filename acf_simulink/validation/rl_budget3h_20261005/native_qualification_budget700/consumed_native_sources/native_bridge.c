/* Packaging and measurement adapter around Simulink Coder's native plant.
 * No mechanics, coil, sensor, observer or controller equation lives here.
 * Each context owns the complete emitted native state and prepared parameters.
 * A thread-local active-instance binding allows independent contexts to run
 * concurrently. The process-wide mutex excludes resets and other API calls
 * while an OpenMP batch is active; each context appears once in that batch.
 */
#include "acf_rl_native.h"
#include <math.h>
#include <pthread.h>
#include <setjmp.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <omp.h>

extern const P_acf_rl_native_T acf_parameter_bank[];
extern const int acf_parameter_bank_count;

typedef struct {
    double normal_min, normal_max, peak_error, normal_sq;
    double travel_min, travel_max, travel_sq, current_max, voltage_max, current_sq;
    uint64_t samples, voltage_clips, current_limits, force_outside;
    int contact_loss, travel_stop, nonfinite;
} Interval;

typedef struct AcfContext {
    P_acf_rl_native_T parameters;
    B_acf_rl_native_T signals;
    DW_acf_rl_native_T states;
    X_acf_rl_native_T continuous;
    XDis_acf_rl_native_T disabled;
    ExtU_acf_rl_native_T inputs;
    ExtY_acf_rl_native_T outputs, major_outputs;
    RT_MODEL_acf_rl_native_T model;
    Interval interval;
    jmp_buf output_jump;
    int scenario, baseline, peek, terminated;
    double interval_start;
    double *record;
    size_t record_capacity, record_count;
    uint64_t record_last_tick;
} AcfContext;

static _Thread_local AcfContext *acf_active;
static pthread_mutex_t acf_lock = PTHREAD_MUTEX_INITIALIZER;
static int acf_threads = 1;

#include "native_bank_aux.h"

static void interval_clear(AcfContext *context) {
    memset(&context->interval, 0, sizeof(context->interval));
    context->interval.normal_min = context->interval.travel_min = INFINITY;
    context->interval.normal_max = context->interval.travel_max = -INFINITY;
    context->interval_start = context->model.Timing.tArray[0];
}

static uint64_t native_tick(AcfContext *context) {
    return ((uint64_t)context->model.Timing.clockTickH0 << 32) |
           context->model.Timing.clockTick0;
}

static void record_major(AcfContext *context) {
    if (!context->record) return;
    uint64_t tick = native_tick(context);
    size_t index;
    if (context->record_count && context->record_last_tick == tick)
        index = context->record_count - 1; /* New action replaces endpoint preview. */
    else {
        if (context->record_count >= context->record_capacity) return;
        index = context->record_count++;
        context->record_last_tick = tick;
    }
    double *row = context->record + 31 * index;
    row[0] = context->model.Timing.tArray[0];
    row[1] = context->inputs.rl_current_request_A;
    row[2] = context->inputs.rl_robot_correction_m_s;
    row[3] = context->inputs.rl_baseline_mode;
    memcpy(row + 4, context->outputs.rl_scoring, 27 * sizeof(double));
}

static void score_major(AcfContext *context, int include_sum) {
    Interval *s = &context->interval;
    const double *truth = context->outputs.rl_scoring;
    const double normal = truth[0], travel = truth[2];
    const double error = normal - context->parameters.Requestedforce_Value;
    if (!isfinite(normal) || !isfinite(travel) || !isfinite(truth[14]) ||
        !isfinite(truth[15])) s->nonfinite = 1;
    s->normal_min = fmin(s->normal_min, normal);
    s->normal_max = fmax(s->normal_max, normal);
    s->peak_error = fmax(s->peak_error, fabs(error));
    s->travel_min = fmin(s->travel_min, travel);
    s->travel_max = fmax(s->travel_max, travel);
    s->current_max = fmax(s->current_max, fabs(truth[14]));
    s->voltage_max = fmax(s->voltage_max, fabs(truth[15]));
    if (truth[18] < .5) s->contact_loss = 1;
    if (truth[21] > .5) s->travel_stop = 1;
    if (include_sum) {
        double offset = travel - context->parameters.Midtravel_Value;
        s->normal_sq += error * error;
        s->travel_sq += offset * offset;
        s->current_sq += truth[14] * truth[14];
        s->voltage_clips += truth[23] > .5;
        s->current_limits += truth[22] > .5;
        s->force_outside += fabs(error) > .5;
        s->samples++;
    }
    if (s->contact_loss || s->travel_stop || s->nonfinite ||
        context->model.Timing.stopRequestedFlag || context->model.errorStatus)
        context->terminated = 1;
}

/* Called once at the emitted output/update boundary, never at ODE minor stages. */
static void acf_native_output_hook(void) {
    AcfContext *context = acf_active;
    context->major_outputs = context->outputs;
    record_major(context);
    score_major(context, !context->peek);
    if (context->peek) longjmp(context->output_jump, 1);
}

/* Evaluate the fresh boundary measurements before changing actions; leave the
 * physical clock, integrators, queue and noise-generator states unadvanced.
 * Output-phase temporaries/caches are allowed to refresh, exactly as native Run.
 */
static void prime_outputs(AcfContext *context) {
    context->peek = 1;
    if (setjmp(context->output_jump) == 0) acf_rl_native_step();
    context->peek = 0;
}

static void result_copy(AcfContext *context, double *obs, double *truth,
                        double *comparator, double *metrics) {
    if (obs) memcpy(obs, context->major_outputs.rl_observation, 18*sizeof(double));
    if (truth) memcpy(truth, context->major_outputs.rl_scoring, 27*sizeof(double));
    if (comparator) memcpy(comparator, context->major_outputs.rl_comparator, 3*sizeof(double));
    if (!metrics) return;
    Interval *s = &context->interval;
    const double n = s->samples ? (double)s->samples : 1.0;
    metrics[0] = context->model.Timing.tArray[0] - context->interval_start;
    metrics[1] = context->interval_start;
    metrics[2] = context->model.Timing.tArray[0];
    metrics[3] = (double)s->samples;
    metrics[4] = s->normal_min;
    metrics[5] = s->normal_max;
    metrics[6] = s->peak_error;
    metrics[7] = s->normal_sq/n;
    metrics[8] = s->travel_min;
    metrics[9] = s->travel_max;
    metrics[10] = s->travel_sq/n;
    metrics[11] = s->current_max;
    metrics[12] = s->voltage_max;
    metrics[13] = s->current_sq/n;
    metrics[14] = (double)s->voltage_clips;
    metrics[15] = (double)s->current_limits;
    metrics[16] = (double)s->force_outside;
    metrics[17] = s->contact_loss;
    metrics[18] = s->travel_stop;
    metrics[19] = s->nonfinite;
    metrics[20] = context->terminated;
    metrics[21] = (double)context->record_count;
    metrics[22] = (double)native_tick(context);
    metrics[23] = context->major_outputs.rl_scoring[0];
}

int acf_scenario_count(void) { return acf_parameter_bank_count; }

/* Exact prepared bank parameters; seed override shifts all native sensor seeds
 * by the same documented delta. No parameters change within an episode.
 */
int acf_reset(void *handle, int scenario, int seed, int baseline, double *obs,
              double *truth, double *comparator, double *metrics) {
    AcfContext *context = handle;
    if (!context || scenario < 0 || scenario >= acf_parameter_bank_count) return -1;
    pthread_mutex_lock(&acf_lock);
    acf_active = context;
    context->parameters = acf_parameter_bank[scenario];
    acf_apply_seed(&context->parameters, scenario, seed);
    context->scenario = scenario;context->baseline = baseline != 0;
    context->peek = context->terminated = 0;
    context->record_count = 0;context->record_last_tick = UINT64_MAX;
    acf_rl_native_initialize();
    context->inputs.rl_current_request_A = context->parameters.Flatholdingcurrent_Value;
    context->inputs.rl_robot_correction_m_s = 0;
    context->inputs.rl_baseline_mode = context->baseline;
    interval_clear(context);prime_outputs(context);
    /* Reset is a measured t=0 snapshot, not an advanced actor transition. */
    context->interval.samples = 1;
    double error = context->major_outputs.rl_scoring[0] - context->parameters.Requestedforce_Value;
    context->interval.normal_sq = error*error;
    double travel = context->major_outputs.rl_scoring[2] - context->parameters.Midtravel_Value;
    context->interval.travel_sq = travel*travel;
    double current = context->major_outputs.rl_scoring[14];
    context->interval.current_sq = current*current;
    result_copy(context,obs,truth,comparator,metrics);
    pthread_mutex_unlock(&acf_lock);return 0;
}

void *acf_create(int scenario, int seed, int baseline) {
    AcfContext *context = calloc(1,sizeof(*context));
    if (!context) return NULL;
    if (acf_reset(context,scenario,seed,baseline,NULL,NULL,NULL,NULL)) {
        free(context);return NULL;
    }
    return context;
}

void acf_destroy(void *handle) { free(handle); }

int acf_snapshot(void *handle,double *obs,double *truth,double *comparator,double *metrics) {
    if (!handle) return -1;
    pthread_mutex_lock(&acf_lock);result_copy(handle,obs,truth,comparator,metrics);
    pthread_mutex_unlock(&acf_lock);return 0;
}

static int step_repeat_unlocked(AcfContext *context,double current,double robot,int repeat,
                          double *obs,double *truth,double *comparator,double *metrics) {
    acf_active = context;interval_clear(context);
    if (!isfinite(current) || !isfinite(robot)) {
        context->interval.nonfinite = context->terminated = 1;
        result_copy(context,obs,truth,comparator,metrics);return 1;
    }
    if (!context->terminated) {
        context->inputs.rl_current_request_A = current;
        context->inputs.rl_robot_correction_m_s = robot;
        for (int action=0;action<repeat && !context->terminated;action++) {
            for (int k=0;k<30 && !context->terminated;k++) acf_rl_native_step();
            if (!context->terminated) prime_outputs(context);
        }
    }
    result_copy(context,obs,truth,comparator,metrics);return context->terminated;
}

static int step_unlocked(AcfContext *context,double current,double robot,
                          double *obs,double *truth,double *comparator,double *metrics) {
    return step_repeat_unlocked(context,current,robot,1,obs,truth,comparator,metrics);
}

int acf_step(void *handle,double current,double robot,double *obs,double *truth,
             double *comparator,double *metrics) {
    if (!handle) return -1;
    pthread_mutex_lock(&acf_lock);
    int status = step_unlocked(handle,current,robot,obs,truth,comparator,metrics);
    pthread_mutex_unlock(&acf_lock);return status;
}

int acf_step_batch(void **handles,size_t count,const double *actions,double *obs,
                   double *truth,double *comparator,double *metrics) {
    pthread_mutex_lock(&acf_lock);
    for (size_t k=0;k<count;k++) {
        if (!handles[k]) { pthread_mutex_unlock(&acf_lock);return -1; }
        for (size_t j=0;j<k;j++) {
            if (handles[j]==handles[k]) {pthread_mutex_unlock(&acf_lock);return -2;}
        }
    }
    #pragma omp parallel for num_threads(acf_threads) schedule(static) if(count>1 && acf_threads>1)
    for (size_t k=0;k<count;k++) {
        step_unlocked(handles[k],actions[2*k],actions[2*k+1],obs+18*k,truth+27*k,
                       comparator+3*k,metrics+24*k);
    }
    pthread_mutex_unlock(&acf_lock);return 0;
}

int acf_step_repeat_batch(void **handles,size_t count,const double *actions,int repeat,
                   double *obs,double *truth,double *comparator,double *metrics) {
    if (repeat<1 || repeat>10000) return -3;
    pthread_mutex_lock(&acf_lock);
    for (size_t k=0;k<count;k++) {
        if (!handles[k]) {pthread_mutex_unlock(&acf_lock);return -1;}
        for (size_t j=0;j<k;j++) {
            if (handles[j]==handles[k]) {pthread_mutex_unlock(&acf_lock);return -2;}
        }
    }
    #pragma omp parallel for num_threads(acf_threads) schedule(static) if(count>1 && acf_threads>1)
    for (size_t k=0;k<count;k++) {
        step_repeat_unlocked(handles[k],actions[2*k],actions[2*k+1],repeat,obs+18*k,
                       truth+27*k,comparator+3*k,metrics+24*k);
    }
    pthread_mutex_unlock(&acf_lock);return 0;
}

int acf_set_threads(int threads) {
    if (threads<1 || threads>64) return -1;
    pthread_mutex_lock(&acf_lock);acf_threads=threads;
    pthread_mutex_unlock(&acf_lock);return 0;
}

int acf_get_threads(void) {return acf_threads;}

void acf_set_record(void *handle,double *buffer,size_t capacity) {
    AcfContext *context=handle;if (!context) return;
    pthread_mutex_lock(&acf_lock);context->record=buffer;context->record_capacity=capacity;
    context->record_count=0;context->record_last_tick=UINT64_MAX;
    context->outputs=context->major_outputs;record_major(context);
    pthread_mutex_unlock(&acf_lock);
}

size_t acf_record_count(void *handle) { return handle ? ((AcfContext*)handle)->record_count : 0; }

/* This catches declaration parsing, ordering or coefficient-packing errors. */
int acf_bank_zero_matches_codegen(void) {
    return acf_parameter_equal(&acf_parameter_bank[0],&acf_rl_native_P);
}

/* Native C below is emitted by Simulink Coder. Only global-storage bindings and
 * the observation hook are adapted; native computations are otherwise intact.
 */
#include "acf_rl_native_context.c"
