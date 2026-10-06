/* Frozen shape; weights may be refreshed between PPO rollouts. */
#ifndef ACF_UNIFIED_POLICY_H
#define ACF_UNIFIED_POLICY_H
#include <stddef.h>
#define ACF_RL_OBS 32
#define ACF_RL_HIDDEN 32
#define ACF_RL_LAYERS 2
#define ACF_RL_FLOAT_COUNT 7493
#define ACF_RL_LOG_STD_OFFSET 7491
#ifdef __cplusplus
extern "C" {
#endif
/* State is [layers,hidden]. Reset only at a physical episode start. */
void acf_unified_policy_step(const float *weights, const float *observation,
    float *state, int reset_before, float *latent_mean, float *value);
/* Batch state is [batch,layers,hidden], observations [batch,obs]. */
void acf_unified_policy_step_many(size_t batch, const float *weights,
    const float *observations, float *states, const unsigned char *resets,
    float *latent_means, float *values);
#ifdef __cplusplus
}
#endif
#endif
