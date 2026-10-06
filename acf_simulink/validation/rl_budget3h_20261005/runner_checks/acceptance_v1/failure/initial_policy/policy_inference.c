#include "policy_inference.h"
#include <math.h>
#include <string.h>
static float acf_sigmoid(float x) {
    if (x >= 0.0f) return 1.0f / (1.0f + expf(-x));
    float e = expf(x); return e / (1.0f + e);
}
static const size_t recurrent_w[ACF_RL_LAYERS] = {1392,4560};
static const size_t recurrent_b[ACF_RL_LAYERS] = {4464,7632};
void acf_unified_policy_step(const float *w, const float *observation,
    float *state, int reset_before, float *mean, float *value) {
    float hidden[ACF_RL_HIDDEN], combined[3*ACF_RL_HIDDEN], normalized[ACF_RL_OBS];
    float aux[ACF_RL_AUX] = {0};
    if (reset_before) memset(state, 0, ACF_RL_LAYERS*ACF_RL_HIDDEN*sizeof(float));
    for (size_t j=0; j<ACF_RL_OBS; j++)
        normalized[j] = (observation[j]-w[0+j]) / w[40+j];
#if ACF_RL_OBS == 40
    float s=2*observation[19], b=observation[20];
    float az=10*observation[3], ax=10*observation[4];
    float denominator=2*fmaxf(25*observation[26],1);
    aux[0]=(s+b)*az/denominator; aux[1]=s*b*az/denominator;
    aux[2]=(5*observation[1]-.5f*observation[0])/denominator;
    aux[3]=b*az/denominator; aux[4]=(s+b)*(.2f*observation[10])/denominator;
    aux[5]=(60*observation[25])*(.02f*observation[6])/denominator;
    aux[6]=(20000*observation[24])*(.005f*observation[7])/denominator;
    aux[7]=(s+b)*ax/denominator;
#endif
    for (size_t i=0; i<ACF_RL_HIDDEN; i++) {
        float x = w[1360 + i];
        for (size_t j=0; j<ACF_RL_OBS; j++)
            x += w[80 + i*ACF_RL_OBS + j]*normalized[j];
        hidden[i] = tanhf(x);
    }
    for (size_t layer=0; layer<ACF_RL_LAYERS; layer++) {
        for (size_t i=0; i<3*ACF_RL_HIDDEN; i++) {
            float x = w[recurrent_b[layer] + i];
            for (size_t j=0; j<ACF_RL_HIDDEN; j++)
                x += w[recurrent_w[layer] + i*ACF_RL_HIDDEN + j]*hidden[j];
            combined[i] = x;
        }
        for (size_t i=0; i<ACF_RL_HIDDEN; i++) {
            float candidate = combined[i] >= 0 ? combined[i]+0.5f : acf_sigmoid(combined[i]);
            float update = acf_sigmoid(combined[ACF_RL_HIDDEN+i]);
            float projection = acf_sigmoid(combined[2*ACF_RL_HIDDEN+i]);
            size_t index = layer*ACF_RL_HIDDEN+i;
            float recurrent = state[index] + update*(candidate-state[index]);
            state[index] = recurrent;
            hidden[i] = projection*recurrent + (1.0f-projection)*hidden[i];
        }
    }
    for (size_t i=0; i<2; i++) {
        float x = w[7792+i];
        for (size_t j=0; j<ACF_RL_HIDDEN; j++)
            x += w[7728+i*ACF_RL_HIDDEN+j]*hidden[j];
        for (size_t j=0; j<ACF_RL_OBS; j++)
            x += w[7794+i*ACF_RL_OBS+j]*normalized[j];
        for (size_t j=0; j<ACF_RL_AUX; j++)
            x += w[7874+i*ACF_RL_AUX+j]*aux[j];
        mean[i] = x;
    }
    if (value) {
        float x = w[7922];
        for (size_t j=0; j<ACF_RL_HIDDEN; j++) x += w[7890+j]*hidden[j];
        *value = x;
    }
}
void acf_unified_policy_step_many(size_t batch, const float *w,
    const float *observations, float *states, const unsigned char *resets,
    float *means, float *values) {
    for (size_t i=0; i<batch; i++) acf_unified_policy_step(w,
        observations+i*ACF_RL_OBS, states+i*ACF_RL_LAYERS*ACF_RL_HIDDEN,
        resets ? resets[i] : 0, means+2*i, values ? values+i : 0);
}
