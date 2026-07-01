#ifndef ML_INFERENCE_H
#define ML_INFERENCE_H

#include <stdint.h>
#include <stdbool.h>

/*
 * TinyML puck resistance predictor.
 *
 * Takes the first 3 seconds of a shot (pressure + flow at 100 Hz = 600 floats)
 * and predicts puck resistance parameters for the remainder of the extraction.
 *
 * Model architecture: small fully-connected network
 *   Input:  300 samples x 2 channels = 600 features (downsampled from 1kHz)
 *   Hidden: 32 neurons, ReLU
 *   Hidden: 16 neurons, ReLU
 *   Output: 3 values [R_initial, alpha (compaction rate), R_final]
 *
 * Weights: ~5 KB (quantized int8 via CMSIS-NN or TFLite Micro)
 */

#define ML_INPUT_SAMPLES    300
#define ML_INPUT_CHANNELS   2
#define ML_INPUT_SIZE       (ML_INPUT_SAMPLES * ML_INPUT_CHANNELS)
#define ML_OUTPUT_SIZE      3

typedef struct {
    float r_initial;    // initial puck resistance (bar·s/mL)
    float alpha;        // compaction rate coefficient
    float r_final;      // predicted final resistance
} ml_prediction_t;

typedef struct {
    float input_buffer[ML_INPUT_SIZE];
    uint16_t sample_count;
    bool ready;
    bool prediction_valid;
    ml_prediction_t prediction;
} ml_context_t;

void ml_init(ml_context_t *ctx);
void ml_feed_sample(ml_context_t *ctx, float pressure, float flow);
bool ml_is_ready(const ml_context_t *ctx);
bool ml_run_inference(ml_context_t *ctx);
const ml_prediction_t *ml_get_prediction(const ml_context_t *ctx);
void ml_reset(ml_context_t *ctx);

#endif
