#include "ml_inference.h"
#include <string.h>

/*
 * Placeholder weights — replaced by trained model exported from
 * host/training/ via TFLite Micro or manual CMSIS-NN port.
 *
 * For now, uses a simple heuristic based on the input data statistics
 * so the control loop can exercise the full pipeline before the real
 * model is trained from collected shot data.
 */

void ml_init(ml_context_t *ctx)
{
    memset(ctx, 0, sizeof(ml_context_t));
}

void ml_feed_sample(ml_context_t *ctx, float pressure, float flow)
{
    if (ctx->sample_count >= ML_INPUT_SAMPLES)
        return;

    // V-11 fix: secondary bounds check on computed index
    uint32_t idx = (uint32_t)ctx->sample_count * ML_INPUT_CHANNELS;
    if (idx + 1 >= ML_INPUT_SIZE)
        return;

    ctx->input_buffer[idx] = pressure;
    ctx->input_buffer[idx + 1] = flow;
    ctx->sample_count++;

    if (ctx->sample_count >= ML_INPUT_SAMPLES)
        ctx->ready = true;
}

bool ml_is_ready(const ml_context_t *ctx)
{
    return ctx->ready;
}

bool ml_run_inference(ml_context_t *ctx)
{
    if (!ctx->ready)
        return false;

    /*
     * Heuristic placeholder until real model is deployed:
     * - Compute mean pressure and flow from the 3-second window
     * - Estimate resistance as mean(pressure) / mean(flow)
     * - Predict compaction from the pressure trend
     */
    float sum_p = 0.0f, sum_f = 0.0f;
    float p_start = 0.0f, p_end = 0.0f;

    for (uint16_t i = 0; i < ML_INPUT_SAMPLES; i++) {
        float p = ctx->input_buffer[i * ML_INPUT_CHANNELS];
        float f = ctx->input_buffer[i * ML_INPUT_CHANNELS + 1];
        sum_p += p;
        sum_f += f;

        if (i < 10) p_start += p;
        if (i >= ML_INPUT_SAMPLES - 10) p_end += p;
    }

    float mean_p = sum_p / ML_INPUT_SAMPLES;
    float mean_f = sum_f / ML_INPUT_SAMPLES;
    p_start /= 10.0f;
    p_end /= 10.0f;

    if (mean_f < 0.01f) mean_f = 0.01f;

    ctx->prediction.r_initial = mean_p / mean_f;
    ctx->prediction.alpha = (p_end - p_start) / (mean_p + 0.01f) * 0.1f;
    ctx->prediction.r_final = ctx->prediction.r_initial * (1.0f + ctx->prediction.alpha * 25.0f);

    if (ctx->prediction.r_initial < 1.0f) ctx->prediction.r_initial = 1.0f;
    if (ctx->prediction.r_final < ctx->prediction.r_initial)
        ctx->prediction.r_final = ctx->prediction.r_initial;

    ctx->prediction_valid = true;
    return true;
}

const ml_prediction_t *ml_get_prediction(const ml_context_t *ctx)
{
    if (!ctx->prediction_valid)
        return NULL;
    return &ctx->prediction;
}

void ml_reset(ml_context_t *ctx)
{
    ctx->sample_count = 0;
    ctx->ready = false;
    ctx->prediction_valid = false;
}
