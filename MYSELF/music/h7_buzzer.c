#include "h7_buzzer.h"
#include "see_you_again_tune.h"
#include "main.h"
#include "tim.h"

#include <stddef.h>
#include <string.h>

h7_buzzer_t g_buzzer;
volatile int16_t music_cmd = 0;

void laoda_music_Init(void)
{
	if (H7_Buzzer_Init(&g_buzzer,
                   &htim12,
                   TIM_CHANNEL_2,
                   240000000U) != HAL_OK)
{
    Error_Handler();
}

/* 使用 ARR=999，占空比 50%，适合无源蜂鸣器 */
if (H7_Buzzer_Config(&g_buzzer, 999U, 50U) != HAL_OK)
{
    Error_Handler();
}

/* 开始异步播放 */
H7_Buzzer_PlayTuneAsync(&g_buzzer,
                        SEE_YOU_AGAIN_____AIGEI_COM_TUNE,
                        SEE_YOU_AGAIN_____AIGEI_COM_TUNE_LEN);
}

void laoda_music_Control(int16_t cmd)
{
    static int16_t last_cmd = 0;

    /* Process only command changes, so a held 600 does not restart forever. */
    if (cmd == last_cmd)
    {
        return;
    }
    last_cmd = cmd;

    if (cmd > 600)
    {
        /* Restart from the beginning. */
        (void)H7_Buzzer_Stop(&g_buzzer);
        (void)H7_Buzzer_PlayTuneAsync(&g_buzzer,
                                      SEE_YOU_AGAIN_____AIGEI_COM_TUNE,
                                      SEE_YOU_AGAIN_____AIGEI_COM_TUNE_LEN);
    }
    else if (cmd < -600)
    {
        /* Pause at the current note position. */
        (void)H7_Buzzer_Pause(&g_buzzer);
    }
    else if (cmd == 0)
    {
        /* Resume only if it was paused; otherwise keep normal playback. */
        if (H7_Buzzer_IsPaused(&g_buzzer))
        {
            (void)H7_Buzzer_Resume(&g_buzzer);
        }
    }
}

static HAL_StatusTypeDef h7_buzzer_stop_output(h7_buzzer_t *buzzer)
{
    if (buzzer == NULL)
    {
        return HAL_ERROR;
    }

    if (buzzer->output_on != 0U)
    {
        (void)HAL_TIM_PWM_Stop(buzzer->htim, buzzer->channel);
        buzzer->output_on = 0U;
    }

    return HAL_OK;
}

static HAL_StatusTypeDef h7_buzzer_start_note(h7_buzzer_t *buzzer, uint32_t frequency_hz)
{
    uint32_t psc;
    uint32_t arr;
    uint32_t ccr;
    HAL_StatusTypeDef status;

    if (frequency_hz == 0U)
    {
        return h7_buzzer_stop_output(buzzer);
    }

    /* Silence the previous note before attempting a new register setting. */
    (void)h7_buzzer_stop_output(buzzer);

    status = H7_Buzzer_GetPwmRegisters(buzzer, frequency_hz, &psc, &arr, &ccr);
    if (status != HAL_OK)
    {
        return status;
    }

    __HAL_TIM_SET_PRESCALER(buzzer->htim, psc);
    __HAL_TIM_SET_AUTORELOAD(buzzer->htim, arr);
    __HAL_TIM_SET_COMPARE(buzzer->htim, buzzer->channel, ccr);
    __HAL_TIM_SET_COUNTER(buzzer->htim, 0U);

    /* Transfer PSC/ARR into their active shadow registers immediately. */
    buzzer->htim->Instance->EGR = TIM_EGR_UG;
    __HAL_TIM_CLEAR_FLAG(buzzer->htim, TIM_FLAG_UPDATE);

    status = HAL_TIM_PWM_Start(buzzer->htim, buzzer->channel);
    if (status == HAL_OK)
    {
        buzzer->output_on = 1U;
    }

    return status;
}

HAL_StatusTypeDef H7_Buzzer_Init(h7_buzzer_t *buzzer,
                                TIM_HandleTypeDef *htim,
                                uint32_t channel,
                                uint32_t timer_clock_hz)
{
    if ((buzzer == NULL) || (htim == NULL) || (timer_clock_hz == 0U))
    {
        return HAL_ERROR;
    }
    if ((channel != TIM_CHANNEL_1) && (channel != TIM_CHANNEL_2) &&
        (channel != TIM_CHANNEL_3) && (channel != TIM_CHANNEL_4))
    {
        return HAL_ERROR;
    }

    (void)memset(buzzer, 0, sizeof(*buzzer));
    buzzer->htim = htim;
    buzzer->channel = channel;
    buzzer->timer_clock_hz = timer_clock_hz;
    buzzer->pwm_arr = 999U;
    buzzer->duty_percent = 50U;
    buzzer->initialized = 1U;
    buzzer->last_status = HAL_OK;
    return HAL_OK;
}

HAL_StatusTypeDef H7_Buzzer_Config(h7_buzzer_t *buzzer,
                                  uint32_t pwm_arr,
                                  uint8_t duty_percent)
{
    if ((buzzer == NULL) || (buzzer->initialized == 0U))
    {
        return HAL_ERROR;
    }
    if ((pwm_arr == 0U) || (pwm_arr > 0xFFFFU) || (duty_percent > 100U))
    {
        return HAL_ERROR;
    }

    buzzer->pwm_arr = pwm_arr;
    buzzer->duty_percent = duty_percent;
    return HAL_OK;
}

HAL_StatusTypeDef H7_Buzzer_GetPwmRegisters(const h7_buzzer_t *buzzer,
                                           uint32_t frequency_hz,
                                           uint32_t *psc,
                                           uint32_t *arr,
                                           uint32_t *ccr)
{
    uint64_t denominator;
    uint64_t divider;
    uint64_t compare;

    if ((buzzer == NULL) || (buzzer->initialized == 0U) ||
        (frequency_hz == 0U) || (psc == NULL) || (arr == NULL) || (ccr == NULL))
    {
        return HAL_ERROR;
    }
    if ((buzzer->pwm_arr == 0U) || (buzzer->pwm_arr > 0xFFFFU))
    {
        return HAL_ERROR;
    }

    denominator = (uint64_t)frequency_hz * ((uint64_t)buzzer->pwm_arr + 1ULL);
    if (denominator == 0ULL)
    {
        return HAL_ERROR;
    }

    divider = ((uint64_t)buzzer->timer_clock_hz + (denominator / 2ULL)) / denominator;
    if ((divider == 0ULL) || (divider > 65536ULL))
    {
        return HAL_ERROR;
    }

    compare = (((uint64_t)buzzer->pwm_arr + 1ULL) * buzzer->duty_percent) / 100ULL;
    if ((compare == 0ULL) && (buzzer->duty_percent != 0U))
    {
        compare = 1ULL;
    }
    if (compare > buzzer->pwm_arr)
    {
        compare = buzzer->pwm_arr;
    }

    *arr = buzzer->pwm_arr;
    *psc = (uint32_t)(divider - 1ULL);
    *ccr = (uint32_t)compare;
    return HAL_OK;
}

HAL_StatusTypeDef H7_Buzzer_PlayNote(h7_buzzer_t *buzzer, uint32_t frequency_hz)
{
    if ((buzzer == NULL) || (buzzer->initialized == 0U))
    {
        return HAL_ERROR;
    }

    buzzer->async_running = 0U;
    buzzer->paused = 0U;
    buzzer->paused_elapsed_ms = 0U;
    buzzer->tune = NULL;
    buzzer->tune_length = 0U;
    buzzer->tune_index = 0U;
    buzzer->last_status = h7_buzzer_start_note(buzzer, frequency_hz);
    return buzzer->last_status;
}

HAL_StatusTypeDef H7_Buzzer_Stop(h7_buzzer_t *buzzer)
{
    HAL_StatusTypeDef status;

    if ((buzzer == NULL) || (buzzer->initialized == 0U))
    {
        return HAL_ERROR;
    }

    buzzer->async_running = 0U;
    buzzer->paused = 0U;
    buzzer->paused_elapsed_ms = 0U;
    buzzer->tune = NULL;
    buzzer->tune_length = 0U;
    buzzer->tune_index = 0U;
    status = h7_buzzer_stop_output(buzzer);
    buzzer->last_status = status;
    return status;
}

HAL_StatusTypeDef H7_Buzzer_Pause(h7_buzzer_t *buzzer)
{
    if ((buzzer == NULL) || (buzzer->initialized == 0U))
    {
        return HAL_ERROR;
    }

    if ((buzzer->async_running == 0U) || (buzzer->paused != 0U))
    {
        return HAL_OK;
    }

    buzzer->paused_elapsed_ms = HAL_GetTick() - buzzer->note_started_ms;
    (void)h7_buzzer_stop_output(buzzer);
    buzzer->paused = 1U;

    return HAL_OK;
}

HAL_StatusTypeDef H7_Buzzer_Resume(h7_buzzer_t *buzzer)
{
    HAL_StatusTypeDef status;

    if ((buzzer == NULL) || (buzzer->initialized == 0U))
    {
        return HAL_ERROR;
    }

    if ((buzzer->async_running == 0U) || (buzzer->paused == 0U))
    {
        return HAL_OK;
    }

    if ((buzzer->tune == NULL) || (buzzer->tune_index >= buzzer->tune_length))
    {
        return HAL_ERROR;
    }

    status = h7_buzzer_start_note(buzzer,
                                  buzzer->tune[buzzer->tune_index].frequency_hz);
    if (status != HAL_OK)
    {
        return status;
    }

    buzzer->note_started_ms = HAL_GetTick() - buzzer->paused_elapsed_ms;
    buzzer->paused = 0U;

    return HAL_OK;
}

HAL_StatusTypeDef H7_Buzzer_PlayTuneBlocking(h7_buzzer_t *buzzer,
                                            const buzzer_note_t *tune,
                                            uint32_t length)
{
    uint32_t index;

    if ((buzzer == NULL) || (buzzer->initialized == 0U) || (tune == NULL) || (length == 0U))
    {
        return HAL_ERROR;
    }

    (void)H7_Buzzer_Stop(buzzer);
    for (index = 0U; index < length; ++index)
    {
        HAL_StatusTypeDef status = h7_buzzer_start_note(buzzer, tune[index].frequency_hz);
        if (status != HAL_OK)
        {
            (void)h7_buzzer_stop_output(buzzer);
            buzzer->last_status = status;
            return status;
        }

        if (tune[index].duration_ms != 0U)
        {
            HAL_Delay(tune[index].duration_ms);
        }
    }

    (void)h7_buzzer_stop_output(buzzer);
    buzzer->last_status = HAL_OK;
    return HAL_OK;
}

HAL_StatusTypeDef H7_Buzzer_PlayTuneAsync(h7_buzzer_t *buzzer,
                                         const buzzer_note_t *tune,
                                         uint32_t length)
{
    HAL_StatusTypeDef status;

    if ((buzzer == NULL) || (buzzer->initialized == 0U) || (tune == NULL) || (length == 0U))
    {
        return HAL_ERROR;
    }

    (void)H7_Buzzer_Stop(buzzer);
    buzzer->tune = tune;
    buzzer->tune_length = length;
    buzzer->tune_index = 0U;
    buzzer->note_started_ms = HAL_GetTick();

    status = h7_buzzer_start_note(buzzer, tune[0].frequency_hz);
    if (status != HAL_OK)
    {
        buzzer->tune = NULL;
        buzzer->tune_length = 0U;
        buzzer->last_status = status;
        return status;
    }

    buzzer->async_running = 1U;
    buzzer->last_status = HAL_OK;
    return HAL_OK;
}

void H7_Buzzer_Update(h7_buzzer_t *buzzer)
{
    uint32_t now;

    if ((buzzer == NULL) ||
        (buzzer->async_running == 0U) ||
        (buzzer->paused != 0U) ||
        (buzzer->tune == NULL))
    {
        return;
    }

    now = HAL_GetTick();

    while (buzzer->tune_index < buzzer->tune_length)
    {
        const buzzer_note_t *note = &buzzer->tune[buzzer->tune_index];
        uint32_t elapsed = now - buzzer->note_started_ms;

        if ((note->duration_ms != 0U) && (elapsed < note->duration_ms))
        {
            return; /* Current note is still playing. */
        }

        ++buzzer->tune_index;
        if (buzzer->tune_index >= buzzer->tune_length)
        {
            (void)H7_Buzzer_Stop(buzzer);
            return;
        }

        buzzer->note_started_ms = now;
        buzzer->last_status = h7_buzzer_start_note(buzzer,
                                                   buzzer->tune[buzzer->tune_index].frequency_hz);
        if (buzzer->last_status != HAL_OK)
        {
            (void)H7_Buzzer_Stop(buzzer);
            return;
        }
        return; /* Handle the next note on the following update call. */
    }

    (void)H7_Buzzer_Stop(buzzer);
}

uint8_t H7_Buzzer_IsBusy(const h7_buzzer_t *buzzer)
{
    return ((buzzer != NULL) && (buzzer->async_running != 0U)) ? 1U : 0U;
}

uint8_t H7_Buzzer_IsPaused(const h7_buzzer_t *buzzer)
{
    return ((buzzer != NULL) && (buzzer->paused != 0U)) ? 1U : 0U;
}

uint8_t H7_Buzzer_IsOutputOn(const h7_buzzer_t *buzzer)
{
    return ((buzzer != NULL) && (buzzer->output_on != 0U)) ? 1U : 0U;
}
