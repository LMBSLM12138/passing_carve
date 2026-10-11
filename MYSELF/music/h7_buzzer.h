#ifndef H7_BUZZER_H
#define H7_BUZZER_H

#include "stm32h7xx_hal.h"
#include <stdint.h>


#ifdef __cplusplus
extern "C" {
#endif

/**
 * One entry in a generated buzzer tune.
 * frequency_hz == 0 is a rest.
 */
typedef struct
{
    uint16_t frequency_hz;
    uint32_t duration_ms;
} buzzer_note_t;

typedef struct
{
    TIM_HandleTypeDef *htim;
    uint32_t channel;
    uint32_t timer_clock_hz;
    uint32_t pwm_arr;
    uint8_t duty_percent;

    uint8_t initialized;
    uint8_t output_on;
    uint8_t async_running;

    /* Pause state: keeps the current note and elapsed time. */
    uint8_t paused;
    uint32_t paused_elapsed_ms;

    const buzzer_note_t *tune;
    uint32_t tune_length;
    uint32_t tune_index;
    uint32_t note_started_ms;
    HAL_StatusTypeDef last_status;
} h7_buzzer_t;

/**
 * Initialize the driver. The timer and GPIO must already be initialized by
 * CubeMX/HAL as a PWM output. Example:
 *
 *   H7_Buzzer_Init(&buzzer, &htim4, TIM_CHANNEL_1, 240000000u);
 *
 * timer_clock_hz is the timer kernel clock, not necessarily the APB clock.
 */
HAL_StatusTypeDef H7_Buzzer_Init(h7_buzzer_t *buzzer,
                                TIM_HandleTypeDef *htim,
                                uint32_t channel,
                                uint32_t timer_clock_hz);

/**
 * Change PWM resolution and duty cycle. pwm_arr is the timer ARR value.
 * Defaults are pwm_arr = 999 and duty_percent = 50.
 *
 * A 50% duty is recommended for a passive buzzer.
 */
HAL_StatusTypeDef H7_Buzzer_Config(h7_buzzer_t *buzzer,
                                  uint32_t pwm_arr,
                                  uint8_t duty_percent);

/**
 * Convert a note frequency to timer registers. The result is:
 *
 *   timer_frequency = timer_clock_hz / ((psc + 1) * (arr + 1))
 *
 * This function is useful if the application wants the raw PWM values before
 * calling H7_Buzzer_PlayNote().
 */
HAL_StatusTypeDef H7_Buzzer_GetPwmRegisters(const h7_buzzer_t *buzzer,
                                           uint32_t frequency_hz,
                                           uint32_t *psc,
                                           uint32_t *arr,
                                           uint32_t *ccr);

/**
 * Play one note. frequency_hz == 0 stops the output and produces a rest.
 * This cancels an asynchronous tune if one is currently running.
 */
HAL_StatusTypeDef H7_Buzzer_PlayNote(h7_buzzer_t *buzzer, uint32_t frequency_hz);

/** Stop PWM output and cancel any blocking/async state owned by this object. */
HAL_StatusTypeDef H7_Buzzer_Stop(h7_buzzer_t *buzzer);

/**
 * Blocking playback using HAL_Delay(). Do not call from an interrupt.
 */
HAL_StatusTypeDef H7_Buzzer_PlayTuneBlocking(h7_buzzer_t *buzzer,
                                            const buzzer_note_t *tune,
                                            uint32_t length);

/**
 * Start non-blocking playback. Call H7_Buzzer_Update() periodically, usually
 * every 1 ms from the main loop or a 1 ms task.
 */
HAL_StatusTypeDef H7_Buzzer_PlayTuneAsync(h7_buzzer_t *buzzer,
                                         const buzzer_note_t *tune,
                                         uint32_t length);

/** Pause asynchronous playback, preserving the current note and elapsed time. */
HAL_StatusTypeDef H7_Buzzer_Pause(h7_buzzer_t *buzzer);

/** Resume asynchronous playback from the position saved by Pause(). */
HAL_StatusTypeDef H7_Buzzer_Resume(h7_buzzer_t *buzzer);

/** Non-zero when asynchronous playback is paused. */
uint8_t H7_Buzzer_IsPaused(const h7_buzzer_t *buzzer);

/** Advance an asynchronous tune. Safe to call frequently; it is time-based. */
void H7_Buzzer_Update(h7_buzzer_t *buzzer);
void laoda_music_Init(void);
void laoda_music_Control(int16_t cmd);
																				 
/** Non-zero while an asynchronous tune is running. */
uint8_t H7_Buzzer_IsBusy(const h7_buzzer_t *buzzer);

/** Non-zero when PWM is currently enabled. */
uint8_t H7_Buzzer_IsOutputOn(const h7_buzzer_t *buzzer);
																				 
extern h7_buzzer_t g_buzzer;
extern volatile int16_t music_cmd;
																				 
#ifdef __cplusplus
}
#endif

#endif /* H7_BUZZER_H */
