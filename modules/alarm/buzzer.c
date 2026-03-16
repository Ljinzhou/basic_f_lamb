#include "bsp_pwm.h"
#include "buzzer.h"
#include "bsp_dwt.h"
#include "string.h"

static PWMInstance *buzzer;
static BuzzzerInstance *buzzer_list[BUZZER_DEVICE_CNT] = {0};

static Note_t startup_melody[STARTUP_MELODY_LENGTH] = {
    {DoFreq, 150},
    {ReFreq, 150},
    {MiFreq, 150},
    {FaFreq, 150},
    {SoFreq, 150},
    {LaFreq, 150},
    {SiFreq, 300},
};

static uint8_t current_note_index = 0;
static float note_start_time = 0;
static uint8_t is_playing_melody = 0;
static float melody_loudness = 0.3f;

void BuzzerInit()
{
    PWM_Init_Config_s buzzer_config = {
        .htim = &htim4,
        .channel = TIM_CHANNEL_3,
        .dutyratio = 0,
        .period = 0.001,
    };
    buzzer = PWMRegister(&buzzer_config);
}

BuzzzerInstance *BuzzerRegister(Buzzer_config_s *config)
{
    if (config->alarm_level > BUZZER_DEVICE_CNT)
        while (1)
            ;
    BuzzzerInstance *buzzer_temp = (BuzzzerInstance *)malloc(sizeof(BuzzzerInstance));
    memset(buzzer_temp, 0, sizeof(BuzzzerInstance));

    buzzer_temp->alarm_level = config->alarm_level;
    buzzer_temp->loudness = config->loudness;
    buzzer_temp->octave = config->octave;
    buzzer_temp->alarm_state = ALARM_OFF;

    buzzer_list[config->alarm_level] = buzzer_temp;
    return buzzer_temp;
}

void AlarmSetStatus(BuzzzerInstance *buzzer, AlarmState_e state)
{
    buzzer->alarm_state = state;
}

void PlayStartupMelody()
{
    if (!is_playing_melody)
    {
        is_playing_melody = 1;
        current_note_index = 0;
        note_start_time = DWT_GetTimeline_ms();
    }
}

uint8_t IsStartupMelodyPlaying()
{
    return is_playing_melody;
}

static void UpdateStartupMelody()
{
    if (!is_playing_melody)
        return;

    float current_time = DWT_GetTimeline_ms();
    Note_t *current_note = &startup_melody[current_note_index];

    if (current_time - note_start_time >= current_note->duration_ms)
    {
        current_note_index++;
        note_start_time = current_time;

        if (current_note_index >= STARTUP_MELODY_LENGTH)
        {
            is_playing_melody = 0;
            current_note_index = 0;
            PWMSetDutyRatio(buzzer, 0);
            return;
        }
        current_note = &startup_melody[current_note_index];
    }

    PWMSetDutyRatio(buzzer, melody_loudness);
    PWMSetPeriod(buzzer, 1.0f / current_note->freq);
}

void BuzzerTask()
{
    if (is_playing_melody)
    {
        UpdateStartupMelody();
        return;
    }

    BuzzzerInstance *buzz;
    for (size_t i = 0; i < BUZZER_DEVICE_CNT; ++i)
    {
        buzz = buzzer_list[i];
        if (buzz == NULL || buzz->alarm_level > ALARM_LEVEL_LOW)
        {
            continue;
        }
        if (buzz->alarm_state == ALARM_OFF)
        {
            PWMSetDutyRatio(buzzer, 0);
        }
        else
        {
            PWMSetDutyRatio(buzzer, buzz->loudness);
            switch (buzz->octave)
            {
            case OCTAVE_1:
                PWMSetPeriod(buzzer, (float)1 / DoFreq);
                break;
            case OCTAVE_2:
                PWMSetPeriod(buzzer, (float)1 / ReFreq);
                break;
            case OCTAVE_3:
                PWMSetPeriod(buzzer, (float)1 / MiFreq);
                break;
            case OCTAVE_4:
                PWMSetPeriod(buzzer, (float)1 / FaFreq);
                break;
            case OCTAVE_5:
                PWMSetPeriod(buzzer, (float)1 / SoFreq);
                break;
            case OCTAVE_6:
                PWMSetPeriod(buzzer, (float)1 / LaFreq);
                break;
            case OCTAVE_7:
                PWMSetPeriod(buzzer, (float)1 / SiFreq);
                break;
            default:
                break;
            }
            break;
        }
    }
}
