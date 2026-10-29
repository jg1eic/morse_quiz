/*
 * morse_quiz.c  -  Morse code quiz kit (ATtiny202, pure C, MPLAB X / XC8 or avr-gcc)
 *
 * Modes
 *   QUIZ   : the device plays a character in Morse (sound + LED), the player taps the
 *            answer with one button (short press = dit, long press = dah).
 *            Correct -> "pin-pon" + LED, wrong -> "buu" + LED blink, then replay.
 *   RANDOM : listening practice. Plays random characters back to back, no input.
 *   Hold the button >= MODE_HOLD_MS in either mode to switch to the other
 *   (QUIZ -> RANDOM -> QUIZ -> ...).
 *   About POWERDOWN_MS of no button activity in QUIZ (or before the very
 *   first press) puts the MCU into full POWER-DOWN sleep; a button press
 *   wakes it via the PA7 pin interrupt (BOTHEDGES: PA7 is not a fully
 *   asynchronous pin, so a single-edge setting would not wake it with all
 *   clocks stopped). RANDOM powers down after RANDOM_OFF_MIN minutes
 *   without button activity (a short tap restarts the count); a press
 *   wakes it and playback resumes in the same mode.
 *
 * Character set (chosen once, by the very first press after power-on)
 *   press < 1 s       -> Latin A-Z        (no confirm tone)
 *   hold  1 .. 5 s    -> Wabun (kana, 50) (confirm: "ホレ" as one run, -..---)
 *   hold  >= 5 s      -> battery check: side tone stops at 5 s; on release the
 *                        supply voltage is sent as two digits in 0.1 V
 *                        ("29" = 2.9 V), then the device waits for the first
 *                        press again.
 *   QUIZ and RANDOM use the selected set.
 *   Power-cycle to choose again.
 *
 * Speed (5 / 10 / 15 / 20 WPM, default 5; saved in EEPROM)
 *   Hold the button while powering on -> speed setup: plays "V" (...-) at
 *   the current speed. Short press = next speed (5 -> 10 -> 15 -> 20 -> 5),
 *   hold reaching MODE_HOLD_MS = save to EEPROM and "pin-pon" right away
 *   (still held), then start normally after release.
 *   Playback and input timing both follow the selected speed.
 *
 * Volume (bridge / single-ended drive, default bridge; saved in EEPROM)
 *   Hold the button >= DRIVE_HOLD_MS while powering on (the LED lights at
 *   that point; then release) -> volume setup: plays "V" with the current
 *   drive. Short press = toggle bridge (loud) / single-ended (about -6 dB),
 *   hold reaching MODE_HOLD_MS = save to EEPROM and "pin-pon" right away
 *   (still held), then start normally after release.
 *   Note: a UPDI chip erase also clears EEPROM unless the EESAVE fuse is set.
 *
 * Pins (ATtiny202, 8-pin SOIC/DIP)   *** PA4/PA5 do not exist on this part ***
 *   pin 1  VDD
 *   pin 2  PA6  LED (via resistor to GND, drive high = on)
 *   pin 3  PA7  push button to GND (internal pull-up)
 *   pin 4  PA1  piezo speaker terminal A  } bridge: opposite phase (2x VDD swing,
 *   pin 5  PA2  piezo speaker terminal B  } +6 dB); single-ended: PA2 stays low
 *   pin 6  PA0  UPDI
 *   pin 7  PA3  (spare)
 *   pin 8  GND
 *
 * Clock: factory default 20 MHz / 6 = 3.333 MHz (no clock setup needed).
 * Flash 2 KB / RAM 128 B: keep it small. No stdio, no malloc, no floating point.
 */

#ifndef F_CPU
#define F_CPU 3333333UL
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/xmega.h>
#include <stdbool.h>
#include <stdint.h>

/* ------------------------------------------------------------------ */
/* Tunables                                                            */
/* ------------------------------------------------------------------ */
/* Morse unit = 1200 / WPM ms, selected at run time (see g_unit) */
#define DASH_MIN_MS      (g_unit * 2u)      /* press >= this -> dah, else dit    */
#define CHAR_GAP_MS      (g_unit * 3u)      /* released this long -> char done   */
#define IDLE_REPLAY_MS   8000u  /* no input for this long -> replay the question */
#define MODE_HOLD_MS     2000u  /* hold this long -> switch mode                 */
#define WABUN_HOLD_MS    1000u  /* first press held this long -> Wabun          */
#define VDD_HOLD_MS      5000u  /* first press held this long -> battery check  */
#define DRIVE_HOLD_MS    3000u  /* held this long at power-on -> volume setup   */
#define DEBOUNCE_MS      20u
#define MAX_SYMBOLS      6u     /* more dit/dah than this is treated as invalid  */
#define RANDOM_GAP_MS    900u   /* pause between characters in RANDOM mode       */
#define POWERDOWN_MS     30000u /* no button activity this long -> power down    */
#define POWERDOWN_IDLE_CYCLES (POWERDOWN_MS / IDLE_REPLAY_MS)  /* same, in QUIZ replay cycles */
#define RANDOM_OFF_MIN   1u    /* RANDOM: no button activity this many minutes -> power down */

#define MORSE_HZ         741u   /* piezo is louder near its resonance; tune by ear */

/* ------------------------------------------------------------------ */
/* Pins                                                                */
/* ------------------------------------------------------------------ */
#define SPKA_bm    PIN1_bm      /* PA1, piezo terminal A */
#define SPKB_bm    PIN2_bm      /* PA2, piezo terminal B */
#define SPK_bm     (SPKA_bm | SPKB_bm)
#define BTN_bm     PIN7_bm      /* PA7 */
#define LED_bm     PIN6_bm      /* PA6 */

/* ------------------------------------------------------------------ */
/* Settings (speed / volume), kept in EEPROM                           */
/* ------------------------------------------------------------------ */
#define N_SPEEDS   4u
static const uint8_t UNIT_MS[N_SPEEDS] = { 240, 120, 80, 60 };  /* 5/10/15/20 WPM */
#define EE_CFG     ((volatile uint8_t *)EEPROM_START)
#define CFG_SPEED  0u           /* speed index                                     */
#define CFG_DRIVE  1u           /* 1 = bridge, 0 = single-ended (blank 0xFF -> bridge) */

static uint8_t g_cfg[2];        /* RAM copy of the settings */
static uint8_t g_unit;          /* current Morse unit in ms */
static uint8_t g_spk_tgl;       /* speaker pins toggled by the tone ISR */

/* ------------------------------------------------------------------ */
/* Time base: TCB0 1 ms tick                                           */
/* ------------------------------------------------------------------ */
static volatile uint16_t g_ms;

ISR(TCB0_INT_vect)
{
    g_ms++;
    TCB0.INTFLAGS = TCB_CAPT_bm;
}

static uint16_t ms_now(void)
{
    uint8_t sreg = SREG;
    cli();
    uint16_t v = g_ms;
    SREG = sreg;
    return v;
}

static void wait_ms(uint16_t ms)
{
    uint16_t t0 = ms_now();
    while ((uint16_t)(ms_now() - t0) < ms) {
        sleep_cpu();          /* IDLE: CPU stops, TCB0/TCA0 keep running */
    }
}

/* ------------------------------------------------------------------ */
/* Piezo speaker tone: bridge = PA1/PA2 in opposite phase, both       */
/* toggled at once; single-ended = only PA1 toggles, PA2 stays low     */
/* ------------------------------------------------------------------ */
ISR(TCA0_OVF_vect)
{
    PORTA.OUTTGL = g_spk_tgl;
    TCA0.SINGLE.INTFLAGS = TCA_SINGLE_OVF_bm;
}

static void tone_off(void)
{
    TCA0.SINGLE.CTRLA = 0;                       /* stop timer            */
    TCA0.SINGLE.INTCTRL = 0;                     /* disable OVF interrupt */
    PORTA.OUTCLR = SPK_bm;                       /* both low: no voltage across the piezo */
}

/* Timer period for a tone, computed at compile time (no runtime divide).
 * Toggling twice per cycle -> overflow at 2x the tone frequency. DIV1 fits 16 bit. */
#define TONE_PER(hz)  ((uint16_t)(F_CPU / (2UL * (hz))) - 1u)

static void tone_on(uint16_t per)       /* per = TONE_PER(hz), 0 = silence */
{
    if (per == 0u) {
        tone_off();
        return;
    }
    TCA0.SINGLE.CTRLA = 0;
    PORTA.OUTSET = SPKA_bm;                      /* start in opposite states */
    PORTA.OUTCLR = SPKB_bm;
    TCA0.SINGLE.CNT   = 0;
    TCA0.SINGLE.PER   = per;
    TCA0.SINGLE.INTCTRL = TCA_SINGLE_OVF_bm;
    TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV1_gc | TCA_SINGLE_ENABLE_bm;
}

/* ------------------------------------------------------------------ */
/* LED                                                                 */
/* ------------------------------------------------------------------ */
/* VPORTA (I/O space): single sbi/cbi instructions, smaller than PORTA.OUTSET */
static inline void led_on(void)  { VPORTA.OUT |= LED_bm; }
static inline void led_off(void) { VPORTA.OUT &= (uint8_t)~LED_bm; }

/* ------------------------------------------------------------------ */
/* Sound sequencer: array of {period, duration}, per==0 is a rest     */
/* ------------------------------------------------------------------ */
typedef struct {
    uint16_t per;               /* TONE_PER(hz), 0 = rest */
    uint16_t ms;
} tone_step_t;

static const tone_step_t SND_START[]   = { {TONE_PER(1000), 80}, {TONE_PER(1500), 80}, {TONE_PER(2000), 120} };
static const tone_step_t SND_CORRECT[] = { {TONE_PER(1000), 100}, {TONE_PER(800), 100} };  /* pin-pon */
static const tone_step_t SND_WRONG[]   = { {TONE_PER(180), 400} };               /* buu     */
static const tone_step_t SND_MODE_QUIZ[] = { {TONE_PER(1200), 80}, {0, 40}, {TONE_PER(1200), 80} };
static const tone_step_t SND_MODE_RANDOM[] = { {TONE_PER(1200), 60}, {0, 40}, {TONE_PER(1200), 60}, {0, 40}, {TONE_PER(1200), 60} };

static void play_seq(const tone_step_t *s, uint8_t n, bool with_led)
{
    for (uint8_t i = 0; i < n; i++) {
        tone_on(s[i].per);
        if (with_led && s[i].per) led_on();
        wait_ms(s[i].ms);
        tone_off();
        led_off();
    }
}
#define PLAY(seq, led) play_seq((seq), (uint8_t)(sizeof(seq) / sizeof((seq)[0])), (led))

/* ------------------------------------------------------------------ */
/* Morse table. Sentinel-bit coding: leading 1 marks the start,        */
/* following bits are 0 = dit, 1 = dah (MSB first).                    */
/* ------------------------------------------------------------------ */
#define N_LATIN 26u              /* Latin : CHARSET[0 .. 25]                   */
#define N_WABUN 50u              /* Wabun : CHARSET[0 .. 49]  iroha incl. ヰ ヱ ン 濁点 半濁点 */
/*
 * Latin and Wabun share 26 codes (all of A-Z; I ".." is the dakuten), so
 * both sets start at index 0 and only the end of the range differs:
 *   [0 .. 25]  shared     (Latin / Wabun)
 *   [26 .. 49] Wabun only
 */
static const uint8_t CHARSET[N_WABUN] = {
    /* shared: Latin / Wabun */
    0x05,   /* A イ    .-    */
    0x18,   /* B ハ    -...  */
    0x1A,   /* C ニ    -.-.  */
    0x0C,   /* D ホ    -..   */
    0x02,   /* E ヘ    .     */
    0x12,   /* F チ    ..-.  */
    0x0E,   /* G リ    --.   */
    0x10,   /* H ヌ    ....  */
    0x04,   /* I 濁点  ..    */
    0x17,   /* J ヲ    .---  */
    0x0D,   /* K ワ    -.-   */
    0x14,   /* L カ    .-..  */
    0x07,   /* M ヨ    --    */
    0x06,   /* N タ    -.    */
    0x0F,   /* O レ    ---   */
    0x16,   /* P ツ    .--.  */
    0x1D,   /* Q ネ    --.-  */
    0x0A,   /* R ナ    .-.   */
    0x08,   /* S ラ    ...   */
    0x03,   /* T ム    -     */
    0x09,   /* U ウ    ..-   */
    0x11,   /* V ク    ...-  */
    0x0B,   /* W ヤ    .--   */
    0x19,   /* X マ    -..-  */
    0x1B,   /* Y ケ    -.--  */
    0x1C,   /* Z フ    --..  */
    /* Wabun only */
    0x15,   /* ロ      .-.-  */
    0x24,   /* ト      ..-.. */
    0x36,   /* ル      -.--. */
    0x1E,   /* ソ      ---.  */
    0x29,   /* ヰ      .-..- */
    0x13,   /* ノ      ..--  */
    0x28,   /* オ      .-... */
    0x1F,   /* コ      ----  */
    0x37,   /* エ      -.--- */
    0x2B,   /* テ      .-.-- */
    0x3B,   /* ア      --.-- */
    0x35,   /* サ      -.-.- */
    0x34,   /* キ      -.-.. */
    0x33,   /* ユ      -..-- */
    0x31,   /* メ      -...- */
    0x25,   /* ミ      ..-.- */
    0x3A,   /* シ      --.-. */
    0x2C,   /* ヱ      .--.. */
    0x39,   /* ヒ      --..- */
    0x32,   /* モ      -..-. */
    0x2E,   /* セ      .---. */
    0x3D,   /* ス      ---.- */
    0x2A,   /* ン      .-.-. */
    0x26,   /* 半濁点  ..--. */
};

/* Active set: CHARSET[0 .. g_hi - 1] */
static uint8_t g_hi = N_LATIN;

static void play_char(uint8_t code)
{
    int8_t top = 7;
    while (top > 0 && !(code & (1u << top))) {
        top--;
    }
    for (int8_t i = top - 1; i >= 0; i--) {
        uint16_t len = (code & (1u << i)) ? (g_unit * 3u) : g_unit;
        tone_on(TONE_PER(MORSE_HZ));
        led_on();
        wait_ms(len);
        tone_off();
        led_off();
        wait_ms(g_unit);
    }
}

/* ------------------------------------------------------------------ */
/* Xorshift16                                                          */
/* ------------------------------------------------------------------ */
static uint16_t g_rng = 0xACE1u;

static uint16_t rng16(void)
{
    uint16_t x = g_rng;
    x ^= (uint16_t)(x << 7);
    x ^= (uint16_t)(x >> 9);
    x ^= (uint16_t)(x << 8);
    g_rng = x;
    return x;
}

#define PICK_MASK 0x3Fu   /* smallest (2^n - 1) >= N_WABUN - 1, for rejection sampling */

/* Returns an absolute CHARSET index inside the active set. */
static uint8_t pick_index(uint8_t avoid)
{
    uint8_t idx;
    do {
        idx = (uint8_t)(rng16() & PICK_MASK);   /* mask only: no divide routine needed */
    } while (idx >= g_hi || idx == avoid);
    return idx;
}

/* ------------------------------------------------------------------ */
/* Button                                                              */
/* ------------------------------------------------------------------ */
static inline bool btn_down(void)
{
    return (VPORTA.IN & BTN_bm) == 0;       /* sbic: smaller than PORTA.IN */
}

static void wait_release(void)
{
    while (btn_down()) {
        sleep_cpu();
    }
    wait_ms(DEBOUNCE_MS);
}

/* PORTA pin interrupt: only used to wake the MCU from POWER-DOWN sleep;
 * the button itself is still read by polling, so nothing else to do here. */
ISR(PORTA_PORT_vect)
{
    PORTA.INTFLAGS = PORTA.INTFLAGS;
}

/* Full power-down: everything (CPU, TCB0 tick, TCA0 tone) stops. Woken only
 * by a button press (PORTA pin interrupt configured in init()). */
static void power_down(void)
{
    SLPCTRL.CTRLA = SLPCTRL_SMODE_PDOWN_gc | SLPCTRL_SEN_bm;
    sleep_cpu();
    SLPCTRL.CTRLA = SLPCTRL_SMODE_IDLE_gc | SLPCTRL_SEN_bm; /* back to IDLE */
    wait_release();                     /* debounce + swallow the waking press  */
}

#define CODE_NONE     0x00u     /* nothing entered before timeout     */
#define CODE_MODE     0xFEu     /* long hold: switch mode             */
#define CODE_INVALID  0xFFu     /* too many symbols                   */

/*
 * Collect one character (sentinel-coded). Shows side tone + LED while the
 * button is held. Returns CODE_NONE / CODE_MODE / CODE_INVALID or the code.
 */
static uint8_t collect_input(void)
{
    uint8_t  acc = 1;
    uint8_t  n = 0;
    bool     over = false;
    uint16_t t_start = ms_now();
    uint16_t t_rel = t_start;

    wait_release();                     /* ignore a press left over from playback */

    for (;;) {
        while (!btn_down()) {
            uint16_t now = ms_now();
            if (n == 0) {
                if ((uint16_t)(now - t_start) >= IDLE_REPLAY_MS) {
                    return CODE_NONE;
                }
            } else if ((uint16_t)(now - t_rel) >= CHAR_GAP_MS) {
                return over ? CODE_INVALID : acc;
            }
            sleep_cpu();
        }
        wait_ms(DEBOUNCE_MS);
        if (!btn_down()) {
            continue;                   /* bounce */
        }

        uint16_t t_press = ms_now();
        tone_on(TONE_PER(MORSE_HZ));
        led_on();
        while (btn_down()) {
            if ((uint16_t)(ms_now() - t_press) >= MODE_HOLD_MS) {
                tone_off();
                led_off();
                wait_release();
                return CODE_MODE;
            }
            sleep_cpu();
        }
        uint16_t dur = (uint16_t)(ms_now() - t_press);
        tone_off();
        led_off();
        wait_ms(DEBOUNCE_MS);

        if (n < MAX_SYMBOLS) {
            acc = (uint8_t)((acc << 1) | (dur >= DASH_MIN_MS ? 1u : 0u));
        } else {
            over = true;
        }
        n++;
        t_rel = ms_now();
    }
}

/* ------------------------------------------------------------------ */
/* Modes                                                               */
/* ------------------------------------------------------------------ */
static void feedback_correct(void)
{
    PLAY(SND_CORRECT, true);
    led_on();
    wait_ms(400);
    led_off();
}

static void feedback_wrong(void)
{
    tone_on(TONE_PER(180));
    for (uint8_t i = 0; i < 4; i++) {           /* LED blinks during the buzz */
        led_on();
        wait_ms(100);
        led_off();
        wait_ms(100);
    }
    tone_off();
}

/* Returns when the player asks for a mode switch. */
static void run_quiz(void)
{
    uint8_t idx = pick_index(0xFF);
    uint8_t idle_cycles = 0;

    for (;;) {
        uint8_t target = CHARSET[idx];

        wait_ms(500);
        play_char(target);

        uint8_t in = collect_input();
        if (in == CODE_MODE) {
            return;
        }
        if (in == CODE_NONE) {
            if (++idle_cycles >= POWERDOWN_IDLE_CYCLES) {
                power_down();
                idle_cycles = 0;
            }
            continue;                           /* replay same character */
        }
        idle_cycles = 0;
        if (in == target) {
            feedback_correct();
            idx = pick_index(idx);              /* next character */
        } else {
            feedback_wrong();                   /* retry same character */
        }
    }
}

#define GAP_NONE  0u            /* no button activity        */
#define GAP_TAP   1u            /* short tap (otherwise ignored) */
#define GAP_MODE  2u            /* held: switch mode         */

/*
 * Waits up to ms, but returns early (GAP_MODE) if the button is held long
 * enough to request a mode switch. A short tap is debounced and reported.
 */
static uint8_t wait_gap_or_modeswitch(uint16_t ms)
{
    uint8_t r = GAP_NONE;
    uint16_t t0 = ms_now();
    while ((uint16_t)(ms_now() - t0) < ms) {
        if (btn_down()) {
            uint16_t t_press = ms_now();
            led_on();
            while (btn_down()) {
                if ((uint16_t)(ms_now() - t_press) >= MODE_HOLD_MS) {
                    led_off();
                    wait_release();
                    return GAP_MODE;
                }
                sleep_cpu();
            }
            led_off();
            wait_ms(DEBOUNCE_MS);
            r = GAP_TAP;
        }
        sleep_cpu();
    }
    return r;
}

/* Listening practice: plays random characters back to back, no input needed.
 * After RANDOM_OFF_MIN minutes without button activity it powers down (a
 * short tap restarts the count); a press wakes it and playback resumes.
 * Returns when the player asks for a mode switch. */
static void run_random(void)
{
    uint8_t idx = pick_index(0xFF);
    uint16_t t_min = ms_now();          /* start of the current minute */
    uint8_t minutes = 0;

    for (;;) {
        play_char(CHARSET[idx]);
        uint8_t g = wait_gap_or_modeswitch(RANDOM_GAP_MS);
        if (g == GAP_MODE) {
            return;
        }
        /* One character + gap is far below the 65 s wrap of ms_now(). */
        if ((uint16_t)(ms_now() - t_min) >= 60000u) {
            t_min += 60000u;
            minutes++;
        }
        if (g == GAP_TAP || minutes >= RANDOM_OFF_MIN) {
            if (g == GAP_NONE) {
                power_down();               /* returns after the waking press */
            }
            t_min = ms_now();
            minutes = 0;
        }
        idx = pick_index(idx);
    }
}

/* ------------------------------------------------------------------ */
/* Battery check: measure the internal 1.1 V reference against VDD     */
/* ------------------------------------------------------------------ */
static const uint8_t DIGITS[10] = {
    0x3F, 0x2F, 0x27, 0x23, 0x21,   /* 0 -----  1 .----  2 ..---  3 ...--  4 ....- */
    0x20, 0x30, 0x38, 0x3C, 0x3E,   /* 5 .....  6 -....  7 --...  8 ---..  9 ----. */
};

static void report_vdd(void)
{
    VREF.CTRLA   = VREF_ADC0REFSEL_1V1_gc;
    ADC0.CTRLC   = ADC_SAMPCAP_bm | ADC_REFSEL_VDDREF_gc | ADC_PRESC_DIV4_gc;
    ADC0.CTRLD   = ADC_INITDLY_DLY64_gc;       /* let the reference settle */
    ADC0.MUXPOS  = ADC_MUXPOS_INTREF_gc;
    ADC0.CTRLA   = ADC_ENABLE_bm;
    uint16_t res = 0;
    for (uint8_t i = 0; i < 2u; i++) {          /* first conversion is discarded */
        ADC0.COMMAND = ADC_STCONV_bm;
        while (!(ADC0.INTFLAGS & ADC_RESRDY_bm)) {
        }
        res = ADC0.RES;                         /* reading RES clears RESRDY */
    }
    ADC0.CTRLA = 0;                             /* ADC off again */

    /* res = 1.1 V / VDD * 1023  ->  VDD in 0.1 V = 11 * 1023 / res (rounded).
     * Divide by repeated subtraction: no divide routine needed. */
    uint16_t x = 11u * 1023u + res / 2u;
    uint8_t v = 0;
    while (x >= res) {
        x -= res;
        v++;
    }
    uint8_t tens = 0;
    while (v >= 10u) {
        v -= 10u;
        tens++;
    }
    play_char(DIGITS[tens]);
    wait_ms(g_unit * 2u);                       /* + 1 unit from play_char = char gap */
    play_char(DIGITS[v]);
}

/* ------------------------------------------------------------------ */
/* Settings setup (button held at power-on)                            */
/* ------------------------------------------------------------------ */
static void apply_cfg(void)
{
    g_unit = UNIT_MS[g_cfg[CFG_SPEED]];
    g_spk_tgl = g_cfg[CFG_DRIVE] ? SPK_bm : SPKA_bm;
}

/* Short press = next value (g_cfg[k] + 1) & mask. A hold is saved as soon as it
 * reaches MODE_HOLD_MS ("pin-pon" while still held), so the player knows when
 * to let go; returns after release. */
static void cfg_setup(uint8_t k, uint8_t mask)
{
    wait_release();
    for (;;) {
        apply_cfg();
        wait_ms(500);
        play_char(0x11);                        /* V ...- with this setting */
        while (!btn_down()) {
            sleep_cpu();
        }
        uint16_t t_press = ms_now();
        while (btn_down() && (uint16_t)(ms_now() - t_press) < MODE_HOLD_MS) {
            sleep_cpu();
        }
        if (btn_down()) {
            break;                              /* held MODE_HOLD_MS: save now */
        }
        wait_ms(DEBOUNCE_MS);
        g_cfg[k] = (uint8_t)((g_cfg[k] + 1u) & mask);
    }
    if (EE_CFG[k] != g_cfg[k]) {
        EE_CFG[k] = g_cfg[k];                   /* load EEPROM page buffer (only */
                                                /* loaded bytes are erased/written) */
        _PROTECTED_WRITE_SPM(NVMCTRL.CTRLA, NVMCTRL_CMD_PAGEERASEWRITE_gc);
        while (NVMCTRL.STATUS & NVMCTRL_EEBUSY_bm) {
        }
    }
    feedback_correct();                         /* saved */
    wait_release();                             /* don't pass the hold on as the first press */
}

/* ------------------------------------------------------------------ */
/* Init / main                                                         */
/* ------------------------------------------------------------------ */
static void init(void)
{
    /* Pins */
    PORTA.DIRSET = SPK_bm | LED_bm;
    PORTA.PIN7CTRL = PORT_PULLUPEN_bm | PORT_ISC_BOTHEDGES_gc; /* button (PA7)  */
    /* PA7 is not a fully async pin: only BOTHEDGES/LEVEL can wake POWER-DOWN */
    PORTA.PIN3CTRL = PORT_ISC_INPUT_DISABLE_gc; /* PA3 unused: no floating-input current */
    tone_off();
    led_off();

    /* 1 ms tick on TCB0 (periodic interrupt) */
    TCB0.CCMP    = (uint16_t)(F_CPU / 1000UL) - 1u;
    TCB0.INTCTRL = TCB_CAPT_bm;
    TCB0.CTRLA   = TCB_ENABLE_bm;               /* CLKSEL = CLK_PER (DIV1)  */

    /* IDLE: CPU only; TCB0/TCA0 keep running. Sleep stays enabled (SEN), so
     * every wait is a single SLEEP instruction (sleep_cpu). */
    SLPCTRL.CTRLA = SLPCTRL_SMODE_IDLE_gc | SLPCTRL_SEN_bm;
    sei();
}

int main(void)
{
    init();

    uint8_t sp = EE_CFG[CFG_SPEED];
    if (sp >= N_SPEEDS) {
        sp = 0;                                 /* blank EEPROM (0xFF) -> 5 WPM */
    }
    g_cfg[CFG_SPEED] = sp;
    g_cfg[CFG_DRIVE] = EE_CFG[CFG_DRIVE] & 1u;  /* blank EEPROM (0xFF) -> bridge */
    apply_cfg();
    wait_ms(DEBOUNCE_MS);
    if (btn_down()) {
        /* Released before DRIVE_HOLD_MS = speed setup, after = volume setup
         * (the LED lights at DRIVE_HOLD_MS to tell the player to release). */
        uint16_t t_press = ms_now();
        uint8_t k = CFG_SPEED;
        uint8_t mask = N_SPEEDS - 1u;
        while (btn_down()) {
            if ((uint16_t)(ms_now() - t_press) >= DRIVE_HOLD_MS) {
                k = CFG_DRIVE;
                mask = 1u;
                led_on();
            }
            sleep_cpu();
        }
        led_off();
        cfg_setup(k, mask);
    }

    PLAY(SND_START, true);

    uint16_t held;
    for (;;) {
        /* Wait for the first press; use its timing as the random seed. */
        uint16_t t_idle = ms_now();
        while (!btn_down()) {
            if ((uint16_t)(ms_now() - t_idle) >= POWERDOWN_MS) {
                power_down();
                t_idle = ms_now();
            }
            sleep_cpu();
        }
        g_rng ^= (uint16_t)(ms_now() ^ (uint16_t)(TCB0.CNT << 5));
        if (g_rng == 0u) {
            g_rng = 0xACE1u;
        }

        /* First press: < 1 s = Latin, 1 .. 5 s = Wabun, >= 5 s = battery check.
         * The side tone stops at 5 s to show the battery check is armed. */
        uint16_t t_press = ms_now();
        tone_on(TONE_PER(MORSE_HZ));
        led_on();
        do {
            held = (uint16_t)(ms_now() - t_press);
            if (held >= VDD_HOLD_MS) {
                tone_off();
                led_off();
            }
            sleep_cpu();
        } while (btn_down());
        tone_off();
        led_off();
        wait_ms(500);
        if (held < VDD_HOLD_MS) {
            break;
        }
        report_vdd();                           /* then wait for the first press again */
    }
    if (held >= WABUN_HOLD_MS) {
        g_hi = N_WABUN;
        play_char(0x67);                        /* ホレ -..--- (no gap) */
    }
    wait_ms(500);

    for (;;) {
        PLAY(SND_MODE_QUIZ, true);
        run_quiz();
        PLAY(SND_MODE_RANDOM, true);
        run_random();
    }
}
