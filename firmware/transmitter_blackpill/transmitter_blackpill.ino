/*
 ============================================================================
  transmitter_blackpill.ino - Software-Defined Sonar Transmitter (AUV demo)
  Target : WeAct "Black Pill" STM32F411CEU6   (STM32duino core, Arduino IDE)
  DAC    : MCP4921 (12-bit, SPI)       Filter : Sallen-Key low-pass (2 poles)
  Inputs : 3 x 10k pots (simulated environment sensors) + on-board KEY button
 ----------------------------------------------------------------------------
  WIRING
    MCP4921 CS   <- PA4  (driven by DMA through GPIOA->BSRR, NOT by the CPU)
    MCP4921 SCK  <- PA5  (SPI1_SCK)
    MCP4921 SDI  <- PA7  (SPI1_MOSI)
    MCP4921 LDAC <- PB0  (firmware holds it LOW => DAC updates on CS rising edge)
    MCP4921 VDD/VREFA = 3.3 V, AVSS = GND, VOUTA -> R -> op-amp IN+ (filter)
    POT1 wiper -> PA1 (ADC1_IN1)  Bandwidth / centre frequency
    POT2 wiper -> PA2 (ADC1_IN2)  Pulse duration
    POT3 wiper -> PA3 (ADC1_IN3)  Amplitude / signal power
    KEY button (on board) PA0, LED PC13 (active low, on board)

  HOW A SAMPLE REACHES THE DAC - ZERO CPU INVOLVEMENT
    TIM1 free-runs at the sample rate Fs. Three timer events per sample each
    trigger a DMA2 stream:
        TIM1 UPDATE (t = 0)     -> DMA2 S5 ch6 : write BSRR  -> CS  LOW
        TIM1 CC1    (t = 15 %)  -> DMA2 S1 ch6 : write SPI1->DR (16-bit word)
        TIM1 CC4    (t = 60 %)  -> DMA2 S4 ch6 : write BSRR  -> CS  HIGH
    The rising edge of CS latches the word and (LDAC=0) updates VOUT.

  BUTTON
    Short press : cycle modulation   LFM -> GEOMETRIC -> PHASE-CODED
    Long  press : cycle window       HANN -> HAMMING -> BLACKMAN
    LED flashes n times to show what was selected (1, 2, 3).

  OUTPUT LEVEL
    The MCU cannot make negative voltages. The sine is centred on DAC code
    2048 (about 1.65 V), so the "negative" half is the part below 1.65 V.

  TEST MODE
    TEST_CW 1 sends a continuous 10 kHz tone (no window, no pings) so you can
    check the DAC, filter and receiver before using real pings.
 ============================================================================
*/
#include <Arduino.h>
#include <math.h>

// ============================ USER CONFIGURATION ===========================
#define FS_HZ              200000UL   // DAC sample rate (5 us/sample; MCP4921 needs ~4.5 us to settle)
#define PRI_MS             100UL      // ping repetition interval
#define PULSE_MIN_MS       2.0f       // POT2 fully CCW
#define PULSE_MAX_MS       20.0f      // POT2 fully CW  (buffer is sized for this)
#define BW_MIN_HZ          5000.0f    // POT1 fully CCW
#define BW_MAX_HZ          25000.0f   // POT1 fully CW
#define FC_MIN_HZ          8000.0f    // centre frequency at POT1 = 0
#define FC_MAX_HZ          17500.0f   // centre frequency at POT1 = 1
#define AMP_MIN            0.10f      // POT3 fully CCW
#define AMP_MAX            1.00f      // POT3 fully CW
#define DEFAULT_MODE       0          // 0 LFM, 1 GEOMETRIC, 2 PHASE-CODED
#define DEFAULT_WINDOW     0          // 0 HANN, 1 HAMMING, 2 BLACKMAN
#define LED_ENABLED        1
#define DEBUG_SERIAL       0          // 1 = print parameters over USB serial
#define TEST_CW            1          // 1 = continuous 10 kHz test tone, 0 = normal pings
// ===========================================================================

// ---- derived constants ----
#define PAD_SAMPLES        8          // mid-scale guard samples before/after pulse
#define MAX_PULSE_SAMPLES  ((uint32_t)(FS_HZ * 20UL / 1000UL))   // 20 ms
#define MAX_SAMPLES        (MAX_PULSE_SAMPLES + 2 * PAD_SAMPLES)
#define DAC_MID            2048
#define DAC_CMD            0x3000u    // MCP4921: DACa, unbuffered Vref, 1x gain, active
#define ADC_AVG            8          // scans averaged per ping
#define LUT_SIZE           1024
#define TWO_PI_F           6.28318530718f

enum { MOD_LFM = 0, MOD_GEO = 1, MOD_PHASE = 2, MOD_COUNT = 3 };
enum { WIN_HANN = 0, WIN_HAMMING = 1, WIN_BLACKMAN = 2, WIN_COUNT = 3 };

// ============================ GLOBAL DATA ==================================
static uint16_t txBuf[MAX_SAMPLES] __attribute__((aligned(4)));   // DAC words
static volatile uint16_t adcBuf[ADC_AVG * 3] __attribute__((aligned(4)));
static float    sineLut[LUT_SIZE + 1];
static uint8_t  mseq[511];                  // 9-bit m-sequence chips (0/1)
static const int8_t barker13[13] = {1,1,1,1,1,-1,-1,1,1,-1,1,-1,1};

// DMA source words must live in SRAM (non-const globals)
static uint32_t csLowWord  = (1u << (4 + 16));   // BSRR: reset PA4
static uint32_t csHighWord = (1u << 4);          // BSRR: set   PA4

static uint8_t  gMode  = DEFAULT_MODE;
static uint8_t  gWin   = DEFAULT_WINDOW;
static bool     gNoWin = false;                  // true = rectangular window (test tone)
static float    gFs    = (float)FS_HZ;           // actual sample rate
static uint32_t gArr   = 0;

struct Params {
  float    bw, fc, f0, f1;      // Hz
  float    amp;                 // 0..1
  uint32_t nPulse;              // samples in pulse
  float    durMs;
};

// ============================ SMALL HELPERS ================================
static inline float sinCyc(float cyc) {            // sin(2*pi*cyc) via LUT + lerp
  cyc -= floorf(cyc);
  float idx = cyc * (float)LUT_SIZE;
  int   i   = (int)idx;
  float fr  = idx - (float)i;
  return sineLut[i] + fr * (sineLut[i + 1] - sineLut[i]);
}
static inline float cosCyc(float cyc) { return sinCyc(cyc + 0.25f); }

static float windowFn(uint8_t type, uint32_t m, uint32_t M) {
  float x = (float)m / (float)(M - 1);             // 0..1 across the pulse
  switch (type) {
    case WIN_HANN:    return 0.5f  - 0.5f  * cosCyc(x);
    case WIN_HAMMING: return 0.54f - 0.46f * cosCyc(x);
    default:          return 0.42f - 0.5f  * cosCyc(x) + 0.08f * cosCyc(2.0f * x);
  }
}

static void initTables() {
  for (int i = 0; i <= LUT_SIZE; i++) sineLut[i] = sinf(TWO_PI_F * (float)i / (float)LUT_SIZE);
  uint16_t l = 0x1FF;                              // x^9 + x^5 + 1 (maximal length)
  for (int i = 0; i < 511; i++) {
    mseq[i] = l & 1;
    uint16_t fb = ((l >> 8) ^ (l >> 4)) & 1;
    l = ((l << 1) | fb) & 0x1FF;
  }
}

// ============================ LOW LEVEL HW =================================
static inline void dmaStop(DMA_Stream_TypeDef* s) {
  s->CR &= ~DMA_SxCR_EN;
  uint32_t t = 100000; while ((s->CR & DMA_SxCR_EN) && t--) {}
}
static inline void dmaClearAll() {                 // clear every flag of S0, S1, S4, S5
  DMA2->LIFCR = (0x3Du << 0) | (0x3Du << 6);
  DMA2->HIFCR = (0x3Du << 0) | (0x3Du << 6);
}
static inline void dmaStart(DMA_Stream_TypeDef* s, uint32_t cr, uint32_t par,
                            uint32_t mar, uint32_t n) {
  s->CR = cr; s->PAR = par; s->M0AR = mar; s->NDTR = n; s->FCR = 0;  // direct mode
  s->CR = cr | DMA_SxCR_EN;
}

// DMA2 stream CR templates (CHSEL=6 => TIM1 requests on S1/S4/S5)
#define CR_CH6          (6u << 25)
#define CR_DATA_SPI     (CR_CH6 | (3u<<16) | (1u<<13) | (1u<<11) | (1u<<10) | (1u<<6)) // 16-bit, MINC, m->p, very high prio
#define CR_BSRR_CONST   (CR_CH6 | (2u<<16) | (2u<<13) | (2u<<11) | (1u<<6))            // 32-bit, no MINC, m->p, high prio

static void spiWriteBlocking(uint16_t w) {         // used only at start-up / idle
  GPIOA->BSRR = csLowWord;
  while (!(SPI1->SR & SPI_SR_TXE)) {}
  SPI1->DR = w;
  while (!(SPI1->SR & SPI_SR_TXE)) {}
  while (SPI1->SR & SPI_SR_BSY) {}
  GPIOA->BSRR = csHighWord;
}

static void hwInit() {
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN |
                  RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_DMA2EN;
  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN | RCC_APB2ENR_TIM1EN | RCC_APB2ENR_ADC1EN;
  (void)RCC->APB2ENR;

  // ---- GPIOA: PA0 key (in, pull-up) PA1..3 analog, PA4 CS (out), PA5/PA7 AF5 ----
  GPIOA->BSRR = (1u << 4);                                     // CS high first
  uint32_t m = GPIOA->MODER;
  m &= ~((3u<<0)|(3u<<2)|(3u<<4)|(3u<<6)|(3u<<8)|(3u<<10)|(3u<<14));
  m |=  (0u<<0) | (3u<<2) | (3u<<4) | (3u<<6) | (1u<<8) | (2u<<10) | (2u<<14);
  GPIOA->MODER = m;
  GPIOA->PUPDR = (GPIOA->PUPDR & ~(3u<<0)) | (1u<<0);          // PA0 pull-up
  GPIOA->OSPEEDR |= (3u<<8) | (3u<<10) | (3u<<14);             // very high speed
  GPIOA->AFR[0]   = (GPIOA->AFR[0] & ~((0xFu<<20)|(0xFu<<28))) | (5u<<20) | (5u<<28);

  // ---- PB0 = LDAC held LOW (DAC output updates on CS rising edge) ----
  GPIOB->BSRR  = (1u << (0 + 16));
  GPIOB->MODER = (GPIOB->MODER & ~(3u<<0)) | (1u<<0);

  // ---- PC13 LED (active low) ----
  GPIOC->BSRR  = (1u << 13);
  GPIOC->MODER = (GPIOC->MODER & ~(3u<<26)) | (1u<<26);

  // ---- SPI1: master, 16-bit, mode 0, TX-only, SW NSS, fPCLK2/8 ----
  SPI1->CR1 = 0;
  SPI1->CR1 = SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE | SPI_CR1_DFF |
              SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_MSTR | (2u << 3);   // BR=010 => /8
  SPI1->CR1 |= SPI_CR1_SPE;

  // ---- TIM1: free-running sample clock with three DMA-request events ----
  uint32_t pclk2  = HAL_RCC_GetPCLK2Freq();
  uint32_t timclk = (RCC->CFGR & RCC_CFGR_PPRE2_2) ? pclk2 * 2 : pclk2;
  gArr = timclk / FS_HZ - 1;
  gFs  = (float)timclk / (float)(gArr + 1);
  TIM1->CR1  = 0;
  TIM1->PSC  = 0;
  TIM1->ARR  = gArr;
  TIM1->CCR1 = ((gArr + 1) * 15u) / 100u;          // data -> SPI (15 %)
  TIM1->CCR4 = ((gArr + 1) * 60u) / 100u;          // CS high     (60 %)
  TIM1->DIER = 0;

  // ---- ADC1: 3-channel scan (IN1, IN2, IN3), 12-bit, 144-cycle sampling, DMA ----
  ADC->CCR   = (ADC->CCR & ~ADC_CCR_ADCPRE) | (1u << 16);      // PCLK2 / 4
  ADC1->CR1  = ADC_CR1_SCAN;
  ADC1->CR2  = 0;
  ADC1->SMPR2 = (6u << 3) | (6u << 6) | (6u << 9);             // ch1..3 : 144 cycles
  ADC1->SQR1 = (2u << 20);                                     // 3 conversions
  ADC1->SQR3 = (1u << 0) | (2u << 5) | (3u << 10);

  spiWriteBlocking(DAC_CMD | DAC_MID);                         // park DAC at mid-scale
}

// Read 3 pots with ADC + DMA (averaged), returns 0..1 floats
static void readPots(float* x) {
  ADC1->CR2 = 0;
  ADC1->SR  = 0;
  dmaStop(DMA2_Stream0);
  dmaClearAll();
  dmaStart(DMA2_Stream0,
           (0u << 25) | (1u << 16) | (1u << 13) | (1u << 11) | (1u << 10),  // ch0, 16-bit, MINC, p->m
           (uint32_t)&ADC1->DR, (uint32_t)adcBuf, ADC_AVG * 3);
  ADC1->CR2 = ADC_CR2_ADON;
  delayMicroseconds(10);                                       // tSTAB
  ADC1->CR2 = ADC_CR2_ADON | ADC_CR2_CONT | ADC_CR2_DMA;
  ADC1->CR2 |= ADC_CR2_SWSTART;
  uint32_t t0 = millis();
  while (!(DMA2->LISR & DMA_LISR_TCIF0)) { if (millis() - t0 > 5) break; }
  ADC1->CR2 = 0;                                               // ADC off => no idle power
  dmaStop(DMA2_Stream0);
  dmaClearAll();
  uint32_t acc[3] = {0, 0, 0};
  for (int i = 0; i < ADC_AVG; i++)
    for (int c = 0; c < 3; c++) acc[c] += adcBuf[i * 3 + c];
  for (int c = 0; c < 3; c++) {
    float v = (float)acc[c] / (float)(ADC_AVG * 4095);
    x[c] = v < 0.f ? 0.f : (v > 1.f ? 1.f : v);
  }
}

// ============================ ADAPTATION LOGIC =============================
static Params mapParams(const float* x) {
  Params p;
  p.bw    = BW_MIN_HZ + x[0] * (BW_MAX_HZ - BW_MIN_HZ);   // range <-> resolution
  p.fc    = FC_MIN_HZ + x[0] * (FC_MAX_HZ - FC_MIN_HZ);
  p.f0    = p.fc - 0.5f * p.bw;
  p.f1    = p.fc + 0.5f * p.bw;
  p.durMs = PULSE_MIN_MS + x[1] * (PULSE_MAX_MS - PULSE_MIN_MS);
  p.nPulse = (uint32_t)(p.durMs * 0.001f * gFs + 0.5f);
  if (p.nPulse > MAX_PULSE_SAMPLES) p.nPulse = MAX_PULSE_SAMPLES;
  p.amp   = AMP_MIN + x[2] * (AMP_MAX - AMP_MIN);
  return p;
}

// ============================ WAVEFORM SYNTHESIS ===========================
// Fills txBuf with MCP4921 command words. Returns total word count.
static uint32_t buildWaveform(const Params& p) {
  const uint32_t M  = p.nPulse;
  const float    dt = 1.0f / gFs;
  const float    Tp = (float)M * dt;
  const uint16_t mid = DAC_CMD | DAC_MID;

  for (uint32_t i = 0; i < PAD_SAMPLES; i++) txBuf[i] = mid;

  const float k    = (p.f1 - p.f0) / Tp;                 // LFM chirp rate [Hz/s]
  const float lnr  = logf(p.f1 / p.f0);                  // geometric sweep
  uint32_t nChips  = (uint32_t)(p.bw * Tp * 0.5f + 0.5f);// chip rate = BW/2
  bool barker = false;
  if (nChips <= 13) { nChips = 13; barker = true; }
  if (nChips > 511) nChips = 511;

  for (uint32_t m = 0; m < M; m++) {
    const float t = (float)m * dt;
    float s;
    switch (gMode) {
      case MOD_GEO: {                                    // f(t) = f0 * (f1/f0)^(t/T)
        float cyc = p.f0 * Tp / lnr * (expf(lnr * t / Tp) - 1.0f);
        s = sinCyc(cyc);
      } break;
      case MOD_PHASE: {                                  // BPSK on carrier fc
        uint32_t c = (uint32_t)(((uint64_t)m * nChips) / M);
        float sign = barker ? (float)barker13[c] : (mseq[c] ? 1.0f : -1.0f);
        s = sign * sinCyc(p.fc * t);
      } break;
      default: {                                         // LFM: phi = f0 t + k t^2 / 2
        float cyc = p.f0 * t + 0.5f * k * t * t;
        s = sinCyc(cyc);
      } break;
    }
    float w = gNoWin ? 1.0f : windowFn(gWin, m, M);      // window (off in test tone)
    float y = p.amp * w * s;
    int   v = DAC_MID + (int)(2047.0f * y + (y >= 0.f ? 0.5f : -0.5f));
    if (v < 0) v = 0; else if (v > 4095) v = 4095;
    txBuf[PAD_SAMPLES + m] = (uint16_t)(DAC_CMD | (uint16_t)v);
  }
  for (uint32_t i = 0; i < PAD_SAMPLES; i++) txBuf[PAD_SAMPLES + M + i] = mid;
  return M + 2 * PAD_SAMPLES;
}

// ============================ TRANSMIT (TIMER + DMA) =======================
static bool transmit(uint32_t n) {
  TIM1->CR1  = 0;
  TIM1->DIER = 0;
  dmaStop(DMA2_Stream1); dmaStop(DMA2_Stream4); dmaStop(DMA2_Stream5);
  dmaClearAll();
  TIM1->SR   = 0;
  TIM1->CNT  = gArr - 2;                // first UPDATE event comes after 3 counts

  dmaStart(DMA2_Stream5, CR_BSRR_CONST, (uint32_t)&GPIOA->BSRR, (uint32_t)&csLowWord,  n); // UPDATE -> CS low
  dmaStart(DMA2_Stream1, CR_DATA_SPI,   (uint32_t)&SPI1->DR,    (uint32_t)txBuf,       n); // CC1    -> SPI data
  dmaStart(DMA2_Stream4, CR_BSRR_CONST, (uint32_t)&GPIOA->BSRR, (uint32_t)&csHighWord, n); // CC4    -> CS high

#if LED_ENABLED
  GPIOC->BSRR = (1u << (13 + 16));      // LED on
#endif
  TIM1->DIER = TIM_DIER_UDE | TIM_DIER_CC1DE | TIM_DIER_CC4DE;
  TIM1->CR1  = TIM_CR1_CEN;             // hardware now plays the whole pulse

  uint32_t t0 = millis(), tmax = (uint32_t)(n * 1000.0f / gFs) + 10;
  bool ok = true;
  while (!(DMA2->HISR & DMA_HISR_TCIF4)) {          // CS-high stream finishes last
    if (millis() - t0 > tmax) { ok = false; break; }
    __WFI();                                        // CPU sleeps; DMA keeps running
  }
  TIM1->CR1  = 0;
  TIM1->DIER = 0;
  dmaStop(DMA2_Stream1); dmaStop(DMA2_Stream4); dmaStop(DMA2_Stream5);
  dmaClearAll();
  while (SPI1->SR & SPI_SR_BSY) {}
  GPIOA->BSRR = csHighWord;
#if LED_ENABLED
  GPIOC->BSRR = (1u << 13);             // LED off
#endif
  return ok;
}

// ============================ UI (KEY button) ==============================
static void flash(uint8_t n) {
#if LED_ENABLED
  for (uint8_t i = 0; i < n; i++) {
    GPIOC->BSRR = (1u << (13 + 16)); delay(120);
    GPIOC->BSRR = (1u << 13);        delay(180);
  }
#endif
}

static void pollButton() {
  static bool     down = false;
  static uint32_t tDown = 0;
  bool pressed = !(GPIOA->IDR & (1u << 0));
  uint32_t now = millis();
  if (pressed && !down) { down = true; tDown = now; }
  else if (!pressed && down) {
    down = false;
    uint32_t held = now - tDown;
    if (held > 30 && held < 800) { gMode = (gMode + 1) % MOD_COUNT; flash(gMode + 1); }
    else if (held >= 800)        { gWin  = (gWin  + 1) % WIN_COUNT; flash(gWin + 1);  }
  }
}

// ============================ ARDUINO ENTRY POINTS =========================
static uint32_t nextPing;

void setup() {
#if DEBUG_SERIAL
  Serial.begin(115200);
#endif
  initTables();
  hwInit();
  flash(1);
  nextPing = millis() + 200;
}

void loop() {
#if TEST_CW
  // Continuous 10 kHz tone. At 200 kS/s, 1600 samples = 8 ms = exactly 80 cycles,
  // so the tone repeats without a phase jump. Use it to check the DAC and filter.
  Params p;
  p.f0 = p.f1 = p.fc = 10000.0f;
  p.bw     = 0.0f;
  p.amp    = 0.8f;
  p.nPulse = 1600;
  p.durMs  = 8.0f;
  gMode  = MOD_LFM;                    // LFM with f0 = f1 is a plain tone
  gNoWin = true;
  uint32_t n = buildWaveform(p);
  transmit(n);
#else
  pollButton();
  uint32_t now = millis();
  if ((int32_t)(now - nextPing) >= 0) {
    nextPing += PRI_MS;
    if ((int32_t)(now - nextPing) >= 0) nextPing = now + PRI_MS;   // resync if late

    float x[3];
    readPots(x);                         // ADC + DMA
    Params p = mapParams(x);             // adaptation logic
    gNoWin = false;
    uint32_t n = buildWaveform(p);       // synthesise next pulse (LUT, windowed)
    bool ok = transmit(n);               // timer + DMA plays it out
    (void)ok;
#if DEBUG_SERIAL
    Serial.print(gMode); Serial.print(' '); Serial.print(gWin);
    Serial.print(" f0="); Serial.print(p.f0); Serial.print(" f1="); Serial.print(p.f1);
    Serial.print(" T="); Serial.print(p.durMs); Serial.print("ms A="); Serial.print(p.amp);
    Serial.println(ok ? " OK" : " DMA-TIMEOUT");
#endif
  } else {
    __WFI();                             // sleep until next SysTick (1 ms)
  }
#endif
}
