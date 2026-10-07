#include "daisy_seed.h"
#include "daisysp.h"

using daisy::DaisySeed;

using daisy::AnalogControl;
using daisy::AudioHandle;
using daisy::Led;
using daisy::Parameter;
using daisy::SaiHandle;
using daisy::Switch;
using daisy::System;
using daisysp::DelayLine;
using daisysp::fonepole;
using daisysp::Limiter;
using daisysp::OnePole;
using daisysp::Oscillator;
using daisysp::WhiteNoise;

// ============================================================================
// CONFIGURATION CONSTANTS
// ============================================================================

// Delay timing parameters
#define GOLDEN_RATIO (1.6180339f) // Used for delay time ratio between channels
#define MIN_DELAY_TIME (0.8f)     // Minimum delay time in seconds
#define MAX_DELAY_TIME (30.0f)    // Maximum delay time in seconds
#define DELAY_BUFFER_MULTIPLIER                                               \
  (1.2f) // Buffer size multiplier for LFO modulation headroom
#define MAX_DELAY_LINE_TIME                                                   \
  ((MAX_DELAY_TIME) * GOLDEN_RATIO                                            \
   * DELAY_BUFFER_MULTIPLIER) // Maximum delay line duration in seconds
#define MAX_DELAY                                                             \
  static_cast<size_t> (                                                       \
      48000 * (MAX_DELAY_LINE_TIME)) // Maximum delay line samples at 48kHz
#define N_DELAYS (2)                 // Number of delay lines

// Delay processing parameters
#define DELAY_SMOOTHING_COEFF                                                 \
  (0.00002f) // Low-pass filter coefficient for delay time smoothing
#define TAPE_HPF_MULTIPLIER                                                   \
  (0.125f) // Tape HPF cutoff frequency multiplier based on delay time
#define INITIAL_FEEDBACK (0.9f) // Initial feedback coefficient for delay lines
#define FEEDBACK_MULTIPLIER                                                   \
  (1.2f) // Feedback gain multiplier to offset level loss

// Filter frequency parameters
#define FEEDBACK_LPF_FREQ                                                     \
  (8000.0f) // Low-pass filter frequency for feedback (Hz)
#define FEEDBACK_HPF_FREQ                                                     \
  (60.0f) // High-pass filter frequency for feedback (Hz)
#define TAPE_HPF_CENTER_FREQ                                                  \
  (12000.0f) // Center frequency for tape HPF filter (Hz)

// LFO parameters

#define LFO_LFO_BASE_FREQ (0.0005f)      // Base LFO frequency (Hz)
#define LFO_LFO_FREQ_INCREMENT (0.0003f) // Per-delay LFO frequency increment (Hz)
#define LFO_LFO_BASE_AMP (0.002f)         // Base LFO amplitude multiplier
#define LFO_LFO_AMP_INCREMENT (0.002f)   // Per-delay LFO amplitude increment


#define LFO_BASE_FREQ (0.005f)      // Base LFO frequency (Hz)
#define LFO_FREQ_INCREMENT (0.003f) // Per-delay LFO frequency increment (Hz)
#define LFO_BASE_AMP (0.10f)         // Base LFO amplitude multiplier
#define LFO_AMP_INCREMENT (0.1f)   // Per-delay LFO amplitude increment
#define TAPE_LFO_AMP (4000.0f)      // Tape HPF LFO amplitude (Hz)
#define TAPE_LFO_FREQ (2.0f)        // Tape HPF LFO frequency (Hz)

// Effect parameters
#define NOISE_AMP (0.00005f) // White noise amplitude for tape effect
#define INPUT_SCALE_SMOOTHING                                                 \
  (0.002f) // Smoothing coefficient for input level fading
#define MIX_OUTPUT_SCALE (0.66f) // Output gain scaling for delay mix

// UI and timing parameters
#define DELAY_TIME_BUCKETS (128.0f) // Number of delay time resolution buckets
#define AUDIO_BLOCK_SIZE (4)        // Audio samples per callback

// Knob slew time in *seconds*. libDaisy turns this into a one-pole coefficient
// internally: coeff = 1 / (slew_seconds * update_rate * 0.5). Each call to
// AnalogControl::Process() advances that filter by one step.
#define KNOB_SMOOTHING (0.05f) // Knob slew time in seconds

// Erase state machine parameters
#define ERASE_STATE_WAIT 0 // Erase button state: waiting for first press
#define ERASE_STATE_WAIT_RELEASE 1 // Erase button state: waiting for release
#define ERASE_STATE_WAIT_PRESS_AGAIN                                          \
  2                             // Erase button state: waiting for second press
#define ERASE_TIMEOUT_MS (800) // Timeout in milliseconds to reset erase state

// ============================================================================
// BOARD SELECT
// ============================================================================
//
// This firmware runs on two physically different boards:
//
//   - the production PCB - the default, built by a plain `make`, and
//   - the older hand-wired prototype board (the one nosmos-fez.cpp ran on),
//     built with `make BOARD=proto`.
//
// The Daisy Seed pins used for the three pots and two footswitches are the same
// on both boards, but three things genuinely differ: the LED pin, the direction
// the pots are wired, and whether the footswitches are normally-open (PCB) or
// normally-closed (prototype). This block
// isolates those three differences so the rest of the file never has to care
// which board it is running on.
//
// Educational note on the structure: the production values are written as the plain,
// unconditional defaults, and the prototype's differences are applied on top by
// #undef-ing and redefining them. That is deliberate. An #ifdef/#else pair
// forces you to keep both boards' definitions mentally in sync and makes it
// easy to add a constant to one arm and forget the other; this "default plus
// override" shape means the common case reads as straight-line code, and the
// override block below is an explicit, self-documenting list of exactly what is
// unusual about the prototype. Nothing can be defined for one board and
// silently missing for the other.
//
// BOARD_PROTO is supplied by the compiler (-DBOARD_PROTO from the Makefile),
// not defined here, so switching boards never means editing this file.

// --- Production PCB defaults ------------------------------------------------

#define LED_RECORD_PIN (2) // LED indicator pin

// The PCB wires the pots' outer legs such that the wiper voltage *falls* as you
// turn clockwise, so we ask AnalogControl to compute (1.0 - input) for us.
// That is all libDaisy's `flip` argument does.
#define KNOB_FLIP true

// Switch sense. Both boards wire one lug of each footswitch to ground and use
// the Seed's internal pull-up, so the pin reads HIGH while the contact is open
// and LOW while it is closed. Both also configure POLARITY_INVERTED, which
// makes libDaisy's Pressed() return true for a LOW pin - i.e. Pressed() means
// "contact closed" on either board.
//
// What differs is the *switch type*, not the wiring: the PCB uses normally-open
// footswitches, so closed == foot down, and Pressed() already means what we
// want with no extra inversion.
#define SW_ACTIVE(sw) ((sw).Pressed ())

// Edge helpers, used by the erase double-tap state machine. These describe the
// *physical* action - foot down, foot up - rather than which way the voltage
// moved, which is what lets that state machine be board-agnostic.
#define SW_ENGAGED_EDGE(sw) ((sw).RisingEdge ())
#define SW_RELEASED_EDGE(sw) ((sw).FallingEdge ())

// --- Hand-wired prototype overrides -----------------------------------------
#ifdef BOARD_PROTO

// The prototype's LED is on D1 rather than D2.
#undef LED_RECORD_PIN
#define LED_RECORD_PIN (1)

// The prototype's pots are wired the other way round to the PCB: wiper voltage
// rises as you turn clockwise, which is what AnalogControl expects by default.
#undef KNOB_FLIP
#define KNOB_FLIP false

// The prototype's footswitches are normally-*closed*: at rest the contact is
// made, so the pin sits LOW and Pressed() reads true with nothing underfoot,
// and pressing opens the contact. Engaged is therefore !Pressed() - the same
// double-inversion the older nosmos-fez.cpp did with `!SWITCHES[n].Pressed()`.
// (This is not a wiring difference - both boards ground one lug - so the
// schematic will not tell you which board is which; the resting continuity of
// the switch will.)
#undef SW_ACTIVE
#define SW_ACTIVE(sw) (!(sw).Pressed ())

// Because the sense is flipped, the edge helpers swap too: libDaisy's
// RisingEdge() fires when the pin transitions to *its* idea of pressed (LOW),
// which on a normally-closed switch is the foot coming *off* the switch.
#undef SW_ENGAGED_EDGE
#undef SW_RELEASED_EDGE
#define SW_ENGAGED_EDGE(sw) ((sw).FallingEdge ())
#define SW_RELEASED_EDGE(sw) ((sw).RisingEdge ())

#endif // BOARD_PROTO

// Hardware pin assignments (identical on both boards)
#define KNOB_FB_PIN daisy::seed::D15 // Feedback knob ADC pin (daisy pin 22)
#define KNOB_DELAY_TIME_PIN                                                   \
  daisy::seed::D16 // Delay time knob ADC pin (daisy pin 23)
#define KNOB_XFB_PIN                                                          \
  daisy::seed::D17 // Cross-feedback knob ADC pin (daisy pin 24)
#define SWITCH_RECORDING_PIN daisy::seed::D27 // Recording switch pin
#define SWITCH_ERASE_PIN daisy::seed::D26     // Erase switch pin

// Control counts. These exist so the array declarations and the loops that
// walk them can never drift apart: previously the debounce loop counted to 3
// while the switch array only held 2 entries, so every audio callback called
// Debounce() on memory one slot past the end of the array.
#define N_KNOBS (3)
#define N_SWITCHES (2)

// Named indices into KNOBS[] and SWITCHES[], so the reads below say what they
// mean rather than relying on you remembering that knob 2 is cross-feedback.
#define KNOB_FB (0)
#define KNOB_DELAY_TIME (1)
#define KNOB_XFB (2)
#define SWITCH_RECORDING (0)
#define SWITCH_ERASE (1)

// ============================================================================

DaisySeed hw;

DelayLine<float, MAX_DELAY> DSY_SDRAM_BSS DELAY_LINES[N_DELAYS];

struct delay_s
{
  DelayLine<float, MAX_DELAY> *del;
  float currentDelay;
  float delayTarget;
  float feedback;
  float lastVal;
  float samplerate;
  float lfo_freq;

  WhiteNoise noise;
  OnePole fb_lpf, fb_hpf;
  OnePole tape_hpf;

  Oscillator lfo_lfo;
  Oscillator lfo;
  Oscillator tape_hpf_lfo;
  float tape_hpf_cf;

  void
  Init (DelayLine<float, MAX_DELAY> *line, float sr, int index)
  {
    del = line;
    del->Init ();

    currentDelay = sr * 1.0f;
    delayTarget = sr * 1.0f;
    feedback = INITIAL_FEEDBACK;
    lastVal = 0;
    samplerate = sr;
    tape_hpf_cf = TAPE_HPF_CENTER_FREQ;

    // init noise
    noise.Init ();
    noise.SetAmp (NOISE_AMP);

    // init feedback filters
    // LPF filter is 8000hz which to my ear is where tape starts to drop
    fb_lpf.Init ();
    fb_lpf.SetFilterMode (OnePole::FILTER_MODE_LOW_PASS);
    fb_lpf.SetFrequency (FEEDBACK_LPF_FREQ / sr);

    // HPF filter at 60hz to make sure the resulting audio mash doesnt
    // get super muddy in the sub frequencies.
    fb_hpf.Init ();
    fb_hpf.SetFilterMode (OnePole::FILTER_MODE_HIGH_PASS);
    fb_hpf.SetFrequency (FEEDBACK_HPF_FREQ / sr);

    // init tape HPF
    tape_hpf.Init ();
    tape_hpf.SetFilterMode (OnePole::FILTER_MODE_HIGH_PASS);

    // LFO of LFOs: a very slow oscillator whose output is added to the delay
    // LFO's frequency below, so the wobble itself drifts in speed rather than
    // staying metronomic. `index` staggers each delay line so the two never
    // line up and phase-lock into an audible pattern.
    lfo_lfo.Init (sr);
    lfo_lfo.SetFreq (LFO_LFO_BASE_FREQ + (LFO_LFO_FREQ_INCREMENT * index));
    lfo_lfo.SetAmp (LFO_LFO_BASE_AMP + (LFO_LFO_AMP_INCREMENT * index));

    // init delay time LFO
    lfo.Init (sr);
    lfo_freq = LFO_BASE_FREQ + (LFO_FREQ_INCREMENT * index);
    lfo.SetFreq (lfo_freq);
    // lfo is 100ms max (plus some variation from the samplerate,
    // we'll scale these for each delay time in the delay loop
    lfo.SetAmp ((LFO_BASE_AMP + (LFO_AMP_INCREMENT * index)) * sr);

    // init tape HPF LFO
    tape_hpf_lfo.Init (sr);
    tape_hpf_lfo.SetAmp (TAPE_LFO_AMP);
    tape_hpf_lfo.SetFreq (TAPE_LFO_FREQ);
  }

  float
  Process (float in)
  {
    fonepole (currentDelay, delayTarget, DELAY_SMOOTHING_COEFF);

    float delay_fraction = currentDelay / (samplerate * MAX_DELAY_TIME);
    lfo.SetFreq(lfo_freq + lfo_lfo.Process ());
    float lfo_val = lfo.Process ();
    del->SetDelay (currentDelay + (lfo_val * delay_fraction));
    float read = del->Read ();

    // add some noise, use delay time to move the LFO
    // tape HPF cutoff around
    tape_hpf_lfo.SetFreq (1.0f / ((1.0f+currentDelay) * TAPE_HPF_MULTIPLIER));
    tape_hpf.SetFrequency ((tape_hpf_cf + tape_hpf_lfo.Process ())
                           / samplerate);
    read = read + tape_hpf.Process (noise.Process ());

    // apply the filters
    read = fb_lpf.Process (fb_hpf.Process (read));

    // write back with softclip
    del->Write (daisysp::SoftClip ((feedback * read) + in));
    lastVal = read;
    return read;
  }
};

// LEDs
Led led_record;

bool RECORDING = false;
bool ERASING = false;

float CROSS_FEEDBACK = 0.0f;
float FEEDBACK = 0.0f;

delay_s DELAYS[N_DELAYS];

// CONTROL HARDWARE
AnalogControl KNOBS[N_KNOBS];
Switch SWITCHES[N_SWITCHES];

float DELAY_1_DEST_VAL = 0;
float DELAY_2_DEST_VAL = 0;

void
InitDelays (float samplerate)
{
  for (int i = 0; i < N_DELAYS; i++)
    {
      DELAYS[i].Init (&DELAY_LINES[i], samplerate, i);
    }
}

void
ProcessAllControls ()
{
  for (int i = 0; i < N_KNOBS; i++)
    {
      KNOBS[i].Process ();
    }

  for (int i = 0; i < N_SWITCHES; i++)
    {
      SWITCHES[i].Debounce ();
    }
}

float INPUT_SCALE = 0;
float CURRENT_INPUT_SCALE = 0;
float DELAY_TIME_KNOB = 0;
uint32_t FRAME_COUNTER = 0;

uint32_t ERASE_STATE = ERASE_STATE_WAIT;
uint32_t ERASE_PRESSED_WHEN = 0;

Limiter limiter;


void
AudioCallback (AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out,
               size_t size)
{
  ProcessAllControls ();

  // SW_ACTIVE() hides the per-board electrical sense (see BOARD SELECT above).
  RECORDING = SW_ACTIVE (SWITCHES[SWITCH_RECORDING]);
  ERASING = SW_ACTIVE (SWITCHES[SWITCH_ERASE]);

  INPUT_SCALE = RECORDING ? 1.0f : 0.0f;

  FEEDBACK = KNOBS[KNOB_FB].Value () * FEEDBACK_MULTIPLIER;
  DELAY_TIME_KNOB = KNOBS[KNOB_DELAY_TIME].Value ();
  CROSS_FEEDBACK = KNOBS[KNOB_XFB].Value () * FEEDBACK_MULTIPLIER;

  // Double tap erase statemachine stuff.
  uint32_t now = System::GetNow ();
  uint32_t tdiff = now - ERASE_PRESSED_WHEN;

  // timeout after 2 seconds
  if (ERASE_STATE > ERASE_STATE_WAIT && tdiff > ERASE_TIMEOUT_MS)
    {
      ERASE_STATE = ERASE_STATE_WAIT;
    }

  // SW_ENGAGED_EDGE / SW_RELEASED_EDGE describe the *physical* action (foot
  // down / foot up) regardless of which way the voltage moves on this board.
  if (SW_ENGAGED_EDGE (SWITCHES[SWITCH_ERASE]))
    {
      if (ERASE_STATE == ERASE_STATE_WAIT)
        {
          ERASE_PRESSED_WHEN = now;
          ERASE_STATE = ERASE_STATE_WAIT_RELEASE;
        }

      if (ERASE_STATE == ERASE_STATE_WAIT_PRESS_AGAIN)
        {
          DELAYS[0].del->Reset ();
          DELAYS[1].del->Reset ();

          // disable feedbacks for this cycle, this prevents a "click"
          // from entering the loop
          ERASING = true;
          ERASE_STATE = ERASE_STATE_WAIT;
        }
    }

  // advance state if the initial press was released.
  if (SW_RELEASED_EDGE (SWITCHES[SWITCH_ERASE]))
    {
      if (ERASE_STATE == ERASE_STATE_WAIT_RELEASE)
        {
          ERASE_STATE = ERASE_STATE_WAIT_PRESS_AGAIN;
        }
    }

  // bit ol' hack, if the erasing button is held down, set both
  // feedbacks to 0, it works.
  if (ERASING)
    {
      CROSS_FEEDBACK = 0;
      FEEDBACK = 0;
    }

  DELAYS[0].feedback = DELAYS[1].feedback = FEEDBACK;

  for (size_t i = 0; i < size; ++i)
    {
      float mix = 0;

      // fade in the input, so new audio comes in smooth and not with a big loud
      // click.. i'm looking at you, other loopers.
      fonepole (CURRENT_INPUT_SCALE, INPUT_SCALE, INPUT_SCALE_SMOOTHING);

      float in_samp = in[0][i] * CURRENT_INPUT_SCALE;

      // manually unrolled the mixing loop. each delay gets the dry input plus a
      // scaled tap of the *other* delay's most recent output. i suspect the
      // compiler would have done this anyway.
      float sig
          = DELAYS[0].Process (in_samp + (CROSS_FEEDBACK * DELAYS[1].lastVal));
      mix += sig;
      sig = DELAYS[1].Process (in_samp + (CROSS_FEEDBACK * DELAYS[0].lastVal));
      mix += sig;

      mix = mix * MIX_OUTPUT_SCALE;
      mix = mix + in[0][i];

      // Limit in place on the single mono sample. This updates the limiter's
      // envelope once per sample, and both channels get the same limited value.
      limiter.ProcessBlock (&mix, 1, 1.0f);
      out[0][i] = out[1][i] = mix;
    }
}

int
main ()
{
  hw.Configure ();
  hw.Init ();
  hw.SetAudioBlockSize (AUDIO_BLOCK_SIZE);
  hw.SetAudioSampleRate (SaiHandle::Config::SampleRate::SAI_48KHZ);


  led_record.Init (hw.GetPin (LED_RECORD_PIN), false);

  // KNOBS
  // The ADC channel order here defines the KNOBS[] index order, which is why
  // the KNOB_* index constants must match the order of these three lines.
  daisy::AdcChannelConfig cfg[N_KNOBS];
  cfg[KNOB_FB].InitSingle (KNOB_FB_PIN);
  cfg[KNOB_DELAY_TIME].InitSingle (KNOB_DELAY_TIME_PIN);
  cfg[KNOB_XFB].InitSingle (KNOB_XFB_PIN);
  hw.adc.Init (cfg, N_KNOBS);

  for (int i = 0; i < N_KNOBS; i++)
    {
      // KNOB_FLIP is the per-board pot direction - see BOARD SELECT at the top.
      KNOBS[i].Init (hw.adc.GetPtr (i), hw.AudioCallbackRate (), KNOB_FLIP,
                     false, KNOB_SMOOTHING);
    }

  hw.adc.Start ();

  // SWITCHES
  // Both boards use momentary switches wired to ground with the Seed's internal
  // pull-up, so the libDaisy config is the same for both; only the resting
  // state differs (the PCB's are normally-open, the prototype's are
  // normally-closed), and that lives in the SW_ACTIVE / SW_*_EDGE macros.
  SWITCHES[SWITCH_RECORDING].Init (SWITCH_RECORDING_PIN, 0.0f,
                                   daisy::Switch::Type::TYPE_MOMENTARY,
                                   daisy::Switch::Polarity::POLARITY_INVERTED,
                                   daisy::Switch::Pull::PULL_UP);
  SWITCHES[SWITCH_ERASE].Init (SWITCH_ERASE_PIN, 0.0f,
                               daisy::Switch::Type::TYPE_MOMENTARY,
                               daisy::Switch::Polarity::POLARITY_INVERTED,
                               daisy::Switch::Pull::PULL_UP);

  InitDelays (hw.AudioSampleRate ());
  limiter.Init();
  
  hw.StartAudio (AudioCallback);

  unsigned int counter = 0;
  unsigned int flasher = 0x4;

  while (true)
    {
      hw.DelayMs (10);

      if (ERASING)
        {
          flasher = 0x8;
        }

      if (RECORDING)
        {
          flasher = 0x10;
        }

      // LED flash logic, RECORD and ERASE flash at different rates.
      led_record.Set (
          (counter & flasher) > 0 && (RECORDING || ERASING) ? 1.0f : 0.0f);
      led_record.Update ();

      int delay_time_bucket
          = (int)round (DELAY_TIME_KNOB * DELAY_TIME_BUCKETS);


      // this guy is a little tricky to read, but basically buckets
      // the delay time knob into 128 slices.
      float base_delay_time = (MIN_DELAY_TIME
                               + ((MAX_DELAY_TIME - MIN_DELAY_TIME)
                                  * (delay_time_bucket / 128.0f)))
                              * hw.AudioSampleRate ();

      DELAYS[0].delayTarget = base_delay_time;
      // set the delay time based off of the golden ratio, this is the
      // estimated co-primish magic.
      DELAYS[1].delayTarget = base_delay_time * GOLDEN_RATIO;

      counter++;
    }

  return 0;
}
