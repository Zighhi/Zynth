# Zynth

This was my Bachelor's degree project, where I designed a monophonic and semi-modular Digitally Controlled Analog Sound Synthesizer named **The Zynth**.

## Key Components

The synthesizer includes the following key components:

- **Voltage-Controlled Oscillators (VCOs):** Generates sine, triangle, sawtooth, and square waveforms.
- **Voltage-Controlled Filter (VCF):** Low-pass filter with adjustable cutoff frequency and resonance.
- **Voltage-Controlled Amplifier (VCA):** Modulates the amplitude of the synthesized signal.
- **Mixer:** Combines signals from the oscillator.
- **Amplitude Modulators:** Configurable ADSR envelope generator and LFO for modulation.
- **MIDI Interface:** Allows real-time musical note control.
- **Display:** Visualizes set parameters.

## Digital Control

The digital control is implemented using a Raspberry Pi Pico microcontroller, which manages the MIDI interface, ADSR envelope generator, LFO, other control signals, and the user interface.

## Features

- Real-time control via MIDI.
- Configurable waveform generation.
- Dynamic amplitude modulation.
- Adjustable filter parameters.
