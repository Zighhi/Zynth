#include <MIDI.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <Wire.h>
#include <adsr.h>
#include <lfo.h>
#include <U8g2lib.h>

//global Defines
#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels
#define DACSIZE 4096      // Vertical resolution of the DACs

//Midi Serial
using Transport = MIDI_NAMESPACE::SerialMIDI<SoftwareSerial>;
int rxPin = 1;
int txPin = 0;
SoftwareSerial mySerial = SoftwareSerial(rxPin, txPin);
Transport serialMIDI(mySerial);
MIDI_NAMESPACE::MidiInterface<Transport> MIDI((Transport &)serialMIDI);

//MCP4822 registers
typedef unsigned int mcp4xxx_register;
const mcp4xxx_register mcp4822_channel_a = 0x5000;
const mcp4xxx_register mcp4822_channel_b = 0XD000;

//pins
const byte ENV_dac_cs = 20;    //ENV
const byte LFO_dac_cs = 21;    //LFO
const byte midi_dac1_cs = 22;  //NoteNumber
const byte TRIG = 26;
const byte ENV2_gate = 28;
const byte LFO2_sync = 27;
const byte ENC_A = 2;
const byte ENC_B = 3;
const byte ENC_btn = 6;

//Demultiplexor adress pins
const byte A_0 = 14;
const byte B_0 = 13;
const byte C_0 = 12;
const byte A_1 = 17;
const byte B_1 = 16;
const byte C_1 = 15;

//buttons
const byte btn0 = 7;
const byte btn1 = 8;
const byte btn2 = 9;
const byte btn3 = 10;
const byte btn4 = 11;

//Encoder Variables
unsigned long _lastIncReadTime = micros();
unsigned long _lastDecReadTime = micros();
int _pauseLength = 25000;
int _fastIncrement = 1000;

//Menu variables
int submenuSelection = 0;     // Variable to store the current submenu selection
unsigned long startTime = 0;  // Initialize variable
bool displayMenu = false;     // Flag to indicate when to display the menu
bool enterLFOMenu = false;    // Flag to indicate when to enter the LFO menu
bool enterENVMenu = false;    // Flag to indicate when to enter the ENV menu
bool enterMODMenu = false;    // Flag to indicate when to enter the MOD menu
int lastDisplayedMenu = 0;    // Variable to store the last displayed menu option
int LFOindex = 1;
int ENVindex = 1;

//MIDI variables
unsigned long startTime_midi = 0;  // Initialize variables
unsigned long elapsedTime = 0;
bool start = false;
int midiClockCount = 0;  // Initialize MIDI clock counter
float bpm = 120;
bool gateOpen = false;
bool noteOnReceived = false;
uint16_t velocity = 0;  //
float velocityPercent = 1;
uint16_t noteNumber = 0;

// adsr variables
unsigned long env1_attack = 100000;   // time in µs
unsigned long env1_decay = 100000;    // time in µs
int env1_sustain = DACSIZE / 2;       // sustain level -> from 0 to 4096-1
unsigned long env1_release = 100000;  // time in µs
float env1_amp = 1;

unsigned long env2_attack = 100000;   // time in µs
unsigned long env2_decay = 100000;    // time in µs
int env2_sustain = DACSIZE / 2;       // sustain level -> from 0 to 4096-1
unsigned long env2_release = 100000;  // time in µs
float env2_amp = 1;


//lfo variables
String wvfrms[5] = { "Off", "Saw", "Triangle", "Sin", "Square" };  // 0 -> off, 1 -> saw, 2 -> triangle, 3 -> sin, 4 -> square [0,4]
bool lfo1Mode = false;                                             //free running
bool lfo2Mode = false;
float lfo_mode1_rate[19] = { 0.125, 0.25, 0.5, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16 };
String lfo_mode1_rateStrings[19] = { "2", "1", "1/2", "1/4", "1/8", "1/12", "1/16", "1/20", "1/24", "1/28", "1/32", "1/36", "1/40", "1/44", "1/48", "1/52", "1/56", "1/60", "1/64" };
int lfo1_waveformIndex = 1;
int lfo2_waveformIndex = 1;
int lfo1_amp = DACSIZE - 1;
int lfo2_amp = DACSIZE - 1;
float lfo1_freq = 10;
float lfo2_freq = 10;
int lfo1_rateIndex = 0;
float lfo1_rate = lfo_mode1_rate[lfo1_rateIndex];
int lfo2_rateIndex = 0;
float lfo2_rate = lfo_mode1_rate[lfo2_rateIndex];

//MOD menu variables
String lfo1_paths[8] = { "None", "Pitch", "PWM", "SYNC", "Cutoff", "Resonance", "Amp", "ENV2 Trigger" };
int lfo1_pathsIndex = 0;
String lfo2_paths[6] = { "None", "Pitch", "SYNC", "Cutoff", "Amp", "ENV2 Trigger" };
int lfo2_pathsIndex = 0;
String env2_paths[5] = { "None", "Pitch", "Amp", "Cutoff", "Resonance" };
int env2_pathsIndex = 0;
String trigger_paths[5] = { "None", "SYNC", "LFO1 SYNC", "LFO2 SYNC", "ENV2 Trigger" };
int trigger_pathsIndex = 0;
String velocity_paths[3] = { "None", "Cutoff", "Pitch" };
int velocity_pathsIndex = 1;
bool trigger2SYNC = false;
bool trigger2LFO1 = false;
bool trigger2LFO2 = false;
bool trigger2ENV2 = false;
bool lfo12ENV2 = false;
bool lfo22ENV2 = false;
bool velocity2Pitch = false;
bool velocity2Cutoff = false;

adsr env1(DACSIZE);  // ADSR class initialization
adsr env2(DACSIZE);
lfo lfo1(DACSIZE);  // LFO class initialization
lfo lfo2(DACSIZE);
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

//Logo bitmap
static const uint8_t PROGMEM logo[] = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0xf8, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x07, 0x00, 0x00, 0x00, 0x00, 0x38, 0x00, 0x00,
  0x00, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x07, 0x00, 0x00, 0x00, 0x0e, 0x38, 0x00, 0x00,
  0x00, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x07, 0x00, 0x00, 0x00, 0x0e, 0x38, 0x00, 0x00,
  0x00, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x07, 0x00, 0x00, 0xe0, 0xff, 0x38, 0x00, 0x00,
  0x00, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x07, 0x00, 0x00, 0xe0, 0x7f, 0x38, 0x00, 0x00,
  0x00, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x07, 0x00, 0x00, 0xe0, 0xff, 0x38, 0x00, 0x00,
  0x00, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0x07, 0x00, 0x00, 0x00, 0x0e, 0x38, 0x00, 0x00,
  0x00, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x07, 0x00, 0x00, 0x00, 0x0e, 0x38, 0x00, 0x00,
  0x00, 0x0c, 0x00, 0x00, 0x78, 0x00, 0x00, 0x06, 0x07, 0x00, 0x00, 0x00, 0x0e, 0x38, 0x00, 0x00,
  0x00, 0x0c, 0x00, 0x00, 0x78, 0x00, 0x00, 0x03, 0x07, 0x00, 0x00, 0x00, 0x0e, 0x38, 0x00, 0x00,
  0x00, 0x0c, 0x00, 0x00, 0x7c, 0x00, 0xc0, 0x11, 0x07, 0x00, 0x00, 0x00, 0x0e, 0x38, 0x00, 0x00,
  0x00, 0x0c, 0x00, 0x00, 0x7e, 0x00, 0xe0, 0x3c, 0x07, 0x00, 0xf8, 0x01, 0x0e, 0xf8, 0x0f, 0x00,
  0x00, 0x0c, 0x00, 0x00, 0x7f, 0x00, 0xe0, 0x7c, 0xe7, 0xc0, 0xf9, 0x77, 0x0e, 0xf8, 0x1f, 0x00,
  0x00, 0x0c, 0x00, 0x80, 0x73, 0x00, 0x70, 0x7c, 0xe7, 0xc0, 0xf9, 0x7f, 0x0e, 0xf8, 0x3e, 0x00,
  0x00, 0x0c, 0x00, 0xc0, 0x61, 0x00, 0x38, 0x38, 0xe7, 0xc0, 0x39, 0x7e, 0x0e, 0x78, 0x38, 0x00,
  0x00, 0x0c, 0x00, 0xe0, 0x60, 0x00, 0x7c, 0x00, 0xe7, 0xc0, 0x39, 0x7c, 0x0e, 0x38, 0x30, 0x00,
  0x00, 0x1c, 0x00, 0x70, 0x60, 0x00, 0x7e, 0x00, 0xe7, 0xc0, 0x39, 0x78, 0x0e, 0x38, 0x70, 0x00,
  0x00, 0xfc, 0xff, 0x3f, 0x60, 0x00, 0xff, 0xff, 0xe7, 0xc0, 0x39, 0x78, 0x0e, 0x38, 0x70, 0x00,
  0x00, 0xfc, 0xff, 0x1f, 0x60, 0x80, 0xff, 0xff, 0xe7, 0xc0, 0x39, 0x70, 0x0e, 0x38, 0x70, 0x00,
  0x00, 0x1c, 0xc0, 0x0f, 0x60, 0xc0, 0x01, 0x00, 0xe7, 0xc0, 0x39, 0x70, 0x0e, 0x38, 0x70, 0x00,
  0x00, 0x0c, 0xc0, 0x07, 0x60, 0xe0, 0x00, 0x00, 0xe7, 0xc0, 0x39, 0x70, 0x0e, 0x38, 0x70, 0x00,
  0x00, 0xcc, 0xc3, 0x03, 0xe0, 0x70, 0x00, 0x00, 0xc7, 0xe1, 0x39, 0x70, 0x0e, 0x38, 0x70, 0x00,
  0x00, 0xcc, 0xc3, 0x01, 0xe0, 0x38, 0x00, 0x00, 0xc7, 0xff, 0x39, 0x70, 0x3c, 0x38, 0x70, 0x00,
  0x00, 0xcc, 0xe7, 0x00, 0xe0, 0x1f, 0x00, 0x00, 0x87, 0xff, 0x39, 0x70, 0xf8, 0x38, 0x30, 0x00,
  0x00, 0xcc, 0x73, 0x00, 0xe0, 0x0f, 0x00, 0x00, 0x07, 0xff, 0x01, 0x50, 0xf0, 0x00, 0x00, 0x00,
  0x00, 0x0c, 0x39, 0x00, 0xe0, 0x07, 0x00, 0x00, 0x07, 0xc0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x0c, 0x1c, 0x00, 0xe0, 0x03, 0x00, 0x00, 0x07, 0xc0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x0c, 0x0e, 0x00, 0xc0, 0x01, 0x00, 0x00, 0x07, 0xc0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x0c, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xc0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0xfc, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xc0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0xfc, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xc0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0xfc, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xc0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0xfc, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xc0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0xfc, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xe0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x07, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x07, 0x7c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0xf8, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x03, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

void setup() {
  pinMode(ENV_dac_cs, OUTPUT);
  digitalWrite(ENV_dac_cs, HIGH);
  pinMode(LFO_dac_cs, OUTPUT);
  digitalWrite(LFO_dac_cs, HIGH);
  pinMode(midi_dac1_cs, OUTPUT);
  digitalWrite(midi_dac1_cs, HIGH);

  pinMode(A_0, OUTPUT);
  digitalWrite(A_0, LOW);
  pinMode(B_0, OUTPUT);
  digitalWrite(B_0, LOW);
  pinMode(C_0, OUTPUT);
  digitalWrite(C_0, LOW);
  pinMode(A_1, OUTPUT);
  digitalWrite(A_1, LOW);
  pinMode(B_1, OUTPUT);
  digitalWrite(B_1, LOW);
  pinMode(C_1, OUTPUT);
  digitalWrite(C_1, LOW);


  pinMode(TRIG, OUTPUT);
  digitalWrite(TRIG, HIGH);

  pinMode(btn0, INPUT_PULLUP);
  pinMode(btn1, INPUT_PULLUP);
  pinMode(btn2, INPUT_PULLUP);
  pinMode(btn3, INPUT_PULLUP);
  pinMode(btn4, INPUT_PULLUP);

  pinMode(ENV2_gate, INPUT_PULLDOWN);
  pinMode(LFO2_sync, INPUT_PULLDOWN);

  // Set encoder pins and attach interrupts
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  pinMode(ENC_btn, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A), read_encoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), read_encoder, CHANGE);
  //attachInterrupt(digitalPinToInterrupt(ENC_btn), toggleControlMode, FALLING);

  Serial.begin(9600);  // Initialize serial communication for printing
  MIDI.begin(1);

  env1.setAttack(env1_attack);    // initialize attack
  env1.setDecay(env1_decay);      // initialize decay
  env1.setSustain(env1_sustain);  // initialize sustain
  env1.setRelease(env1_release);  // initialize release

  env2.setAttack(env2_attack);    // initialize attack
  env2.setDecay(env2_decay);      // initialize decay
  env2.setSustain(env2_sustain);  // initialize sustain
  env2.setRelease(env2_release);  // initialize release

  lfo1.setWaveForm(1);
  lfo1.setAmpl(DACSIZE - 1);
  lfo1.setAmplOffset(0);
  lfo1.setMode(0);
  lfo1.setMode0Freq(10);

  lfo2.setWaveForm(1);
  lfo2.setAmpl(DACSIZE - 1);
  lfo2.setAmplOffset(0);
  lfo1.setMode(0);
  lfo2.setMode0Freq(10);

  u8g2.begin();
  u8g2.clearBuffer();  // clear the internal memory

  u8g2.firstPage();
  do {
    u8g2.drawXBMP(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, logo);
  } while (u8g2.nextPage());

  u8g2.sendBuffer();     // transfer internal memory to the display
  startTime = millis();  // Record the start time

  SPI.begin();
}

void loop() {
  // Check if it's time to display the menu
  if (millis() - startTime >= 5000 && !displayMenu) {
    displayMenu = true;
    updateDisplay(submenuSelection);  // Display the menu
  }
  // Check for button press to toggle MIDI menu
  if (digitalRead(ENC_btn) == LOW) {
    if (submenuSelection == 1) {
      enterENVMenu = !enterENVMenu;  // Toggle ENV menu flag
      if (enterENVMenu) {
        // Reset the ENV index to 1 when entering the ENV menu
        ENVindex = 1;
      }
      if (!enterENVMenu) {
        // Store the last displayed menu option before exiting the menu
        lastDisplayedMenu = submenuSelection;
      }
      updateDisplay(enterENVMenu ? submenuSelection : lastDisplayedMenu);  // Update the display
      delay(200);                                                          // Add a small delay to debounce the button
    } else if (submenuSelection == 0) {
      enterLFOMenu = !enterLFOMenu;  // Toggle LFO menu flag
      if (enterLFOMenu) {
        // Reset the LFO index to 1 when entering the LFO menu
        LFOindex = 1;
      }
      if (!enterLFOMenu) {
        // Store the last displayed menu option before exiting the menu
        lastDisplayedMenu = submenuSelection;
      }
      updateDisplay(enterLFOMenu ? submenuSelection : lastDisplayedMenu);  // Update the display
      delay(200);                                                          // Add a small delay to debounce the button
    } else if (submenuSelection == 2) {
      enterMODMenu = !enterMODMenu;  // Toggle MOD menu flag
      if (!enterMODMenu) {
        // Store the last displayed menu option before exiting the menu
        lastDisplayedMenu = submenuSelection;
      }
      updateDisplay(enterMODMenu ? submenuSelection : lastDisplayedMenu);  // Update the display
      delay(200);                                                          // Add a small delay to debounce the button
    } else if (enterENVMenu) {
      // If in the ENV menu and the button is pressed again, exit the menu
      enterENVMenu = false;
      updateDisplay(lastDisplayedMenu);  // Update the display to show the last displayed menu option
      delay(200);                        // Add a small delay to debounce the button
    } else if (enterLFOMenu) {
      // If in the LFO menu and the button is pressed again, exit the menu
      enterLFOMenu = false;
      updateDisplay(lastDisplayedMenu);  // Update the display to show the last displayed menu option
      delay(200);                        // Add a small delay to debounce the button
    } else if (enterMODMenu) {
      // If in the MOD menu and the button is pressed again, exit the menu
      enterMODMenu = false;
      updateDisplay(lastDisplayedMenu);  // Update the display to show the last displayed menu option
      delay(200);                        // Add a small delay to debounce the button
    }
  }

  // Add this condition to exit the ENV menu when you press the button again
  if (enterENVMenu && digitalRead(ENC_btn) == LOW) {
    enterENVMenu = false;              // Exit the ENV menu
    updateDisplay(lastDisplayedMenu);  // Update the display to show the last displayed menu option
    delay(200);                        // Add a small delay to debounce the button
  }

  // Add this condition to exit the LFO menu when you press the button again
  if (enterLFOMenu && digitalRead(ENC_btn) == LOW) {
    enterLFOMenu = false;              // Exit the LFO menu
    updateDisplay(lastDisplayedMenu);  // Update the display to show the last displayed menu option
    delay(200);                        // Add a small delay to debounce the button
  }

  // Add this condition to exit the LFO menu when you press the button again
  if (enterMODMenu && digitalRead(ENC_btn) == LOW) {
    enterMODMenu = false;              // Exit the LFO menu
    updateDisplay(lastDisplayedMenu);  // Update the display to show the last displayed menu option
    delay(200);                        // Add a small delay to debounce the button
  }

  setVoltage(ENV_dac_cs, mcp4822_channel_b, env1.getWave(micros()) * env1_amp);

  switch (env2_pathsIndex) {  //"None", "Pitch", "Amp", "Cutoff", "Resonance"
    case 0:
      setVoltage(ENV_dac_cs, mcp4822_channel_a, 0);
      break;
    case 1:
      setVoltage(ENV_dac_cs, mcp4822_channel_a, env2.getWave(micros()) * env2_amp);
      digitalWrite(A_1, LOW);
      digitalWrite(B_1, LOW);
      digitalWrite(C_1, HIGH);
      break;
    case 2:
      setVoltage(ENV_dac_cs, mcp4822_channel_a, env2.getWave(micros()) * env2_amp);
      digitalWrite(A_1, HIGH);
      digitalWrite(B_1, LOW);
      digitalWrite(C_1, HIGH);
      break;
    case 3:
      setVoltage(ENV_dac_cs, mcp4822_channel_a, env2.getWave(micros()) * env2_amp);
      digitalWrite(A_1, LOW);
      digitalWrite(B_1, HIGH);
      digitalWrite(C_1, HIGH);
      break;
    case 4:
      setVoltage(ENV_dac_cs, mcp4822_channel_a, env2.getWave(micros()) * env2_amp);
      digitalWrite(A_1, HIGH);
      digitalWrite(B_1, HIGH);
      digitalWrite(C_1, HIGH);
      break;
    default:
      break;
  }


  lfo1.setWaveForm(lfo1_waveformIndex);
  lfo1.setAmpl(lfo1_amp);
  if (!lfo1Mode) {
    lfo1.setMode(lfo1Mode);
    lfo1.setMode0Freq(lfo1_freq, micros());
  } else {
    lfo1.setMode(lfo1Mode);
    lfo1.setMode1Bpm(bpm);
    lfo1.setMode1Rate(lfo1_rate);
  }

  switch (lfo1_pathsIndex) {  //"None", "Pitch", "PWM", "SYNC", "Cutoff", "Resonance", "Amp", "ENV2 EXT Trigger"
    case 0:
      lfo12ENV2 = false;
      setVoltage(LFO_dac_cs, mcp4822_channel_a, 0);
      break;
    case 1:
      lfo12ENV2 = false;
      setVoltage(LFO_dac_cs, mcp4822_channel_a, lfo1.getWave(micros()));
      digitalWrite(A_0, HIGH);
      digitalWrite(B_0, LOW);
      digitalWrite(C_0, LOW);
      break;
    case 2:
      lfo12ENV2 = false;
      setVoltage(LFO_dac_cs, mcp4822_channel_a, lfo1.getWave(micros()));
      digitalWrite(A_0, LOW);
      digitalWrite(B_0, LOW);
      digitalWrite(C_0, LOW);
      break;
    case 3:
      lfo12ENV2 = false;
      setVoltage(LFO_dac_cs, mcp4822_channel_a, lfo1.getWave(micros()));
      digitalWrite(A_0, LOW);
      digitalWrite(B_0, HIGH);
      digitalWrite(C_0, LOW);
      break;
    case 4:
      lfo12ENV2 = false;
      setVoltage(LFO_dac_cs, mcp4822_channel_a, lfo1.getWave(micros()));
      digitalWrite(A_0, HIGH);
      digitalWrite(B_0, HIGH);
      digitalWrite(C_0, LOW);
      break;
    case 5:
      lfo12ENV2 = false;
      setVoltage(LFO_dac_cs, mcp4822_channel_a, lfo1.getWave(micros()));
      digitalWrite(A_0, LOW);
      digitalWrite(B_0, LOW);
      digitalWrite(C_0, HIGH);
      break;
    case 6:
      lfo12ENV2 = false;
      setVoltage(LFO_dac_cs, mcp4822_channel_a, lfo1.getWave(micros()));
      digitalWrite(A_0, HIGH);
      digitalWrite(B_0, LOW);
      digitalWrite(C_0, HIGH);
      break;
    case 7:
      lfo12ENV2 = true;
      static bool prevEnv2GateState1 = false;
      static bool currentEnv2GateState1 = false;
      if (lfo1.getWave(micros()) > 2047) {
        currentEnv2GateState1 = true;
      } else {
        currentEnv2GateState1 = false;
      }
      if (currentEnv2GateState1 && !prevEnv2GateState1 && trigger2ENV2 == false) {
        // Start the envelope
        env2.noteOn(micros());
      } else if (!currentEnv2GateState1 && prevEnv2GateState1 && trigger2ENV2 == false) {
        // Stop the envelope
        env2.noteOff(micros());
      }
      prevEnv2GateState1 = currentEnv2GateState1;

      break;
    default:
  
      break;
  }

  lfo2.setWaveForm(lfo2_waveformIndex);
  lfo2.setAmpl(lfo2_amp);
  if (!lfo2Mode) {
    lfo2.setMode(lfo2Mode);
    lfo2.setMode0Freq(lfo2_freq, micros());
    if (digitalRead(LFO2_sync) == HIGH && trigger2LFO2 == false) {  //EXT Trigger
      lfo2.sync(micros());
    }
  } else {
    lfo2.setMode(lfo2Mode);
    lfo2.setMode1Bpm(bpm);
    lfo2.setMode1Rate(lfo2_rate);
  }

  switch (lfo2_pathsIndex) {  //"None", "Pitch", "SYNC", "Cutoff", "Amp", "ENV2 EXT Trigger"
    case 0:
      lfo22ENV2 = false;
      setVoltage(LFO_dac_cs, mcp4822_channel_b, 0);
      break;
    case 1:
      lfo22ENV2 = false;
      setVoltage(LFO_dac_cs, mcp4822_channel_b, lfo2.getWave(micros()));
      digitalWrite(A_1, HIGH);
      digitalWrite(B_1, HIGH);
      digitalWrite(C_1, LOW);
      break;
    case 2:
      lfo22ENV2 = false;
      setVoltage(LFO_dac_cs, mcp4822_channel_b, lfo2.getWave(micros()));
      digitalWrite(A_1, LOW);
      digitalWrite(B_1, LOW);
      digitalWrite(C_1, LOW);
      break;
    case 3:
      lfo22ENV2 = false;
      setVoltage(LFO_dac_cs, mcp4822_channel_b, lfo2.getWave(micros()));
      digitalWrite(A_1, HIGH);
      digitalWrite(B_1, LOW);
      digitalWrite(C_1, LOW);
      break;
    case 4:
      lfo22ENV2 = false;
      setVoltage(LFO_dac_cs, mcp4822_channel_b, lfo2.getWave(micros()));
      digitalWrite(A_1, LOW);
      digitalWrite(B_1, HIGH);
      digitalWrite(C_1, LOW);
      break;
    case 5:
      lfo22ENV2 = true;
      static bool prevEnv2GateState2 = false;
      static bool currentEnv2GateState2 = false;
      if (lfo2.getWave(micros()) > 2047) {
        currentEnv2GateState2 = true;
      } else {
        currentEnv2GateState2 = false;
      }
      if (currentEnv2GateState2 && !prevEnv2GateState2 && trigger2ENV2 == false) {
        // Start the envelope
        env2.noteOn(micros());
      } else if (!currentEnv2GateState2 && prevEnv2GateState2 && trigger2ENV2 == false) {
        // Stop the envelope
        env2.noteOff(micros());
      }
      prevEnv2GateState2 = currentEnv2GateState2;

      break;
    default:
      break;
  }

  switch (trigger_pathsIndex) {  //"None", "SYNC", "LFO1 SYNC", "LFO2 SYNC", "ENV2 Trigger"
    case 0:
      trigger2SYNC = false;
      trigger2LFO1 = false;
      trigger2LFO2 = false;
      trigger2ENV2 = false;
      break;
    case 1:
      trigger2SYNC = true;
      trigger2LFO1 = false;
      trigger2LFO2 = false;
      trigger2ENV2 = false;
      break;
    case 2:
      trigger2SYNC = false;
      trigger2LFO1 = true;
      trigger2LFO2 = false;
      trigger2ENV2 = false;
      break;
    case 3:
      trigger2SYNC = false;
      trigger2LFO1 = false;
      trigger2LFO2 = true;
      trigger2ENV2 = false;
      break;
    case 4:
      trigger2SYNC = false;
      trigger2LFO1 = false;
      trigger2LFO2 = false;
      trigger2ENV2 = true;
      break;
    default:
      break;
  }

  switch (velocity_pathsIndex) {  //"None", "Cutoff", "Pitch"
    case 0:
      velocity2Cutoff = false;
      velocity2Pitch = false;
      setVoltage(midi_dac1_cs, mcp4822_channel_a, 1500);
      digitalWrite(A_0, LOW);
      digitalWrite(B_0, HIGH);
      digitalWrite(C_0, HIGH);
      break;
    case 1:
      velocity2Cutoff = true;
      velocity2Pitch = false;
      digitalWrite(A_0, HIGH);
      digitalWrite(B_0, HIGH);
      digitalWrite(C_0, HIGH);
      break;
    case 2:
      velocity2Cutoff = false;
      velocity2Pitch = true;
      digitalWrite(A_0, LOW);
      digitalWrite(B_0, HIGH);
      digitalWrite(C_0, HIGH);
      break;
    default:
      break;
  }



  // Check the state of the ENV2_gate pin, EXT TRIGGER
  static bool prevEnv2GateState = false;
  bool currentEnv2GateState = digitalRead(ENV2_gate);

  // Check for rising edge on ENV2_gate pin
  if (currentEnv2GateState && !prevEnv2GateState && !trigger2ENV2 && !lfo12ENV2 && !lfo22ENV2) {
    // Start the envelope
    env2.noteOn(micros());
  }
  // Check for falling edge on ENV2_gate pin
  else if (!currentEnv2GateState && prevEnv2GateState && !trigger2ENV2 && !lfo12ENV2 && !lfo22ENV2) {
    // Stop the envelope
    env2.noteOff(micros());
  }

  // Update previous state of ENV2_gate pin
  prevEnv2GateState = currentEnv2GateState;


  if (MIDI.read()) {
    // Serial.print("Status byte: ");
    // Serial.print(MIDI.getType(), HEX);
    // Serial.print(", Data byte 1: ");
    // Serial.print(MIDI.getData1(), HEX);
    // Serial.print(", Data byte 2: ");
    // Serial.println(MIDI.getData2(), HEX);
    byte type = MIDI.getType();
    switch (type) {
      case midi::Clock:
        if (midiClockCount == 0) {
          startTime_midi = micros();
        }
        midiClockCount++;
        if (midiClockCount == 24) {
          elapsedTime = micros() - startTime_midi;
          float quarterNoteDuration = elapsedTime / 24.0;
          bpm = 60000000.0 / quarterNoteDuration / 25;
          midiClockCount = 0;
        }
        break;
      case midi::NoteOn:
        if (!noteOnReceived) {
          noteOnReceived = true;
          gateOpen = true;
          velocity = MIDI.getData2();
          env1_amp = velocity / 127.0;
          noteNumber = constrain(MIDI.getData1() - 0xC, 21, 93);
          noteNumber = 42 * (noteNumber - 21);
        }
        break;
      case midi::NoteOff:
        if (noteOnReceived) {
          noteOnReceived = false;
        }
        break;
      default:
        break;
    }
  }
  if (!noteOnReceived && gateOpen) {
    // Check if gate has been open for a while and no NoteOn received
    gateOpen = false;
  }

  updateGate();
}

// Function to set voltage using MCP4822 DAC
void setVoltage(byte dac_cs_pin, mcp4xxx_register channel, int voltage) {
  SPI.beginTransaction(SPISettings(133000000, MSBFIRST, SPI_MODE0));
  digitalWrite(dac_cs_pin, LOW);
  SPI.transfer16(channel | voltage);
  digitalWrite(dac_cs_pin, HIGH);
  SPI.endTransaction();
}

void updateGate() {
  static bool prevGateOpen = false;
  if (gateOpen != prevGateOpen) {
    if (gateOpen && !prevGateOpen) {
      env1.noteOn(micros());
      if (trigger2SYNC == true) {
        digitalWrite(TRIG, LOW);
        delay(5);  // Ensure trigger pulse width
        digitalWrite(TRIG, HIGH);
      } else {
        digitalWrite(TRIG, HIGH);
      }
      if (trigger2LFO1 && !lfo1Mode) {
        lfo1.sync(micros());
      }
      if (trigger2LFO2 && !lfo2Mode) {
        lfo2.sync(micros());
      }
      if (trigger2ENV2 && !lfo12ENV2 && !lfo22ENV2) {
        env2.noteOn(micros());
      }
      if (velocity2Cutoff || velocity2Pitch) {
        setVoltage(midi_dac1_cs, mcp4822_channel_a, velocity * 24 * velocityPercent);  // Output velocity CV
      }
      setVoltage(midi_dac1_cs, mcp4822_channel_b, noteNumber);  // Output V/Oct CV

    } else {
      if (trigger2ENV2 && !lfo12ENV2 && !lfo22ENV2) {
        env2.noteOff(micros());
      }
      env1.noteOff(micros());
      setVoltage(midi_dac1_cs, mcp4822_channel_b, 0);  // Reset V/Oct CV
      if (!velocity2Cutoff || !velocity2Pitch) {
        setVoltage(midi_dac1_cs, mcp4822_channel_a, 0);  // Reset velocity CV
      }
    }
    prevGateOpen = gateOpen;
  }
}
void read_encoder() {
  static uint8_t old_AB = 3;                                                                  // Lookup table index
  static int8_t encval = 0;                                                                   // Encoder value
  static const int8_t enc_states[] = { 0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0 };  // Lookup table
  int changevalue;

  old_AB <<= 2;  // Remember previous state

  if (digitalRead(ENC_A))
    old_AB |= 0x02;  // Add current state of pin A
  if (digitalRead(ENC_B))
    old_AB |= 0x01;  // Add current state of pin B

  encval += enc_states[(old_AB & 0x0f)];

  // Update counter if encoder has rotated a full indent, that is at least 4 steps
  if (encval > 3) {  // Four steps forward
    changevalue = 1;
    if ((micros() - _lastIncReadTime) < _pauseLength) {
      changevalue = _fastIncrement * changevalue;
    }
    _lastIncReadTime = micros();

    if (enterLFOMenu) {
      if (digitalRead(btn0) == LOW) {
        // Rotate the encoder to select between LFO1 and LFO2
        if (LFOindex == 1) {
          // Change waveform for LFO1
          lfo1_waveformIndex = (lfo1_waveformIndex + 1) % 5;
        } else if (LFOindex == 2) {
          // Change waveform for LFO2
          lfo2_waveformIndex = (lfo2_waveformIndex + 1) % 5;
        }
      } else if (digitalRead(btn1) == LOW) {
        // Rotate the encoder to select between LFO1 and LFO2
        if (LFOindex == 1) {
          lfo1Mode = !lfo1Mode;
        }
        if (LFOindex == 2) {
          // Change waveform for LFO2
          lfo2Mode = !lfo2Mode;
        }
      } else if (digitalRead(btn2) == LOW) {
        // If button 2 is pressed, change the frequency of the selected LFO
        if (LFOindex == 1 && !lfo1Mode) {
          lfo1_freq += changevalue * adjust_lfo_increment(lfo1_freq);  // Increment/decrement frequency of LFO1
          lfo1_freq = constrain(lfo1_freq, 0.1, 200);                  // Ensure frequency stays within [0.1, 200] range
        } else if (LFOindex == 2 && !lfo2Mode) {
          lfo2_freq += changevalue * adjust_lfo_increment(lfo2_freq);  // Increment/decrement frequency of LFO2
          lfo2_freq = constrain(lfo2_freq, 0.1, 200);                  // Ensure frequency stays within [0.1, 200] range
        }
        if (LFOindex == 1 && lfo1Mode) {
          lfo1_rateIndex += changevalue;
          lfo1_rateIndex = constrain(lfo1_rateIndex, 0, 18);
          lfo1_rate = lfo_mode1_rate[lfo1_rateIndex];
        } else if (LFOindex == 2 && lfo2Mode) {
          lfo2_rateIndex += changevalue;
          lfo2_rateIndex = constrain(lfo2_rateIndex, 0, 18);
          lfo2_rate = lfo_mode1_rate[lfo2_rateIndex];
        }
      } else if (digitalRead(btn3) == LOW) {

        if (LFOindex == 1) {
          lfo1_amp += changevalue * 250;
          lfo1_amp = constrain(lfo1_amp, 1, DACSIZE - 1);
        } else if (LFOindex == 2) {
          lfo2_amp += changevalue * 250;                   // Increment/decrement frequency of LFO2
          lfo2_amp = constrain(lfo2_amp, 1, DACSIZE - 1);  // Ensure frequency stays within [0.1, 200] range
        }
      } else {
        LFOindex = constrain(LFOindex + changevalue, 1, 2);
      }
    } else if (enterENVMenu) {
      if (digitalRead(btn0) == LOW) {
        if (ENVindex == 1) {
          env1_attack += changevalue * 100000;
          env1_attack = constrain(env1_attack, 0, 3000000);
          env1.setAttack(env1_attack);
        } else if (ENVindex == 2) {
          env2_attack += changevalue * 100000;
          env2_attack = constrain(env2_attack, 0, 3000000);
          env2.setAttack(env2_attack);
        }
      } else if (digitalRead(btn1) == LOW) {
        if (ENVindex == 1) {
          env1_decay += changevalue * 10000;
          env1_decay = constrain(env1_decay, 0, 500000);
          env1.setDecay(env1_decay);
        } else if (ENVindex == 2) {
          env2_decay += changevalue * 10000;
          env2_decay = constrain(env2_decay, 0, 500000);
          env2.setDecay(env2_decay);
        }
      } else if (digitalRead(btn2) == LOW) {
        if (ENVindex == 1) {
          env1_sustain += changevalue * 250;
          env1_sustain = constrain(env1_sustain, 0, DACSIZE - 1);
          env1.setSustain(env1_sustain);
        } else if (ENVindex == 2) {
          env2_sustain += changevalue * 250;
          env2_sustain = constrain(env2_sustain, 0, DACSIZE - 1);
          env2.setSustain(env2_sustain);
        }
      } else if (digitalRead(btn3) == LOW) {
        if (ENVindex == 1) {
          env1_release += changevalue * 100000;
          env1_release = constrain(env1_release, 50000, 5000000);
          env1.setRelease(env1_release);
        } else if (ENVindex == 2) {
          env2_release += changevalue * 100000;
          env2_release = constrain(env2_release, 50000, 5000000);
          env2.setRelease(env2_release);
        }
      } else if (digitalRead(btn4) == LOW) {
        if (ENVindex == 2) {
          env2_amp += changevalue * 0.1;
          env2_amp = constrain(env2_amp, 0, 1);
        }
      } else {
        ENVindex = constrain(ENVindex + changevalue, 1, 2);
      }
    } else if (enterMODMenu) {
      if (digitalRead(btn0) == LOW) {
        if (trigger2ENV2) {
          lfo1_pathsIndex = (lfo1_pathsIndex + 1) % 7;
        } else {
          lfo1_pathsIndex = (lfo1_pathsIndex + 1) % 8;
        }
      } else if (digitalRead(btn1) == LOW) {
        if (trigger2ENV2) {
          lfo2_pathsIndex = (lfo2_pathsIndex + 1) % 5;
        } else {
          lfo2_pathsIndex = (lfo2_pathsIndex + 1) % 6;
        }
      } else if (digitalRead(btn2) == LOW) {
        env2_pathsIndex = (env2_pathsIndex + 1) % 5;
      } else if (digitalRead(btn4) == LOW && digitalRead(btn3) == LOW) {
        velocityPercent += changevalue * 0.1;
        velocityPercent = constrain(velocityPercent, 0, 1);

      } else if (digitalRead(btn3) == LOW) {
        trigger_pathsIndex = (trigger_pathsIndex + 1) % 5;
        if (lfo1_pathsIndex == 7) {
          lfo1_pathsIndex = 0;
        }
        if (lfo2_pathsIndex == 5) {
          lfo2_pathsIndex = 0;
        }
      } else if (digitalRead(btn4) == LOW) {
        velocity_pathsIndex = (velocity_pathsIndex + 1) % 3;
      }
    } else {
      submenuSelection = constrain(submenuSelection + changevalue, 0, 2);  // Update submenu selection
    }

    encval = 0;
  } else if (encval < -3) {  // Four steps backward
    changevalue = -1;
    if ((micros() - _lastDecReadTime) < _pauseLength) {
      changevalue = _fastIncrement * changevalue;
    }
    _lastDecReadTime = micros();

    if (enterLFOMenu) {
      if (digitalRead(btn0) == LOW) {
        // Rotate the encoder to select between LFO1 and LFO2
        if (LFOindex == 1) {
          // Change waveform for LFO1
          lfo1_waveformIndex = (lfo1_waveformIndex - 1 + 5) % 5;
        } else if (LFOindex == 2) {
          // Change waveform for LFO2
          lfo2_waveformIndex = (lfo2_waveformIndex - 1 + 5) % 5;
        }
      } else if (digitalRead(btn1) == LOW) {
        // Rotate the encoder to select between LFO1 and LFO2
        if (LFOindex == 1) {
          lfo1Mode = !lfo1Mode;
        }
        if (LFOindex == 2) {
          // Change waveform for LFO2
          lfo2Mode = !lfo2Mode;
        }
      } else if (digitalRead(btn2) == LOW) {
        // If button 2 is pressed, change the frequency of the selected LFO
        if (LFOindex == 1 && !lfo1Mode) {
          lfo1_freq += changevalue * adjust_lfo_increment(lfo1_freq);  // Increment/decrement frequency of LFO1
          lfo1_freq = constrain(lfo1_freq, 0.1, 200);                  // Ensure frequency stays within [0.1, 200] range
        } else if (LFOindex == 2 && !lfo2Mode) {
          lfo2_freq += changevalue * adjust_lfo_increment(lfo2_freq);  // Increment/decrement frequency of LFO2
          lfo2_freq = constrain(lfo2_freq, 0.1, 200);                  // Ensure frequency stays within [0.1, 200] range
        }
        if (LFOindex == 1 && lfo1Mode) {
          lfo1_rateIndex += changevalue;
          lfo1_rateIndex = constrain(lfo1_rateIndex, 0, 18);
          lfo1_rate = lfo_mode1_rate[lfo1_rateIndex];
        } else if (LFOindex == 2 && lfo2Mode) {
          lfo2_rateIndex += changevalue;
          lfo2_rateIndex = constrain(lfo2_rateIndex, 0, 18);
          lfo2_rate = lfo_mode1_rate[lfo2_rateIndex];
        }
      } else if (digitalRead(btn3) == LOW) {

        if (LFOindex == 1) {
          lfo1_amp += changevalue * 250;
          lfo1_amp = constrain(lfo1_amp, 1, DACSIZE - 1);
        } else if (LFOindex == 2) {
          lfo2_amp += changevalue * 250;                   
          lfo2_amp = constrain(lfo2_amp, 1, DACSIZE - 1);  
        }
      } else {
        LFOindex = constrain(LFOindex + changevalue, 1, 2);
      }
    } else if (enterENVMenu) {

      if (digitalRead(btn0) == LOW) {
        if (ENVindex == 1) {
          env1_attack += changevalue * 10000;
          env1_attack = constrain(env1_attack, 0, 3000000);
          env1.setAttack(env1_attack);
        } else if (ENVindex == 2) {
          env2_attack += changevalue * 10000;
          env2_attack = constrain(env2_attack, 0, 3000000);
          env2.setAttack(env2_attack);
        }
      } else if (digitalRead(btn1) == LOW) {
        if (ENVindex == 1) {
          env1_decay += changevalue * 10000;
          env1_decay = constrain(env1_decay, 0, 500000);
          env1.setDecay(env1_decay);
        } else if (ENVindex == 2) {
          env2_decay += changevalue * 10000;
          env2_decay = constrain(env2_decay, 0, 500000);
          env2.setDecay(env2_decay);
        }
      } else if (digitalRead(btn2) == LOW) {
        if (ENVindex == 1) {
          env1_sustain += changevalue * 409;
          env1_sustain = constrain(env1_sustain, 0, DACSIZE - 1);
          env1.setSustain(env1_sustain);
        } else if (ENVindex == 2) {
          env2_sustain += changevalue * 409;
          env2_sustain = constrain(env2_sustain, 0, DACSIZE - 1);
          env2.setSustain(env2_sustain);
        }
      } else if (digitalRead(btn3) == LOW) {
        if (ENVindex == 1) {
          env1_release += changevalue * 100000;
          env1_release = constrain(env1_release, 50000, 5000000);
          env1.setRelease(env1_release);
        } else if (ENVindex == 2) {
          env2_release += changevalue * 100000;
          env2_release = constrain(env2_release, 50000, 5000000);
          env2.setRelease(env2_release);
        }
      } else if (digitalRead(btn4) == LOW) {
        if (ENVindex == 2) {
          env2_amp += changevalue * 0.1;
          env2_amp = constrain(env2_amp, 0, 1);
        }
      } else {
        ENVindex = constrain(ENVindex + changevalue, 1, 2);
      }
    } else if (enterMODMenu) {
      if (digitalRead(btn0) == LOW) {
        if (trigger2ENV2) {
          lfo1_pathsIndex = (lfo1_pathsIndex - 1 + 7) % 7;
        } else {
          lfo1_pathsIndex = (lfo1_pathsIndex - 1 + 8) % 8;
        }
      } else if (digitalRead(btn1) == LOW) {
        if (trigger2ENV2) {
          lfo2_pathsIndex = (lfo2_pathsIndex - 1 + 5) % 5;
        } else {
          lfo2_pathsIndex = (lfo2_pathsIndex - 1 + 6) % 6;
        }
      } else if (digitalRead(btn2) == LOW) {
        env2_pathsIndex = (env2_pathsIndex - 1 + 5) % 5;
      } else if (digitalRead(btn4) == LOW && digitalRead(btn3) == LOW) {
        velocityPercent += changevalue * 0.1;
        velocityPercent = constrain(velocityPercent, 0, 1);

      } else if (digitalRead(btn3) == LOW) {
        trigger_pathsIndex = (trigger_pathsIndex - 1 + 5) % 5;
        if (lfo1_pathsIndex == 7) {
          lfo1_pathsIndex = 0;
        }
        if (lfo2_pathsIndex == 5) {
          lfo2_pathsIndex = 0;
        }
      } else if (digitalRead(btn4) == LOW) {
        velocity_pathsIndex = (velocity_pathsIndex - 1 + 3) % 3;
      }
    } else {
      submenuSelection = constrain(submenuSelection + changevalue, 0, 2);  // Update submenu selection
    }

    encval = 0;
  }

  // Ensure submenu selection stays within bounds
  submenuSelection = constrain(submenuSelection, 0, 2);
  updateDisplay(submenuSelection);
}
void updateDisplay(int submenu) {
  // Clear the display buffer
  u8g2.clearBuffer();

  if (enterLFOMenu) {
    //u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);  // Choose a font for submenu text
    u8g2.drawStr(0, 15, "LFO");          // Write "Menu" at coordinates (0, 15) if not in the MIDI menu
    u8g2.drawStr(27, 15, String(LFOindex).c_str());
    u8g2.drawLine(0, 19, 128, 19);  // Draw a line under the text
    u8g2.setFont(u8g2_font_5x7_mr);
    u8g2.drawStr(5, 30, "Waveform: ");
    u8g2.drawStr(5, 39, "Mode: ");

    if (LFOindex == 1) {
      displayLFOParameters(lfo1Mode, lfo1_waveformIndex, lfo1_freq, lfo_mode1_rateStrings[lfo1_rateIndex], lfo1_amp);
    } else if (LFOindex == 2) {
      displayLFOParameters(lfo2Mode, lfo2_waveformIndex, lfo2_freq, lfo_mode1_rateStrings[lfo2_rateIndex], lfo2_amp);
    }
  } else if (enterENVMenu) {
    u8g2.setFont(u8g2_font_ncenB08_tr);  // Choose a font for submenu text
    u8g2.drawStr(0, 15, "ENV");          // Write "Menu" at coordinates (0, 15) if not in the MIDI menu
    u8g2.drawStr(27, 15, String(ENVindex).c_str());
    u8g2.drawLine(0, 19, 128, 19);  // Draw a line under the text

    if (ENVindex == 1) {
      displayENVParameters(ENVindex, env1_attack, env1_decay, env1_sustain, env1_release, env1_amp);
    } else if (ENVindex == 2) {
      displayENVParameters(ENVindex, env2_attack, env2_decay, env2_sustain, env2_release, env2_amp);
    }
  } else if (enterMODMenu) {
    u8g2.setFont(u8g2_font_ncenB08_tr);  // Choose a font for submenu text
    u8g2.drawStr(0, 15, "MOD");          // Write "Menu" at coordinates (0, 15) if not in the MIDI menu
    u8g2.drawLine(0, 19, 128, 19);       // Draw a line under the text
    u8g2.setFont(u8g2_font_5x7_mr);
    u8g2.drawStr(5, 30, "LFO1: ");
    u8g2.drawStr(5, 39, "LFO2: ");
    u8g2.drawStr(5, 47, "ENV2: ");
    u8g2.drawStr(4, 55, "Trigger: ");
    u8g2.drawStr(5, 63, "Velocity: ");
    u8g2.drawStr(115, 63, "%");

    u8g2.drawStr(50, 30, String(lfo1_paths[lfo1_pathsIndex]).c_str());
    u8g2.drawStr(50, 39, String(lfo2_paths[lfo2_pathsIndex]).c_str());
    u8g2.drawStr(50, 47, String(env2_paths[env2_pathsIndex]).c_str());
    u8g2.drawStr(50, 55, String(trigger_paths[trigger_pathsIndex]).c_str());
    u8g2.drawStr(50, 63, String(velocity_paths[velocity_pathsIndex]).c_str());
    u8g2.drawStr(85, 63, String(velocityPercent * 100).c_str());

  } else {
    u8g2.setFont(u8g2_font_ncenB08_tr);  // Choose a font for submenu text
    u8g2.drawStr(0, 15, "Menu");         // Write "Menu" at coordinates (0, 15) if not in the MIDI menu
    u8g2.drawLine(0, 19, 128, 19);       // Draw a line under the text

    // Display submenu text and circle cursor
    switch (submenu) {
      case 0:
        u8g2.setFont(u8g2_font_ncenB24_tr);  // Choose a larger font for submenu text
        u8g2.drawStr(30, 58, "LFO");

        break;
      case 1:
        u8g2.setFont(u8g2_font_ncenB24_tr);  // Choose a larger font for submenu text
        u8g2.drawStr(30, 58, "ENV");

        break;
      case 2:
        u8g2.setFont(u8g2_font_ncenB24_tr);  // Choose a larger font for submenu text
        u8g2.drawStr(30, 58, "MOD");

        break;
      default:
        break;
    }
  }

  // Send the buffer to the display
  u8g2.sendBuffer();
}

void displayLFOParameters(bool mode, int waveformIndex, float freq, String rate, int amp) {
  if (!mode) {  // Free running mode
    u8g2.drawStr(65, 39, "Free");
    u8g2.drawStr(5, 47, "Frequency: ");
    u8g2.drawStr(105, 47, "Hz");
    u8g2.drawStr(5, 55, "Amplitude: ");
    u8g2.drawStr(105, 55, "%");
    u8g2.drawStr(65, 30, String(wvfrms[waveformIndex]).c_str());
    u8g2.drawStr(65, 47, String(freq).c_str());
    u8g2.drawStr(65, 55, String(amp * 100 / 4095).c_str());
  } else {  // Locked mode
    u8g2.drawStr(65, 39, "Locked");
    u8g2.drawStr(5, 47, "Rate: ");
    if (rate == "2") {
      u8g2.drawStr(105, 47, "bars");
    } else if (rate == "1") {
      u8g2.drawStr(105, 47, "bar");
    } else {
      u8g2.drawStr(105, 47, "note");
    }
    u8g2.drawStr(5, 55, "Amplitude: ");
    u8g2.drawStr(105, 55, "%");
    u8g2.drawStr(65, 30, String(wvfrms[waveformIndex]).c_str());
    u8g2.drawStr(65, 47, rate.c_str());
    u8g2.drawStr(65, 55, String(amp * 100 / 4095).c_str());
  }
}

void displayENVParameters(int index, int attack, int decay, int sustain, int release, float amp) {
  u8g2.setFont(u8g2_font_5x7_mr);
  u8g2.drawStr(5, 30, "Atack: ");
  u8g2.drawStr(5, 39, "Decay: ");
  u8g2.drawStr(5, 47, "Sustain: ");
  u8g2.drawStr(5, 55, "Release: ");
  if (index == 2) {
    u8g2.drawStr(5, 63, "Amplitude: ");
  }
  u8g2.drawStr(65, 30, String(static_cast<float>(attack) / 1000000.0).c_str());
  u8g2.drawStr(105, 30, "s");
  u8g2.drawStr(65, 39, String(static_cast<float>(decay) / 1000.0, 0).c_str());
  u8g2.drawStr(105, 39, "ms");
  float sustain_print = map(sustain, 0, DACSIZE - 1, 0, 100);
  u8g2.drawStr(65, 47, String(sustain_print).c_str());
  u8g2.drawStr(105, 47, "%");
  u8g2.drawStr(65, 55, String(static_cast<float>(release) / 1000000.0).c_str());
  u8g2.drawStr(105, 55, "s");
  if (index == 2) {
    u8g2.drawStr(65, 63, String(amp * 100, 1).c_str());
    u8g2.drawStr(105, 63, "%");
  }
}

float adjust_lfo_increment(float current_frequency) {
  float increment;
  if (current_frequency < 2) {
    increment = 0.1;
  } else if (current_frequency < 5) {
    increment = 0.25;
  } else if (current_frequency < 10) {
    increment = 0.5;
  } else if (current_frequency < 50) {
    increment = 1;
  } else {
    increment = 10;
  }
  return increment;
}
