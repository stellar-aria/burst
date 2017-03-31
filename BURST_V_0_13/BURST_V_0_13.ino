/************************************
  BURST 0.13 - BEFACO
  written by Eloi Flores - Winter Modular



  NOTES, KNOWN BUGS and IMPROVEMENTS :

  When cycle on, no trigger, and external ping. There is a process to resync the burst with the ping, because if not the burst is not in phase with the ping.
  This is done taking the difference time between the cycle and the moment when the trigger button has been pressed.
  A proportion is calculated when the trigger button is pressed to provide the position of the trigger in relation with the time window.
  This calculation uses float, known as the evil datatype in terms of processing consume. Another way to do that?

  The resync doesn't work properly with the combination of :
  - Distribution   o     o    o   o  o oo
  - Small amount of repetitions (depending on tempo but 8 7 or less; as lower tempo, the amount of repetitions where we have de-sync is higher)

  Testing the module we noticed that an attenuator in the quantity input could be very useful.
  One solution could be to use [encoder + encoder button hold] to adjust this attenuationof quantity CV in 32 steps.


  At high frequencies there is a gap at the end of the burst
*/




///////////////// LIBRARYS

#include <ClickEncoder.h>
#include <TimerOne.h>
#include <EEPROM.h>
#include "DebugUtils.h"

///////////////// PIN DEFINITIONS


////// ANALOG INS

#define CV_DIVISIONS    A0
#define CV_PROBABILITY  A1
#define CV_QUANTITY     A2
#define CV_DISTRIBUTION A3

////// DIGITAL INS

#define CYCLE_SWITCH    A4
#define CYCLE_STATE     A5

#define PING_STATE      2
#define PING_BUTTON     3
#define ENCODER_1       4
#define ENCODER_2       5
#define TRIGGER_STATE   8
#define TRIGGER_BUTTON  11

////// DIGITAL OUTS

#define EOC_LED         0
#define OUT_LED         1
#define OUT_STATE       7
#define EOC_STATE       9

const int led_pin[4] = { 12, 13, 10, 6 };

////// Encoder

ClickEncoder *encoder;
int16_t encoder_value, last_encoder_value;

void timerIsr() {
  encoder->service();
}

////// VARIABLES

// tempo and counters
unsigned long master_clock = 0;             /// the master clock is the result of averaged time of ping input or encoder button
unsigned long master_clock_temp = 0;        /// we use the temp variables to avoid the parameters change during a burst execution
unsigned long clock_divided = 0;            /// the result of div/mult the master clock depending on the div/mult potentiometer/input
unsigned long clock_divided_temp = 0;            /// we use the temp variables to avoid the parameters change during a burst execution

int time_portions = 0;                      /// the linear portions of time depending on clock divided and number of repetitions in the burst. If the distribution is linear this will give us the duration of every repetition.
/// if it is not linear it will be used to calculate the distribution
int time_portions_temp = 0;                 /// we use the temp variables to avoid the parameters change during a burst execution
unsigned long current_time = 0;
unsigned long old_time = 0;

unsigned long burst_time_start = 0;         /// the moment when the burst start
unsigned long burst_time_accu = 0;          /// the accumulation of all repetitions times since the burst have started

unsigned long repetition_raise_start = 0;       /// the time when the previous repetition pulse have raised

byte repetition_counter = 0;            /// the current repetition number since the burst has started

unsigned long elapsed_time_since_prev_repetition = 0;               /// the difference between previous repetition time and current repetition time. the difference between one repetition and the next one
unsigned long elapsed_time_since_prev_repetition_new = 0;           /// the position of the new repetition inside the time window
unsigned long elapsed_time_since_prev_repetition_old = 0;           /// the position of the previous repetition inside the time window

unsigned long led_quantity_time = 0;        /// time counter used to turn off the led when we are chosing the number of repetitions with the encoder

unsigned long eoc_counter = 0;              /// a counter to turn off the eoc led and the eoc output


//    divisions
int divisions;                            //// value of the time division or the time multiplier
int divisions_old;
int divisions_pot;
int sub_division_counter = 0;

////// Repetitions

#define MAX_REPETITIONS 32                  /// max number of repetitions

byte repetitions = 0;                       /// number of repetitions
byte repetitions_old = 0;
byte repetitions_temp = 0;                  /// temporal value that adds the number of repetitions in the encoder and the number number added by the CV quantity input
byte repetitions_encoder = 0;
byte repetitions_encoder_temp = 0;
unsigned long position_temp = 0;


///// Random
int random_pot = 0;
int random_dif = 0;

///// Distribution
int distribution_pot = 0;
byte distribution_sign = 1;               /// 0 negative , 1 postive, 2 zero
float distribution = 0;
float distribution_index_array [9];       /// used to calculate the position of the repetitions
int curve = 5;                            /// the curved we apply to the  pow function to calculate the distribution of repetitions

//// Trigger
uint8_t trigger_button_state = LOW;           /// the trigger button
bool trigger_cv_state = 0;               /// the trigger input
bool triggered = false;                        /// the result of both trigger button and trigger input
bool trigger_ready = true;          /// a falg to know if it is the first time we ahve pressed a trigger ( after release it is HIGH again)
long trigger_difference = 0;            /// the time difference between the trigger and the ping
bool trigger_first_pressed = 0;
float trigger_dif_proportional = 0;
//// Cycle
bool cycle_switch_state = 0;             /// the cycle switch
bool cycle_in_state = 0;                 /// the cycle input
bool cycle = 0;                          /// the result of both cycle switch and cycle input


/// ping
byte ping_in_state = 0;


/// output
bool output_state = HIGH;


/// flags
bool burst_started = 0;                   // if the burst is active or not
bool no_more_bursts = HIGH;               // used for probability to stop bursts, HIGH -> NO repetitions, LOW -> repetitions
bool first_burst = 0;                     // high if we are in the first repetition, LOW if we are in any of the other repetitions
bool resync = 0;                          // active when the resync in cycle can be done

//// encoder button and tap tempo
byte encoder_button_state = 0;
unsigned long tempo_tic = 0;                        /// everytime a pulse is received in ping input or the encoder button (tap) is pressed we store the time
unsigned long tempo_tic_temp = 0;
unsigned long old_tempo_tic = 0;                    /// previous stored tempo_tic time value
unsigned long tap_tempo_array[4] = {0, 0, 0, 0};    /// we use this array to store the 4 last tempo_tic values in order to make an average
byte max_taps = 0;                                  /// the number of tempo_tics that have been stored in tap_tempo_array[]  (from 0 to 3)
byte tap_index = 0;                                 /// and index used to know the last tempo_tic stored in the tap_tempo_array[]
unsigned long averaged_taps = 0;                    /// the sum of all tempo_tic differences

void setup() {

  //// remove to activate eoc_led and out_led
  //Serial.begin(9600);
  //Serial.println("HOLA");

  /// Encoder
  encoder = new ClickEncoder(ENCODER_2, ENCODER_1, 3);

  Timer1.initialize(1000); // maybe 100u?
  Timer1.attachInterrupt(timerIsr); // why do we only service the encoder here, actually we should get the values of all the interface elements. the variables we set need to be 'volatile' and we need to disabled interrupts when reading them (we can just copy them and then re-enable interrupts).

  // Encoder pins
  pinMode (ENCODER_1, INPUT_PULLUP);
  pinMode (ENCODER_2, INPUT_PULLUP);


  pinMode (CYCLE_SWITCH, INPUT_PULLUP);
  pinMode (CYCLE_STATE, INPUT_PULLUP);
  pinMode (PING_STATE, INPUT_PULLUP);
  pinMode (TRIGGER_STATE, INPUT_PULLUP);
  pinMode (TRIGGER_BUTTON, INPUT_PULLUP);
  pinMode (PING_BUTTON, INPUT_PULLUP);



  pinMode (EOC_LED, OUTPUT);
  pinMode (OUT_LED, OUTPUT);

  pinMode (OUT_STATE, OUTPUT);
  digitalWrite(OUT_STATE, HIGH);

  pinMode (EOC_STATE, OUTPUT);
  digitalWrite(EOC_STATE, HIGH);

  create_distribution_index();

  for (int i = 0; i < 4; i++) {
    pinMode(led_pin[i], OUTPUT);
  }

  master_clock = (EEPROM.read(0) & 0xFF) + (((long)EEPROM.read(1) << 8) & 0xFFFF) + (((long)EEPROM.read(2) << 16) & 0xFFFFFF) + (((long)EEPROM.read(3) << 24) & 0xFFFFFFFF);
  master_clock_temp = master_clock;
  repetitions = EEPROM.read(4);
  repetitions_temp = repetitions;
  repetitions_old = repetitions;
  repetitions_encoder = repetitions;
  repetitions_encoder_temp = repetitions;
  read_division();

  if ( divisions > 0) {
    clock_divided_temp = master_clock_temp * divisions;
  }
  if ( divisions < 0) {
    clock_divided_temp = master_clock_temp / (-divisions);
  }
  if ( divisions == 0) {
    clock_divided_temp = master_clock_temp;
  }
  time_portions_temp = clock_divided_temp / repetitions_temp;


}

void loop() {

  if ((triggered == HIGH) && (trigger_first_pressed == HIGH))  {    ///// we read the values and pots and inputs, and store the time difference between ping clock and trigger
    if (repetitions_encoder_temp != repetitions_encoder) {
      repetitions_encoder = repetitions_encoder_temp;
      EEPROM.write(4, repetitions_encoder_temp);
    }
    repetitions = repetitions_temp;
    clock_divided = clock_divided_temp;
    master_clock = master_clock_temp;
    time_portions = time_portions_temp;
    read_division();
    read_random();
    read_distribution();
    read_cycle();
    start_burst_init();
    trigger_difference = burst_time_start - tempo_tic_temp;       /// when we press the trigger button we define the phase difference between the external clock and our burst
    trigger_dif_proportional = (float)master_clock_temp / (float)trigger_difference;
    trigger_ready = false;
  }

  calculate_clock();          /// we read the ping in and the encoder button to get : master clock, clock divided and time_portions
  read_trigger();             /// we read the trigger input and the trigger button to see if it is high or low, if it is high and it is the first time it is.
  read_repetitions();         /// we read the number of repetitions in the encoder, we have to attend this process often to avoid missing encoder tics.



  current_time = millis();
  if (cycle == HIGH) {                                                      //////// CYCLE ON

    if (( current_time > tempo_tic + (clock_divided * sub_division_counter) + trigger_difference ) && resync) {                  //////// RESYNC BETWEEN CYCLE AND PING MAINTAINING PHASE bearing in mind the difference, the divisions and the


      if (repetitions != repetitions_temp) {
        EEPROM.write(4, repetitions_temp);
      }
      repetitions = repetitions_temp;
      clock_divided = clock_divided_temp;
      master_clock = master_clock_temp;
      time_portions = time_portions_temp;
      read_division();
      read_random();
      read_distribution();
      read_cycle();
      start_burst_init();
      resync = LOW;

    }




    if (current_time >=  eoc_counter + 25) {                                 //////// CHECK EOC OUTPUT AND LED
      digitalWrite(EOC_LED, LOW);
      digitalWrite(EOC_STATE, HIGH);
    }

    if ((current_time >= led_quantity_time + 25) && (no_more_bursts)) {     //////// ONCE THE NUMBER OF REPETITIONS IS CHOSEN LEDS LIGHT UP WITH THE CURRENT REPETITION
      for (int i = 0 ; i < 4 ; i++) {
        digitalWrite(led_pin[i], bitRead(repetition_counter, i));
      }
    }

    // pulse down
    if ( (output_state == HIGH) && (burst_started == HIGH) ) {
      if ( current_time >= burst_time_start + burst_time_accu + 2) {      /////  WE COUNT THE TIME DURING THE PULSE IS HIGH

        output_state = !output_state;
        digitalWrite(OUT_STATE, !(output_state * no_more_bursts));
        digitalWrite(OUT_LED, (output_state * no_more_bursts));
        random_dif = random_pot - random(1023);
        if (( first_burst == HIGH ) && (random_dif <= 0) && trigger_button_state)  {   //// WE DECIDE IF THE BURST WILL OCCUR OR NOT DEPENDING ON PROBABILITY
          no_more_bursts = LOW;

        }
        first_burst = LOW;
      }
    }

    current_time = millis();
    // pulse up - burst time
    if ( (output_state == LOW) && (burst_started == HIGH) ) {
      if ( current_time >= (burst_time_start + elapsed_time_since_prev_repetition + burst_time_accu) ) { ///// WE COUNT THE TIME TO THE NEXT PULSE
        if (repetition_counter < repetitions - 1) {  /// WE CHECK IF IT IS NOT THE LAST REPETITION
          output_state = !output_state;
          digitalWrite(OUT_STATE, !(output_state * no_more_bursts));
          digitalWrite(OUT_LED, (output_state * no_more_bursts));
          burst_time_accu += elapsed_time_since_prev_repetition;
          repetition_counter++;
          if (repetition_counter != repetitions - 1 ) {     /// WE CALCULATE THE TIME FOR THE NEXT PULSE
            switch ( distribution_sign ) {
              case 1:
                elapsed_time_since_prev_repetition_old = elapsed_time_since_prev_repetition_new;
                elapsed_time_since_prev_repetition_new = fscale( 0, clock_divided, 0, clock_divided, time_portions * (repetition_counter + 1), distribution);
                elapsed_time_since_prev_repetition = elapsed_time_since_prev_repetition_new - elapsed_time_since_prev_repetition_old;
                break;
              case 0:
                //if (repetition_counter == repetitions - 1 ) {
                elapsed_time_since_prev_repetition_old = elapsed_time_since_prev_repetition_new;
                position_temp = repetitions - repetition_counter - 2;
                if (position_temp > 0) {
                  elapsed_time_since_prev_repetition_new = fscale( 0, clock_divided, 0, clock_divided, time_portions * position_temp, distribution);
                  elapsed_time_since_prev_repetition = elapsed_time_since_prev_repetition_old - elapsed_time_since_prev_repetition_new;
                }
                break;
              case 2:
                elapsed_time_since_prev_repetition_old = elapsed_time_since_prev_repetition_new;
                elapsed_time_since_prev_repetition_new = time_portions *  (repetition_counter + 1);
                elapsed_time_since_prev_repetition = elapsed_time_since_prev_repetition_new - elapsed_time_since_prev_repetition_old;
                break;
            }
          }
        }
        else {                                                  ///// tHE END OF THE BURST

          sub_division_counter++;
          switch (divisions) {                                  ///// WE ADJUST THE VALUE OF TEMPO_TIC ( FOR THE RESYNC) DEPENDING ON TIME_DIVISIONS AND THE AMOUNT OF SUB-BURSTS DONE
            case 0:
              tempo_tic = tempo_tic_temp;
              sub_division_counter = 0;
              break;
            case -2:
              if (sub_division_counter == 2) {
                tempo_tic = tempo_tic_temp;
                sub_division_counter = 0;
              }
              break;
            case -3:
              if (sub_division_counter == 3) {
                tempo_tic = tempo_tic_temp;
                sub_division_counter = 0;
              }
              break;
            case -4:
              if (sub_division_counter == 4) {
                tempo_tic = tempo_tic_temp;
                sub_division_counter = 0;
              }
              break;
            case -5:
              if (sub_division_counter == 5) {
                tempo_tic = tempo_tic_temp;
                sub_division_counter = 0;
              }
              break;
          }
          resync = HIGH;
          digitalWrite(EOC_LED, HIGH);
          digitalWrite(EOC_STATE, LOW);
          eoc_counter = current_time;
          no_more_bursts = HIGH;
          burst_started = LOW;
          read_cycle();
          /// try to mantain proportional difference between ping and trigger, with external clock and cycle, and the clock changes

          if (master_clock_temp != master_clock) {
            trigger_difference = (float)master_clock_temp / trigger_dif_proportional;
          }


          for (int i = 0 ; i < 4 ; i++) {
            digitalWrite(led_pin[i], bitRead(repetitions - 1, i));
          }
        }
      }
    }
  }




  else {                                                                    ////// CYCLE OFF

    if (current_time >=  eoc_counter + 90) {
      digitalWrite(EOC_LED, LOW);
      digitalWrite(EOC_STATE, HIGH);
    }

    if ((current_time >= led_quantity_time + 250) && (no_more_bursts)) {
      for (int i = 0 ; i < 4 ; i++) {
        digitalWrite(led_pin[i], bitRead(repetition_counter, i));
      }
    }


    // pulse down
    if ( (output_state == HIGH) && (burst_started == HIGH) ) {
      if ( current_time >=  elapsed_time_since_prev_repetition_old + 2) {
        output_state = !output_state;
        digitalWrite(OUT_STATE, !(output_state * no_more_bursts));
        digitalWrite(OUT_LED, (output_state * no_more_bursts));
        random_dif = random_pot - random(1023);
        if (( first_burst == HIGH ) && (random_dif <= 0) && trigger_button_state) {
          no_more_bursts = LOW;
        }
        first_burst = LOW;
      }
    }

    // pulse up - burst time
    if ( (output_state == LOW) && (burst_started == HIGH) ) {
      if ( current_time >= (burst_time_start + elapsed_time_since_prev_repetition + burst_time_accu) ) {
        resync = HIGH;
        if (repetition_counter < repetitions - 1) {
          output_state = !output_state;
          digitalWrite(OUT_STATE, !(output_state * no_more_bursts));
          digitalWrite(OUT_LED, (output_state * no_more_bursts));
          elapsed_time_since_prev_repetition_old = current_time;
          burst_time_accu += elapsed_time_since_prev_repetition;
          repetition_counter++;

          switch ( distribution_sign ) {
            case 1:
              elapsed_time_since_prev_repetition_old = elapsed_time_since_prev_repetition_new;
              elapsed_time_since_prev_repetition_new = fscale( 0, clock_divided, 0, clock_divided, time_portions * (repetition_counter + 1), distribution);
              elapsed_time_since_prev_repetition = elapsed_time_since_prev_repetition_new - elapsed_time_since_prev_repetition_old;
              break;
            case 0:
              //if (repetition_counter == repetitions - 1 ) {
              elapsed_time_since_prev_repetition_old = elapsed_time_since_prev_repetition_new;
              elapsed_time_since_prev_repetition_new = fscale( 0, clock_divided, 0, clock_divided, time_portions * (repetitions - repetition_counter - 2), distribution);
              elapsed_time_since_prev_repetition = elapsed_time_since_prev_repetition_old - elapsed_time_since_prev_repetition_new;
              break;
            case 2:
              elapsed_time_since_prev_repetition_old = elapsed_time_since_prev_repetition_new;
              elapsed_time_since_prev_repetition_new = time_portions *  (repetition_counter + 1);
              elapsed_time_since_prev_repetition = elapsed_time_since_prev_repetition_new - elapsed_time_since_prev_repetition_old;
              break;
          }
        }
        else {
          digitalWrite(EOC_LED, HIGH);
          digitalWrite(EOC_STATE, LOW);
          eoc_counter = current_time;
          no_more_bursts = HIGH;
          burst_started = LOW;
          for (int i = 0 ; i < 4 ; i++) {
            digitalWrite(led_pin[i], bitRead(repetitions - 1, i));
          }
          //repetition_counter = repetitions - 1 ;
        }
      }
    }
  }
}
