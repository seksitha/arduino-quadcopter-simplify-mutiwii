
// #include <InterruptHandler.h>   <-- You may need this on some versions of Arduino
// Initialize a PPMReader on digital pin 3 with 6 expected channels.


#define IBUS_BUFFSIZE 32 // Max iBus packet size (2 byte header, 14 channels x 2 bytes, 2 byte checksum)
#define IBUS_MAXCHANNELS 6
static uint8_t ibusIndex = 0;
static uint8_t ibus[IBUS_BUFFSIZE] = {0};
static uint16_t rcValue[IBUS_MAXCHANNELS];
boolean rxFrameDone = false;
int serialInput; // without assing it is 0 by default
int setMotorTorun = 0, start = 0; // without assing it is 0 by default
unsigned long loop_timer; // without assing it is 0 by default
int esc_1, esc_2, esc_3, esc_4; // without assing it is 0 by default
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer;
unsigned long valueThrottle, valuePitch, valueRoll, valueYaw, valueAux1, valueAux2;
boolean readingRx = false;



void setup() {
  Serial.begin(115200);
  DDRD |= B10001000;      //Configure digital port D pin 3 as output.
  DDRB |= B00111110;      // pin 9 10 11 12 13
  loop_timer = micros();
}

void loop() {
  loop_timer = micros();
  if (readingRx == false) {
    readingRx == true;
    while (micros() - loop_timer < 650) {   

      if (Serial.available()) {
        uint8_t val = Serial.read();
        uint16_t chksum, rxsum;
        uint8_t i;

        // Look for 0x2040 as start of packet
        if (ibusIndex == 0 && val != 0x20) {
          return;
        }
        if (ibusIndex == 1 && val != 0x40) {
          ibusIndex = 0;
          return;
        }

        if (ibusIndex < IBUS_BUFFSIZE) {
          ibus[ibusIndex] = val;
          ibusIndex++;

        }

        if (ibusIndex == IBUS_BUFFSIZE) {
          ibusIndex = 0;
          chksum = 0xFFFF;
          for (i = 0; i < 30; i++)
            chksum -= ibus[i];
          rxsum = ibus[30] + (ibus[31] << 8);

          if (chksum == rxsum) {
            rcValue[0] = (ibus[3] << 8) + ibus[2];
            rcValue[1] = (ibus[5] << 8) + ibus[4];
            rcValue[2] = (ibus[7] << 8) + ibus[6];
            rcValue[3] = (ibus[9] << 8) + ibus[8];
            rcValue[4] = (ibus[11] << 8) + ibus[10];
            rcValue[5] = (ibus[13] << 8) + ibus[12];
            readingRx = false;
          }
        }
      }
      if (readingRx == false) {
//        PORTD ^= (1 << 7);
        valueRoll = rcValue[0];     // 2 is the channel
        valuePitch = rcValue[1];    // 1 is the channel
        valueThrottle = rcValue[2]; // 3 is the channel
        valueYaw = rcValue[3];      // 4 is the channel
        valueAux1 = rcValue[4];     // 5 is the channel
        valueAux2 = rcValue[5];     // 5 is the channel
      }
    }

  }
  readingRx = false;
//  if(micros()-loop_timer>700) PORTD ^= (1 << 7);
  while(micros()-loop_timer<700); 
 while(micros()-loop_timer<700);
  loop_timer = micros();
  PORTD |= B00001000;                                                       //Set digital outputs 3 high.
  PORTB |= B00001110;


  // Yaw right to start the first motor
  if (valueThrottle < 1020 && valueYaw > 1800 && start == 0) {
    start = 1;
    PORTB |= B00100000;
  }

  // yaw left to disarm all 
  if (valueThrottle < 1020 && valueYaw < 1020 && start >= 1 && valueAux1 < 1020) {
    start = 0;
    setMotorTorun = 0;
    PORTB &= B11011111;
  }

  if (start == 1 && valueAux1 < 1020 ) {
    setMotorTorun = 1 ;
    start = 2;
  }

  if (start == 2 && valueAux1 > 1800 ) {
    setMotorTorun = 2;
    start = 3;
  }

  if (start == 3 && valueAux1 > 1800  && valueAux2 > 1200 && valueAux2 < 1800 ) {
    setMotorTorun = 3;
    start = 4;
  }
  if (start == 4 && valueAux1 > 1800 && valueAux2 > 1900 && valueAux2 < 2020 ) {
    setMotorTorun = 4;
    start = 5;
  }

  if (start == 5 && valueAux1 > 1800 && valueAux2 > 1900 && valueYaw < 1020) {
    setMotorTorun = 5;
    start = 6;
  }
  //  Serial.println(setMotorTorun);
  if (setMotorTorun == 0 ) {
    esc_1 = 1000;
    esc_2 = 1000;
    esc_3 = 1000;
    esc_4 = 1000;
    //    PORTB |= B00100000;
  }
  else {
    if (setMotorTorun == 1 || setMotorTorun == 5) esc_1 = valueThrottle;
    if (setMotorTorun == 2 || setMotorTorun == 5) esc_2 = valueThrottle;
    if (setMotorTorun == 3 || setMotorTorun == 5) esc_3 = valueThrottle;
    if (setMotorTorun == 4 || setMotorTorun == 5) esc_4 = valueThrottle;
    //    PORTB &= B11011111;

  }



  if (micros() - loop_timer > 950) PORTD |= B10000000;
  if (micros() - loop_timer <= 950) PORTD &= B01111111;
  while (micros() - loop_timer < 1000);  // delay untill 1000us
  loop_timer = micros();
  while ( start > 0 && (PIND & B00001000 || PINB & B00001110)) {                       //Stay in this loop until output all port are low number is bite
    esc_loop_timer = micros();
    if (esc_loop_timer - loop_timer >= esc_1 - 1000) PORTB &= B11111011;               //Set digital output 10 to low if the time is expired.
    if (esc_loop_timer - loop_timer >= esc_2 - 1000) PORTD &= B11110111;               //Set digital output  3 to low if the time is expired.
    if (esc_loop_timer - loop_timer >= esc_3 - 1000) PORTB &= B11110111;               //Set digital output 11 to low if the time is expired.
    if (esc_loop_timer - loop_timer >= esc_4 - 1000) PORTB &= B11111101;               // pin 9
    //    Serial.println(timer_channel_1 - esc_loop_timer );
  }
  loop_timer = micros();
  while (micros() - loop_timer < 3000);

}




