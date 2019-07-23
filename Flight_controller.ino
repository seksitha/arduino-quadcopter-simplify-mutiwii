#include <Wire.h> //Include the Wire.h library so we can communicate with the gyro.
#include <EEPROM.h>
//#include <SoftwareSerial.h>
#include <string.h>
#include "I2Cdev.h"
#include "MPU6050.h"

#define IBUS_BUFFSIZE 32 // Max iBus packet size (2 byte header, 14 channels x 2 bytes, 2 byte checksum)
#define IBUS_MAXCHANNELS 6
static uint8_t ibusIndex = 0;
static uint8_t ibus[IBUS_BUFFSIZE] = {0};
static uint16_t rcValue[IBUS_MAXCHANNELS];
boolean rxFrameDone = false;
//SoftwareSerial mySerial(8, 12);
uint8_t p_light = 5;
uint8_t d_light = 1;
uint8_t i_light = 1;
uint16_t p_light_k, d_light_k, i_light_k, button_pushed;
int p_mul = 40, i_mul = 25, d_mul = 25;

int valueThrottle, valuePitch, valueRoll, valueYaw, valueAux1, valueAux2;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//PID gain and limit settings
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
uint8_t pid_p_gain_roll = 0; //Gain setting for the roll P-controller (1.3)
uint8_t pid_i_gain_roll = 0; //Gain setting for the roll I-controller (0.3)
uint8_t pid_d_gain_roll = 0; //Gain setting for the roll D-controller (15)
// int pid_max_roll = 400;      //Maximum output of the PID-controller (+/-)

// float pid_p_gain_pitch = pid_p_gain_roll; //Gain setting for the pitch P-controller.
// float pid_i_gain_pitch = pid_i_gain_roll; //Gain setting for the pitch I-controller.
// float pid_d_gain_pitch = pid_d_gain_roll; //Gain setting for the pitch D-controller.
// int pid_max_pitch = pid_max_roll;         //Maximum output of the PID-controller (+/-)

uint8_t pid_p_gain_yaw = 0; //Gain setting for the pitch P-controller. //4.0
uint8_t pid_i_gain_yaw = 0; //Gain setting for the pitch I-controller. //0.02
uint8_t pid_d_gain_yaw = 0; //Gain setting for the pitch D-controller.
// int pid_max_yaw = 400;      //Maximum output of the PID-controller (+/-)

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Declaring Variables
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int16_t Pterm = 0, Iterm = 0, Dterm, delta, axis;
int16_t gyroInput[2], pidConfig[2], lastgyro[2], delta1[2], delta2[2], rc_command[2], pidOutput[2];
// byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;
// int counter_channel_1, counter_channel_2, counter_channel_3, counter_channel_4, loop_counter;
int esc_1 = 1000, esc_2 = 1000, esc_3 = 1000, esc_4 = 1000;
int throttle, battery_voltage;
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer;
unsigned long timer_1, timer_2, timer_3, timer_4, current_time;
int cal_int, start;
unsigned long loop_timer;
int16_t gyro_pitch, gyro_roll, gyro_yaw, acc_pitch, acc_roll, acc_yaw, temperature;
//double gyro_roll_cal , gyro_pitch_cal, gyro_yaw_cal;
int32_t gyro_roll_cal = 0, gyro_pitch_cal = 0, gyro_yaw_cal = 0;
byte highByte, lowByte;

int16_t pid_error_temp;

int16_t pid_i_error[2], pid_roll_command, gyro_roll_meter, pid_output_roll, pid_last_roll_d_error;
int16_t pid_pitch_command, gyro_pitch_meter, pid_output_pitch, pid_last_pitch_d_error;
int16_t pid_i_mem_yaw, rc_yaw_command, gyro_yaw_meter, pid_output_yaw, pid_last_yaw_d_error;

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;
boolean readingRx = false;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Setup routine
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup()
{
  //  mySerial.begin(115200);
  Serial.begin(115200);
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000);
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
  accelgyro.initialize();
  accelgyro.setDLPFMode(MPU6050_DLPF_BW_10);
  // verify connection
  pid_p_gain_roll = EEPROM.read(3);
  pid_d_gain_roll = EEPROM.read(4);
  pid_i_gain_roll = EEPROM.read(5);
  pid_p_gain_yaw = EEPROM.read(6);
  pid_d_gain_yaw = EEPROM.read(7);
  pid_i_gain_yaw = EEPROM.read(8);
  Wire.begin(); //Start the I2C as master.
  TWBR = 12;
  //
  DDRD |= B11101000; //Configure digital port D pin 6 7 as output.
  DDRB |= B00101110; // pin 9 10 11 12 13 as output

  PORTC |= B00000111; // pull the 14 15 16 hight

  //Use the led on the Arduino for startup indication
  digitalWrite(13, LOW); //Turn on the warning led.
  delay(2000);           //Wait 2 second befor continuing.

  start = 0; //Set start back to 0.
}








///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Main program loop
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop()
{
  loop_timer = micros();
  if (readingRx == false) {
    readingRx == true;
    while (micros() - loop_timer < 650) {
      if (Serial.available()) {
        uint8_t val = Serial.read(); //https://arduino.stackexchange.com/questions/10088/what-is-the-difference-between-serial-write-and-serial-print-and-when-are-they/48147#48147?newreg=054f10a9de0f4013bb1fb4ed2115e030
        uint16_t chksum, rxsum;
        uint8_t i;

        // Look for 0x2040 as start of packet
        if (ibusIndex == 0 && val != 0x20) { // 32
          return;
        }
        if (ibusIndex == 1 && val != 0x40) { // 64
          ibusIndex = 0;                                // index to 0000
          return;
        }

        if (ibusIndex < IBUS_BUFFSIZE) {
          ibus[ibusIndex] = val;
          ibusIndex++;                                  // index increment

        }

        if (ibusIndex == IBUS_BUFFSIZE) {

          ibusIndex = 0;
          chksum = 0xFFFF; // 16bits == 65535

          for (i = 0; i < 30; i++) {
            chksum -= ibus[i];
          }

          rxsum = ibus[30] + (ibus[31] << 8);

          if (chksum == rxsum) {
            rcValue[0] = (ibus[3] << 8) + ibus[2]; // shift 8 of second byte [0,1], [2,3], where second byte is mostly small number.
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
  while (micros() - loop_timer < 700);                                                      /////////////////////////// End of Rx loop //////////700//////////////////////////












  loop_timer = micros();                                                     // 1
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  gyro_roll_meter =   (gyro_roll_meter * 0.9) +   (((gy - 28  ) / 10) * 0.1); //Gyro pid meter is deg/sec.
  gyro_pitch_meter =  (gyro_pitch_meter * 0.9) +  (((gx + 343 ) / 10) * 0.1) * -1; //Gyro pid meter is deg/sec.
  gyro_yaw_meter =    (gyro_yaw_meter * 0.9) +    (((gz + 234 ) / 10) * 0.1) * -1; //Gyro pid meter is deg/sec.
  //  Serial.print("a/g:\t");
  //  Serial.print(gyro_roll_meter); Serial.print("\t");
  //  Serial.print(gyro_pitch_meter); Serial.print("\t");
  //  Serial.println(gyro_yaw_meter);

  rc_command[0] = 0; // Roll
  //We need a little dead band of 16us for better results.
  if (valueRoll > 1508 ){
    rc_command[0] = (valueRoll - 1508); // convert to degree per socond  +500 //when roll to right +degree
  } else if (valueRoll < 1492){
    rc_command[0] = (valueRoll - 1492); // - 500
  }
    

  rc_command[1] = 0; // Pitch
  //We need a little dead band of 16us for better results. prevent RX noise
  if (valuePitch > 1508 ) {
    rc_command[1] = (valuePitch - 1508);
  } else if (valuePitch < 1492) {
    rc_command[1] = (valuePitch - 1492);
  }
   

  rc_yaw_command = 0; // Yaw
  //We need a little dead band of 16us for better results.
  if (valueThrottle > 1050) // only if the throttle is hight we can yaw!!!
  { //Do not yaw when turning off the motors.
    if (valueYaw > 1508)
      rc_yaw_command = (valueYaw - 1508);
    else if (valueYaw < 1492)
      rc_yaw_command = (valueYaw - 1492);
  }

  valueThrottle = constrain(valueThrottle, 1010, 2000);

  //  Serial.print("a/g:\t");
  //  Serial.print(rc_command[0]); Serial.print("\t");
  //  Serial.print(rc_command[1]); Serial.print("\t");
  //  Serial.println(rc_yaw_command);

  gyroInput[0] = gyro_roll_meter;
  gyroInput[1] = gyro_pitch_meter;
  if (valueThrottle > 1000) {

    for (axis = 0; axis < 2; axis++) {
      // PPPPPP
      pid_error_temp = rc_command[axis] * 4  - gyroInput[axis];
      Pterm = pid_error_temp * (float) pid_p_gain_roll / 150; // https://forum.arduino.cc/index.php?topic=28742.0


      // IIIIIII
      pid_i_error[axis] = constrain(pid_i_error[axis] + pid_error_temp, -16000, 16000);
      if (abs(gyroInput[axis]) > 1500) pid_i_error[axis] = 0;
      Iterm = pid_i_error[axis] * (float)pid_i_gain_roll / 3000 ; // pid_i_error[axis] * (pid_i_gain_roll / 100);


      // DDDDDD

      delta           = gyroInput[axis] - lastgyro[axis];
      lastgyro[axis]  = gyroInput[axis];
      Dterm           = delta + delta1[axis] + delta2[axis];
      delta2[axis]    = delta1[axis];
      delta1[axis]    = delta;

      Dterm = Dterm * (float)pid_d_gain_roll / 35;

      pidOutput[axis] = Pterm + Iterm - Dterm ;
    }

    // Yaw pid calculation
    pid_error_temp  =   rc_yaw_command * 4  - gyro_yaw_meter;
    pid_i_mem_yaw   +=  pid_error_temp;
    pid_i_mem_yaw   =   constrain(pid_i_mem_yaw, -16000, 16000);
    (abs(gyro_yaw_meter) > 650) && (pid_i_mem_yaw = 0);
    // there is no d gain for yaw because no ocilation gravity
    pid_output_yaw = (pid_error_temp * (float)pid_p_gain_yaw / 150) + (pid_i_mem_yaw * (float)pid_i_gain_yaw / 3000);
  }


  //      Serial.println( (pid_i_error[0]* pid_i_gain_roll) / 200);
  (micros() - loop_timer > 1500 && start == 1) && (PORTD ^= (1 << 7));
  while (micros() - loop_timer < 1500)   ;                                           ///////////////////////////////////////End of pid loop ///////////////1500///////////////////










  //  PORTD ^= (1<<7);
  PORTD |= B00001000;    //Set digital outputs 3 high.
  PORTB |= B00001110;    // set pin 9,10,11
  loop_timer = micros(); // 1

  // Arming the quad
  if (valueThrottle < 1050 && valueAux1 > 1800 && start == 0) {
    PORTB |= B00100000;
    start = 1;
  }

  // un-arm the quad
  if (valueThrottle < 1050 && valueAux1 < 1200 && start >= 1 && valueAux2 < 1020) {
    PORTB &= B11011111;
    start = 0;
    lastgyro[0] = 0;
    lastgyro[1] = 0;
    pid_i_error[0] = 0;
    pid_i_error[1] = 0;
    pid_error_temp = 0;

    delta = 0;

    delta1[0] = 0;
    delta1[1] = 0;

    delta2[0] = 0;
    delta2[1] = 0;
  }


  // set tune variable to pitch and roll
  // if tuning is active set p i d key by button
  // throught p i d key we can set pid gain base on that.
  if (valueThrottle < 1090 && valueAux2 > 1200 ) {
    if (valueAux2 < 1600 && (start == 0 || start == 4) ) {
      start = 3;
      p_light_k = EEPROM.read(3) * p_mul; // 1*50 = 50, 2*50 = 100 ;   P : 1/10=0.1 ; 2/10=0.2;
      d_light_k = EEPROM.read(4) * d_mul; // 2*25 = 50, 4*25 = 100 ;   D : 1/10=0.1 ; 2/10=0.2;
      i_light_k = EEPROM.read(5) * i_mul; // 1*50 = 50, 2*50 = 100     I : 1/100 = 0.01

    }

    if (valueAux2 > 1700 && start == 3) {
      start = 4;
      p_light_k = EEPROM.read(6) * p_mul;
      d_light_k = EEPROM.read(7) * d_mul;
      i_light_k = EEPROM.read(8) * i_mul;
    }

  }

  //    Serial.print("\t");
  //    Serial.print(pid_p_gain_roll); Serial.print("\t");
  //    Serial.print(pid_d_gain_roll); Serial.print("\t");
  //    Serial.print(pid_i_gain_roll); Serial.print("\t");
  //    Serial.print(pid_p_gain_yaw); Serial.print("\t");
  //    Serial.print(pid_d_gain_yaw); Serial.print("\t");
  //    Serial.println(pid_i_gain_yaw);

  if (valueThrottle < 1090 && valueAux2 < 1200 && start >= 3) {
    start = 0;
    p_light_k = 0;
    d_light_k = 0;
    i_light_k = 0;

  }


  if (bitRead(PINC, 0) == 0 && button_pushed == 0)
  {
    //    PORTD ^= (1 << 7);
    (start == 3) && (pid_p_gain_roll += 1);
    (start == 4) && (pid_p_gain_yaw += 1);

    p_light_k += p_mul; // 1000/40 = 25
    if (p_light_k > 1000) {
      p_light_k = 0;
      (start == 3) && (pid_p_gain_roll = 0);
      (start == 4) && (pid_p_gain_yaw = 0);
    }

    button_pushed = 1;
  }

  if (bitRead(PINC, 2) == 0 && button_pushed == 0)
  {
    //    PORTD ^= (1 << 7);
    (start == 3) && (pid_i_gain_roll += 1);
    (start == 4) && (pid_i_gain_yaw += 1);

    i_light_k += i_mul; // 1000/40 = 25
    if (i_light_k > 1000) {
      (i_light_k = 0);
      (start == 3) && (pid_i_gain_roll = 0);
      (start == 4) && (pid_i_gain_yaw = 0);

    }

    button_pushed = 1;
  }

  if (bitRead(PINC, 1) == 0 && button_pushed == 0)
  {
    //    PORTD ^= (1 << 7);
    (start == 3) && (pid_d_gain_roll += 1);
    (start == 4) && (pid_d_gain_yaw += 1);

    d_light_k += d_mul; // 1000/25 = 40
    if (d_light_k > 1000) {
      d_light_k = 0;
      (start == 3) && (pid_d_gain_roll = 0);
      (start == 4) && (pid_d_gain_yaw = 0);

    }

    button_pushed = 1;
  }



  if (start >= 3 && valueYaw < 1090 && valuePitch < 1100 )
  {
    EEPROM.write(3, pid_p_gain_roll);
    EEPROM.write(4, pid_d_gain_roll);
    EEPROM.write(5, pid_i_gain_roll);

    EEPROM.write(6, pid_p_gain_yaw);
    EEPROM.write(7, pid_d_gain_yaw);
    EEPROM.write(8, pid_i_gain_yaw);
    start = 2;
    PORTB |= B00100000;
  }

  //////////////////////////////////End of pid command////////////////////////
  //  Serial.print(start);
  //  Serial.print(EEPROM.read(3));
  //  Serial.print(EEPROM.read(4));
  //  Serial.print(EEPROM.read(5));
  //  Serial.print(EEPROM.read(6));
  //  Serial.print(EEPROM.read(7));
  //  Serial.println(EEPROM.read(8));


  

  (micros() - loop_timer > 950 && start == 1) && (PORTD ^= (1 << 5));                    //d5
  while (micros() - loop_timer <= 1000);                                                 /////////////////////////////// End of ESC command loop ////////// 1000///////////////////////////
  // cpp.sh/9nhqh // url used to learn the code





  if (start >= 3) {
    (p_light_k > 0) && (PORTD |= B10000000);
    (d_light_k > 0) && (PORTD |= B01000000);
    (i_light_k > 0) && (PORTD |= B00100000);
    PORTD &= B11110111;                                                                 //Set digital output 10 to low if the time is expired.
    PORTB &= B11110001;                                                                 //Set digital output  3 to low if the time is expired.
    loop_timer = micros();
    while (micros() - loop_timer < 1500 ) {                                             //Stay in this loop until output all port are low number is bite
      esc_loop_timer = micros() - loop_timer;
      (esc_loop_timer >= p_light_k ) && (PORTD &= B01111111);                           // pin
      (esc_loop_timer >= d_light_k ) && (PORTD &= B10111111);                           // pin
      (esc_loop_timer >= i_light_k ) && (PORTD &= B11011111);                           // pin
    }
  }





  if (start <= 1 ) {                                                                     // TODO: this will make esc beaping when tune pid
    loop_timer = micros();                                                               // 2
    while ((PIND & B00001000 || PINB & B00001110)) {                                     //Stay in this loop until output all port are low number is bite
      esc_loop_timer = micros();
      if (esc_loop_timer - loop_timer >= esc_1 - 1000) PORTB &= B11111011;               //Set digital output 10 to low if the time is expired.
      if (esc_loop_timer - loop_timer >= esc_2 - 1000) PORTD &= B11110111;               //Set digital output  3 to low if the time is expired.
      if (esc_loop_timer - loop_timer >= esc_3 - 1000) PORTB &= B11110111;               //Set digital output 11 to low if the time is expired.
      if (esc_loop_timer - loop_timer >= esc_4 - 1000) PORTB &= B11111101;               // pin 9
    }
  }


  (micros() - loop_timer > 1600 && start == 1) && (PORTD ^= (1 << 6)); // D6            ////////////////////////////////////////End of motor loop //////////////1600//////////////////////////////

  while (micros() - loop_timer < 1600) ;
  (button_pushed >= 1) && (button_pushed += 1);
  (button_pushed > 50) && (button_pushed = 0);

// Total loop rx 700 + Gyro+pid 1500 + esc min up 1000+ esc to loow time 1600 = 4800

  

}
