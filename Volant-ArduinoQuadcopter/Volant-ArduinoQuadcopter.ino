#include <Wire.h> //Include the Wire.h library so we can communicate with the gyro.

/////////////////////////////////
//PID gain and limit settings
/////////////////////////////////
float pid_p_gain_roll = 0.95;               //Gain setting for the roll P-controller
float pid_i_gain_roll = 0.03;              //Gain setting for the roll I-controller
float pid_d_gain_roll = 17.0;                //Gain setting for the roll D-controller
int pid_max_roll = 400;                    //Maximum output of the PID-controller (+/-)

float pid_p_gain_pitch = pid_p_gain_roll;  //Gain setting for the pitch P-controller
float pid_i_gain_pitch = pid_i_gain_roll;  //Gain setting for the pitch I-controller
float pid_d_gain_pitch = pid_d_gain_roll;  //Gain setting for the pitch D-controller
int pid_max_pitch = pid_max_roll;          //Maximum output of the PID-controller (+/-)

float pid_p_gain_yaw = 3.0;                //Gain setting for the pitch P-controller
float pid_i_gain_yaw = 0.02;               //Gain setting for the pitch I-controller
float pid_d_gain_yaw = 0.0;                //Gain setting for the pitch D-controller
int pid_max_yaw = 400;                     //Maximum output of the PID-controller (+/-)

/////////////////////////////////
//Declaring Variables
/////////////////////////////////
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;
int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;
int counter_channel_1, counter_channel_2, counter_channel_3, counter_channel_4, loop_counter;
int esc_1, esc_2, esc_3, esc_4;
int throttle, battery_voltage;
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer;
unsigned long timer_1, timer_2, timer_3, timer_4, current_time;
int cal_int, start;
unsigned long loop_timer;
double gyro_pitch, gyro_roll, gyro_yaw;
double gyro_roll_cal, gyro_pitch_cal, gyro_yaw_cal;
byte highByte, lowByte;

float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;
float                 x_gyro = 0;
float                 y_gyro = 0;
float                 z_gyro = 0;

float gyro[3];
float gyroScaleFactor = radians(1000.0 / 32768.0);
//float gyroScaleFactor = (0.0174532 / 16.4);


uint16_t sensors_detected = 0x00;

uint8_t gyroSamples = 0;

int16_t gyroRaw[3];
float gyroSum[3];

int16_t gyro_offset[3];
float gyro_x_cal=0.0;
float gyro_y_cal=0.0;
float gyro_z_cal=0.0;



/////////////////////////////////
//Defining Variables
/////////////////////////////////
#define XAXIS 0
#define YAXIS 1
#define ZAXIS 2

#define MPU6050_ADDRESS         0x68
#define MPUREG_WHOAMI           0x75
#define MPUREG_SMPLRT_DIV       0x19
#define MPUREG_CONFIG           0x1A
#define MPUREG_GYRO_CONFIG      0x1B
#define MPUREG_ACCEL_CONFIG     0x1C
#define MPUREG_FIFO_EN          0x23
#define MPUREG_INT_PIN_CFG      0x37
#define MPUREG_INT_ENABLE       0x38
#define MPUREG_INT_STATUS       0x3A
#define MPUREG_ACCEL_XOUT_H     0x3B
#define MPUREG_ACCEL_XOUT_L     0x3C
#define MPUREG_ACCEL_YOUT_H     0x3D
#define MPUREG_ACCEL_YOUT_L     0x3E
#define MPUREG_ACCEL_ZOUT_H     0x3F
#define MPUREG_ACCEL_ZOUT_L     0x40
#define MPUREG_TEMP_OUT_H       0x41
#define MPUREG_TEMP_OUT_L       0x42
#define MPUREG_GYRO_XOUT_H      0x43
#define MPUREG_GYRO_XOUT_L      0x44
#define MPUREG_GYRO_YOUT_H      0x45
#define MPUREG_GYRO_YOUT_L      0x46
#define MPUREG_GYRO_ZOUT_H      0x47
#define MPUREG_GYRO_ZOUT_L      0x48
#define MPUREG_USER_CTRL        0x6A
#define MPUREG_PWR_MGMT_1       0x6B
#define MPUREG_PWR_MGMT_2       0x6C
#define MPUREG_FIFO_COUNTH      0x72
#define MPUREG_FIFO_COUNTL      0x73
#define MPUREG_FIFO_R_W         0x74


// Configuration bits
#define BIT_SLEEP               0x40
#define BIT_H_RESET             0x80
#define BITS_CLKSEL             0x07
#define MPU_CLK_SEL_PLLGYROX    0x01
#define MPU_CLK_SEL_PLLGYROZ    0x03
#define MPU_EXT_SYNC_GYROX      0x02
#define BITS_FS_250DPS          0x00
#define BITS_FS_500DPS          0x08
#define BITS_FS_1000DPS         0x10
#define BITS_FS_2000DPS         0x18
#define BITS_FS_MASK            0x18
#define BITS_DLPF_CFG_256HZ_NOLPF2  0x00
#define BITS_DLPF_CFG_188HZ         0x01
#define BITS_DLPF_CFG_98HZ          0x02
#define BITS_DLPF_CFG_42HZ          0x03
#define BITS_DLPF_CFG_20HZ          0x04
#define BITS_DLPF_CFG_10HZ          0x05
#define BITS_DLPF_CFG_5HZ           0x06
#define BITS_DLPF_CFG_2100HZ_NOLPF  0x07
#define BITS_DLPF_CFG_MASK          0x07
#define BIT_INT_ANYRD_2CLEAR    0x10
#define BIT_RAW_RDY_EN          0x01
#define BIT_I2C_IF_DIS          0x10
#define BIT_INT_STATUS_DATA     0x01

#define pi 3.14159 
#define RAD_TO_DEG 57.295779513082320876798154814105

/////////////////////////////////
//Setup routine
/////////////////////////////////
void setup(){
  
  DDRD |= B11110000;                                           //Configure digital port 4, 5, 6 and 7 as output.
  DDRB |= B00110000;                                           //Configure digital port 12 and 13 as output.
  //Arduino (Atmega) pins default to inputs, so they don't need to be explicitly declared as inputs.
  
  Wire.begin();
  
  //Use the led on the Arduino for startup indication
  digitalWrite(13,HIGH);                                       //Turn on the warning led.
  delay(3000);                                                 //Wait 2 second befor continuing.
  
  Serial.begin(115200);   

  // I2C bus hardware specific settings
  #if defined(__MK20DX128__)
      I2C0_F = 0x00; // 2.4 MHz (prescaler 20)
      I2C0_FLT = 4;
  #endif
      
  #if defined(__AVR__)
      TWBR = 12; // 400 KHz (maximum supported frequency)
  #endif   
  
  mpu6050_initialize();    

  delay(20);
 
 //Let's take multiple samples so we can determine the average gyro offset
  Serial.print("Starting calibration...");           //Print message 
   for (int cal_int = 0 ; cal_int <= 100 ; cal_int++){
    gyro_signalen();
    gyro_roll = (gyroRaw[XAXIS]*gyroScaleFactor)*RAD_TO_DEG;
    gyro_pitch = (gyroRaw[YAXIS]*gyroScaleFactor)*RAD_TO_DEG;
    gyro_yaw = (gyroRaw[ZAXIS]*gyroScaleFactor)*RAD_TO_DEG;
     
    gyro_x_cal += gyro_roll;
    gyro_y_cal += gyro_pitch;
    gyro_z_cal += gyro_yaw;
     
    if(cal_int%10 == 0)Serial.print(".");           //Print a dot every 100 readings
     
     digitalWrite(13, LOW);
     delay(20);
     digitalWrite(13, HIGH);
     delay(20);
   } 
   //Now that we have 2000 measures, we need to devide by 2000 to get the average gyro offset
  Serial.println(" done!");                          //2000 measures are done!
   
   gyro_x_cal = gyro_x_cal/100.0;
   gyro_y_cal = gyro_y_cal/100.0;
   gyro_z_cal = gyro_z_cal/100.0;  
   

    Serial.print("gyro_x_cal:");Serial.print(gyro_x_cal);Serial.print("\t");
    Serial.print("gyro_y_cal:");Serial.print(gyro_y_cal);Serial.print("\t");
    Serial.print("gyro_z_cal:");Serial.print(gyro_z_cal);Serial.print("\t");    
    delay(200);

  
  PCICR  |= (1 << PCIE0);                                      //Set PCIE0 to enable PCMSK0 scan.
  PCMSK0 |= (1 << PCINT0);                                     //Set PCINT0 (digital input 8) to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT1);                                     //Set PCINT1 (digital input 9)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT2);                                     //Set PCINT2 (digital input 10)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT3);                                     //Set PCINT3 (digital input 11)to trigger an interrupt on state change.

  //Wait until the receiver is active and the throtle is set to the lower position.
   while(receiver_input_channel_3 < 990 || receiver_input_channel_3 > 1020 || receiver_input_channel_4 < 1400){
    start ++;                                                  //While waiting increment start whith every loop.
    //We don't want the esc's to be beeping annoyingly. So let's give them a 1000us puls while waiting for the receiver inputs.
    PORTD |= B11110000;                                        //Set digital poort 4, 5, 6 and 7 high.
    delayMicroseconds(1000);                                   //Wait 1000us.
    PORTD &= B00001111;                                        //Set digital poort 4, 5, 6 and 7 low.
    delay(3);                                                  //Wait 3 milliseconds before the next loop.
    if(start == 125){                                          //Every 125 loops (500ms).
      digitalWrite(13, !digitalRead(13));                      //Change the led status.
      start = 0;                                               //Start again at 0.
    }
  }
  start = 0;                                                   //Set start back to 0.
  
  //Load the battery voltage to the battery_voltage variable.
  //65 is the voltage compensation for the diode.
  //12.6V equals ~5V @ Analog 0.
  //12.6V equals 1023 analogRead(0).
  //1260 / 1023 = 1.2317.
  //The variable battery_voltage holds 1050 if the battery voltage is 10.5V.
  battery_voltage = (analogRead(0) + 65) * 1.2317;
  
  
  //When everything is done, turn off the led.
  digitalWrite(13,LOW);                                        //Turn off the warning led.
}
/////////////////////////////////
//Main program loop
/////////////////////////////////
void loop(){
  //Let's get the current gyro data and scale it to degrees per second for the pid calculations.
  gyro_signalen();
  
    gyro_roll = (gyroRaw[XAXIS]*gyroScaleFactor)*RAD_TO_DEG-gyro_x_cal;
    gyro_pitch = ((gyroRaw[YAXIS]*gyroScaleFactor)*RAD_TO_DEG-gyro_y_cal)*-1;
    gyro_yaw = ((gyroRaw[ZAXIS]*gyroScaleFactor)*RAD_TO_DEG-gyro_z_cal)*-1;
    
    gyro_roll_input = (gyro_roll_input * 0.8) + ((gyro_roll) * 0.2);            //Gyro pid input is deg/sec.
    gyro_pitch_input = (gyro_pitch_input * 0.8) + ((gyro_pitch) * 0.2);         //Gyro pid input is deg/sec.
    gyro_yaw_input = (gyro_yaw_input * 0.8) + ((gyro_yaw) * 0.2);               //Gyro pid input is deg/sec.
    
         //   Serial.print(gyro_roll_input);Serial.print("\t");
         //   Serial.print(gyro_pitch_input);Serial.print("\t");
         //   Serial.print(gyro_yaw_input);Serial.print("\t");  
         //   Serial.print("\n");  

  
  
  //For starting the motors: throttle low and yaw left (step 1).
  if(receiver_input_channel_3 < 1050 && receiver_input_channel_4 < 1150)start = 1;
  //When yaw stick is back in the center position start the motors (step 2).
  if(start == 1 && receiver_input_channel_3 < 1050 && receiver_input_channel_4 > 1450){
    start = 2;
    //Reset the pid controllers for a bumpless start.
    pid_i_mem_roll = 0;
    pid_last_roll_d_error = 0;
    pid_i_mem_pitch = 0;
    pid_last_pitch_d_error = 0;
    pid_i_mem_yaw = 0;
    pid_last_yaw_d_error = 0;
  }
  //Stopping the motors: throttle low and yaw right.
  if(start == 2 && receiver_input_channel_3 < 1050 && receiver_input_channel_4 > 1750)start = 0;
  
  //The PID set point in degrees per second is determined by the roll receiver input.
  //In the case of deviding by 3 the max roll rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_roll_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if(receiver_input_channel_1 > 1510)pid_roll_setpoint = (receiver_input_channel_1 - 1510)/4.0;
  else if(receiver_input_channel_1 < 1490)pid_roll_setpoint = (receiver_input_channel_1 - 1490)/4.0;
  
  //The PID set point in degrees per second is determined by the pitch receiver input.
  //In the case of deviding by 3 the max pitch rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_pitch_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if(receiver_input_channel_2 > 1510)pid_pitch_setpoint = (receiver_input_channel_2 - 1510)/4.0;
  else if(receiver_input_channel_2 < 1490)pid_pitch_setpoint = (receiver_input_channel_2 - 1490)/4.0;
  
  //The PID set point in degrees per second is determined by the yaw receiver input.
  //In the case of deviding by 3 the max yaw rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_yaw_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if(receiver_input_channel_3 > 1050){ //Do not yaw when turning off the motors.
    if(receiver_input_channel_4 > 1510)pid_yaw_setpoint = (receiver_input_channel_4 - 1510)/4.0;
    else if(receiver_input_channel_4 < 1490)pid_yaw_setpoint = (receiver_input_channel_4 - 1490)/4.0;
  }
  //PID inputs are known. So we can calculate the pid output.
  calculate_pid();
  
  //The battery voltage is needed for compensation.
  //A complementary filter is used to reduce noise.
  //0.09853 = 0.08 * 1.2317.
  battery_voltage = battery_voltage * 0.92 + (analogRead(0) + 65) * 0.09853;
  
  //Turn on the led if battery voltage is to low.
  if(battery_voltage < 1050 && battery_voltage > 600)digitalWrite(13, HIGH);
  
 
  throttle = receiver_input_channel_3;                                      //We need the throttle signal as a base signal.
  
  if (start == 2){                                                          //The motors are started.
    if (throttle > 1800) throttle = 1800;                                   //We need some room to keep full control at full throttle.
    esc_1 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 1 (front-right - CCW)
    esc_2 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 2 (rear-right - CW)
    esc_3 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 3 (rear-left - CCW)
    esc_4 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 4 (front-left - CW)

    if (battery_voltage < 1240 && battery_voltage > 800){                   //Is the battery connected?
      esc_1 += esc_1 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-1 pulse for voltage drop.
      esc_2 += esc_2 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-2 pulse for voltage drop.
      esc_3 += esc_3 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-3 pulse for voltage drop.
      esc_4 += esc_4 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-4 pulse for voltage drop.
    } 
    
    if (esc_1 < 1200) esc_1 = 1200;                                         //Keep the motors running.
    if (esc_2 < 1200) esc_2 = 1200;                                         //Keep the motors running.
    if (esc_3 < 1200) esc_3 = 1200;                                         //Keep the motors running.
    if (esc_4 < 1200) esc_4 = 1200;                                         //Keep the motors running.
    
    if(esc_1 > 2000)esc_1 = 2000;                                           //Limit the esc-1 pulse to 2000us.
    if(esc_2 > 2000)esc_2 = 2000;                                           //Limit the esc-2 pulse to 2000us.
    if(esc_3 > 2000)esc_3 = 2000;                                           //Limit the esc-3 pulse to 2000us.
    if(esc_4 > 2000)esc_4 = 2000;                                           //Limit the esc-4 pulse to 2000us.  
  }
  
  else{
    esc_1 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-1.
    esc_2 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-2.
    esc_3 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-3.
    esc_4 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-4.
  }
  
  //All the information for controlling the motor's is available.
  //The refresh rate is 250Hz. That means the esc's need there pulse every 4ms.
  while(micros() - loop_timer < 4000);                                      //We wait until 4000us are passed.
  loop_timer = micros();                                                    //Set the timer for the next loop.

  PORTD |= B11110000;                                                       //Set digital outputs 4,5,6 and 7 high.
  timer_channel_1 = esc_1 + loop_timer;                                     //Calculate the time of the faling edge of the esc-1 pulse.
  timer_channel_2 = esc_2 + loop_timer;                                     //Calculate the time of the faling edge of the esc-2 pulse.
  timer_channel_3 = esc_3 + loop_timer;                                     //Calculate the time of the faling edge of the esc-3 pulse.
  timer_channel_4 = esc_4 + loop_timer;                                     //Calculate the time of the faling edge of the esc-4 pulse.
  
  while(PORTD >= 16){                                                       //Stay in this loop until output 4,5,6 and 7 are low.
    esc_loop_timer = micros();                                              //Read the current time.
    if(timer_channel_1 <= esc_loop_timer)PORTD &= B11101111;                //Set digital output 4 to low if the time is expired.
    if(timer_channel_2 <= esc_loop_timer)PORTD &= B11011111;                //Set digital output 5 to low if the time is expired.
    if(timer_channel_3 <= esc_loop_timer)PORTD &= B10111111;                //Set digital output 6 to low if the time is expired.
    if(timer_channel_4 <= esc_loop_timer)PORTD &= B01111111;                //Set digital output 7 to low if the time is expired.
  }
}

/////////////////////////////////
//This routine is called every time input 8, 9, 10 or 11 changed state
/////////////////////////////////
ISR(PCINT0_vect){
  current_time = micros();
  //Channel 1=========================================
  if(PINB & B00000001){                                        //Is input 8 high?
    if(last_channel_1 == 0){                                   //Input 8 changed from 0 to 1
      last_channel_1 = 1;                                      //Remember current input state
      timer_1 = current_time;                                  //Set timer_1 to current_time
    }
  }
  else if(last_channel_1 == 1){                                //Input 8 is not high and changed from 1 to 0
    last_channel_1 = 0;                                        //Remember current input state
    receiver_input_channel_1 = current_time - timer_1;         //Channel 1 is current_time - timer_1
  }
  //Channel 2=========================================
  if(PINB & B00000010 ){                                       //Is input 9 high?
    if(last_channel_2 == 0){                                   //Input 9 changed from 0 to 1
      last_channel_2 = 1;                                      //Remember current input state
      timer_2 = current_time;                                  //Set timer_2 to current_time
    }
  }
  else if(last_channel_2 == 1){                                //Input 9 is not high and changed from 1 to 0
    last_channel_2 = 0;                                        //Remember current input state
    receiver_input_channel_2 = current_time - timer_2;         //Channel 2 is current_time - timer_2
  }
  //Channel 3=========================================
  if(PINB & B00000100 ){                                       //Is input 10 high?
    if(last_channel_3 == 0){                                   //Input 10 changed from 0 to 1
      last_channel_3 = 1;                                      //Remember current input state
      timer_3 = current_time;                                  //Set timer_3 to current_time
    }
  }
  else if(last_channel_3 == 1){                                //Input 10 is not high and changed from 1 to 0
    last_channel_3 = 0;                                        //Remember current input state
    receiver_input_channel_3 = current_time - timer_3;         //Channel 3 is current_time - timer_3

  }
  //Channel 4=========================================
  if(PINB & B00001000 ){                                       //Is input 11 high?
    if(last_channel_4 == 0){                                   //Input 11 changed from 0 to 1
      last_channel_4 = 1;                                      //Remember current input state
      timer_4 = current_time;                                  //Set timer_4 to current_time
    }
  }
  else if(last_channel_4 == 1){                                //Input 11 is not high and changed from 1 to 0
    last_channel_4 = 0;                                        //Remember current input state
    receiver_input_channel_4 = current_time - timer_4;         //Channel 4 is current_time - timer_4
  }
}

/////////////////////////////////
//Subroutine for reading the gyro
/////////////////////////////////
void gyro_signalen()
{
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(MPUREG_GYRO_XOUT_H);
    Wire.endTransmission();
    
    Wire.requestFrom(MPU6050_ADDRESS, 6);
    while(Wire.available() < 6);                       //Wait until the 6 bytes are received
    
    gyroRaw[XAXIS] = ((Wire.read() << 8) | Wire.read());
    gyroRaw[YAXIS] = ((Wire.read() << 8) | Wire.read());
    gyroRaw[ZAXIS] = ((Wire.read() << 8) | Wire.read());
 
}

/////////////////////////////////
//Subroutine for calculating pid outputs
/////////////////////////////////

void calculate_pid(){
  //Roll calculations
  pid_error_temp = gyro_roll_input - pid_roll_setpoint;
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
  if(pid_i_mem_roll > pid_max_roll)pid_i_mem_roll = pid_max_roll;
  else if(pid_i_mem_roll < pid_max_roll * -1)pid_i_mem_roll = pid_max_roll * -1;
  
  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  if(pid_output_roll > pid_max_roll)pid_output_roll = pid_max_roll;
  else if(pid_output_roll < pid_max_roll * -1)pid_output_roll = pid_max_roll * -1;
  
  pid_last_roll_d_error = pid_error_temp;
  
  //Pitch calculations
  pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  if(pid_i_mem_pitch > pid_max_pitch)pid_i_mem_pitch = pid_max_pitch;
  else if(pid_i_mem_pitch < pid_max_pitch * -1)pid_i_mem_pitch = pid_max_pitch * -1;
  
  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  if(pid_output_pitch > pid_max_pitch)pid_output_pitch = pid_max_pitch;
  else if(pid_output_pitch < pid_max_pitch * -1)pid_output_pitch = pid_max_pitch * -1;
    
  pid_last_pitch_d_error = pid_error_temp;
    
  //Yaw calculations
  pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
  if(pid_i_mem_yaw > pid_max_yaw)pid_i_mem_yaw = pid_max_yaw;
  else if(pid_i_mem_yaw < pid_max_yaw * -1)pid_i_mem_yaw = pid_max_yaw * -1;
  
  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  if(pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;
  else if(pid_output_yaw < pid_max_yaw * -1)pid_output_yaw = pid_max_yaw * -1;
    
  pid_last_yaw_d_error = pid_error_temp;
}

/////////////////////////////////
//Gyro Initilization
/////////////////////////////////

void mpu6050_initialize()
{
    // Chip reset
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(MPUREG_PWR_MGMT_1);
    Wire.write(BIT_H_RESET);
    Wire.endTransmission();  
       
    
    // Startup delay 
    delay(100);  
    
    // Check if sensor is alive
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(MPUREG_WHOAMI);
    Wire.endTransmission();
    
    Wire.requestFrom(MPU6050_ADDRESS, 1);
    
    uint8_t register_value = Wire.read();
    
//    if (register_value == 0x68) {
//        sensors_detected |= GYROSCOPE_DETECTED;
//        sensors_detected |= ACCELEROMETER_DETECTED;
//    } else {
//        return;
//    }   
    
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(MPUREG_INT_PIN_CFG);
    Wire.write(0x02);
    Wire.endTransmission();       

    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(MPUREG_PWR_MGMT_1);
    Wire.write(MPU_CLK_SEL_PLLGYROZ);
    Wire.endTransmission();   

    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(MPUREG_PWR_MGMT_2);
    Wire.write(0);
    Wire.endTransmission();   

    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(MPUREG_SMPLRT_DIV);
    Wire.write(0x00);
    Wire.endTransmission();   

    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(MPUREG_CONFIG);
    Wire.write(BITS_DLPF_CFG_42HZ);
    Wire.endTransmission();   

    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(MPUREG_GYRO_CONFIG);
    Wire.write(BITS_FS_1000DPS);
    Wire.endTransmission();   

    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(MPUREG_ACCEL_CONFIG);
    Wire.write(0x08);
    Wire.endTransmission();    
 
    delay(1500);   
}



