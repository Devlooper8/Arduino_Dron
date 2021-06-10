//////////////////////////////////////////////////////////////////////////////////////
//Channel_1 = roll                              PIN 2                               //
//Channel_2 = pitch                             PIN 3                               //
//Channel_3 = throttle                          PIN 4                               //
//Channel_4 = yaw                               PIN 5                               //
//X axis rotation = roll                                                            //
//Y axis rotation = pitch                                                           //
//Z axis rotation = yaw                                                             //
//                                                                                  //
//                                                                                  //
//                                                                                  //
//                                                                                  //
// Gyro pinout(SPI):                                                                //
//                                                                                  //
//  5V  -> 5V                                                                       //
//  GND -> GND                                                                      //
//  SCL -> PIN 13                                                                   //
//  SDA -> PIN 11                                                                   //
//  ADO -> PIN 12                                                                   //
//  NCS -> PIN 10                                                                   //
//////////////////////////////////////////////////////////////////////////////////////


#include <MPU9250.h> //Include the MPU9250.h library so we can communicate with the gyro
//Mahony filter
#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f

unsigned long current_time, timer_1, timer_2, timer_3, timer_4,
         zero_timer, esc_loop_timer,
         timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4;
short esc_1, esc_2, esc_3, esc_4;
byte last_channels;
byte channel_1_state = 0b00000001;
byte channel_2_state = 0b00000010;
byte channel_3_state = 0b00000100;
byte channel_4_state = 0b00001000;
volatile int  receiver_input_channels[4];
byte deadband = 8;

short low_channels[4] = {1070, 1127, 1140, 1047};
short center_channels[4] = {1495, 1493, 1457, 1502};
short high_channels[4] = {1936, 1933, 1787, 1954};

int throttle;

//Mahony filter
unsigned long lastUpdate = 0; // used to calculate integration interval
unsigned long Now = 0;
float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
float eInt[3] = {0.0f, 0.0f, 0.0f};
float deltat = 0.0f;
float yaw, pitch, roll;

//MPU9250 Gyroskop
MPU9250 IMU(SPI, 10); //Setup routine


//PID gain and limit settings
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float pid_p_gain_roll = 1.3;               //Gain setting for the roll P-controller
float pid_i_gain_roll = 0.04;              //Gain setting for the roll I-controller
float pid_d_gain_roll = 18.0;              //Gain setting for the roll D-controller
int pid_max_roll = 400;                    //Maximum output of the PID-controller (+/-)

float pid_p_gain_pitch = pid_p_gain_roll;  //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = pid_i_gain_roll;  //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = pid_d_gain_roll;  //Gain setting for the pitch D-controller.
int pid_max_pitch = pid_max_roll;          //Maximum output of the PID-controller (+/-)

float pid_p_gain_yaw = 4.0;                //Gain setting for the pitch P-controller. //4.0
float pid_i_gain_yaw = 0.02;               //Gain setting for the pitch I-controller. //0.02
float pid_d_gain_yaw = 0.0;                //Gain setting for the pitch D-controller.
int pid_max_yaw = 400;                     //Maximum output of the PID-controller (+/-)

boolean auto_level = true;                 //Auto level on (true) or off (false)

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float roll_level_adjust, pitch_level_adjust;
float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;
float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll;
boolean gyro_angles_set;

float obmedzovac = 0.7;


void setup() {
  pinMode(A1, OUTPUT);
  DDRD |= 0b11000000;      //Configure digital port 6 and 7 as output.
  DDRB |= 0b00000011;      //Configure digital port 8 and 9 as output.
  //Arduino (Atmega) pins default to inputs, so they don't need to be explicitly declared as inputs
  PCICR |= 0b0000100; // set PCIE2 to enable PCMSK2 scan
  PCMSK2 |= 0b00111100; // set PCINT18, PCINT19, PCINT20, PCINT21 (digital input 2, 3, 4, 5) to trigger an interrupt on state change
  Serial.begin(57600); //Start the serial connetion @ 57600bps
  if (IMU.begin()) {
    Serial.println(F("Gyroskop funguje, nastavujem potrebne registre"));
    IMU.setAccelRange(MPU9250::ACCEL_RANGE_16G);
    IMU.setGyroRange(MPU9250::GYRO_RANGE_2000DPS);
    IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_41HZ);
    IMU.setSrd(0);
  }
  else {
    Serial.println(F("Gyroskop nefunguje, skontrolujte zapojenie"));
  }


  Serial.println(F("Nastavte max vykon, po nastaveni zadajte 1"));
  delay(5000);
  
    esc_1 = 1787    ;                                            //Set the pulse for ESC 1 to high throttle pulse.
    esc_2 = 1787   ;                                             //Set the pulse for ESC 2 to high throttle pulse.
    esc_3 = 1787  ;                                           //Set the pulse for ESC 3 to high throttle pulse.
    esc_4 = 1787  ;                                             //Set the pulse for ESC 4 to high throttle pulse.
    esc_pulse_output();
  
  Serial.println(F("Maximalna hodnota esc bola zadana"));


  Serial.println(F("Nastavte min vykon, po nastaveni zadajte 1"));
  delay(5000);
    esc_1 = 1140;                                                //Set the pulse for ESC 1 to low throttle pulse.
    esc_2 = 1140;                                                //Set the pulse for ESC 2 to low throttle pulse.
    esc_3 = 1140;                                                //Set the pulse for ESC 3 to low throttle pulse.
    esc_4 = 1140;                                                //Set the pulse for ESC 4 to low throttle pulse.
    esc_pulse_output();
  
  Serial.println(F("Minimalna hodnota esc bola zadana"));
}

void loop() {
  if((analogRead(0) * 1,6422287390029325513196480938416)<=1480)
  digitalWrite(A1, HIGH);;
  
  receiver_input_channels[0] = convert_receiver_channel(0);
  receiver_input_channels[1] = convert_receiver_channel(1);
  receiver_input_channels[2] = convert_receiver_channel(2);
  receiver_input_channels[3] = convert_receiver_channel(3);
  IMU.readSensor();
  ax = IMU.getAccelX_mss();
  ay = IMU.getAccelY_mss();
  az = IMU.getAccelZ_mss();
  gx = IMU.getGyroX_rads();
  gy = IMU.getGyroY_rads();
  gz = IMU.getGyroZ_rads();
  mx = IMU.getMagX_uT();
  my = IMU.getMagY_uT();
  mz = IMU.getMagZ_uT();


  
  Now = micros();
  deltat = ((Now - lastUpdate) / 1000000.0f); // set integration time by time elapsed since last filter update
  lastUpdate = Now;
  MahonyQuaternionUpdate(ax, ay, az, gx, gy, gz, mx, my, mz);
  yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
  pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
  roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
  pitch *= 180.0f / PI;
  yaw   *= 180.0f / PI;
  yaw   -= 5.24; // Declination at Benice, Slovakia is 5 degrees 14 minutes and 24 seconds on 2020-08-15
  roll  *= 180.0f / PI;


  pitch_level_adjust = pitch * 15;                                    //Calculate the pitch angle correction
  roll_level_adjust = roll * 15;                                      //Calculate the roll angle correction

  pid_i_mem_roll = 0;
  pid_last_roll_d_error = 0;
  pid_i_mem_pitch = 0;
  pid_last_pitch_d_error = 0;
  pid_i_mem_yaw = 0;
  pid_last_yaw_d_error = 0;


  pid_roll_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if (receiver_input_channels[0] > center_channels[0] + deadband)pid_roll_setpoint = receiver_input_channels[0] - center_channels[0] - deadband;
  else if (receiver_input_channels[0] < center_channels[0] - deadband)pid_roll_setpoint = receiver_input_channels[0] - center_channels[0] + deadband;

  pid_roll_setpoint -= roll_level_adjust;                                   //Subtract the angle correction from the standardized receiver roll input value.
  pid_roll_setpoint /= 3.0;                                                 //Divide the setpoint for the PID roll controller by 3 to get angles in degrees.


  //The PID set point in degrees per second is determined by the pitch receiver input.
  //In the case of deviding by 3 the max pitch rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_pitch_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if (receiver_input_channels[1] > center_channels[1] + deadband)pid_pitch_setpoint = receiver_input_channels[1] - center_channels[1] - deadband ;
  else if (receiver_input_channels[1] < center_channels[1] - deadband)pid_pitch_setpoint = receiver_input_channels[1] - center_channels[1] + deadband;

  pid_pitch_setpoint -= pitch_level_adjust;                                  //Subtract the angle correction from the standardized receiver pitch input value.
  pid_pitch_setpoint /= 3.0;                                                 //Divide the setpoint for the PID pitch controller by 3 to get angles in degrees.

  //The PID set point in degrees per second is determined by the yaw receiver input.
  //In the case of deviding by 3 the max yaw rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_yaw_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if (receiver_input_channels[3] > center_channels[3] + deadband)pid_yaw_setpoint = (receiver_input_channels[3] - center_channels[3] - deadband) / 3.0;
  else if (receiver_input_channels[3] < center_channels[3] - deadband)pid_yaw_setpoint = (receiver_input_channels[3] - center_channels[3] + deadband) / 3.0;

  calculate_pid();


  throttle = receiver_input_channels[3];                                      //We need the throttle signal as a base signal.

  if (throttle > 1800) throttle = 1800;                                   //We need some room to keep full control at full throttle.
  esc_1 = (throttle - pid_output_pitch + pid_output_roll - pid_output_yaw) * obmedzovac; //Calculate the pulse for esc 1 (front-right - CCW)
  esc_2 = (throttle + pid_output_pitch + pid_output_roll + pid_output_yaw) * obmedzovac; //Calculate the pulse for esc 2 (rear-right - CW)
  esc_3 = (throttle + pid_output_pitch - pid_output_roll - pid_output_yaw) * obmedzovac; //Calculate the pulse for esc 3 (rear-left - CCW)
  esc_4 = (throttle - pid_output_pitch - pid_output_roll + pid_output_yaw) * obmedzovac; //Calculate the pulse for esc 4 (front-left - CW)

  if (esc_1 > 2000)esc_1 = 2000;                                          //Limit the esc-1 pulse to 2000us.
  if (esc_2 > 2000)esc_2 = 2000;                                          //Limit the esc-2 pulse to 2000us.
  if (esc_3 > 2000)esc_3 = 2000;                                          //Limit the esc-3 pulse to 2000us.
  if (esc_4 > 2000)esc_4 = 2000;                                          //Limit the esc-4 pulse to 2000us.


  esc_pulse_output();
}





//This routine is called every time input 2, 3, 4 or 5 changed state
ISR(PCINT2_vect) {
  current_time = micros(); 
  
  
  //Channel 1=========================================
  if (PIND & 0b00000100) { //Is input 2 high?
    if (!(last_channels & channel_1_state)) { //Input 2 changed from 0 to 1
      last_channels |= channel_1_state; //Remember current input state
      timer_1 = current_time; //Set timer_1 to current_time
    }
  } else if (last_channels & channel_1_state) {
    //Input 2 is not high and changed from 1 to 0
    last_channels &= 0b11111110; //Remember current input state
    receiver_input_channels[0] = current_time - timer_1; //Channel 1 is current_time - timer_1
} 

  
  //Channel 2=========================================
  if (PIND & 0b00001000 ) { //Is input 3 high?
    if (!(last_channels & channel_2_state)) { //Input 3 changed from 0 to 1
      last_channels |= channel_2_state; //Remember current input state
      timer_2 = current_time; //Set timer_2 to current_time
    }
  } else if (last_channels & channel_2_state) { //Input 3 is not high and changed from 1 to 0

    last_channels &= 0b11111101; //Remember current input state
    receiver_input_channels[1] = current_time - timer_2; //Channel 2 is current_time - timer_2
} 
  
  
  //Channel 3=========================================
  if (PIND & 0b00010000 ) { //Is input 4 high?
    if (!(last_channels & channel_3_state )) { //Input 4 changed from 0 to 1
      last_channels |= channel_3_state ; //Remember current input state
      timer_3 = current_time; //Set timer_3 to current_time
    }
  } else if (last_channels & channel_3_state) {
    //Input 4 is not high and changed from 1 to 0
    last_channels &= 0b11111011; //Remember current input state
    receiver_input_channels[2] = current_time - timer_3; //Channel 3 is current_time - timer_3
} 

  
  //Channel 4=========================================
  if (PIND & 0b00100000 ) { //Is input 5 high?
    if (!(last_channels & channel_4_state)) { //Input 5 changed from 0 to 1
      last_channels |= channel_4_state; //Remember current input state
      timer_4 = current_time; //Set timer_4 to current_time
    }
  } else if (last_channels & channel_4_state) {
    //Input 5 is not high and changed from 1 to 0
    last_channels &= 0b11110111; //Remember current input state
    receiver_input_channels[3] = current_time - timer_4; //Channel 4 is current_time - timer_4

  }
}




void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz) {
  float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
  float norm;
  float hx, hy, bx, bz;
  float vx, vy, vz, wx, wy, wz;
  float ex, ey, ez;
  float pa, pb, pc;

  // Auxiliary variables to avoid repeated arithmetic
  float q1q1 = q1 * q1;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q1q4 = q1 * q4;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q2q4 = q2 * q4;
  float q3q3 = q3 * q3;
  float q3q4 = q3 * q4;
  float q4q4 = q4 * q4;

  // Normalise accelerometer measurement
  norm = sqrtf(ax * ax + ay * ay + az * az);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f / norm;        // use reciprocal for division
  ax *= norm;
  ay *= norm;
  az *= norm;

  // Normalise magnetometer measurement
  norm = sqrtf(mx * mx + my * my + mz * mz);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f / norm;        // use reciprocal for division
  mx *= norm;
  my *= norm;
  mz *= norm;

  // Reference direction of Earth's magnetic field
  hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
  hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
  bx = sqrtf((hx * hx) + (hy * hy));
  bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);

  // Estimated direction of gravity and magnetic field
  vx = 2.0f * (q2q4 - q1q3);
  vy = 2.0f * (q1q2 + q3q4);
  vz = q1q1 - q2q2 - q3q3 + q4q4;
  wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
  wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
  wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);

  // Error is cross product between estimated direction and measured direction of gravity
  ex = (ay * vz - az * vy) + (my * wz - mz * wy);
  ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
  ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
  if (Ki > 0.0f)
  {
    eInt[0] += ex;      // accumulate integral error
    eInt[1] += ey;
    eInt[2] += ez;
  }
  else
  {
    eInt[0] = 0.0f;     // prevent integral wind up
    eInt[1] = 0.0f;
    eInt[2] = 0.0f;
  }

  // Apply feedback terms
  gx = gx + Kp * ex + Ki * eInt[0];
  gy = gy + Kp * ey + Ki * eInt[1];
  gz = gz + Kp * ez + Ki * eInt[2];

  // Integrate rate of change of quaternion
  pa = q2;
  pb = q3;
  pc = q4;
  q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * deltat);
  q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * deltat);
  q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * deltat);
  q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * deltat);

  // Normalise quaternion
  norm = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
  norm = 1.0f / norm;
  q[0] = q1 * norm;
  q[1] = q2 * norm;
  q[2] = q3 * norm;
  q[3] = q4 * norm;

}


void esc_pulse_output() {
  zero_timer = micros();
  PORTD |= 0b11000000; //Set port 6 and 7 high at once
  PORTB |= 0b00000011; //Set port 8 and 9 high at once
  timer_channel_1 = esc_1 + zero_timer;                          //Calculate the time when digital port 4 is set low.
  timer_channel_2 = esc_2 + zero_timer;                          //Calculate the time when digital port 5 is set low.
  timer_channel_3 = esc_3 + zero_timer;                          //Calculate the time when digital port 6 is set low.
  timer_channel_4 = esc_4 + zero_timer;                          //Calculate the time when digital port 7 is set low.

  while (PORTD > 64 || PORTB & 0b00000010 || PORTB & 0b00000001) {                                        //Execute the loop until digital port 6 to 9 is low.
    esc_loop_timer = micros();                                   //Check the current time.
    if (timer_channel_1 <= esc_loop_timer)PORTD ^= 0b01000000;    //When the delay time is expired, digital port 6 is set low.
    if (timer_channel_2 <= esc_loop_timer)PORTD ^= 0b10000000;    //When the delay time is expired, digital port 7 is set low.
    if (timer_channel_3 <= esc_loop_timer)PORTB ^= 0b00000001;    //When the delay time is expired, digital port 8 is set low.
    if (timer_channel_4 <= esc_loop_timer)PORTB ^= 0b00000010;    //When the delay time is expired, digital port 9 is set low.
  }
}


int convert_receiver_channel(byte channel) {
  short actual, difference;


  actual = receiver_input_channels[channel];
  if (actual < center_channels[channel]) {                                                       //The actual receiver value is lower than the center value
    if (actual < low_channels[channel])actual = low_channels[channel];                                             //Limit the lowest value to the value that was detected during setup
    difference = ((long)(center_channels[channel] - actual) * (long)500) / (center_channels[channel] - low_channels[channel]);       //Calculate and scale the actual value to a 1000 - 2000us value
    return 1500 - difference;                                             //If the channel is not reversed
  }
  else if (actual > center_channels[channel]) {                                                                      //The actual receiver value is higher than the center value
    if (actual > high_channels[channel])actual = high_channels[channel];                                           //Limit the lowest value to the value that was detected during setup
    difference = ((long)(actual - center_channels[channel]) * (long)500) / (high_channels[channel] - center_channels[channel]);      //Calculate and scale the actual value to a 1000 - 2000us value
    return 1500 + difference;                                             //If the channel is not reversed
  }
  else return 1500;
}


void calculate_pid() {
  gyro_roll_input = roll;
  gyro_pitch_input = pitch;
  gyro_yaw_input = yaw;
  //Roll calculations
  pid_error_temp = gyro_roll_input - pid_roll_setpoint;
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
  if (pid_i_mem_roll > pid_max_roll)pid_i_mem_roll = pid_max_roll;
  else if (pid_i_mem_roll < pid_max_roll * -1)pid_i_mem_roll = pid_max_roll * -1;

  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  if (pid_output_roll > pid_max_roll)pid_output_roll = pid_max_roll;
  else if (pid_output_roll < pid_max_roll * -1)pid_output_roll = pid_max_roll * -1;

  pid_last_roll_d_error = pid_error_temp;

  //Pitch calculations
  pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  if (pid_i_mem_pitch > pid_max_pitch)pid_i_mem_pitch = pid_max_pitch;
  else if (pid_i_mem_pitch < pid_max_pitch * -1)pid_i_mem_pitch = pid_max_pitch * -1;

  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  if (pid_output_pitch > pid_max_pitch)pid_output_pitch = pid_max_pitch;
  else if (pid_output_pitch < pid_max_pitch * -1)pid_output_pitch = pid_max_pitch * -1;

  pid_last_pitch_d_error = pid_error_temp;

  //Yaw calculations
  pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
  if (pid_i_mem_yaw > pid_max_yaw)pid_i_mem_yaw = pid_max_yaw;
  else if (pid_i_mem_yaw < pid_max_yaw * -1)pid_i_mem_yaw = pid_max_yaw * -1;

  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  if (pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;
  else if (pid_output_yaw < pid_max_yaw * -1)pid_output_yaw = pid_max_yaw * -1;

  pid_last_yaw_d_error = pid_error_temp;
}
