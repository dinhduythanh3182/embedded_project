#include <MPU6050_tockn.h>
#include <Wire.h>
MPU6050 mpu6050(Wire);

#define _Base_Freq          100000     
#define _Working_Freq       100       // Hz    định nghĩa sẽ tạo tần số làm việc (điều khiển PID tốc độ và giao tiếp LabVIEW) là 100Hz
#define dt                  (1.0/_Working_Freq)
//-------------------------------------------------------------------------------
#define _Sys_CLK            16000000 
#define _BaseTimer_CLK      16000000  
#define _WrkCycle_MAX       (_Base_Freq / _Working_Freq)            //    định nghĩa số chu kỳ cơ sở để tạo thành chu kỳ làm việc
#define _BaseTimer_TOP      (((_BaseTimer_CLK / _Base_Freq)) - 1)
#define PWM_period          0.02
#define PWM_MAX             (PWM_period*_Base_Freq)   
#define PWM_MIN             (0.001*_Base_Freq)   
#define min_adc_value       495
#define max_adc_value       510
//-------------------------------------------------------------------------------
// độ nhạy p
// độ văng i
// đọ nhún >< p 
#define kp_roll             0.6   
#define ki_roll             0.024
#define kd_roll             0.05
#define kp_pitch            0.6 
#define ki_pitch            0.024
#define kd_pitch            0.05
#define kp_yaw               0.6  
#define ki_yaw              0.024
#define kd_yaw              0.05
#define kp_throttle        0.6 
#define ki_throttle         0.024
#define kd_throttle        0.05
//-------------------------------------------------------------------------------
float e_roll, e_old_roll, out_roll, out_P_roll, out_I_roll, out_D_roll;      
float e_pitch, e_old_pitch, out_pitch, out_P_pitch, out_I_pitch, out_D_pitch;      
float e_yaw, e_old_yaw, out_yaw, out_P_yaw, out_I_yaw, out_D_yaw;           
float e_throttle, e_old_throttle, out_throttle, out_P_throttle, out_I_throttle, out_D_throttle;   
float duty_PWM1, duty_PWM2, duty_PWM3, duty_PWM4;   
uint32_t WrkCycle_Counter = _WrkCycle_MAX;      //    khai báo biến đếm chu kỳ cơ sở
volatile bool F_Working_Cycle = false;          //    khai báo cờ báo bắt đầu chu kỳ làm việc
uint32_t PWM_Counter = PWM_MAX;
uint32_t PWM1_compare, PWM2_compare, PWM3_compare, PWM4_compare;
float angle_x, angle_y, angle_z;
float acc_x, acc_y, acc_z;
float offset_angle_x, offset_angle_y, offset_angle_z;
float offset_acc_x, offset_acc_y, offset_acc_z;
int16_t tien_lui, trai_phai, temp_tien_lui, temp_trai_phai;
//-------------------------------------------------------------------------------
void onBaseTimer()
{
  TCCR2A = 0; TCCR2B = 0; TIMSK2 = 0;   
  TCCR2A |= (1 << WGM21);  
  TCCR2B |= (1 << CS20);   
  TIMSK2 |= (1 << OCIE2A);   
  OCR2A = _BaseTimer_TOP;   
}
//-------------------------------------------------------------------------------
ISR (TIMER2_COMPA_vect)     
{
  if (!(--WrkCycle_Counter))    
  {
    WrkCycle_Counter = _WrkCycle_MAX;    
    F_Working_Cycle = true;       
  } 
  if (!(--PWM_Counter))    
  {
    PWM_Counter = PWM_MAX;
    digitalWrite(9, LOW);    
    digitalWrite(10, LOW);   
    digitalWrite(11, LOW);   
    digitalWrite(12, LOW);     
  } 
  if (PWM_Counter == PWM1_compare)
    digitalWrite(9, HIGH);  
  if (PWM_Counter == PWM1_compare)
    digitalWrite(10, HIGH);   
  if (PWM_Counter == PWM1_compare)
    digitalWrite(11, HIGH); 
  if (PWM_Counter == PWM1_compare)
    digitalWrite(12, HIGH); 
}

void setup() {  
  pinMode(2, INPUT); 
  pinMode(3, INPUT); 
  pinMode(4, INPUT); 
  pinMode(5, INPUT); 
  pinMode(6, INPUT); 
  pinMode(7, INPUT); 
  pinMode(8, INPUT);
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
   
  pinMode(9, OUTPUT); 
  pinMode(10, OUTPUT); 
  pinMode(11, OUTPUT); 
  pinMode(12, OUTPUT); 
  
  Serial.begin(115200);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);

  delay(500);
  mpu6050.update();
  offset_angle_x = mpu6050.getAngleX();
  offset_angle_y = mpu6050.getAngleY();
  offset_angle_z = mpu6050.getAngleZ();
  offset_acc_x = mpu6050.getAccX();
  offset_acc_y = mpu6050.getAccY();
  offset_acc_z = mpu6050.getAccZ();
  
  onBaseTimer();
}

void loop() {
  if (F_Working_Cycle)            // có cờ bắt đầu chu kỳ làm việc
  {
    F_Working_Cycle = false;      // tắt cờ bắt đầu chu kỳ làm việc  
    mpu6050.update();
    angle_x = offset_angle_x - mpu6050.getAngleX();
    angle_y = offset_angle_y - mpu6050.getAngleY();
    angle_z = offset_angle_z - mpu6050.getAngleZ();
    acc_x = offset_acc_x - mpu6050.getAccX();
    acc_y = offset_acc_y - mpu6050.getAccY();
    acc_z = offset_acc_z - mpu6050.getAccZ();
    temp_tien_lui = analogRead(A1);
    temp_trai_phai = analogRead(A0);
    tien_lui = (temp_tien_lui > max_adc_value)?(temp_tien_lui - max_adc_value):((temp_tien_lui < min_adc_value)?(temp_tien_lui - min_adc_value):0);
    trai_phai = (temp_trai_phai > max_adc_value)?(temp_trai_phai - max_adc_value):((temp_trai_phai < min_adc_value)?(temp_trai_phai - min_adc_value):0);
    Serial.print("angle X: "); Serial.print(angle_x); Serial.print("   angle Y: "); Serial.print(angle_y); Serial.print("   angle Z: "); Serial.print(angle_z);
    Serial.print("   acc X: "); Serial.print(acc_x); Serial.print("   acc Y: "); Serial.print(acc_y); Serial.print("   acc Z: "); Serial.println(acc_z);
    if (!digitalRead(2))           // button A - UP
    {
      duty_PWM1 = PID_throttle(acc_z, 0.3) + PID_roll(angle_y, 0) - PID_pitch(angle_x, 0) - PID_yaw(angle_z, angle_z);
      duty_PWM2 = PID_throttle(acc_z, 0.3) + PID_roll(angle_y, 0) + PID_pitch(angle_x, 0) + PID_yaw(angle_z, angle_z);
      duty_PWM3 = PID_throttle(acc_z, 0.3) - PID_roll(angle_y, 0) + PID_pitch(angle_x, 0) - PID_yaw(angle_z, angle_z);
      duty_PWM4 = PID_throttle(acc_z, 0.3) - PID_roll(angle_y, 0) - PID_pitch(angle_x, 0) + PID_yaw(angle_z, angle_z);
      Serial.println("Move up.");
    }
    else if (!digitalRead(3))       // button B - RIGHT
    {
      duty_PWM1 = PID_throttle(acc_z, 0) + PID_roll(angle_y, 0) - PID_pitch(angle_x, 0) - PID_yaw(angle_z, angle_z + 1);
      duty_PWM2 = PID_throttle(acc_z, 0) + PID_roll(angle_y, 0) + PID_pitch(angle_x, 0) + PID_yaw(angle_z, angle_z + 1);
      duty_PWM3 = PID_throttle(acc_z, 0) - PID_roll(angle_y, 0) + PID_pitch(angle_x, 0) - PID_yaw(angle_z, angle_z + 1);
      duty_PWM4 = PID_throttle(acc_z, 0) - PID_roll(angle_y, 0) - PID_pitch(angle_x, 0) + PID_yaw(angle_z, angle_z + 1); 
      Serial.println("Rotate right.");   
    }
    else if (!digitalRead(4))       // button C - DOWN
    {
      duty_PWM1 = PID_throttle(acc_z, -0.3) + PID_roll(angle_y, 0) - PID_pitch(angle_x, 0) - PID_yaw(angle_z, angle_z);
      duty_PWM2 = PID_throttle(acc_z, -0.3) + PID_roll(angle_y, 0) + PID_pitch(angle_x, 0) + PID_yaw(angle_z, angle_z);
      duty_PWM3 = PID_throttle(acc_z, -0.3) - PID_roll(angle_y, 0) + PID_pitch(angle_x, 0) - PID_yaw(angle_z, angle_z);
      duty_PWM4 = PID_throttle(acc_z, -0.3) - PID_roll(angle_y, 0) - PID_pitch(angle_x, 0) + PID_yaw(angle_z, angle_z);
      Serial.println("Move down.");
    }
    else if (!digitalRead(5))       // button D - LEFT
    {
      duty_PWM1 = PID_throttle(acc_z, 0) + PID_roll(angle_y, 0) - PID_pitch(angle_x, 0) - PID_yaw(angle_z, angle_z - 1);
      duty_PWM2 = PID_throttle(acc_z, 0) + PID_roll(angle_y, 0) + PID_pitch(angle_x, 0) + PID_yaw(angle_z, angle_z - 1);
      duty_PWM3 = PID_throttle(acc_z, 0) - PID_roll(angle_y, 0) + PID_pitch(angle_x, 0) - PID_yaw(angle_z, angle_z - 1);
      duty_PWM4 = PID_throttle(acc_z, 0) - PID_roll(angle_y, 0) - PID_pitch(angle_x, 0) + PID_yaw(angle_z, angle_z - 1); 
      Serial.println("Rotate left.");  
    }
    else if (tien_lui)              // joystick X - FORWARD + BACKWARD -
    {
      duty_PWM1 = PID_throttle(acc_z, 0) + PID_roll(angle_y, 0) - PID_pitch(angle_x/90.0, 0.3*((float)tien_lui/500.0)) - PID_yaw(angle_z, angle_z);
      duty_PWM2 = PID_throttle(acc_z, 0) + PID_roll(angle_y, 0) + PID_pitch(angle_x/90.0, 0.3*((float)tien_lui/500.0)) + PID_yaw(angle_z, angle_z);
      duty_PWM3 = PID_throttle(acc_z, 0) - PID_roll(angle_y, 0) + PID_pitch(angle_x/90.0, 0.3*((float)tien_lui/500.0)) - PID_yaw(angle_z, angle_z);
      duty_PWM4 = PID_throttle(acc_z, 0) - PID_roll(angle_y, 0) - PID_pitch(angle_x/90.0, 0.3*((float)tien_lui/500.0)) + PID_yaw(angle_z, angle_z);
      if (tien_lui > 0)        
        Serial.println("Move forward");
      else        
        Serial.println("Move backward.");      
    }
    else if (trai_phai)             // joystick Y - RIGHT + LEFT -
    {
      duty_PWM1 = PID_throttle(acc_z, 0) + PID_roll(angle_y/90.0, -0.3*((float)trai_phai/500.0)) - PID_pitch(angle_x, 0) - PID_yaw(angle_z, angle_z);
      duty_PWM2 = PID_throttle(acc_z, 0) + PID_roll(angle_y/90.0, -0.3*((float)trai_phai/500.0)) + PID_pitch(angle_x, 0) + PID_yaw(angle_z, angle_z);
      duty_PWM3 = PID_throttle(acc_z, 0) - PID_roll(angle_y/90.0, -0.3*((float)trai_phai/500.0)) + PID_pitch(angle_x, 0) - PID_yaw(angle_z, angle_z);
      duty_PWM4 = PID_throttle(acc_z, 0) - PID_roll(angle_y/90.0, -0.3*((float)trai_phai/500.0)) - PID_pitch(angle_x, 0) + PID_yaw(angle_z, angle_z); 
      if (trai_phai > 0)        
        Serial.println("Bend left.");
      else        
        Serial.println("Bend right.");          
    }
    else                            // balance      
    {
      duty_PWM1 = PID_throttle(acc_z, 0) + PID_roll(angle_y, 0) - PID_pitch(angle_x, 0) - PID_yaw(angle_z, angle_z);
      duty_PWM2 = PID_throttle(acc_z, 0) + PID_roll(angle_y, 0) + PID_pitch(angle_x, 0) + PID_yaw(angle_z, angle_z);
      duty_PWM3 = PID_throttle(acc_z, 0) - PID_roll(angle_y, 0) + PID_pitch(angle_x, 0) - PID_yaw(angle_z, angle_z);
      duty_PWM4 = PID_throttle(acc_z, 0) - PID_roll(angle_y, 0) - PID_pitch(angle_x, 0) + PID_yaw(angle_z, angle_z); 
      Serial.println("Balance.");         
    }    
    PWM1_compare = PWM_MIN + PWM_MIN*((duty_PWM1 > 1)?1:((duty_PWM1 < 0)?0:duty_PWM1));
    PWM2_compare = PWM_MIN + PWM_MIN*((duty_PWM2 > 1)?1:((duty_PWM2 < 0)?0:duty_PWM2));
    PWM3_compare = PWM_MIN + PWM_MIN*((duty_PWM3 > 1)?1:((duty_PWM3 < 0)?0:duty_PWM3));
    PWM4_compare = PWM_MIN + PWM_MIN*((duty_PWM4 > 1)?1:((duty_PWM4 < 0)?0:duty_PWM4));
    Serial.print("PWM1 "); Serial.print(PWM1_compare); Serial.print("   PWM2 "); Serial.print(PWM2_compare); Serial.print("   PWM3 "); Serial.print(PWM3_compare); Serial.print("   PWM4 "); Serial.println(PWM4_compare); Serial.println();
  }
}

float PID_roll(float cur, float req)   //  hàm tính output bộ PID
{  
  e_roll = req - cur;              // tính sai số
  out_P_roll = kp_roll*e_roll;               // tính thành phần out P
  out_I_roll += ki_roll*e_roll*dt;           // tính thành phần out I
  out_I_roll = (out_I_roll < -1.0)?-1.0:((out_I_roll > 1.0)?1.0:out_I_roll);  // giới hạn thành phần out I
  out_D_roll = kd_roll*((e_roll - e_old_roll)/dt);  // tính thành phần out D
  e_old_roll = e_roll;    // lưu sai số chu kỳ điều khiển trước dùng để tính thành phần out D
  out_roll = out_P_roll + out_I_roll + out_D_roll;      //tinh toan output cho bo dieu khien PID
  out_roll = (out_roll <- 1.0)?-1.0:((out_roll > 1.0)?1.0:out_roll);  // giới hạn output PID
  return out_roll;
}

float PID_pitch(float cur, float req)   //  hàm tính output bộ PID
{  
  e_pitch = req - cur;              // tính sai số
  out_P_pitch = kp_pitch*e_pitch;               // tính thành phần out P
  out_I_pitch += ki_pitch*e_pitch*dt;           // tính thành phần out I
  out_I_pitch = (out_I_pitch < -1.0)?-1.0:((out_I_pitch > 1.0)?1.0:out_I_pitch);  // giới hạn thành phần out I
  out_D_pitch = kd_pitch*((e_pitch - e_old_pitch)/dt);  // tính thành phần out D
  e_old_pitch = e_pitch;    // lưu sai số chu kỳ điều khiển trước dùng để tính thành phần out D
  out_pitch = out_P_pitch + out_I_pitch + out_D_pitch;      //tinh toan output cho bo dieu khien PID
  out_pitch = (out_pitch <- 1.0)?-1.0:((out_pitch > 1.0)?1.0:out_pitch);  // giới hạn output PID
  return out_pitch;
}

float PID_yaw(float cur, float req)   //  hàm tính output bộ PID
{  
  e_yaw = req - cur;              // tính sai số
  out_P_yaw = kp_yaw*e_yaw;               // tính thành phần out P
  out_I_yaw += ki_yaw*e_yaw*dt;           // tính thành phần out I
  out_I_yaw = (out_I_yaw < -1.0)?-1.0:((out_I_yaw > 1.0)?1.0:out_I_yaw);  // giới hạn thành phần out I
  out_D_yaw = kd_yaw*((e_yaw - e_old_yaw)/dt);  // tính thành phần out D
  e_old_yaw = e_yaw;    // lưu sai số chu kỳ điều khiển trước dùng để tính thành phần out D
  out_yaw = out_P_yaw + out_I_yaw + out_D_yaw;      //tinh toan output cho bo dieu khien PID
  out_yaw = (out_yaw <- 1.0)?-1.0:((out_yaw > 1.0)?1.0:out_yaw);  // giới hạn output PID
  return out_yaw;
}

float PID_throttle(float cur, float req)   //  hàm tính output bộ PID
{  
  e_throttle = req - cur;              // tính sai số
  out_P_throttle = kp_throttle*e_throttle;               // tính thành phần out P
  out_I_throttle += ki_throttle*e_throttle*dt;           // tính thành phần out I
  out_I_throttle = (out_I_throttle < -1.0)?-1.0:((out_I_throttle > 1.0)?1.0:out_I_throttle);  // giới hạn thành phần out I
  out_D_throttle = kd_throttle*((e_throttle - e_old_throttle)/dt);  // tính thành phần out D
  e_old_throttle = e_throttle;    // lưu sai số chu kỳ điều khiển trước dùng để tính thành phần out D
  out_throttle = out_P_throttle + out_I_throttle + out_D_throttle;      //tinh toan output cho bo dieu khien PID
  out_throttle = (out_throttle <- 1.0)?-1.0:((out_throttle > 1.0)?1.0:out_throttle);  // giới hạn output PID
  return out_throttle;
}