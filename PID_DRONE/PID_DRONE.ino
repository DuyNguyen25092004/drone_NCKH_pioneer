

#include <Wire.h>

#define roll_axis           1 // định nghĩa trục roll
#define pitch_axis          2
#define yaw_axis            3
#define mpu_i2c_address  0x68 // địa chỉ của module MPU 6050

struct MPU{
  double gyro_pitch, gyro_roll, gyro_yaw;
  long acc_pitch, acc_roll, acc_yaw, acc_total;
  double gyro_x_raw, gyro_y_raw, gyro_z_raw;
  int acc_x_raw, acc_y_raw, acc_z_raw;
  double gyro_x_offset, gyro_y_offset, gyro_z_offset;
  float temperature;
};

struct PID{
  float pid_input;
  float pid_output;
  float pid_setpoint;
  float pid_error;
  float pid_pre_error;
  float pid_p_part;
  float pid_i_part;
  float pid_d_part;
  float pid_p_gain;
  float pid_i_gain;
  float pid_d_gain;
  int   pid_limit;
};

struct ANGLE{
  float pitch;
  float roll;
  float pitch_acc;
  float roll_acc;
  float pitch_correct;
  float roll_correct;
};


MPU mpu;
PID roll, pitch, yaw, altitude;
ANGLE angle;

int calib_counter;

void initial_pid_gain(){
  roll.pid_p_gain = 1.4;        
  roll.pid_i_gain = 0.05;                   // Ki thực = 0.05/0.005 = 10.0
  roll.pid_d_gain = 15.0;                   // Kd thực = 15.0/200 = 0.075
  
  pitch.pid_p_gain = 1.4;                   
  pitch.pid_i_gain = 0.05;                  // giống PID của roll
  pitch.pid_d_gain = 15.0;

  yaw.pid_p_gain = 4.0;
  yaw.pid_i_gain = 0.02;                    // Ki thực = 0.02/0.005 = 4.0
  yaw.pid_d_gain = 0.0;
  
  roll.pid_limit     = 400.0;               // giới hạn cho các hệ số PID để không bị vọt lố quá mức
  pitch.pid_limit    = 400.0;
  yaw.pid_limit      = 400.0;
}

void gy521_mpu6050_init(){
  Wire.beginTransmission(mpu_i2c_address);                            // Tạo trạng thái Start I2C và gửi địa chỉ slave I2C 0x68 đến mpu 
  Wire.write(0x6B);                                                   // Gửi địa chỉ thanh ghi muốn ghi dữ liệu vào là 0x6B
  Wire.write(0x00);                                                   // Ghi 0x00 vào thanh ghi 0x6B để reset mp6050, tắt chế độ sleep công suất thấp và đánh thức mpu6050
  Wire.endTransmission();                                             // Tạo trạng thái Stop I2C 
  
  Wire.beginTransmission(mpu_i2c_address);                            // Tạo trạng thái Start I2C và gửi địa chỉ slave I2C 0x68 đến mpu 
  Wire.write(0x1B);                                                   // Gửi địa chỉ thanh ghi muốn ghi dữ liệu vào là 0x1B
  Wire.write(0x08);                                                   // Ghi 0x08 vào thanh ghi 0x1B để chọn scale cho gyro là +- 500 deg/sec
  Wire.endTransmission();                                             // Tạo trạng thái Stop I2C

  Wire.beginTransmission(mpu_i2c_address);                            // Tạo trạng thái Start I2C và gửi địa chỉ slave I2C 0x68 đến mpu 
  Wire.write(0x1C);                                                   // Gửi địa chỉ thanh ghi muốn ghi dữ liệu vào là 0x1C      
  Wire.write(0x10);                                                   // Ghi 0x10 vào thanh ghi 0x1C để chọn scale cho accel là +/- 8g       
  Wire.endTransmission();                                             // Tạo trạng thái Stop I2C

  Wire.beginTransmission(mpu_i2c_address);                            // Tạo trạng thái Start I2C và gửi địa chỉ slave I2C 0x68 đến mpu 
  Wire.write(0x1A);                                                   // Gửi địa chỉ thanh ghi muốn ghi dữ liệu vào là 0x1A
  Wire.write(0x03);                                                   // Ghi 0x03 vào thanh ghi 0x1A để chọn tần số bộ lọc thông thấp khoảng 43 Hz để lọc gyro và accel
  Wire.endTransmission();                                             // Tạo trạng thái Stop I2C

  
  Wire.beginTransmission(mpu_i2c_address);                            // Tạo trạng thái Start I2C và gửi địa chỉ slave I2C 0x68 đến mpu 
  Wire.write(0x6B);                                                   // Gửi địa chỉ thanh ghi muốn đọc dữ liệu là 0x6B
  Wire.endTransmission();                                             // Tạo trạng thái Stop I2C
  Wire.requestFrom(mpu_i2c_address, 1);                               // Yêu cầu mpu trả về 1 byte data
  while(Wire.available() < 1);                                        // Chờ tới khi nào có 1 byte data gửi về
  if(Wire.read() != 0x00) goto error_init;                            // Nếu đọc lại mà khác 0x00 
  

  Wire.beginTransmission(mpu_i2c_address);                            // Tạo trạng thái Start I2C và gửi địa chỉ slave I2C 0x68 đến mpu 
  Wire.write(0x1B);                                                   // Gửi địa chỉ thanh ghi muốn đọc dữ liệu là 0x1B
  Wire.endTransmission();                                             // Tạo trạng thái Stop I2C
  Wire.requestFrom(mpu_i2c_address, 1);                               // Yêu cầu mpu trả về 1 byte data
  while(Wire.available() < 1);                                        // Chờ tới khi nào có 1 byte data gửi về
  if(Wire.read() != 0x08) goto error_init;                            // Nếu đọc lại mà khác 0x08 

                                    
  Wire.beginTransmission(mpu_i2c_address);                            // Tạo trạng thái Start I2C và gửi địa chỉ slave I2C 0x68 đến mpu 
  Wire.write(0x1C);                                                   // Gửi địa chỉ thanh ghi muốn đọc dữ liệu là 0x1C
  Wire.endTransmission();                                             // Tạo trạng thái Stop I2C
  Wire.requestFrom(mpu_i2c_address, 1);                               // Yêu cầu mpu trả về 1 byte data
  while(Wire.available() < 1);                                        // Chờ tới khi nào có 1 byte data gửi về
  if(Wire.read() != 0x10) goto error_init;                            // Nếu đọc lại mà khác 0x10 

  
  Wire.beginTransmission(mpu_i2c_address);                            // Tạo trạng thái Start I2C và gửi địa chỉ slave I2C 0x68 đến mpu 
  Wire.write(0x1A);                                                   // Gửi địa chỉ thanh ghi muốn đọc dữ liệu là 0x1C
  Wire.endTransmission();                                             // Tạo trạng thái Stop I2C
  Wire.requestFrom(mpu_i2c_address, 1);                               // Yêu cầu mpu trả về 1 byte data
  while(Wire.available() < 1);                                        // Chờ tới khi nào có 1 byte data gửi về
  if(Wire.read() != 0x03){                                            // Nếu đọc lại mà khác 0x10 
    error_init:
    while(1) {
      PORTC |= B00000010;
      delay(200);
      PORTC &= B11111101;                                        
      delay(200);
    }                                        
  }
}


void read_mpu_data(){
  Wire.beginTransmission(mpu_i2c_address);                                  // Tạo trạng thái Start I2C và gửi địa chỉ slave I2C 0x68 đến mpu 
  Wire.write(0x3B);                                                         // Gửi địa chỉ thanh ghi muốn đọc dữ liệu là 0x3B
  Wire.endTransmission();                                                   // Tạo trạng thái Stop I2C
  Wire.requestFrom(mpu_i2c_address,14);                                     // Yêu cầu mpu trả về 14 byte data
  while(Wire.available() < 14);                                             // Chờ tới khi nào có đủ 14 byte data gửi về
  mpu.acc_x_raw    = Wire.read()<<8|Wire.read();                            // Đọc 8 bit cao trước rồi nối với 8 bit thấp để ra dữ liệu accel trục X 16 bit
  mpu.acc_y_raw    = Wire.read()<<8|Wire.read();                            // Đọc 8 bit cao trước rồi nối với 8 bit thấp để ra dữ liệu accel trục Y 16 bit
  mpu.acc_z_raw    = Wire.read()<<8|Wire.read();                            // Đọc 8 bit cao trước rồi nối với 8 bit thấp để ra dữ liệu accel trục Z 16 bit
  mpu.temperature  = Wire.read()<<8|Wire.read();                            // Đọc 8 bit cao trước rồi nối với 8 bit thấp để ra dữ liệu nhiệt độ 16 bit
  mpu.gyro_x_raw   = Wire.read()<<8|Wire.read();                            // Đọc 8 bit cao trước rồi nối với 8 bit thấp để ra dữ liệu gyro trục X 16 bit
  mpu.gyro_y_raw   = Wire.read()<<8|Wire.read();                            // Đọc 8 bit cao trước rồi nối với 8 bit thấp để ra dữ liệu gyro trục Y 16 bit
  mpu.gyro_z_raw   = Wire.read()<<8|Wire.read();                            // Đọc 8 bit cao trước rồi nối với 8 bit thấp để ra dữ liệu gyro trục Z 16 bit
}


void gyro_get_offset() {
  for (calib_counter = 0; calib_counter < 2000 ; calib_counter++){    // Đọc giá trị gyro 2000 lần và cộng dồn lại    
    if(calib_counter % 20 == 0)digitalWrite(2, !digitalRead(2));   
    read_mpu_data();                                      
    mpu.gyro_x_offset += mpu.gyro_x_raw;                          
    mpu.gyro_y_offset += mpu.gyro_y_raw;                          
    mpu.gyro_z_offset += mpu.gyro_z_raw;           

    
  }
  
  mpu.gyro_x_offset /= 2000;                                               // Chia giá trị gyro_x_calib cho 2000 để lấy offset cho gyro trục X (để thực hiện LPF)
  mpu.gyro_y_offset /= 2000;                                               // Chia giá trị gyro_y_calib cho 2000 để lấy offset cho gyro trục Y (để thực hiện LPF)
  mpu.gyro_z_offset /= 2000;                                               // Chia giá trị gyro_z_calib cho 2000 để lấy offset cho gyro trục Z (để thực hiện LPF)
}

void calib_gyro_3_axis() {
  mpu.gyro_x_raw -= mpu.gyro_x_offset;                                             // Trừ gyro_x cho offset ban đầu đã tìm được
  mpu.gyro_y_raw -= mpu.gyro_y_offset;                                             // Trừ gyro_y cho offset ban đầu đã tìm được
  mpu.gyro_z_raw -= mpu.gyro_z_offset;                                             // Trừ gyro_z cho offset ban đầu đã tìm được
}

void correct_roll_pitch_yaw_data() {
  mpu.gyro_roll  = mpu.gyro_x_raw;                                           // quy đổi chiều dương gyro_roll là chiều dương gyro_x;
  mpu.gyro_pitch = mpu.gyro_y_raw * -1;                                      // quy đổi chiều dương gyro_pitch là chiều âm gyro_y;
  mpu.gyro_yaw   = mpu.gyro_z_raw * -1;                                      // quy đổi chiều dương gyro_yaw là chiều âm gyro_z;

  mpu.acc_pitch = mpu.acc_x_raw;                                             // quy đổi chiều dương accel_pitch là chiều dương accel_x;
  mpu.acc_roll  = mpu.acc_y_raw * -1;                                        // quy đổi chiều dương accel_roll là chiều âm accel_y; 
  mpu.acc_yaw   = mpu.acc_z_raw * -1;                                        // quy đổi chiều dương accel_yaw là chiều âm accel_z;

  /*
                    <--------------- Y ---------------- 
                               ------------
                               | MPU-6050 |
                               ------------  
                                    / \                              
                                     |
                                     | X
                                     |
                                     |
  */
}

void low_pass_filter_pid_input(){  
  roll.pid_input  = (roll.pid_input * 0.8)  + ((mpu.gyro_roll / 65.5) * 0.2);            // ngõ vào bộ pid trục roll đổi sang deg/sec và qua 1 bộ lọc thông thấp với alpha = 0.8 để lọc nhiễu 
  pitch.pid_input = (pitch.pid_input * 0.8) + ((mpu.gyro_pitch / 65.5) * 0.2);           // ngõ vào bộ pid trục pitch đổi sang deg/sec và qua 1 bộ lọc thông thấp với alpha = 0.8 để lọc nhiễu 
  yaw.pid_input   = (yaw.pid_input * 0.8)   + ((mpu.gyro_yaw / 65.5) * 0.2);             // ngõ vào bộ pid trục yaw đổi sang deg/sec và qua 1 bộ lọc thông thấp với alpha = 0.8 để lọc nhiễu   
}

void check_quadrotor_status() {
  // if(rx.standard_signal_channel[3] < 1050 && rx.standard_signal_channel[4] < 1050)    quadrotor_status = 1;   // Để cần ga min và gạt cần yaw qua trái max
  //                                                                                                             // Set biến trạng thái quadrotor = 1
  // if(quadrotor_status == 1 && rx.standard_signal_channel[3] < 1050 && rx.standard_signal_channel[4] > 1450){  // Sau đó giữ cần ga min và thả cần Yaw về giữa thì cho biến quadrotor = 2
  //   quadrotor_status = 2;                                                                                     // Lúc này 4 motor quay với độ rộng xung cấp cho ESC là 1100 us để chờ cất cánh
    roll.pid_i_part = 0.0;                                                                                    // Đồng thời reset các thành phần của bộ điều khiển PID về 0.0
    pitch.pid_i_part = 0.0;
    yaw.pid_i_part = 0.0;
    roll.pid_pre_error = 0.0;
    pitch.pid_pre_error = 0.0;    
    yaw.pid_pre_error = 0.0;
    altitude.pid_pre_error = 0.0;
    angle.pitch = angle.pitch_acc;
    angle.roll = angle.roll_acc;
  //}
  // if(quadrotor_status == 2 && rx.standard_signal_channel[3] < 1050 && rx.standard_signal_channel[4] > 1950) quadrotor_status = 0;  
  // // Nếu muốn dừng motor thì giữ cần ga min và gạt cần Yaw qua phải max  
}

void calculate_angle_roll_pitch() {
  angle.pitch += mpu.gyro_pitch * 0.00007634;                              // 0.00007634 = 1 / (200Hz / 65.5)      
  angle.roll  += mpu.gyro_roll * 0.00007634;                                     
  
  angle.pitch -= angle.roll * sin(mpu.gyro_yaw * 0.000001332);             // 0.000001332 = 0.00007634 * (3.142(PI) / 180)
  angle.roll  += angle.pitch * sin(mpu.gyro_yaw * 0.000001332);           

  mpu.acc_total = sqrt((mpu.acc_roll*mpu.acc_roll)+(mpu.acc_pitch*mpu.acc_pitch)+(mpu.acc_yaw*mpu.acc_yaw));                                              
  if(abs(mpu.acc_pitch) < mpu.acc_total) angle.pitch_acc = asin((float)mpu.acc_pitch/mpu.acc_total)* 57.296;          
  if(abs(mpu.acc_roll) < mpu.acc_total) angle.roll_acc = asin((float)mpu.acc_roll/mpu.acc_total)* -57.296;          
      
  angle.pitch_acc -= 4.8;                                                  // offset này thì phải debug để trừ hoặc cộng lại cho về 0  
  angle.roll_acc  += 1.0;                                                  // vì MPU gá trên Quad ko bao giờ nằm phẳng 1 góc 0độ đc nên phải tự canh chỉnh lại làm sao 
                                                                           // cho góc pitch và roll của gia tốc về 0 khi để Quad nằm yên
                                                                           // ở đây trừ 4.8độ cho góc pitch và cộng 1độ cho góc roll
  
  angle.pitch = angle.pitch * 0.9995 + angle.pitch_acc * 0.0005;           // hệ số 0.9995 và 0.0005 thì xem tài liệu nước ngoài về bộ lọc bù có giới hạn về hệ số này 
  angle.roll = angle.roll * 0.9995 + angle.roll_acc * 0.0005;     
}


void calculate_pid_setpoint() {
  angle.pitch_correct = angle.pitch * 15.0;
  angle.roll_correct = angle.roll * 15.0;
  
  roll.pid_setpoint = 0.0;              // Chọn setpoint trục roll là 0.0 deg/sec
  pitch.pid_setpoint = 0.0;             // Chọn setpoint trục pitch là 0.0 deg/sec
  yaw.pid_setpoint = 0.0; 
}


float calculate_pid(char axis) {                                                                  // Hàm bộ điều khiển PID
  static float pid_output;
  switch(axis) {
    case roll_axis: {                                                                             // case 1 là trục roll
      roll.pid_error  =  roll.pid_input - roll.pid_setpoint;                                      // error roll = input roll - setpoint roll

      roll.pid_p_part =  roll.pid_p_gain * roll.pid_error;                                        // Thành phần P roll
      roll.pid_i_part += roll.pid_i_gain * roll.pid_error;                                        // Thành phần I roll
      roll.pid_d_part =  roll.pid_d_gain * (roll.pid_error - roll.pid_pre_error);                 // Thành phần D roll
      if(roll.pid_i_part > roll.pid_limit)roll.pid_i_part = roll.pid_limit;                       // Giới hạn thành phần I roll vì cộng dồn
      else if(roll.pid_i_part < roll.pid_limit * -1)roll.pid_i_part = roll.pid_limit * -1;
  
      pid_output = roll.pid_p_part + roll.pid_i_part + roll.pid_d_part;                           // Ngõ ra bộ điều khiển PID roll = thành phần P roll + thành phần I roll + thành phần D roll
      if(pid_output > roll.pid_limit)pid_output = roll.pid_limit;
      else if(pid_output < roll.pid_limit * -1)pid_output = roll.pid_limit * -1;
      
      roll.pid_pre_error = roll.pid_error;                                                        // Gán sai số trước đó = sai số hiện tại để lần lấy mẫu kế tiếp tính toán tiếp
      return pid_output;
      break; 
    }
    case pitch_axis: {                                                                            // case 2 là trục pitch
      pitch.pid_error  =  pitch.pid_input - pitch.pid_setpoint;                                   // error pitch = input pitch - setpoint pitch
      
      pitch.pid_p_part =  pitch.pid_p_gain * pitch.pid_error;                                     // Thành phần P pitch
      pitch.pid_i_part += pitch.pid_i_gain * pitch.pid_error;                                     // Thành phần I pitch
      pitch.pid_d_part =  pitch.pid_d_gain * (pitch.pid_error - pitch.pid_pre_error);             // Thành phần D pitch
      if(pitch.pid_i_part > pitch.pid_limit)pitch.pid_i_part = pitch.pid_limit;                   // Giới hạn thành phần I pitch vì cộng dồn
      else if(pitch.pid_i_part < pitch.pid_limit * -1)pitch.pid_i_part = pitch.pid_limit * -1;
  
      pid_output = pitch.pid_p_part + pitch.pid_i_part + pitch.pid_d_part;                        // Ngõ ra bộ điều khiển PID pitch = thành phần P pitch + thành phần I pitch + thành phần D pitch
      if(pid_output > pitch.pid_limit)pid_output = pitch.pid_limit;
      else if(pid_output < pitch.pid_limit * -1)pid_output = pitch.pid_limit * -1;
      
      pitch.pid_pre_error = pitch.pid_error;                                                      // Gán sai số trước đó = sai số hiện tại để lần lấy mẫu kế tiếp tính toán tiếp
      return pid_output;
      break; 
    }
    case yaw_axis: {                                                                              // case 3 là trục yaw
      yaw.pid_error  =  yaw.pid_input - yaw.pid_setpoint;                                         // error yaw = input yaw - setpoint yaw
      
      yaw.pid_p_part =  yaw.pid_p_gain * yaw.pid_error;                                           // Thành phần P yaw
      yaw.pid_i_part += yaw.pid_i_gain * yaw.pid_error;                                           // Thành phần I yaw
      yaw.pid_d_part =  yaw.pid_d_gain * (yaw.pid_error - yaw.pid_pre_error);                     // Thành phần D yaw
      if(yaw.pid_i_part > yaw.pid_limit)yaw.pid_i_part = yaw.pid_limit;                           // Giới hạn thành phần I yaw vì cộng dồn
      else if(yaw.pid_i_part < yaw.pid_limit * -1)yaw.pid_i_part = yaw.pid_limit * -1;
  
      pid_output = yaw.pid_p_part + yaw.pid_i_part + yaw.pid_d_part;                              // Ngõ ra bộ điều khiển PID yaw = thành phần P yaw + thành phần I yaw + thành phần D yaw
      if(pid_output > yaw.pid_limit)pid_output = yaw.pid_limit;
      else if(pid_output < yaw.pid_limit * -1)pid_output = yaw.pid_limit * -1;
      
      yaw.pid_pre_error = yaw.pid_error;                                                          // Gán sai số trước đó = sai số hiện tại để lần lấy mẫu kế tiếp tính toán tiếp
      return pid_output;                              
      break; 
    }
    default : {
      break;
    }
  }
}

void setup() {
  Serial.begin(9600);
  initial_pid_gain();
  Wire.begin();
  gy521_mpu6050_init();
  gyro_get_offset();
}

void loop() {
  read_mpu_data();                                                    // Đọc dữ liệu accel và gyro thô từ mpu6050                                   
  calib_gyro_3_axis(); 
  correct_roll_pitch_yaw_data(); 
  low_pass_filter_pid_input();
  check_quadrotor_status(); 
  calculate_angle_roll_pitch();
  calculate_pid_setpoint(); 
  roll.pid_output  = calculate_pid(roll_axis);                        // Tính toán ngõ ra bộ PID cho trục roll
  pitch.pid_output = calculate_pid(pitch_axis);                       // Tính toán ngõ ra bộ PID cho trục pitch
  yaw.pid_output   = calculate_pid(yaw_axis); 

    Serial.println("=================================");
    Serial.print( 10 - pitch.pid_output - roll.pid_output + yaw.pid_output);             // Tính xung cho ESC 4 ClockWise (motor 4 quay cùng chiều kim đồng hồ)
    Serial.print("------");
    Serial.println(10 - pitch.pid_output + roll.pid_output - yaw.pid_output );          // Tính xung cho ESC 1 CounterClockWise (motor 1 quay ngược chiều kim đồng hồ)

    Serial.print(10 + pitch.pid_output - roll.pid_output - yaw.pid_output);             // Tính xung cho ESC 3 CounterClockWise (motor 3 quay ngược chiều kim đồng hồ)
    Serial.print("------");
    Serial.println(10 + pitch.pid_output + roll.pid_output + yaw.pid_output);  
    Serial.println("=================================");                                // Tính xung cho ESC 2 ClockWise (motor 2 quay cùng chiều kim đồng hồ)
    Serial.println("\n");

             
    /*
    phần cứng
          motor_4 CW (pin 7)   front    motor_1 CCW(pin 4)
                   \\                         //
                    \\                       //
          left       = = = = = = = = = = = = =     right
                    //                       \\
                   //                         \\
          motor_3 CCW (pin 6)   back    motor_2 CW (pin 5)
    */
  //delay(500);
}
