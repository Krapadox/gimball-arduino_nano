#include "I2Cdev.h"
#include "MPU6050.h"
#include <Wire.h>
#include <Servo.h>

MPU6050 mpu;

// Servos
Servo servoRoll;
Servo servoPitch;

// MPU RAW
int ax, ay, az;
int gx, gy, gz;

// Tiempo
unsigned long tiempo_prev;
float dt;

// Ángulos
float ang_x, ang_y;
float ang_x_prev, ang_y_prev;

/* -- Config de PID -- */
float roll_setpoint = 0;   // Setpoints
float pitch_setpoint = 0;

float roll_error, pitch_error;  // Errores
float roll_prev_error = 0;
float pitch_prev_error = 0;

float roll_p, roll_i = 0, roll_d;     // PID términos
float pitch_p, pitch_i = 0, pitch_d;

float Kp = 4.0;   // Constantes PID
float Ki = 0.1;
float Kd = 0.1;

float roll_PID, pitch_PID; // Salida PID

void lecturaMPU()
{
  // Lectura de giroscopo y acelerometro
  mpu.getAcceleration(&ax, &ay, &az);
  mpu.getRotation(&gx, &gy, &gz);
  
  dt = (millis()-tiempo_prev)/1000.0;
  tiempo_prev=millis();
  
  //Calcular los ángulos con acelerometro
  float accel_ang_x=atan(ay/sqrt(pow(ax,2) + pow(az,2)))*(180.0/3.14);
  float accel_ang_y=atan(-ax/sqrt(pow(ay,2) + pow(az,2)))*(180.0/3.14);
  
  //Calcular angulo de rotación con giroscopo y filtro complemento  
  ang_x = 0.98*(ang_x_prev+(gx/131)*dt) + 0.02*accel_ang_x;
  ang_y = 0.98*(ang_y_prev+(gy/131)*dt) + 0.02*accel_ang_y;
  
  
  ang_x_prev=ang_x;
  ang_y_prev=ang_y;
}

void calcularPID() {
  // Error
  roll_error  = ang_y - roll_setpoint;
  pitch_error = ang_x - pitch_setpoint;

  // Proporcional
  roll_p  = Kp * roll_error;
  pitch_p = Kp * pitch_error;

  // Integral
  if (roll_error > -3 && roll_error < 3)
    roll_i += Ki * roll_error;

  if (pitch_error > -3 && pitch_error < 3)
    pitch_i += Ki * pitch_error;

  // Derivativo
  roll_d  = Kd * ((roll_error - roll_prev_error) / dt);
  pitch_d = Kd * ((pitch_error - pitch_prev_error) / dt);

  // PID total
  roll_PID  = roll_p + roll_i + roll_d;
  pitch_PID = pitch_p + pitch_i + pitch_d;

  // Limitar salida
  roll_PID  = constrain(roll_PID,  -90, 90);
  pitch_PID = constrain(pitch_PID, -90, 90);

  roll_prev_error  = roll_error;
  pitch_prev_error = pitch_error;
}

void moverServos() {
  servoRoll.write(90 - roll_PID);
  servoPitch.write(90 + pitch_PID);
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu.initialize();

  servoRoll.attach(3);
  servoPitch.attach(5);

  tiempo_prev = millis();

  if (mpu.testConnection())
    Serial.println("MPU OK");
  else
    Serial.println("Error MPU");
}

void loop() {
  lecturaMPU();
  calcularPID();
  moverServos();

  //Mostrar los angulos separadas por un [tab]
  Serial.print("Rotacion en X:  ");
  Serial.print(ang_x); 
  Serial.print("\tRotacion en Y: ");
  Serial.println(ang_y);

}
