#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>
#include <Timer.h>

// Define ROS Settings
#define TOPIC_NAME "/speeds/A"
#define MAX_PUBLISHERS 20
#define MAX_SUBSCRIBERS 20
#define PUBLISHERS_BUFFER_SIZE 2048
#define SUBSCRIBERS_BUFFER_SIZE 2048

// Settings
#define SEND_PERIOD 5
#define DELAY 10000
#define DURATION 10000
#define PWM_VAL 32767
#define BAUD_RATE 115200
#define ENCODER_RESOLUTION 2400

// Pins
int PIN_ENCODER_A = PB12;
int PIN_ENCODER_B = PB13;
int PIN_MOTOR_DIR = PA7;
int PIN_MOTOR_PWM = PA6;

// Define TTL pins
#define PIN_UART1_RX PA10
#define PIN_UART1_TX PA9
#define PIN_UART3_RX PB11
#define PIN_UART3_TX PB10

#define SERIAL_ROS Serial3
#define SERIAL_DEBUG Serial1
HardwareSerial Serial1(PIN_UART1_RX, PIN_UART1_TX);
HardwareSerial Serial3(PIN_UART3_RX, PIN_UART3_TX);

void encoderISR_AA(void);
void encoderISR_BA(void);

long int counter = 0 ,lastCounter = 0;
unsigned long current_pwm = 0;

Timer myTimer;

// Handle ROS node settings
ros::NodeHandle_<ArduinoHardware, MAX_PUBLISHERS, MAX_SUBSCRIBERS, PUBLISHERS_BUFFER_SIZE, SUBSCRIBERS_BUFFER_SIZE> nh;

std_msgs::String msg;
ros::Publisher pupA(TOPIC_NAME, &msg);

void setup()
{
  init_ros();
  init_encoders();
  init_motor_drivers();

  // new core pwm setup
  analogWriteFrequency(2000);
  analogWriteResolution(16);

  myTimer.after(DELAY, apply_input);
  myTimer.every(SEND_PERIOD, send_data);

  //  Start scheduler and LED indication
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
}

void loop()
{
  myTimer.update();
  nh.spinOnce();
}

void apply_input()
{
  current_pwm = PWM_VAL;
  digitalWrite(PIN_MOTOR_DIR, HIGH);
  analogWrite(PIN_MOTOR_PWM, PWM_VAL);
  myTimer.after(DURATION, stop_input);
}

void stop_input()
{
  current_pwm = 0;
  analogWrite(PIN_MOTOR_PWM, 0);
}

void send_data()
{
  String row;
//  long long int m_sec = millis();
  row = String(millis()) + String(',');
  row += String(current_pwm) + String(',');
  row += String(counter) + String(',');
  row += String(((counter - lastCounter) / 0.005) * 2 * PI / ENCODER_RESOLUTION);
  lastCounter = counter;
  
  std_msgs::String msg;
  msg.data = row.c_str();
  
  pupA.publish(&msg);
}

void encoderISR_AA(void)
{
  counter += digitalRead(PIN_ENCODER_A) == digitalRead(PIN_ENCODER_B) ? -1 : 1;
}
void encoderISR_BA(void)
{
  counter += digitalRead(PIN_ENCODER_A) != digitalRead(PIN_ENCODER_B) ? -1 : 1;
}

void init_encoders()
{
  pinMode(PIN_ENCODER_A, INPUT_PULLUP);
  pinMode(PIN_ENCODER_B, INPUT_PULLUP);
  attachInterrupt(PIN_ENCODER_A, encoderISR_AA, CHANGE);
  attachInterrupt(PIN_ENCODER_B, encoderISR_BA, CHANGE);
}

void init_motor_drivers()
{
  pinMode(PIN_MOTOR_DIR, OUTPUT);
  pinMode(PIN_MOTOR_PWM, OUTPUT);
  digitalWrite(PIN_MOTOR_DIR, HIGH);
  digitalWrite(PIN_MOTOR_PWM, LOW);
}

void init_ros()
{
  (nh.getHardware())->setPort(&SERIAL_ROS);
  (nh.getHardware())->setBaud(BAUD_RATE);
  nh.initNode();
  nh.advertise(pupA);
}
