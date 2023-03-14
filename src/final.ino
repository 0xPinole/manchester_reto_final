#include <ros.h>
#include "motor_input.h"
#include "motor_output.h"
#include <digitalWriteFast.h>

const int EncA = 2;       //  Entrada encoder A 
const int EncB = 3;       //  Entrada encoder B
const int In1 = 4;        //  Puente H 1 
const int In2 = 5;        //  Puente H 2 
const int EnA = 11;       //  PWM
const float resolution = 0.0109986;  //Definir resolución del encoder - radianes

float sgn_ros, dir = 0;
long counter = 0;
volatile float pos = 0, pos_last = 0, velocity = 0;

ros::NodeHandle nh;
reto_final::motor_output str_msg;

void direction_fcn(const reto_final::motor_input& msg){
  sgn_ros = msg.input;
  digitalWrite(In1, sgn_ros>0.01); 
  digitalWrite(In2, sgn_ros<-0.01);
  analogWrite(EnA, min(abs(sgn_ros*255), 255));
  dir = (sgn_ros < 0.0) ? -1 : 1;
  str_msg.st = (sgn_ros>0.01 || sgn_ros<-0.01) ? ((dir) ? "Right" : "Left") : "Stopped";  
  str_msg.tm = msg.tm;
}

ros::Subscriber<reto_final::motor_input> sub_1("motor_input", direction_fcn);
ros::Publisher pub_3("motor_output", &str_msg);

void setup() {   
  pinMode(EnA, OUTPUT);         //Salida de PWM      
  pinMode(In1, OUTPUT);         //Pin declarado como salida para el motor
  pinMode(In2, OUTPUT);         //Pin declarado como salida para el motor
  pinMode(EncA, INPUT_PULLUP);          //Pin declarado como entrada, señal A del encoder de cuadratura
  pinMode(EncB, INPUT_PULLUP);          //Pin declarado como entrada, señal B del encoder de cuadratura
  attachInterrupt(0, Encoder, CHANGE);  //Leer señal A del encoder por interrupción, y asignar a Encoder
  
  TCCR1A = 0;
  TCCR1B = (1 << WGM12) | (1 << CS10) | (1 << CS12);
  OCR1A = 78; //( (78 + 1) * 1024 ) / 16Mhz (Arduino Uno)
  TIMSK1 = (1 << OCIE1A);
  sei();
  
  nh.initNode();
  nh.subscribe(sub_1);
  nh.advertise(pub_3);
}

void loop() { 
  str_msg.output = velocity*dir;
  pub_3.publish( &str_msg );
  nh.spinOnce();
  delay(1);
}

void Encoder() {
  counter += (digitalReadFast(EncB) == digitalReadFast(EncA)) || -1;
  pos = counter * resolution;
}

ISR(TIMER1_COMPA_vect) {
  velocity = abs((pos - pos_last) / 0.005); //min_delta_time =  0.00053
  pos_last = pos;
}