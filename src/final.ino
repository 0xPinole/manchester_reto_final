#include <ros.h>
#include "motor_input.h"
#include "motor_output.h"
#include <std_msgs/Float32.h>
#include <digitalWriteFast.h>

const int EncA = 2;       //  Entrada encoder A 
const int EncB = 3;       //  Entrada encoder B
const int In1 = 4;        //  Puente H 1 
const int In2 = 5;        //  Puente H 2 
const int EnA = 11;       //  PWM

float sgn_ros;

volatile float pos = 0, pos_last = 0, velocity = 0;
float resolution = 0.0183;  //Definir resolución del encoder - radianes

long counter = 0;
volatile bool BSet = 0, ASet=0;

ros::NodeHandle nh;

reto_final::motor_output str_msg;

void direction_fcn(const reto_final::motor_input& msg){
  sgn_ros = msg.input;
  digitalWrite(In1, sgn_ros>0.01); 
  digitalWrite(In2, sgn_ros<-0.01);
  analogWrite(EnA, abs(sgn_ros*255));
  str_msg.st = (sgn_ros>0.01 || sgn_ros<-0.01) ? "Running" : "Stopped";  
  str_msg.tm = msg.tm;
}

ros::Publisher pub_3("motor_output", &str_msg);
ros::Subscriber<reto_final::motor_input> sub_1("motor_input", direction_fcn);
//ros::Rate loop_rate(100);

void setup (){   
  pinMode(EnA, OUTPUT);         //Salida de PWM      
  pinMode(In1, OUTPUT);         //Pin declarado como salida para el motor
  pinMode(In2, OUTPUT);         //Pin declarado como salida para el motor
  pinMode(EncA, INPUT_PULLUP);          //Pin declarado como entrada, señal A del encoder de cuadratura
  pinMode(EncB, INPUT_PULLUP);          //Pin declarado como entrada, señal B del encoder de cuadratura
  attachInterrupt(0, Encoder, CHANGE);  //Leer señal A del encoder por interrupción, y asignar a Encoder
  
  TCCR1A = 0;
  TCCR1B = (1 << WGM12) | (1 << CS10) | (1 << CS12);
  OCR1A = 312;
  TIMSK1 = (1 << OCIE1A);

  sei();
  
  nh.initNode();
  nh.subscribe(sub_1);
}

void loop() 
{ 
  str_msg.output = velocity;
  pub_3.publish( &str_msg );
  nh.spinOnce(); 
  delay(1);
}

void Encoder(){
  BSet = digitalReadFast(EncB);
  ASet = digitalReadFast(EncA);
  counter += (BSet == ASet) || -1;
  pos = counter * resolution;
}

ISR(TIMER1_COMPA_vect) {
  velocity = abs((pos - pos_last) / 0.02);
  pos_last = pos;
}
