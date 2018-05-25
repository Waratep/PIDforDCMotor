#define motor1 8
#define motor2 9
#define motorPwm 10
#define ADC A0 

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

double Kp = 30.00;
double Ki = 0;
double Kd = 100.00;
double error = 0.00;
double last_error = 0.00;
double sum_error = 0.00;
double Pwm_output = 0.00;

void setup() {

  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  cbi(ADCSRA, ADPS0);
  
  Serial.begin(115200);

  pinMode(motor1,OUTPUT);
  pinMode(motor2,OUTPUT);
  pinMode(motorPwm,OUTPUT);

   cli();//stop interrupts
  //set timer interrupt at 8kHz
  TCCR2A = 0;// set entire TCCR2A register to 0
  TCCR2B = 0;// same for TCCR2B
  TCNT2  = 0;//initialize counter value to 0
  // set compare match register for 8khz increments
  OCR2A = 249;// = (16*10^6) / (8000*8) - 1 (must be <256)
  // turn on CTC mode
  TCCR2A |= (1 << WGM21);
  // Set CS21 bit for 8 prescaler
  TCCR2B |= (1 << CS21);
  // enable timer compare interrupt
  TIMSK2 |= (1 << OCIE2A);
  sei();//allow interrupts


}

void loop() {
  
}

ISR(TIMER2_COMPA_vect) { //timer1 interrupt 8kHz

  /// range 40 - 540
  last_error = error;
  error = map(analogRead(ADC),40,540,-50,50);
  error = error > 50 ? 50:error; 
  error = error < -50 ? -50:error;
  sum_error += error;

  if(error < 0){ // motor1 = 1 and motor2 = 0 is left
      digitalWrite(motor1,0);
      digitalWrite(motor2,1);
      Serial.print("  left  ");
  }else{ // motor1 = 0 and motor2 = 1 is right
      digitalWrite(motor1,1);
      digitalWrite(motor2,0);
      Serial.print("  right  ");
  }
  
  Pwm_output = (Kp*error) + (Ki*sum_error) + (Kd*(error - last_error));
  Pwm_output = Pwm_output > 255 ? 255:Pwm_output;
  Pwm_output = Pwm_output < -255 ? -255:Pwm_output;

  if(Pwm_output < 0){
    Pwm_output = Pwm_output*-1;
  }
  analogWrite(motorPwm,(int)Pwm_output);

  Serial.print(" error : ");
  Serial.print(error);
  Serial.print("  pwm : ");
  Serial.println(Pwm_output);

}
