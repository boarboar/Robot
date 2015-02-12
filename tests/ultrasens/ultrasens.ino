
/*
echo DLR=100,100 > /dev/ttyATH0
cat /dev/ttyATH0
*/

//#define TTY_SPEED 38400
#define TTY_SPEED 38400

#define US_IN    P1_6
#define US_OUT   P1_7

#define inputPin    P1_6
#define outputPin   P1_7

volatile int16_t us_dist=0; 
      
void setup()
{
  //analogFrequency(100);

  pinMode(US_OUT, OUTPUT);     
  pinMode(US_IN, INPUT);       
  //pinMode(RED_LED, OUTPUT);     
 
   Serial.begin(TTY_SPEED);
   //digitalWrite(RED_LED, HIGH);  
   //delay(100);
   //digitalWrite(RED_LED, LOW);

    //while (Serial.available()) Serial.read();  // eat garbage

}

void loop()
{
  // put your main code here, to run repeatedly:
  
  digitalWrite(outputPin, LOW);  
  delayMicroseconds(2); 
  digitalWrite(outputPin, HIGH); // Pulse for 10 s to trigger ultrasonic  detection 
  delayMicroseconds(10); 
  digitalWrite(outputPin, LOW);    
  //uint32_t distance = pulseIn(inputPin, HIGH, 25000);  // Read receiver pulse time. TO is 25ms.
 uint32_t distance = pulseIn(inputPin, HIGH); 
  int16_t d=(int16_t)(distance/58);
  Serial.println(d);   //Ourput distance                  
  
  
  delay(1000);   
  
  /*
  readUSDist(); 
  
   Notify();
  */ 
}

void Notify() {
  //addJson("U", (int16_t)(us_dist));
  Serial.println(us_dist);
}

void readUSDist() {
  digitalWrite(US_OUT, LOW);
  delayMicroseconds(2);
  digitalWrite(US_OUT, HIGH);
  delayMicroseconds(10);
  digitalWrite(US_OUT, LOW);
  //uint32_t d=pulseIn(US_IN, HIGH, 25000);
  uint32_t d=pulseIn(US_IN, HIGH, 50000);
  us_dist=(int16_t)(d/58);
}

 void addJson(const char *name, int16_t value) {
  Serial.print("\"");
  Serial.print(name);
  Serial.print("\":");
  Serial.print(value);
  Serial.print(",");
 }
 
