
//Returns the length of the pulse in microseconds.
//pulseIn(pin, value, timeout) 
//timeout (optional): the number of microseconds to wait for the pulse to start; default is one second (unsigned long) 

int inputPin=P1_6;  // define ultrasonic signal receiver pin  ECHO to D4 
int outputPin=P1_7; // define ultrasonic signal transmitter pin  TRIG to D5 
      
void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600); 
  pinMode(inputPin, INPUT); 
  pinMode(outputPin, OUTPUT); 
}

void loop()
{
  // put your main code here, to run repeatedly:
  digitalWrite(outputPin, LOW);  
  delayMicroseconds(2); 
  digitalWrite(outputPin, HIGH); // Pulse for 10 s to trigger ultrasonic  detection 
  delayMicroseconds(10); 
  digitalWrite(outputPin, LOW);    
  uint32_t distance = pulseIn(inputPin, HIGH, 25000);  // Read receiver pulse time. TO is 25ms. 
  distance= distance/58;   // Transform pulse time to distance 
  Serial.println(distance);   //Ourput distance                  
  delay(1000);   
}
