//RECEIVER


void setup()
{
   Serial.begin(57600);
}

void loop()
{
  
  int val = Serial.read();
  //Serial.println(val);
  
  if(val != -1)
  {
    
    Serial.println(val);
  }
}

