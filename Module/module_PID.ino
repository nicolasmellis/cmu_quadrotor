//Velocity: positive if going up, negative if going down
//CurHeight: current sonar reading median
//SetHeight: where we want to be

int height_error = setHeight - curHeight - velocity;
int throttle = height_error * KP;
if(throttle > 255) throttle = 255;
if(throttle < -255) throttle = -255;

if(throttle >= 0){

  Wire.write(5); //Up
  Wire.write(throttle); //magnitude
}
else{
  
  Wire.write(6); //Down
  Wire.write(-throttle); //magnitude
  
}

