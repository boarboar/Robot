#include "robot_reader.h"

void CommandReader::Init() {
  bytes = 0;
  buf[bytes] = 0;
  while (Serial.available()) Serial.read();  // eat garbage
}

// read serial data into buffer. execute command
boolean CommandReader::ReadSerialCommand()
{
  while (Serial.available() && bytes < BUF_SIZE)
  {
    buf[bytes] = Serial.read();
    if (buf[bytes] == 10 || buf[bytes] == 13)
    {
      if (bytes > 0) { 
        buf[bytes]=0; 
        return true; 
      } 
      return false; // skip 10 or 13 left         
    }
    bytes++;
  }
  if(bytes>=BUF_SIZE) { 
    bytes=0; //overflow, probably caused hang up at start...    
    buf[bytes]=0; 
    return true; // this is for test only... !!!!!!!!!!!!!!!!!!!!!!!!!!
  }
  return false;
}

boolean CommandReader::Match(const char *cmd) {
  while(pos<bytes && *cmd && buf[pos]==*cmd) {
    pos++;
    cmd++;
  }
  /*
  if(!*cmd) return pos;
  else return 0;
  */
  return *cmd==0;
}

//boolean CommandReader::ReadInt(int16_t *val) {
int16_t CommandReader::ReadInt() {
  int16_t i=0;
  boolean sign=false;
  while(isspace(buf[pos])) pos++;
  if(buf[pos]=='+') pos++;
  else if(buf[pos]=='-') { 
    sign=true; 
    pos++;
  }
  while (isdigit(buf[pos]))
  {
    i *= 10;
    i += buf[pos] - '0';
    pos++;
  }
  //*val=sign ? -i : i;
  //return true;
  if(sign) i=-i;
  return i;
}

char CommandReader::ReadChar() {
  if(pos>=bytes) return 0;
  char c = buf[pos];
  pos++;
  return c;
}

