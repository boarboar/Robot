
// TODO
// do not clear log on drive
// clear log after output
// do not redrive if the drive parameters are the same

#define BUF_SIZE 32
byte buf[BUF_SIZE];
byte bytes = 0;

// common vars //
const unsigned long cycleTimeout = 100;
const unsigned long cmdTimeout = 600;
unsigned long lastCommandTime;
unsigned long lastCycleTime;
unsigned long last_dur=0;

enum EnumCmd { EnumCmdDrive=1, EnumCmdTest, EnumCmdStop, EnumCmdLog, EnumCmdContinueDrive };  
enum EnumError { EnumErrorUnknown=-1, EnumErrorBadSyntax=-2, EnumErrorBadParam=-3, EnumErrorNone=-100};  

int8_t cmdResult=EnumErrorNone;
uint8_t drv_dir[2]={0,0}; 
uint8_t cur_power[2]={0,0}; 
uint8_t cmd_power[2]={0,0}; 
uint16_t enc_cnt[2]={0,0}; 
uint8_t last_enc_rate[2]={0,0}; 
uint8_t trg_rate[2]={0,0}; 
uint8_t calib_enc_rate=0; 

#define PID_LOG_SZ 16
uint8_t pid_log_cnt=0;
uint8_t pid_log_dur[PID_LOG_SZ];
uint8_t pid_log_rate[PID_LOG_SZ][2];
int8_t pid_log_err[PID_LOG_SZ][2];
uint8_t pid_log_pow[PID_LOG_SZ][2];

#define M_POW_LOW  150
#define M_POW_HIGH 200
#define M_POW_MAX  255

#define M_PID_KP   4

// flags
uint8_t needNotify=0;
boolean IsDrive = false;

uint8_t green_st=0; 

#define M1_OUT_1  P1_3
#define M1_OUT_2  P1_4
#define M1_EN     P2_1 // analog write

#define M2_OUT_1  P2_3
#define M2_OUT_2  P2_4
#define M2_EN     P2_5 // analog write

//    NOT_ON_TIMER, /*  2 - P1.0 */
//    T0A0,         /*  3 - P1.1, note: A0 output cannot be used with analogWrite */
//    T0A1,         /*  4 - P1.2 */
//    NOT_ON_TIMER, /*  5 - P1.3 */
//    NOT_ON_TIMER, /*  6 - P1.4, TODO: T0A2??, not on DIP20 devices according tot datasheet  */
//    T0A0,         /*  7 - P1.5, note: A0 output cannot be used with analogWrite  */
//    T1A0,         /*  8 - P2.0, note: A0 output cannot be used with analogWrite */
//    T1A1,         /*  9 - P2.1 */
//    T1A1,         /* 10 - P2.3 */
//    T1A0,         /* 11 - P2.4 note: A0 output cannot be used with analogWrite  */
//    T1A2,         /* 12 - P2.5 */
//    T1A2,         /* 13 - P2.6 */
//    T0A1,         /* 14 - P1.6 */
//    NOT_ON_TIMER /* 15 - P1.7 */

// Besides, the different pins for the separate analogwrite channels should be on the separate timer+compare 

#define ENC1_IN  P1_5
#define ENC2_IN  P2_0

void setup()
{
  //analogFrequency(800);
  
  int ports[6]={M1_OUT_1,M1_OUT_2,M2_OUT_1,M2_OUT_2, M1_EN, M2_EN };
  for(int i=0;i<6;i++) {
    digitalWrite(ports[i], LOW); 
    pinMode(ports[i], OUTPUT);
  }
  pinMode(ENC1_IN, INPUT_PULLUP);     
  attachInterrupt(ENC1_IN, encodeInterrupt_1, RISING); 
  pinMode(ENC2_IN, INPUT_PULLUP);     
  attachInterrupt(ENC2_IN, encodeInterrupt_2, RISING); 

  pinMode(RED_LED, OUTPUT);     
  pinMode(GREEN_LED, OUTPUT);     
  for(int i=0; i<6; i++) {
    digitalWrite(RED_LED, HIGH); delay(100); digitalWrite(RED_LED, LOW); 
    digitalWrite(GREEN_LED, HIGH); delay(100); digitalWrite(GREEN_LED, LOW);
  }
  Serial.begin(9600);
  lastCommandTime = lastCycleTime = millis();  
  digitalWrite(RED_LED, HIGH);  
  /*
    // test sequence
  IsDrive=1; Drive(1, 254, 1, 170);   // right - to amplify
  delay(1000);
  StopDrive();
  delay(500);
  IsDrive=1; Drive(1, 170, 1, 254);
  delay(1000);
  digitalWrite(RED_LED, LOW);  
  StopDrive();
  delay(500);  
  digitalWrite(RED_LED, HIGH);  
  IsDrive=1; Drive(2, 180, 2, 180);
  delay(500);
  */
  // calibration sequence
  IsDrive=1; Drive(1, M_POW_HIGH, 1, M_POW_HIGH);
  delay(1000);
  StopDrive();
  calib_enc_rate = (last_enc_rate[0]+last_enc_rate[1])/2;
  digitalWrite(RED_LED, LOW);

  //enc_cnt[0]=enc_cnt[1]=0;  
}

void loop()
{
  long cycleTime = millis();
  if ( cycleTime < lastCycleTime) lastCycleTime=0; // wraparound   
  uint16_t ctime = cycleTime - lastCycleTime;
  if ( ctime >= cycleTimeout) { // working cycle    
    if (IsDrive) {
      if (CheckCommandTimeout()) StopDrive();
      else PID(ctime);
    }
    lastCycleTime=cycleTime; //NB!
    if(needNotify) Notify();    
  }  

  if(ReadSerialCommand()) {
    if(needNotify) { delay(cycleTimeout); Notify(); }
    cmdResult = Parse(); // postpone cmd report for 100ms
    bytes = 0; // empty input buffer (only one command at a time)
    if(cmdResult==EnumCmdDrive || (cmdResult==EnumCmdContinueDrive && !IsDrive) ) {
      lastCycleTime=millis(); //NB!
      StartDrive();
      lastCommandTime = millis();
    } else if(cmdResult==EnumCmdContinueDrive && IsDrive) {
      lastCommandTime = millis();
    }else if(cmdResult==EnumCmdStop) {
      StopDrive();
    }
    needNotify=1;    
    //lastCycleTime=millis(); //NB!
  }
}

// read serial data into buffer. execute command
boolean ReadSerialCommand()
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
  if(bytes>=BUF_SIZE) bytes=0; //overflow, probably caused hang up at start...    
  return false;
}

void Notify() {
  Serial.print("RET:"); Serial.print(cmdResult); Serial.print(":");
  switch(cmdResult) {
    //case EnumCmdDrive: Serial.print(drv_dir[0]);Serial.print(","); Serial.print(trg_power[0]);Serial.print(";");Serial.print(drv_dir[1]);Serial.print(","); Serial.print(trg_power[1]); break;    
    case EnumCmdDrive: 
    case EnumCmdContinueDrive: 
      Serial.print(drv_dir[0]);Serial.print(","); Serial.print(cmd_power[0]);Serial.print(";");
      Serial.print(drv_dir[1]);Serial.print(","); Serial.print(cmd_power[1]);Serial.print(";");
      Serial.print(trg_rate[0]);Serial.print(","); Serial.print(trg_rate[1]);
      break; 
    case EnumCmdTest: 
      Serial.print(last_dur); Serial.print(";"); Serial.print(enc_cnt[0]);Serial.print(","); Serial.print(enc_cnt[1]);Serial.print(";"); 
      Serial.print(last_enc_rate[0]);Serial.print(","); Serial.print(last_enc_rate[1]);Serial.print(";"); 
      Serial.print(calib_enc_rate);Serial.print(";"); Serial.print(trg_rate[0]);Serial.print(","); Serial.print(trg_rate[1]); 
      break;
    case EnumCmdStop:
      Serial.print(cur_power[0]);Serial.print(","); Serial.print(cur_power[1]);Serial.print(";");
      //Serial.print(trg_rate[0]);Serial.print(","); Serial.print(trg_rate[1]); Serial.print(";"); 
      Serial.print(last_dur);Serial.print(","); 
      Serial.print(last_enc_rate[0]);Serial.print(","); Serial.print(last_enc_rate[1]); 
      break;
    case EnumCmdLog: {
      Serial.print(pid_log_cnt);Serial.print(";");
      for(uint8_t i=0; i<pid_log_cnt; i++) {
        //Serial.println();
        Serial.print(pid_log_dur[i]);Serial.print(";"); Serial.print(pid_log_rate[i][0]);Serial.print(","); Serial.print(pid_log_rate[i][1]);
        Serial.print("("); Serial.print(pid_log_err[i][0]);Serial.print(","); Serial.print(pid_log_err[i][1]); Serial.print(")"); 
        Serial.print(":"); Serial.print(pid_log_pow[i][0]);Serial.print(","); Serial.print(pid_log_pow[i][1]); Serial.print(";"); 
      }
      pid_log_cnt=0;
      break;
    }
    default:;
   }
  Serial.println();
  needNotify=0; 
}

boolean CheckCommandTimeout()
{
  long commandTime = millis();
  if ( commandTime >= lastCommandTime) commandTime -= lastCommandTime; 
  else lastCommandTime = 0;
  if (commandTime > cmdTimeout) return true;
}

int8_t Parse()
// Expect: "DrvLR=100,100"
{  
  byte pos;
  int m;
  
  if(pos=Match("DrvLR")) {    
    boolean chg=false;
    if(pos>=bytes || buf[pos]!='=') return EnumErrorBadSyntax;
    for(int i=0; i<2; i++) {      
      pos++;    
      pos = bctoi(pos, &m);      
      if(m<-255 || m>254) return EnumErrorBadParam;
      if(buf[pos] != (i==0 ? ',' : 0)) return EnumErrorBadSyntax;
      if(m==0)       { if(drv_dir[i]!=0) { drv_dir[i]=0; chg=true;} cmd_power[i]=0; } 
      else if (m>0)  { if(drv_dir[i]!=1 || cmd_power[i]!=m) { cmd_power[i]=m; drv_dir[i]=1; chg=true;} } 
      else           { if(drv_dir[i]!=2 || cmd_power[i]!=-m) {cmd_power[i]=-m; drv_dir[i]=2; chg=true;} }  
    }
    if(!drv_dir[0] && !drv_dir[1]) return EnumCmdStop;
    return chg ? EnumCmdDrive : EnumCmdContinueDrive;
  } else if(pos=Match("Test")) {
    return EnumCmdTest;
  }
  else if(pos=Match("Log")) {
    return EnumCmdLog;
  }
  else return EnumErrorUnknown;
}

void StartDrive() 
{
  //pid_log_cnt=0;
  
  for(int i=0; i<2; i++) {
    if(drv_dir[i]) {
       trg_rate[i]=map(cmd_power[i], 0, 100, 0, calib_enc_rate);
       cur_power[i]=map(cmd_power[i], 0, 100, M_POW_LOW, M_POW_HIGH);        
     } else {
       trg_rate[i]=0;
       cur_power[i]=0;
     }    
    pid_log_rate[pid_log_cnt][i]=trg_rate[i];
    pid_log_err[pid_log_cnt][i]=-111;
    pid_log_pow[pid_log_cnt][i]=cur_power[i];
  }
  pid_log_dur[pid_log_cnt]=0; 
  
  if(pid_log_cnt<PID_LOG_SZ-1) pid_log_cnt++;

  enc_cnt[0]=enc_cnt[1]=0;  
  Drive(drv_dir[0], cur_power[0], drv_dir[1], cur_power[1]); // change interface
  if (!IsDrive)
  {
    IsDrive = true;
    digitalWrite(RED_LED, HIGH);
  }        
  //lastCommandTime = millis();
}

void StopDrive() 
{
  if(!IsDrive) return;
  //pid_log_cnt=0;
  Drive(0, 0, 0, 0);
  IsDrive = false;
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, LOW);
  unsigned long n=millis();
  if(n>lastCommandTime) {
    last_dur=n-lastCommandTime;
    last_enc_rate[0]=(uint8_t)(enc_cnt[0]*100/last_dur); // per 100ms (ca 10)
    last_enc_rate[1]=(uint8_t)(enc_cnt[1]*100/last_dur);
  }
  else { last_dur=0; last_enc_rate[0]=last_enc_rate[1]=0;}
}

void PID(uint16_t ctime)
{
  if(ctime>0) {
    for(int i=0; i<2; i++) {
      uint8_t rate = (uint8_t)(enc_cnt[i]*100/ctime);
      int8_t err = trg_rate[i]-rate;
      int16_t pow=cur_power[i]+err*M_PID_KP;
      if(pow<0) pow=0;
      if(pow>M_POW_MAX) pow=M_POW_MAX;
      cur_power[i]=pow;
      enc_cnt[i]=0;
      last_enc_rate[i]=rate;
      pid_log_rate[pid_log_cnt][i]=rate;
      pid_log_err[pid_log_cnt][i]=err;
      pid_log_pow[pid_log_cnt][i]=pow;
    }
    pid_log_dur[pid_log_cnt]=ctime;

    if(pid_log_cnt<PID_LOG_SZ-1) pid_log_cnt++;
    
    //Drive(drv_dir[0], cur_power[0], drv_dir[1], cur_power[1]);
    
    // only do analogWrite if needed!
    
  }
}

void Drive(uint8_t ldir, uint8_t lpow, uint8_t rdir, uint8_t rpow) 
{
  Drive_s(ldir, lpow, M1_EN, M1_OUT_1, M1_OUT_2);
  Drive_s(rdir, rpow, M2_EN, M2_OUT_1, M2_OUT_2);
}

void Drive_s(uint8_t dir, uint8_t pow, int16_t p_en, uint8_t p1, uint8_t p2) 
{
  if(dir==0) {
    digitalWrite(p_en, LOW);
    digitalWrite(p1, LOW); 
    digitalWrite(p2, LOW); 
    return;
  }
  else if(dir==1) {
    digitalWrite(p1, LOW); 
    digitalWrite(p2, HIGH); 
  }
  else {
    digitalWrite(p1, HIGH); 
    digitalWrite(p2, LOW);
  }
  
  analogWrite(p_en, pow);
}

void encodeInterrupt_1() {
 if(!IsDrive) return; 
 enc_cnt[0]++; 
 digitalWrite(GREEN_LED, green_st ? HIGH : LOW);
 green_st=!green_st;
}

void encodeInterrupt_2() {
  if(!IsDrive) return;
  enc_cnt[1]++; 
}

//=======================================

byte Match(char *cmd) 
{
  byte pos=0;
  char *p=(char *)buf;
  while(pos<bytes && *cmd && buf[pos]==*cmd) {
    pos++;
    cmd++;
  }
  if(!*cmd) return pos;
  else return 0;
}

byte bctoi(byte index, int *val) 
{
  int i=0;
  boolean sign=false;
  while(isspace(buf[index])) index++;
  if(buf[index]=='+') index++;
  else if(buf[index]=='-') { 
    sign=true; 
    index++;
  }
  while (isdigit(buf[index]))
  {
    i *= 10;
    i += buf[index] - '0';
    index++;
  }
  *val=sign ? -i : i;
  return index;
}


