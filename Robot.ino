
// TODO
// fix RATE_SAMPLE_PERIOD inconsistency
// at setup - eat garbadge from serial input

//#define TTY_SPEED 38400
#define TTY_SPEED 9600

#define BUF_SIZE 20
byte buf[BUF_SIZE];
byte bytes = 0;

// common vars //
const unsigned int CYCLE_TIMEOUT = 100;
const unsigned int PID_TIMEOUT = 400;
const unsigned int CMD_TIMEOUT = 1000; 
const unsigned int RATE_SAMPLE_PERIOD = 400; // ????????????????????  SHOULD BE 1000 ??????????

unsigned long lastCommandTime;
unsigned long lastCycleTime;
unsigned long lastPidTime;

uint16_t last_dur=0;

enum EnumCmd { EnumCmdDrive=1, EnumCmdTest, EnumCmdStop, EnumCmdLog, EnumCmdContinueDrive };  
enum EnumError { EnumErrorUnknown=-1, EnumErrorBadSyntax=-2, EnumErrorBadParam=-3, EnumErrorNone=-100};  

uint16_t test_enc_cnt[2]={0,0}; 
uint16_t test_prev_enc_cnt[2]={0,0}; 


int8_t cmdResult=EnumErrorNone;

uint8_t drv_dir[2]={0,0}; 
uint8_t cur_power[2]={0,0}; 
uint8_t cmd_power[2]={0,0}; 
uint8_t trg_rate[2]={0,0}; 

uint8_t enc_cnt[2]={0,0}; 
uint8_t last_enc_rate[2]={0,0}; 
uint8_t calib_enc_rate=0; // target rate (counts per RATE_SAMPLE_PERIOD) for 100 power

int8_t last_err[2]={0,0};
int8_t int_err[2]={0,0};

#define PID_LOG_SZ 10
uint8_t pid_log_cnt=0;
uint8_t pid_log_ptr=0;
uint8_t pid_log_idx[PID_LOG_SZ];
uint8_t pid_log_rate[PID_LOG_SZ][2];
//int8_t pid_log_err[PID_LOG_SZ][2];
//int8_t pid_log_derr[PID_LOG_SZ][2];
//int8_t pid_log_ierr[PID_LOG_SZ][2];
uint8_t pid_log_pow[PID_LOG_SZ][2];

#define M_POW_LOW   50
#define M_POW_HIGH 150
#define M_POW_MAX  254

#define M_PID_KP   1

// flags - TODO - bitfield
boolean needNotify=0;
boolean IsDrive = false;
uint8_t enc_state[2]={0,0};

//uint8_t green_st=0; 

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
  //analogFrequency(255);
  
  int ports[6]={M1_OUT_1,M1_OUT_2,M2_OUT_1,M2_OUT_2, M1_EN, M2_EN };
  for(int i=0;i<6;i++) {
    digitalWrite(ports[i], LOW); 
    pinMode(ports[i], OUTPUT);
  }
  analogFrequency(100);
  //analogFrequency(64);
  
  //pinMode(ENC1_IN, INPUT_PULLUP);   
  pinMode(ENC1_IN, INPUT);     
  //attachInterrupt(ENC1_IN, encodeInterrupt_1, RISING);
   attachInterrupt(ENC1_IN, encodeInterrupt_1, CHANGE); 
  //pinMode(ENC2_IN, INPUT_PULLUP);
  pinMode(ENC2_IN, INPUT);  
  //attachInterrupt(ENC2_IN, encodeInterrupt_2, RISING);
 attachInterrupt(ENC2_IN, encodeInterrupt_2, CHANGE); 

  pinMode(RED_LED, OUTPUT);     
  pinMode(GREEN_LED, OUTPUT);     
  for(int i=0; i<6; i++) {
    digitalWrite(RED_LED, HIGH); delay(100); digitalWrite(RED_LED, LOW); 
    digitalWrite(GREEN_LED, HIGH); delay(100); digitalWrite(GREEN_LED, LOW);
  }
  Serial.begin(TTY_SPEED);
  lastCommandTime = lastCycleTime = millis();  
  digitalWrite(RED_LED, HIGH);  
  
    // test sequence
    /*
  IsDrive=1; Drive(1, 254, 1, 170);   
  delay(500);
  StopDrive();
  delay(100);
  IsDrive=1; Drive(1, 170, 1, 254);
  delay(500);
  digitalWrite(RED_LED, LOW);  
  StopDrive();
  
  delay(100);  
  digitalWrite(RED_LED, HIGH);  
  IsDrive=1; Drive(2, 180, 2, 180);
  delay(500);
  StopDrive();
  */
  
  // calibration sequence
  lastCommandTime=millis();
  //enc_cnt[0]=enc_cnt[1]=0;  
  IsDrive=1; Drive(1, M_POW_HIGH, 1, M_POW_HIGH);
  //delay(1000);
  delay(RATE_SAMPLE_PERIOD);
  enc_cnt[0]=enc_cnt[1]=0;  
  delay(RATE_SAMPLE_PERIOD);
  calib_enc_rate = (enc_cnt[0]+enc_cnt[1])/2;
  StopDrive();
//calib_enc_rate = (last_enc_rate[0]+last_enc_rate[1])/2; // calib period = 1 sec- SHOULD BE RATE_SAMPLE_PERIOD !!!
  
  digitalWrite(RED_LED, LOW);

  //calib_enc_rate = 18;  // !!!  
  
  enc_state[0]=digitalRead(ENC1_IN);  
  enc_state[1]=digitalRead(ENC2_IN);      
  
  for(uint8_t i=0; i<PID_LOG_SZ; i++) pid_log_idx[i]=255;
        
  test_prev_enc_cnt[0]=test_enc_cnt[0]=0;
  test_prev_enc_cnt[1]=test_enc_cnt[1]=0;
  
}

void loop()
{
  unsigned long cycleTime = millis();
  if ( cycleTime < lastCycleTime) lastCycleTime=0; // wraparound   
  if ( cycleTime - lastCycleTime >= CYCLE_TIMEOUT) { // working cycle    
  
  
#ifdef _DRV_DBG_
  if(test_prev_enc_cnt[0]!=test_enc_cnt[0] || test_prev_enc_cnt[1]!=test_enc_cnt[1]) {    
    Serial.print(test_enc_cnt[0]);Serial.print("("); Serial.print(test_enc_cnt[0]-test_prev_enc_cnt[0]);Serial.print("); "); 
    Serial.print(test_enc_cnt[1]);Serial.print("("); Serial.print(test_enc_cnt[1]-test_prev_enc_cnt[1]);Serial.println(")"); 
    test_prev_enc_cnt[0]=test_enc_cnt[0];
    test_prev_enc_cnt[1]=test_enc_cnt[1];
  }
#endif  
  
    if (IsDrive) {
      if (CheckCommandTimeout()) StopDrive();
      else { 
        if ( cycleTime < lastPidTime) lastPidTime=0; // wraparound   
        uint16_t ctime = cycleTime - lastPidTime;
        if ( ctime >= PID_TIMEOUT) { // working cycle    
          PID(ctime); 
          lastPidTime=cycleTime;
        }
      }
    }
    lastCycleTime=cycleTime; 
    // if(needNotify) Notify();    // commented 06.10.2014
  }  

  if(ReadSerialCommand()) {
    //if(needNotify) { delay(CYCLE_TIMEOUT); Notify(); } // i.e. if we have queued notifications, means prev commend was in less than 100ms // commented 06.10.2014
    cmdResult = Parse(); // postpone cmd report for 100ms
    bytes = 0; // empty input buffer (only one command at a time)
    if(cmdResult==EnumCmdDrive || (cmdResult==EnumCmdContinueDrive && !IsDrive) ) {
      lastCycleTime=lastPidTime=millis(); //NB!
      StartDrive();
      lastCommandTime = millis();
    } else if(cmdResult==EnumCmdContinueDrive && IsDrive) {
      lastCommandTime = millis();
    }else if(cmdResult==EnumCmdStop) {
      StopDrive();
    }
    //needNotify=1;   // commented 06.10.2014
    delay(CYCLE_TIMEOUT); 
    Notify(); // added 06.10.2014
  }
}

void Notify() {
  addJson("Q", cmdResult);
  switch(cmdResult) {
    case EnumCmdDrive: 
    case EnumCmdContinueDrive:     
    case EnumCmdStop:
      addJsonArr8U("D", drv_dir); addJsonArr8U("C", cmd_power); addJsonArr8U("T", trg_rate); addJsonArr8U("P", cur_power); addJsonArr8U("R", last_enc_rate);
      break; 
    case EnumCmdTest:       
      addJson("TB", calib_enc_rate); addJson("TD", last_dur); addJsonArr8U("R", last_enc_rate); addJsonArr8U("EC", enc_cnt);
      /*
      Serial.print(last_dur); Serial.print(";"); Serial.print(enc_cnt[0]);Serial.print(","); Serial.print(enc_cnt[1]);Serial.print(";"); 
      Serial.print(last_enc_rate[0]);Serial.print(","); Serial.print(last_enc_rate[1]);Serial.print(";"); 
      Serial.print(calib_enc_rate);
      */
      break;
//    case EnumCmdStop:
//      addJsonArr8U("P", cur_power);
//      break;
    case EnumCmdLog: {
      Serial.print(pid_log_cnt);Serial.print(";");
      for(uint8_t i=0; i<PID_LOG_SZ; i++) {
        if(pid_log_idx[i]!=255) {
        Serial.print(pid_log_idx[i]);Serial.print(":"); 
        Serial.print(pid_log_rate[i][0]);Serial.print(","); Serial.print(pid_log_rate[i][1]);
        //Serial.print("("); Serial.print(pid_log_err[i][0]);Serial.print(","); Serial.print(pid_log_err[i][1]); Serial.print(")"); 
        Serial.print("("); Serial.print(trg_rate[0]-pid_log_rate[i][0]);Serial.print(","); Serial.print(trg_rate[1]-pid_log_rate[i][1]); Serial.print(")"); 
        //Serial.print("("); Serial.print(pid_log_derr[i][0]);Serial.print(","); Serial.print(pid_log_derr[i][1]); Serial.print(")"); 
        //Serial.print("("); Serial.print(pid_log_ierr[i][0]);Serial.print(","); Serial.print(pid_log_ierr[i][1]); Serial.print(")"); 
        Serial.print(":"); Serial.print(pid_log_pow[i][0]);Serial.print(","); Serial.print(pid_log_pow[i][1]); Serial.print(";"); 
        pid_log_idx[i]=255;
        delay(10);
        }
      }
      pid_log_cnt=0;
      pid_log_ptr=0;
      break;
    }
    default:;
   }
  Serial.println();
  needNotify=0; 
}

boolean CheckCommandTimeout()
{
  unsigned long commandTime = millis();
  if ( commandTime >= lastCommandTime) commandTime -= lastCommandTime; 
  else lastCommandTime = 0;
  return commandTime > CMD_TIMEOUT;
}

void StartDrive() 
{
  //pid_log_cnt=0;
  
  for(int i=0; i<2; i++) {
    if(drv_dir[i]) {
       trg_rate[i]=map(cmd_power[i], 0, 100, 0, calib_enc_rate);
       //cur_power[i]=map(cmd_power[i], 0, 100, M_POW_LOW, M_POW_HIGH);
        cur_power[i]=map(cmd_power[i], 0, 100, 0, M_POW_HIGH);        
     } else {
       trg_rate[i]=0;
       cur_power[i]=0;
     }    
    last_err[i]=0;
    //int_err[i]=0; // ?
    //pid_log_rate[pid_log_ptr][i]=trg_rate[i];
    pid_log_rate[pid_log_ptr][i]=0;
    //pid_log_err[pid_log_ptr][i]=-111;
    //pid_log_derr[pid_log_ptr][i]=pid_log_ierr[pid_log_ptr][i]=0;
    pid_log_pow[pid_log_ptr][i]=cur_power[i];
  }
  pid_log_idx[pid_log_ptr]=pid_log_cnt; 
  pid_log_cnt++;
  
  if(++pid_log_ptr>=PID_LOG_SZ) pid_log_ptr=0;

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
  cur_power[0]=cur_power[1]=0;
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, LOW);
  unsigned long n=millis();
  if(n>lastCommandTime) {
    last_dur=n-lastCommandTime;
    last_enc_rate[0]=(uint8_t)((uint16_t)enc_cnt[0]*RATE_SAMPLE_PERIOD/last_dur); 
    last_enc_rate[1]=(uint8_t)((uint16_t)enc_cnt[1]*RATE_SAMPLE_PERIOD/last_dur);
  }
  else { last_dur=0; last_enc_rate[0]=last_enc_rate[1]=0;}
}

void PID(uint16_t ctime)
{
  if(ctime>0) {
#ifdef _DRV_DBG_    
    Serial.print("PID T=");Serial.print(ctime);Serial.print(", C=(");Serial.print(enc_cnt[0]);Serial.print(", ");Serial.print(enc_cnt[1]);Serial.print(")");
#endif    
    for(int i=0; i<2; i++) 
      last_enc_rate[i]=(uint8_t)((uint16_t)enc_cnt[i]*RATE_SAMPLE_PERIOD/ctime);    
    
#ifdef _DRV_DBG_    
    Serial.print(", R=(");Serial.print(last_enc_rate[0]);Serial.print(", ");Serial.print(last_enc_rate[1]);Serial.print(")");
#endif

    for(int i=0; i<2; i++) {
      //uint8_t rate = (uint8_t)(enc_cnt[i]*100/ctime);
      int8_t err = trg_rate[i]-last_enc_rate[i];
      //int8_t err_d = err-last_err[i];
      int_err[i]=int_err[i]/2+err;
      int16_t pow=cur_power[i]+err*M_PID_KP;
      if(pow<0) pow=0;
      if(pow>M_POW_MAX) pow=M_POW_MAX;
      cur_power[i]=pow;
      last_err[i]=err;
      
      pid_log_rate[pid_log_ptr][i]=last_enc_rate[i];
      //pid_log_err[pid_log_ptr][i]=err;
      //pid_log_derr[pid_log_ptr][i]=err_d;
      //pid_log_ierr[pid_log_ptr][i]=int_err[i];
      pid_log_pow[pid_log_ptr][i]=pow;      
   
      //if(err) analogWrite(i==0 ? M1_EN : M2_EN , pow);
    }
  
  if(last_err[0]) analogWrite(M1_EN, cur_power[0]);
  if(last_err[1]) analogWrite(M2_EN, cur_power[1]);
 
  enc_cnt[0]=0;
  enc_cnt[1]=0;

#ifdef _DRV_DBG_
  Serial.print(", E=(");Serial.print(last_err[0]);Serial.print(", ");Serial.print(last_err[1]);Serial.print(")");
  Serial.print(", P=(");Serial.print(cur_power[0]);Serial.print(", ");Serial.print(cur_power[1]);Serial.print(")");
  Serial.println();
#endif

  pid_log_idx[pid_log_ptr]=pid_log_cnt++; 
  
  if(++pid_log_ptr>=PID_LOG_SZ) pid_log_ptr=0;
        
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
    digitalWrite(p_en, LOW); digitalWrite(p1, LOW); digitalWrite(p2, LOW); 
    return;
  }
  else if(dir==1) {
    digitalWrite(p1, LOW); digitalWrite(p2, HIGH); 
  }
  else {
    digitalWrite(p1, HIGH); digitalWrite(p2, LOW);
  } 
  analogWrite(p_en, pow);
}

volatile uint8_t es1=0;
volatile uint8_t es2=0;

void encodeInterrupt_1() {
  uint8_t v=digitalRead(ENC1_IN);
  //if(enc_state[0]==v) return;
  //enc_state[0]=v;
  if(es1==v) return;
  es1=v;
  
  test_enc_cnt[0]++; 
  
 if(!IsDrive) return; 
 enc_cnt[0]++; 
}

void encodeInterrupt_2() {
  uint8_t v=digitalRead(ENC2_IN);
  //if(enc_state[1]==v) return;
  //enc_state[1]=v;
  
  if(es2==v) return;
  es2=v;
  
    test_enc_cnt[1]++; 

  if(!IsDrive) return;
  enc_cnt[1]++; 
}

//=======================================

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

int8_t Parse()
// Expect: "DrvLR=100,100"
{  
  byte pos;
  int m;
  
  if((pos=Match("DrvLR"))) {    
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
  } else if((pos=Match("Test"))) {
    return EnumCmdTest;
  }
  else if((pos=Match("Log"))) {
    return EnumCmdLog;
  }
  else return EnumErrorUnknown;
}

byte Match(const char *cmd) 
{
  byte pos=0;
  //char *p=(char *)buf;
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

void addJson(const char *name, int value) {
  Serial.print("\"");
  Serial.print(name);
  Serial.print("\":\"");
  Serial.print(value);
  Serial.print("\",");
 }
 
 void addJsonArr8U(const char *name, uint8_t *va) {
  for(int i=0; i<2; i++) { 
    Serial.print("\"");
    Serial.print(name);
    Serial.print(i?"R":"L");
    Serial.print("\":\"");
    Serial.print(va[i]);
    Serial.print("\",");
  }
 }
