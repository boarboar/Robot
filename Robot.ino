
// TODO
// 
// do not start moving fwd on US condition
// UI - distance on arrow

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

#define TTY_SPEED 38400
//#define TTY_SPEED 57600
//#define TTY_SPEED 9600

/*
// head 1
#define M1_OUT_1  P1_3
#define M1_OUT_2  P1_4
#define M1_EN     P2_1 // analog write

#define M2_OUT_1  P2_3
#define M2_OUT_2  P2_4
#define M2_EN     P2_5 // analog write

#define ENC1_IN  P1_5
#define ENC2_IN  P2_0

*/

#define M2_OUT_1  P1_4
#define M2_OUT_2  P1_3
#define M2_EN     P2_1 // analog write

#define M1_OUT_1  P2_4
#define M1_OUT_2  P2_3
#define M1_EN     P2_5 // analog write

#define ENC2_IN  P1_5
#define ENC1_IN  P2_0

#define US_IN    P1_6
#define US_OUT   P1_7

// common vars 
//const unsigned int CYCLE_TIMEOUT = 50;
const unsigned int CYCLE_TIMEOUT = 75;
const unsigned int PID_TIMEOUT = (CYCLE_TIMEOUT*4);
//const unsigned int PID_TIMEOUT = CYCLE_TIMEOUT;
//const unsigned int RESP_TIMEOUT = 90;
const unsigned int RESP_TIMEOUT = 1;
const unsigned int CMD_TIMEOUT = 1500; 
const unsigned int RATE_SAMPLE_PERIOD = 400;
const unsigned int WHEEL_CHGSTATES = 40;
const unsigned int WHEEL_RATIO_RPM = (60000/RATE_SAMPLE_PERIOD/WHEEL_CHGSTATES);
const unsigned int WHEEL_RAD_MM = 33; // 34?
const unsigned int WHEEL_BASE_MM = 140;// approx... carriage base 145 - too high, 135 - too low
const unsigned int WHEEL_RATIO_SMPS = (WHEEL_RAD_MM*628/RATE_SAMPLE_PERIOD/WHEEL_CHGSTATES);
const unsigned int US_WALL_DIST=40;
const unsigned int US_WALL_CNT_THR=2;

#define V_NORM 10000

#define CHGST_TO_MM(CNT)  ((int32_t)(CNT)*62832*WHEEL_RAD_MM/WHEEL_CHGSTATES/10000)

// approx=30*628/400/44 = 1

#define M_POW_LOW   50
//#define M_POW_HIGH 150
#define M_POW_HIGH 175
#define M_POW_MAX  254

//#define M_PID_KP   1
#define M_PID_KP   2
#define M_PID_KI   0
#define M_PID_KD   0


#define BUF_SIZE 20
byte buf[BUF_SIZE];
byte bytes = 0;

uint32_t lastCommandTime;
uint32_t lastCycleTime;
uint32_t lastPidTime;
uint16_t last_dur=0;
uint8_t cmd_id=0;

int32_t dist=0;
int16_t diff=0;
int32_t x=0, y=0;
int32_t nx=0, ny=V_NORM;
int32_t tx=-V_NORM, ty=0;
/*
float nx=0.0, ny=1.0;
float tx=-1.0, ty=0.0;
*/

volatile int16_t us_dist=0; 
volatile int8_t us_wall_cnt_up=0; 
volatile int8_t us_wall_cnt_dn=0; 

enum EnumCmd { EnumCmdDrive=1, EnumCmdTest, EnumCmdStop, EnumCmdLog, EnumCmdContinueDrive, EnumCmdRst };  
enum EnumError { EnumErrorUnknown=-1, EnumErrorBadSyntax=-2, EnumErrorBadParam=-3, EnumErrorNone=-100};  

int8_t cmdResult=EnumErrorNone;

uint8_t drv_dir[2]={0,0}; 
uint8_t cur_power[2]={0,0}; 
uint8_t cmd_power[2]={0,0}; 
uint8_t trg_rate[2]={0,0}; 

volatile uint8_t enc_cnt[2]={0,0}; 
uint8_t last_enc_rate[2]={0,0}; 
uint8_t calib_enc_rate=0; // target rate (counts per RATE_SAMPLE_PERIOD) for 100 power

int8_t last_err[2]={0,0};
int8_t int_err[2]={0,0};

#define PID_LOG_SZ 7
uint8_t pid_log_cnt=0;
uint8_t pid_log_ptr=0;
/*
uint8_t pid_log_idx[PID_LOG_SZ];
uint8_t pid_log_rate[PID_LOG_SZ][2];
int8_t pid_log_derr[PID_LOG_SZ][2];
int8_t pid_log_ierr[PID_LOG_SZ][2];
uint8_t pid_log_pow[PID_LOG_SZ][2];
*/
struct LogRec {
  uint8_t cmd_id;
  uint8_t pid_log_idx;
  uint8_t ctime;
  uint8_t pid_log_rate[2];
  int8_t pid_log_derr[2];
  int8_t pid_log_ierr[2];
  uint8_t pid_log_pow[2];
} logr[PID_LOG_SZ];


// flags - TODO - bitfield
boolean IsDrive = false;
volatile uint8_t EncOverflow=0;


void setup()
{ 
  int ports[6]={M1_OUT_1,M1_OUT_2,M2_OUT_1,M2_OUT_2, M1_EN, M2_EN };
  for(int i=0;i<6;i++) {
    digitalWrite(ports[i], LOW); 
    pinMode(ports[i], OUTPUT);
  }
  //analogFrequency(100);
  analogFrequency(255);
 
  pinMode(ENC1_IN, INPUT);     
  attachInterrupt(ENC1_IN, encodeInterrupt_1, CHANGE); 
  pinMode(ENC2_IN, INPUT);  
  attachInterrupt(ENC2_IN, encodeInterrupt_2, CHANGE); 
  
  pinMode(US_OUT, OUTPUT);     
  pinMode(US_IN, INPUT);     
  
  pinMode(RED_LED, OUTPUT);     
  //pinMode(GREEN_LED, OUTPUT);     
  for(int i=0; i<5; i++) {
    digitalWrite(RED_LED, HIGH); delay(100); digitalWrite(RED_LED, LOW); 
    //digitalWrite(GREEN_LED, HIGH); delay(100); digitalWrite(GREEN_LED, LOW);
  }
  Serial.begin(TTY_SPEED);
  lastCommandTime = lastCycleTime = millis();  
  digitalWrite(RED_LED, HIGH);  
   
  // calibration sequence
  lastCommandTime=millis();
  IsDrive=1; Drive(1, M_POW_HIGH, 1, M_POW_HIGH);
  delay(RATE_SAMPLE_PERIOD*2); // warmup
  enc_cnt[0]=enc_cnt[1]=0;  
  delay(RATE_SAMPLE_PERIOD*2); // calibration
  calib_enc_rate = (enc_cnt[0]+enc_cnt[1])/4;
  StopDrive();
  
  digitalWrite(RED_LED, LOW);

  //for(uint8_t i=0; i<PID_LOG_SZ; i++) pid_log_idx[i]=255;
  for(uint8_t i=0; i<PID_LOG_SZ; i++) logr[i].pid_log_idx=255;

  while (Serial.available()) Serial.read();  // eat garbage
}

void loop()
{
  uint32_t cycleTime = millis();
  if ( cycleTime < lastCycleTime) lastCycleTime=0; // wraparound   
  if ( cycleTime - lastCycleTime >= CYCLE_TIMEOUT) { // working cycle    
  
  //readUSDist();
  
  if (IsDrive) {    
      readUSDist();      
      if (CheckCommandTimeout() ||  
        (us_wall_cnt_up>=US_WALL_CNT_THR && drv_dir[0]==1 && drv_dir[1]==1) // wall ahead && forwared drive
        ) StopDrive();
      else { 
        if ( cycleTime < lastPidTime) lastPidTime=0; // wraparound   
        uint16_t ctime = cycleTime - lastPidTime;
        if ( ctime >= PID_TIMEOUT) { // PID cycle    
          PID(ctime); 
          lastPidTime=cycleTime;
        }
      }
    }
    lastCycleTime=cycleTime; 
  }  
  
  if(ReadSerialCommand()) {
    cmdResult = Parse(); // postpone cmd report for 100ms
    bytes = 0; // empty input buffer (only one command at a time)
    cmd_id++;
    if(cmdResult==EnumCmdDrive || (cmdResult==EnumCmdContinueDrive && !IsDrive) ) {
      if(us_wall_cnt_up>=US_WALL_CNT_THR && drv_dir[0]==1 && drv_dir[1]==1) {
        last_dur=millis()-lastCommandTime;
        StopDrive();
      }
      else {
        lastCycleTime=lastPidTime=millis(); //NB!
        StartDrive();
        last_dur=0;
        lastCommandTime = millis();
      }
    } else if(cmdResult==EnumCmdContinueDrive && IsDrive) {
      last_dur=millis()-lastCommandTime;
      lastCommandTime = millis();
    } else if(cmdResult==EnumCmdStop) {
      last_dur=millis()-lastCommandTime;
      StopDrive();
    } else if(cmdResult==EnumCmdRst) {
      StopDrive();
      x=y=0;
      nx=0; ny=V_NORM;
      tx=-V_NORM;ty=0;
    }
    delay(RESP_TIMEOUT);
    //readUSDist(); 
    Notify(); // added 06.10.2014
  }
}

void Notify() {
  addJson("Q", cmdResult);
  addJson("I", cmd_id);
  switch(cmdResult) {
    case EnumCmdDrive: 
    case EnumCmdContinueDrive:     
    case EnumCmdStop: 
    case EnumCmdRst: {
      uint32_t t=millis();
      addJson("L", last_dur); 
      addJsonArr16_2("P", cur_power[0], cur_power[1]);
      addJsonArr16_2("T", trg_rate[0], trg_rate[1]);
      addJsonArr16_2("R", last_enc_rate[0], last_enc_rate[1]);
      addJsonArr16_2("W", last_enc_rate[0]*WHEEL_RATIO_RPM, last_enc_rate[1]*WHEEL_RATIO_RPM); // in cm
      int16_t s[2];
      for(uint8_t i=0; i<2; i++) {         
        if(drv_dir[i]==0) s[i]=0;
        else {
          s[i]=last_enc_rate[i]*WHEEL_RATIO_SMPS;
          if(drv_dir[i]==2) s[i]=-s[i];
        } 
      }
      addJson("S", (s[0]+s[1])/2);
      addJson("D", (int16_t)(dist/10));
      addJson("F", (int16_t)(diff/10));
      addJsonArr16_2("N", (int16_t)nx, (int16_t)ny); // in cm
      addJsonArr16_2("X", (int16_t)(x/10), (int16_t)(y/10)); // in cm
      addJson("U", (int16_t)(us_dist));
      addJson("UC", (int16_t)(us_wall_cnt_up));
      }
      break; 
    case EnumCmdTest:       
      addJson("TB", calib_enc_rate); addJson("TD", last_dur); addJsonArr8U("R", last_enc_rate); //addJsonArr8U("EC", enc_cnt);
      addJson("OVF", (int16_t)(EncOverflow));
      addJson("U", (int16_t)(us_dist));
      break;
    case EnumCmdLog: {
      addJson("LCNT", pid_log_cnt);
      Serial.print("\""); Serial.print("LOGR"); Serial.print("\":\"");
      for(uint8_t i=0; i<PID_LOG_SZ; i++) {
        /*
        if(pid_log_idx[i]!=255) {
        Serial.print(pid_log_idx[i]);Serial.print(":"); 
        Serial.print(pid_log_rate[i][0]);Serial.print(","); Serial.print(pid_log_rate[i][1]);
        Serial.print("("); Serial.print(trg_rate[0]-pid_log_rate[i][0]);Serial.print(","); Serial.print(trg_rate[1]-pid_log_rate[i][1]); Serial.print(")"); 
        Serial.print("("); Serial.print(pid_log_ierr[i][0]);Serial.print(","); Serial.print(pid_log_ierr[i][1]); Serial.print(")"); 
        Serial.print("("); Serial.print(pid_log_derr[i][0]);Serial.print(","); Serial.print(pid_log_derr[i][1]); Serial.print(")"); 
        Serial.print(":"); Serial.print(pid_log_pow[i][0]);Serial.print(","); Serial.print(pid_log_pow[i][1]); Serial.print(";"); 
        pid_log_idx[i]=255;
        */
        if(logr[i].pid_log_idx!=255) {
        Serial.print(logr[i].pid_log_idx);Serial.print(":"); Serial.print(logr[i].cmd_id);Serial.print(":"); Serial.print(logr[i].ctime); Serial.print(":"); 
        Serial.print("("); Serial.print(logr[i].pid_log_rate[0]);Serial.print(","); Serial.print(logr[i].pid_log_rate[1]); Serial.print(")"); 
        Serial.print("("); Serial.print(trg_rate[0]-logr[i].pid_log_rate[0]);Serial.print(","); Serial.print(trg_rate[1]-logr[i].pid_log_rate[1]); Serial.print(")"); 
        Serial.print("("); Serial.print(logr[i].pid_log_ierr[0]);Serial.print(","); Serial.print(logr[i].pid_log_ierr[1]); Serial.print(")"); 
        Serial.print("("); Serial.print(logr[i].pid_log_derr[0]);Serial.print(","); Serial.print(logr[i].pid_log_derr[1]); Serial.print(")"); 
        Serial.print("("); Serial.print(logr[i].pid_log_pow[0]);Serial.print(","); Serial.print(logr[i].pid_log_pow[1]); Serial.print(");"); 
        logr[i].pid_log_idx=255;
        
        delay(10);
        }
      }
      Serial.print("\",");
      pid_log_cnt=0;
      pid_log_ptr=0;
      break;
    }
    default:;
   }
  Serial.println();
}

boolean CheckCommandTimeout()
{
  unsigned long commandTime = millis();
  if ( commandTime >= lastCommandTime) commandTime -= lastCommandTime; 
  else lastCommandTime = 0;
  return commandTime > CMD_TIMEOUT;
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
  if(us_dist<=US_WALL_DIST) {
    if(us_wall_cnt_up<US_WALL_CNT_THR) us_wall_cnt_up++;
    else us_wall_cnt_dn=0;
  }
  else {
    if(us_wall_cnt_dn<US_WALL_CNT_THR) us_wall_cnt_dn++;
    else us_wall_cnt_up=0;
  }
}

void StartDrive() 
{
  for(int i=0; i<2; i++) {
    if(drv_dir[i]) {
       trg_rate[i]=map(cmd_power[i], 0, 100, 0, calib_enc_rate);
       cur_power[i]=map(cmd_power[i], 0, 100, 0, M_POW_HIGH);        
     } else {
       trg_rate[i]=0;
       cur_power[i]=0;
     }    
    last_err[i]=0;
    //int_err[i]=0; // ?
    /*
    pid_log_rate[pid_log_ptr][i]=0;
    pid_log_ierr[pid_log_ptr][i]=0;
    pid_log_derr[pid_log_ptr][i]=0;
    pid_log_pow[pid_log_ptr][i]=cur_power[i];
    */
    /*
    logr[pid_log_ptr].pid_log_rate[i]=0;
    logr[pid_log_ptr].pid_log_ierr[i]=0;
    logr[pid_log_ptr].pid_log_derr[i]=0;
    logr[pid_log_ptr].pid_log_pow[i]=cur_power[i];
    */
  }
  //pid_log_idx[pid_log_ptr]=pid_log_cnt; 
  /*
  logr[pid_log_ptr].pid_log_idx=pid_log_cnt; 
  pid_log_cnt++;  
  if(++pid_log_ptr>=PID_LOG_SZ) pid_log_ptr=0;
*/
  enc_cnt[0]=enc_cnt[1]=0;  
  Drive(drv_dir[0], cur_power[0], drv_dir[1], cur_power[1]); // change interface
  if (!IsDrive)
  {
    IsDrive = true;
    digitalWrite(RED_LED, HIGH);
  }        
}

void StopDrive() 
{
  if(!IsDrive) return;
  unsigned long n=millis();
  if(n>lastPidTime) PID(n-lastPidTime); // finalize
  Drive(0, 0, 0, 0);
  IsDrive = false;
  digitalWrite(RED_LED, LOW);  
  last_enc_rate[0]=last_enc_rate[1]=0;
  cur_power[0]=cur_power[1]=0;
  //us_wall_cnt=0;
}

void PID(uint16_t ctime)
{
  if(ctime>0) {
    int16_t s[2];
    for(uint8_t i=0; i<2; i++) {
      if(drv_dir[i]==2) s[i]=-enc_cnt[i];
      else s[i]=enc_cnt[i];
      s[i]=CHGST_TO_MM(s[i]);
      last_enc_rate[i]=(uint8_t)((uint16_t)enc_cnt[i]*RATE_SAMPLE_PERIOD/ctime);    
      enc_cnt[i]=0; 
      int8_t err = trg_rate[i]-last_enc_rate[i];
      int8_t err_d = err-last_err[i];
      int_err[i]=int_err[i]/2+err;      
      int16_t pow=cur_power[i]+(int16_t)err*M_PID_KP+(int16_t)int_err[i]*M_PID_KI+(int16_t)err_d*M_PID_KD;
      if(pow<0) pow=0;
      if(pow>M_POW_MAX) pow=M_POW_MAX;
      if(err) analogWrite(i==0 ? M1_EN : M2_EN , pow); 
      cur_power[i]=pow;
      last_err[i]=err;
      // log entry
      /*
      pid_log_rate[pid_log_ptr][i]=last_enc_rate[i];
      pid_log_derr[pid_log_ptr][i]=err_d;
      pid_log_ierr[pid_log_ptr][i]=int_err[i];
      pid_log_pow[pid_log_ptr][i]=pow;      
      */
      logr[pid_log_ptr].pid_log_rate[i]=last_enc_rate[i];
      logr[pid_log_ptr].pid_log_derr[i]=err_d;
      logr[pid_log_ptr].pid_log_ierr[i]=int_err[i];
      logr[pid_log_ptr].pid_log_pow[i]=pow;      
    } 
  if(s[0] || s[1]) {  
    dist+=(s[0]+s[1])/2; // drive distance, mm  
    diff+=(s[0]-s[1])/2; // drive diff, mm  
    
    tx += nx*(s[0]-s[1])/WHEEL_BASE_MM;
    ty += ny*(s[0]-s[1])/WHEEL_BASE_MM;
/*
// opt1 start    
    uint16_t tl=isqrt32(tx*tx+ty*ty);
    tx=tx*V_NORM/tl;  
    ty=ty*V_NORM/tl;
    nx=ty; 
    ny=-tx;
// opt1 end    
 */
   
// opt2 start       
    nx=(nx*2+ty*4)/6;
    ny=(ny*2-tx*4)/6;
    uint16_t nl=isqrt32(nx*nx+ny*ny);
    nx=nx*V_NORM/nl;  
    ny=ny*V_NORM/nl;
    ty=nx; tx=-ny;
// opt2 end        
    
    
    /*
// float option    
    tx += nx*(s[1]-s[0])/WHEEL_BASE_MM;
    ty += ny*(s[1]-s[0])/WHEEL_BASE_MM;
    nx=(2.0*nx+4.0*ty)/6.0;
    ny=(2.0*ny-4.0*tx)/6.0;
    float nl=sqrt(nx*nx+ny*ny);
    nx/=nl; ny/=nl;
    ty=nx, tx=-ny;
    
		float dx=nx*(s1+s2)/2.0;
		float dy=ny*(s1+s2)/2.0;		
*/
    x+=(int32_t)nx*(s[0]+s[1])/(2*V_NORM);
    y+=(int32_t)ny*(s[0]+s[1])/(2*V_NORM);
  }
  // log advance/wrap  
  //pid_log_idx[pid_log_ptr]=pid_log_cnt++;   
  
  logr[pid_log_ptr].cmd_id=cmd_id;
  logr[pid_log_ptr].ctime=ctime;
  logr[pid_log_ptr].pid_log_idx=pid_log_cnt++;   
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
  if(es1==v) return;
  es1=v;  
  if(!IsDrive) return; 
  enc_cnt[0]++; 
  if(enc_cnt[0]>123) EncOverflow&=0x01;
}

void encodeInterrupt_2() {
  uint8_t v=digitalRead(ENC2_IN);  
  if(es2==v) return;
  es2=v;  
  if(!IsDrive) return;
  enc_cnt[1]++; 
  if(enc_cnt[1]>123) EncOverflow&=0x02;
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
  if(bytes>=BUF_SIZE) { 
    bytes=0; //overflow, probably caused hang up at start...    
    return true; // this is for test only... !!!!!!!!!!!!!!!!!!!!!!!!!!
  }
  return false;
}

int8_t Parse()
// Expect: "DLR=100,100"
{  
  byte pos;
  int m;
  
  if((pos=Match("DLR"))) {    
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
  else if((pos=Match("Rst"))) {
    return EnumCmdRst;
  }
  else return EnumErrorUnknown;
}

byte Match(const char *cmd) 
{
  byte pos=0;
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

 void addJson(const char *name, int16_t value) {
  Serial.print("\"");
  Serial.print(name);
  Serial.print("\":");
  Serial.print(value);
  Serial.print(",");
 }
 
 void addJsonArr8U(const char *name, uint8_t *va) {
  for(int i=0; i<2; i++) { 
    Serial.print("\"");
    Serial.print(name);
    Serial.print(i?"R":"L");
    Serial.print("\":");
    Serial.print(va[i]);
    Serial.print(",");
  }
 }

 void addJsonArr16_2(const char *name, int16_t v1, int16_t v2) {
    Serial.print("\"");
    Serial.print(name);
    Serial.print("\":[");
    Serial.print(v1);
    Serial.print(",");
    Serial.print(v2);
    Serial.print("],");
 }
 
 uint16_t isqrt32(uint32_t n)  
    {  
        uint16_t c = 0x8000;  
        uint16_t g = 0x8000;  
      
        for(;;) {  
            if((uint32_t)g*g > n)  
                g ^= c;  
            c >>= 1;  
            if(c == 0)  
                return g;  
            g |= c;  
        }  
    }  
