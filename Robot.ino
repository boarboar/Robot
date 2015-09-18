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

int32_t isin32d(int32_t xd);
uint16_t isqrt32(uint32_t n);  
//int32_t invsin(int16_t ax, int16_t ay, int16_t bx, int16_t by, uint16_t norm); 
int16_t invsin(int16_t ax, int16_t ay, int16_t bx, int16_t by, uint16_t norm); 
int16_t asin32d(int16_t x, uint16_t norm);
void addJson(const char *name, int16_t value);
void addJsonArr16_2(const char *name, int16_t v1, int16_t v2);

#define TTY_SPEED 38400
//#define TTY_SPEED 9600

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
//const unsigned int PID_TIMEOUT = 100;
const unsigned int PID_TIMEOUT = 200;
//const unsigned int RESP_TIMEOUT = 90;
const unsigned int RESP_TIMEOUT = 25; // C_T+R_T >= 75ms
const unsigned int CMD_TIMEOUT = 600; 
const unsigned int RATE_SAMPLE_PERIOD = 400;
const unsigned int RATE_SAMPLE_TARGET_LOW = 3;
const unsigned int RATE_SAMPLE_TARGET_HIGH = 18;
const unsigned int WHEEL_CHGSTATES = 40;
const unsigned int WHEEL_RATIO_RPM = (60000/RATE_SAMPLE_PERIOD/WHEEL_CHGSTATES);
const unsigned int WHEEL_RAD_MM_10 = 350; 
const unsigned int WHEEL_BASE_MM_10 = 1400;// approx... carriage base 145 - too high, 135 - too low
const unsigned int WHEEL_RATIO_SMPS_10 = (WHEEL_RAD_MM_10/10*628/RATE_SAMPLE_PERIOD/WHEEL_CHGSTATES);
const unsigned int US_WALL_DIST=20;
const unsigned int US_WALL_CNT_THR=100;
const unsigned int M_COAST_TIME=400;
const unsigned int M_WUP_PID_CNT=1;

const unsigned int TASK_TIMEOUT = 10000; 

const unsigned int R_F_ISDRIVE=0x01;
const unsigned int R_F_ISOVERFLOW=0x02;
const unsigned int R_F_ISTASK=0x04;
const unsigned int R_F_ISTASKMOV=0x08;
const unsigned int R_F_ISTASKROT=0x16;

#define F_ISDRIVE() (flags&R_F_ISDRIVE)
#define F_SETDRIVE() (flags|=R_F_ISDRIVE)
#define F_CLEARDRIVE() (flags&=~R_F_ISDRIVE)
#define F_ISOVERFLOW() (flags&R_F_ISOVERFLOW)
#define F_SETOVERFLOW() (flags|=R_F_ISOVERFLOW)
#define F_CLEAROVERFLOW() (flags&=~R_F_ISOVERFLOW)
#define F_ISTASKANY() (flags&R_F_ISTASK)
#define F_ISTASKMOV() (flags&R_F_ISTASKMOV)
#define F_ISTASKROT() (flags&R_F_ISTASKROT)
#define F_SETTASKMOV() (flags|=(R_F_ISTASK|R_F_ISTASKMOV))
#define F_SETTASKROT() (flags|=(R_F_ISTASK|R_F_ISTASKROT))
#define F_CLEARTASK() (flags&=~(R_F_ISTASK|R_F_ISTASKMOV|R_F_ISTASKROT))

#define CHGST_TO_MM_10(CNT)  ((int32_t)(CNT)*62832*WHEEL_RAD_MM_10/WHEEL_CHGSTATES/10000)

#define RADN_TO_GRAD(R)      ((int32_t)(R)*180/V_NORM_PI)
#define GRAD_TO_RADN(D)      ((int32_t)(D)*V_NORM_PI/180)

// for 5v
/*
#define M_POW_LOW   50
#define M_POW_HIGH 175
#define M_POW_MAX  254

#define M_PID_KP   2
#define M_PID_KI   0
#define M_PID_KD   0
*/

// for 7.5v
#define M_POW_LOWEST_LIM   10
//#define M_POW_LOW   30
#define M_POW_HIGH_LIM 100
#define M_POW_MAX  120
#define M_POW_STEP 2

#define M_PID_KP   2
#define M_PID_KI   0
#define M_PID_KD   2
#define M_PID_DIV  4

#define BUF_SIZE 16
byte buf[BUF_SIZE];
byte bytes = 0;

uint32_t lastCommandTime; // =lastTaskTime
uint32_t lastPidTime;
uint16_t last_dur=0;
//uint16_t us_meas_dur=0;

uint8_t cmd_id=0;

// tracking
#define V_NORM 10000
#define V_NORM_PI 31400

int32_t dist=0;  // in 10thmm
int16_t diff=0;  // in 10tmm
int32_t x=0, y=0;// in 10thmm
int16_t nx=0, ny=V_NORM;
int32_t angle=0; 

uint16_t us_dist=9999; 
volatile uint8_t v_enc_cnt[2]={0,0}; 
volatile uint8_t v_es[2]={0,0};

enum EnumCmd { EnumCmdDrive=1, EnumCmdTest=2, EnumCmdStop=3, EnumCmdLog=4, EnumCmdContinueDrive=5, EnumCmdRst=6, EnumCmdTaskMove=7, EnumCmdTaskRotate=8, EnumCmdWallLog=9};  
enum EnumError { EnumErrorUnknown=-1, EnumErrorBadSyntax=-2, EnumErrorBadParam=-3, EnumErrorNone=-100};  

int32_t task_target=0;   // in mm or degrees

int16_t t_nx, t_ny;
int16_t t_x, t_y; //in 10thmm - up to 320 cm
int16_t t_ang;
int16_t  t_adv;

int8_t cmdResult=EnumErrorNone;
uint8_t drv_dir[2]={0,0}; 
uint8_t cur_power[2]={0,0}; 
uint8_t cmd_power[2]={0,0}; 
uint8_t trg_rate[2]={0,0}; 

uint8_t last_enc_cnt[2]={0,0}; 
uint8_t last_enc_rate[2]={0,0}; 
uint8_t calib_enc_rate_high=0; // target rate (counts per RATE_SAMPLE_PERIOD) for 100 power
uint8_t calib_enc_rate_low=0; // target rate (counts per RATE_SAMPLE_PERIOD) for low power
uint8_t pow_low=0;
uint8_t pow_high=0;

int8_t last_err[2]={0,0};
int8_t int_err[2]={0,0};

uint8_t flags=0; 

#define PID_LOG_SZ 8
uint8_t pid_cnt=0;
uint8_t pid_log_ptr=0;

struct __attribute__((__packed__)) LogRec {
  uint8_t cmd_id;       // ref to cmd id
  uint8_t pid_log_idx;   
  uint8_t ctime;         // pid interval 
  uint8_t ec[2];
  uint8_t pid_t_rate[2];
  uint8_t pid_log_rate[2];
  int8_t pid_log_derr[2];
  //int8_t pid_log_ierr[2];
  uint8_t pid_log_pow[2];
  int8_t pid_t_err;
  int8_t t_adv;
  int16_t t_ang;
} logr[PID_LOG_SZ];

#define WALL_LOG_SZ 8
//uint8_t wl_ptr=0;
struct __attribute__((__packed__)) WallRec {
  //uint8_t idx;
  int8_t adv; //cm
  int8_t usd; //cm
} logw[WALL_LOG_SZ];

void setup()
{ 
  uint8_t i;
  int ports[6]={M1_OUT_1,M1_OUT_2,M2_OUT_1,M2_OUT_2, M1_EN, M2_EN };
  for(i=0;i<6;i++) {
    digitalWrite(ports[i], LOW); 
    pinMode(ports[i], OUTPUT);
  }
  analogFrequency(32);
  //analogFrequency(100); // better
  //analogFrequency(255); // bad
 
  pinMode(ENC1_IN, INPUT);     
  attachInterrupt(ENC1_IN, encodeInterrupt_1, CHANGE); 
  pinMode(ENC2_IN, INPUT);  
  attachInterrupt(ENC2_IN, encodeInterrupt_2, CHANGE); 
  
  pinMode(US_OUT, OUTPUT);     
  pinMode(US_IN, INPUT);     
  
  pinMode(RED_LED, OUTPUT);     
  for(i=0; i<5; i++) {
    digitalWrite(RED_LED, HIGH); delay(100); digitalWrite(RED_LED, LOW); 
  }
  Serial.begin(TTY_SPEED);
  
  digitalWrite(RED_LED, HIGH);  
   
  // calibration sequence
  F_SETDRIVE();
  pow_low=M_POW_LOWEST_LIM;
  while(pow_low<M_POW_HIGH_LIM) {
   ReadEnc(); 
   Drive(1, pow_low, 1, pow_low);
   delay(RATE_SAMPLE_PERIOD);
   ReadEnc();   
   if(last_enc_cnt[0]>=RATE_SAMPLE_TARGET_LOW && last_enc_cnt[1]>=RATE_SAMPLE_TARGET_LOW) break;
   pow_low+=M_POW_STEP;
  }
  
  //ReadEnc();
  //delay(RATE_SAMPLE_PERIOD); // calibration
  //ReadEnc();
  calib_enc_rate_low = (last_enc_cnt[0]+last_enc_cnt[1])/2;
 
  pow_high=pow_low+M_POW_STEP;
  while(pow_high<M_POW_HIGH_LIM) {
   ReadEnc();
   Drive(1, pow_high, 1, pow_high);
   delay(RATE_SAMPLE_PERIOD);
   ReadEnc();   
   if(last_enc_cnt[0]>=RATE_SAMPLE_TARGET_HIGH && last_enc_cnt[1]>=RATE_SAMPLE_TARGET_HIGH) break;
   pow_high+=M_POW_STEP;
  }
  
  //ReadEnc();
  //delay(RATE_SAMPLE_PERIOD); // calibration
  //ReadEnc();
  calib_enc_rate_high = (last_enc_cnt[0]+last_enc_cnt[1])/2;
  StopDrive();
  
  digitalWrite(RED_LED, LOW);

  for(i=0; i<PID_LOG_SZ; i++) logr[i].pid_log_idx=255;
  
  for(i=0; i<WALL_LOG_SZ; i++) logw[i].adv=logw[i].usd=0;

  while (Serial.available()) Serial.read();  // eat garbage
  
  lastCommandTime = lastPidTime = millis();  
}

void loop()
{  
  uint32_t cycleTime = millis();
  if (F_ISDRIVE() && (/*CheckCommandTimeout() ||*/ (us_dist<US_WALL_DIST && drv_dir[0]+drv_dir[1]==2))) StopDrive(); 
  if ( cycleTime < lastPidTime) lastPidTime=0; // wraparound, not correct   
  uint16_t ctime = cycleTime - lastPidTime;
  if ( ctime >= PID_TIMEOUT) { // PID cycle    
    ReadEnc();
    if (F_ISDRIVE()) {
      PID(ctime); 
      if(F_ISTASKANY()) { 
          if(TrackTask() || CheckCommandTimeout(TASK_TIMEOUT)) StopTask();
      } else if(CheckCommandTimeout(CMD_TIMEOUT)) StopDrive();
    }
    readUSDist(); 
    lastPidTime=cycleTime;
  } // PID cycle 
          
  if(ReadSerialCommand()) {
    cmdResult = Parse(); 
    bytes = 0; // empty input buffer (only one command at a time)
    cmd_id++;  
    if(cmdResult==EnumCmdDrive || (cmdResult==EnumCmdContinueDrive && !F_ISDRIVE())) {
      F_CLEARTASK();
      lastCommandTime = millis();        
      last_dur=0;
      if(!(us_dist<US_WALL_DIST && drv_dir[0]+drv_dir[1]==2)) StartDrive();
    } else if(cmdResult==EnumCmdContinueDrive && F_ISDRIVE()) {
      last_dur=millis()-lastCommandTime;
      lastCommandTime = millis();
    } else if(cmdResult==EnumCmdStop) {
      StopDrive();
    } else if(cmdResult==EnumCmdRst) {
      StopDrive();
      x=y=0;
      nx=0; ny=V_NORM;
      dist=diff=0;
      angle=0;
    } 
    else if(cmdResult==EnumCmdTest) { ; }
    else if(cmdResult==EnumCmdTaskMove || cmdResult==EnumCmdTaskRotate) {
        lastCommandTime = millis();        
        StartTask();
    }
    delay(RESP_TIMEOUT);
    Notify(); 
  } // read serial
}

void Notify() {
  addJson("Q", cmdResult);
  addJson("I", cmd_id);
  switch(cmdResult) {
    case EnumCmdDrive: 
    case EnumCmdContinueDrive:     
    case EnumCmdStop: 
    case EnumCmdRst: {
      // speeed calc
      int16_t s[2];
      for(uint8_t i=0; i<2; i++) {         
        if(drv_dir[i]==0) s[i]=0;
        else {
          s[i]=last_enc_rate[i]*WHEEL_RATIO_SMPS_10;
          if(drv_dir[i]==2) s[i]=-s[i];
        } 
      }
 
      addJson("L", last_dur); 
      addJsonArr16_2("P", cur_power[0], cur_power[1]);
      addJsonArr16_2("T", trg_rate[0], trg_rate[1]);
      addJsonArr16_2("R", last_enc_rate[0], last_enc_rate[1]);
      addJsonArr16_2("W", last_enc_rate[0]*WHEEL_RATIO_RPM, last_enc_rate[1]*WHEEL_RATIO_RPM);      
      addJson("S", (s[0]+s[1])/2);
      addJson("D", (int16_t)(dist/100)); // in cm
      addJson("F", (int16_t)(diff/100)); // in cm
      addJsonArr16_2("N", (int16_t)nx, (int16_t)ny); // in normval
      addJsonArr16_2("X", (int16_t)(x/100), (int16_t)(y/100)); // in cm
      addJson("U", (int16_t)(us_dist));
      }
      break; 
    case EnumCmdTest:       
      addJson("PL", pow_low);
      addJson("PH", pow_high);
      addJson("CRL", calib_enc_rate_low);       
      addJson("CRH", calib_enc_rate_high); 
      addJson("TD", last_dur); 
      addJsonArr16_2("R", last_enc_rate[0], last_enc_rate[1]);
      addJsonArr16_2("EC", last_enc_cnt[0], last_enc_cnt[1]);
      delay(10);
      addJson("OVF", (int16_t)(F_ISOVERFLOW()));
      addJson("U", (int16_t)(us_dist));      
      addJson("FT", flags&R_F_ISTASK);
      addJson("FM", flags&R_F_ISTASKMOV);
      addJson("FR", flags&R_F_ISTASKROT);
      addJson("TG", task_target);
      delay(10);
//      addJson("TP", task_progress);
      addJsonArr16_2("TN", (int16_t)t_nx, (int16_t)t_ny); // in normval
      addJsonArr16_2("TX", (int16_t)(t_x/100), (int16_t)(t_y/100)); // in cm 
      //addJson("NSIN", invsin(0, V_NORM, nx, ny, V_NORM)/100);
      addJson("ANG", RADN_TO_GRAD(t_ang));
      addJson("TADV", RADN_TO_GRAD(t_adv)); 
      addJson("L", last_dur); 
      break;
    case EnumCmdLog: {
      uint8_t i;
      addJson("LCNT", pid_cnt);
      Serial.print("\""); Serial.print("LOGR"); Serial.print("\":\"");      
      for(i=0; i<PID_LOG_SZ; i++) {
        if(logr[i].pid_log_idx!=255) { // if not empty
        Serial.print(logr[i].pid_log_idx);Serial.print(":"); Serial.print(logr[i].cmd_id);Serial.print(":"); Serial.print(logr[i].ctime); Serial.print(":"); 
        Serial.print("("); Serial.print(logr[i].ec[0]);Serial.print(","); Serial.print(logr[i].ec[1]); Serial.print(")"); 
        Serial.print("("); Serial.print(logr[i].pid_t_rate[0]);Serial.print(","); Serial.print(logr[i].pid_t_rate[1]); Serial.print(")"); 
        Serial.print("("); Serial.print(logr[i].pid_log_rate[0]);Serial.print(","); Serial.print(logr[i].pid_log_rate[1]); Serial.print(")"); 
        Serial.print("("); Serial.print(trg_rate[0]-logr[i].pid_log_rate[0]);Serial.print(","); Serial.print(trg_rate[1]-logr[i].pid_log_rate[1]); Serial.print(")"); 
//        Serial.print("("); Serial.print(logr[i].pid_log_ierr[0]);Serial.print(","); Serial.print(logr[i].pid_log_ierr[1]); Serial.print(")"); 
        Serial.print("("); Serial.print(logr[i].pid_log_derr[0]);Serial.print(","); Serial.print(logr[i].pid_log_derr[1]); Serial.print(")"); 
        Serial.print("("); Serial.print(logr[i].pid_log_pow[0]);Serial.print(","); Serial.print(logr[i].pid_log_pow[1]); Serial.print("),"); 
        Serial.print(logr[i].pid_t_err); Serial.print(","); 
        Serial.print(logr[i].t_ang); Serial.print(","); 
        Serial.print(logr[i].t_adv); 
        Serial.print(";"); 
        logr[i].pid_log_idx=255; // mark as empty      
        delay(10);
        }
      }      
      Serial.print("\",");
      pid_cnt=0;
      pid_log_ptr=0;
      break;
    }
    case EnumCmdWallLog: {
      Serial.print("\""); Serial.print("\"LOGR\":\""); 
      for(uint8_t i=WALL_LOG_SZ-1; i>=0; i--) {
        Serial.print(logw[i].adv);Serial.print(","); Serial.print(logw[i].usd);Serial.print(";"); 
        delay(10);
      }
      Serial.print("\",");
      break;
    }

    case EnumCmdTaskMove:       
    case EnumCmdTaskRotate:           
      addJson("FT", flags&R_F_ISTASK);
      addJson("FM", flags&R_F_ISTASKMOV);
      addJson("TG", task_target);
//      addJson("TP", task_progress); 
      break;
    default:;
   }
  Serial.println();
}

boolean CheckCommandTimeout(uint16_t t)
{
  unsigned long commandTime = millis();
  if ( commandTime >= lastCommandTime) commandTime -= lastCommandTime; 
  else lastCommandTime = 0;
  return commandTime > t;
}

void readUSDist() {
  int16_t usd_prev = us_dist;
  digitalWrite(US_OUT, LOW);
  delayMicroseconds(2);
  digitalWrite(US_OUT, HIGH);
  delayMicroseconds(10);
  digitalWrite(US_OUT, LOW);
  //uint32_t ms=millis();
  uint32_t d=pulseIn(US_IN, HIGH, 25000);
  if(!d) return;
  //us_meas_dur = millis()-ms;
  us_dist=(int16_t)(d/58);  
  if(F_ISDRIVE()) {
      for(uint8_t i=WALL_LOG_SZ-1; i>=1; i--) logw[i]=logw[i-1];
      logw[0].adv=t_adv/10; //cm
      logw[0].usd=us_dist-usd_prev;
  }
}

void StartDrive() 
{
  for(int i=0; i<2; i++) {
    if(drv_dir[i]) {
       //trg_rate[i]=map(cmd_power[i], 0, 100, 0, calib_enc_rate);
       //cur_power[i]=map(cmd_power[i], 0, 100, 0, M_POW_HIGH);   
       //cur_power[i]=map(cmd_power[i], 0, 100, pow_low, pow_high);        
       cur_power[i]=cmd_power[i];
       if(cur_power[i]>pow_high) cur_power[i]=pow_high;
       trg_rate[i]=map(cur_power[i], pow_low, pow_high, calib_enc_rate_low, calib_enc_rate_high);
     } else {
       trg_rate[i]=0;
       cur_power[i]=0;
     }    
    last_err[i]=0;
    //int_err[i]=0; // ?
  }
  ReadEnc();
  Drive(drv_dir[0], cur_power[0], drv_dir[1], cur_power[1]); // change interface
  if (!F_ISDRIVE())
  {
    F_SETDRIVE();
    digitalWrite(RED_LED, HIGH);
  }        
  pid_cnt=0;
  pid_log_ptr=0;
  lastPidTime=millis(); //NB!
}

void StopDrive() 
{
  if(!F_ISDRIVE()) return;
  Drive(0, 0, 0, 0);
  F_CLEARDRIVE();
  digitalWrite(RED_LED, LOW);  
  last_enc_rate[0]=last_enc_rate[1]=0;
  cur_power[0]=cur_power[1]=0;
  last_dur=millis()-lastCommandTime;
  //us_wall_cnt=0;
}

void StartTask() 
{
//  task_progress=0;
  t_nx=0; t_ny=V_NORM;
  t_x=t_y=0; t_ang=0; t_adv=0;
  
  if(F_ISTASKMOV()) { 
    cmd_power[0]=cmd_power[1]=(pow_low+pow_high)/2;  
    drv_dir[0]=drv_dir[1]=1;
  } else { // rot
    cmd_power[0]=cmd_power[1]=pow_low;  
    if(task_target>0) { // clockwise
      drv_dir[0]=1; drv_dir[1]=2; 
    } else { //counterclockwise
      drv_dir[0]=2; drv_dir[1]=1; 
    }
  }
  StartDrive();
}

void StopTask() 
{
  StopDrive();
  F_CLEARTASK();
}

boolean TrackTask() 
{
  //if(F_ISTASKMOV()) return task_progress>=task_target;
  if(F_ISTASKMOV()) return t_y/10+t_adv>=task_target;
  else {
    // test. use global angle. So do this only after Reset !!!
    if(task_target>0) { //clockwise
      return t_ang+t_adv>=task_target;
    }
    else { //counterclockwise
      return t_ang+t_adv<=task_target;
    }
  }
}
  
void ReadEnc()
{
  int16_t s[2];
  int16_t tx, ty;
  int16_t dd, df;
  //uint16_t tl;
  int16_t tl;

  for(uint8_t i=0; i<2; i++) {
    last_enc_cnt[i]=v_enc_cnt[i];
    v_enc_cnt[i] = 0;
    if(drv_dir[i]==2) s[i]=-last_enc_cnt[i];
    else s[i]=last_enc_cnt[i];
  }

  // dead reckoning 
  if(s[0] || s[1]) {  
    dd = CHGST_TO_MM_10(s[0]+s[1]); // in 10th mm
    df = CHGST_TO_MM_10(s[0]-s[1]); // in 10th mm
    dist+=dd/2; // drive distance, 10th mm
    diff+=df; // drive diff, 10th mm 
    tx=-ny; ty=nx;     
    tx += (int32_t)nx*df/WHEEL_BASE_MM_10;
    ty += (int32_t)ny*df/WHEEL_BASE_MM_10;
    tl=isqrt32((int32_t)tx*tx+(int32_t)ty*ty);
    tx=(int32_t)tx*V_NORM/tl;  
    ty=(int32_t)ty*V_NORM/tl;
    angle += asin32d(invsin(nx, ny, ty, -tx, V_NORM), V_NORM);
    nx=ty; ny=-tx;
    x+=(int32_t)nx*dd/(2*V_NORM); // in 10th mm
    y+=(int32_t)ny*dd/(2*V_NORM); // in 10th mm
    // task vars
    if(F_ISTASKANY()) {     
      tx=-t_ny; ty=t_nx;     
      tx += (int32_t)t_nx*df/WHEEL_BASE_MM_10;
      ty += (int32_t)t_ny*df/WHEEL_BASE_MM_10;
      tl=isqrt32((int32_t)tx*tx+(int32_t)ty*ty);
      tx=(int32_t)tx*V_NORM/tl;  
      ty=(int32_t)ty*V_NORM/tl;
      tl=invsin(t_nx, t_ny, ty, -tx, V_NORM); //vector product = sin
      tl=asin32d(tl, V_NORM); //asin
      t_ang += tl;
      /*
      if(F_ISTASKMOV()) t_adv=dd/2/10; // in mm     
      //else t_adv=(int32_t)tl*180/V_NORM_PI; 
      else t_adv=tl; 
      */
      if(F_ISTASKROT()) t_adv=tl; // in radn
      else t_adv=dd/2/10; // in mm     
      t_nx=ty; t_ny=-tx;
      t_x+=(int32_t)t_nx*dd/(2*V_NORM); // in 10th mm
      t_y+=(int32_t)t_ny*dd/(2*V_NORM); // in 10th mm
    }
  }
}

void PID(uint16_t ctime)
{
  if(ctime>0) {
    int16_t task_progress;
    uint8_t task_incr;    
    uint8_t t_rate[2];
    uint8_t i;
    t_rate[0]=trg_rate[0]; t_rate[1]=trg_rate[1];
    
    if(F_ISTASKANY()) {      
      if(F_ISTASKMOV()) {
        /*
        task_incr=calib_enc_rate_high-calib_enc_rate_low;
        task_progress=t_y/10; // mm
        logr[pid_log_ptr].pid_t_err=t_x/10;  // mm
        for(i=0; i<2; i++) {
          if(task_progress<task_target/2) t_rate[i] = t_rate[i] + (uint32_t)task_incr*task_progress*2/task_target; //0..1
          else {
            t_rate[i] = t_rate[i] + (uint32_t)task_incr*2*(task_target-task_progress)/task_target; //1..0
          }
        } */
      } else { // rotate
      }     
    }
    else
      logr[pid_log_ptr].pid_t_err=0;    

    for(i=0; i<2; i++) {
      int8_t err=0, err_d=0;
      last_enc_rate[i]=(uint8_t)((uint16_t)last_enc_cnt[i]*RATE_SAMPLE_PERIOD/ctime);    
      logr[pid_log_ptr].ec[i]=last_enc_cnt[i];
      if(pid_cnt>=M_WUP_PID_CNT) { // do not correct for the first cycles - ca 100-200ms(warmup)
        //err = trg_rate[i]-last_enc_rate[i];
        err = t_rate[i]-last_enc_rate[i];
        err_d = err-last_err[i];
        //int_err[i]=int_err[i]/2+err;
        int_err[i]=0;      
        int16_t pow=cur_power[i]+((int16_t)err*M_PID_KP+(int16_t)int_err[i]*M_PID_KI+(int16_t)err_d*M_PID_KD)/M_PID_DIV;
        if(pow<0) pow=0;
        if(pow>M_POW_MAX) pow=M_POW_MAX;
        if(err) analogWrite(i==0 ? M1_EN : M2_EN , pow); 
        cur_power[i]=pow;
      }
      last_err[i]=err;
      // log entry
      logr[pid_log_ptr].pid_t_rate[i]=t_rate[i];
      logr[pid_log_ptr].pid_log_rate[i]=last_enc_rate[i];
      logr[pid_log_ptr].pid_log_derr[i]=err_d;
      //logr[pid_log_ptr].pid_log_ierr[i]=int_err[i];
      //logr[pid_log_ptr].pid_log_pow[i]=pow;      
      logr[pid_log_ptr].pid_log_pow[i]=cur_power[i];      
    } 
  logr[pid_log_ptr].t_ang=RADN_TO_GRAD(t_ang);  
  logr[pid_log_ptr].t_adv=RADN_TO_GRAD(t_adv); 
  logr[pid_log_ptr].cmd_id=cmd_id;
  logr[pid_log_ptr].ctime=ctime;
  logr[pid_log_ptr].pid_log_idx=pid_cnt++;   
  // log advance/wrap  
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
  if(dir==0 || pow==0) {
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
// Expect: "D=100,100"
{  
  byte pos;
  int16_t m;
  
  if((pos=Match("D"))) {    
    boolean chg=false;
    if(pos>=bytes || buf[pos]!='=') return EnumErrorBadSyntax;
    for(int i=0; i<2; i++) {      
      pos++;    
      pos = bctoi(pos, &m);      
      if(m<-255 || m>254) return EnumErrorBadParam;
      if(buf[pos] != (i==0 ? ',' : 0)) return EnumErrorBadSyntax;
      if(m==0)       { if(cmd_power[i]) { cmd_power[i]=0; chg=true;}} 
      else if (m>0)  { if(m<pow_low) m=pow_low; if(m>M_POW_HIGH_LIM) m=M_POW_HIGH_LIM; if(drv_dir[i]!=1 || cmd_power[i]!=m) { cmd_power[i]=m; drv_dir[i]=1; chg=true;} } 
      //else           { if((-m)<pow_low) m=-pow_low; if(drv_dir[i]!=2 || cmd_power[i]!=-m) {cmd_power[i]=-m; drv_dir[i]=2; chg=true;} }  
      else           { m=-m; if(m<pow_low) m=pow_low; if(m>M_POW_HIGH_LIM) m=M_POW_HIGH_LIM; if(drv_dir[i]!=2 || cmd_power[i]!=m) {cmd_power[i]=m; drv_dir[i]=2; chg=true;} }  
    }
    if(!cmd_power[0] && !cmd_power[1]) return EnumCmdStop;
    return chg ? EnumCmdDrive : EnumCmdContinueDrive;
  } 
  else if((pos=Match("L"))) {
    return EnumCmdLog;
  }
  else if((pos=Match("W"))) {
    return EnumCmdWallLog;
  } 
  else if((pos=Match("R"))) {
    return EnumCmdRst;
  }
  else if((pos=Match("TM"))) {
    if(pos>=bytes || buf[pos]!='=') return EnumErrorBadSyntax;
    pos++;
    pos = bctoi(pos, &m);      
    if(!m) return EnumErrorBadParam;
    task_target=m*10; // mm
    F_SETTASKMOV();
    return EnumCmdTaskMove;
  }
  else if((pos=Match("TR"))) {
    pos++;
    pos = bctoi(pos, &m);      
    if(!m) return EnumErrorBadParam;
    task_target=GRAD_TO_RADN(m);  // rad norm  
    F_SETTASKROT();
    return EnumCmdTaskRotate;
  }
  else if((pos=Match("T"))) {
    return EnumCmdTest;
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

byte bctoi(byte index, int16_t *val) 
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

void encodeInterrupt_1() { baseInterrupt(0); }

void encodeInterrupt_2() { baseInterrupt(1); } 

void baseInterrupt(uint8_t i) {
  const uint8_t encp[]={ENC1_IN, ENC2_IN};
  uint8_t v=digitalRead(encp[i]);  
  if(v_es[i]==v) return;
  v_es[i]=v;  
  if(v_enc_cnt[i]==255) F_SETOVERFLOW();
  else v_enc_cnt[i]++; 
} 



