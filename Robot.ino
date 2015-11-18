
#include "utils.h"
#include "robot_hw.h"
#include "robot_flags.h"
#include "robot_reader.h"

// common vars 
const unsigned int PID_TIMEOUT_HIGH = 200;
const unsigned int PID_TIMEOUT_LOW = 100;
//const unsigned int RESP_TIMEOUT = 90;
const unsigned int RESP_TIMEOUT = 25; // C_T+R_T >= 75ms
const unsigned int CMD_TIMEOUT = 600; 
const unsigned int RATE_SAMPLE_PERIOD = 400;
const unsigned int RATE_SAMPLE_TARGET_LOW = 8;
const unsigned int RATE_SAMPLE_TARGET_ROT_LOW = 2;
const unsigned int RATE_SAMPLE_TARGET_HIGH = 18;
const unsigned int WHEEL_RATIO_RPM = (60000/RATE_SAMPLE_PERIOD/WHEEL_CHGSTATES);
const unsigned int WHEEL_RATIO_SMPS_10 = (WHEEL_RAD_MM_10/10*628/RATE_SAMPLE_PERIOD/WHEEL_CHGSTATES);
const unsigned int US_WALL_DIST=20;
//const unsigned int US_WALL_CNT_THR=100;
const unsigned int M_COAST_TIME=400;
const unsigned int M_WUP_PID_CNT=3;
const unsigned int TASK_TIMEOUT = 20000; 
const int US_STALL_ADV_LIM = 4;
const int US_STALL_USD_LIM = 2;

#define CHGST_TO_MM_10(CNT)  ((int32_t)(CNT)*V_NORM_PI2*WHEEL_RAD_MM_10/WHEEL_CHGSTATES/10000)

// for 7.5v
#define M_POW_LOWEST_LIM   10
#define M_POW_HIGH_LIM 100
#define M_POW_MAX  120
#define M_POW_STEP 2

#define M_PID_KP_0   25
#define M_PID_KD_0  140
#define M_PID_KI_0    2
#define M_PID_DIV   50

uint8_t M_PID_KP = M_PID_KP_0;
uint8_t M_PID_KD = M_PID_KD_0;
uint8_t M_PID_KI = M_PID_KI_0;

const int8_t M_PID_TERR_LIM=100; // prev was 4. 

#define SENS_K_K  5
#define SENS_K_DIV 10

struct TaskStruct {
  int32_t target;  // in mm or nrads  
  //int16_t nx, ny;    // NORM
  //int32_t x, y;      //in 10thmm - up to 320 cm
  int32_t angle;        // in nrads
  int32_t x_abs, y_abs;      //in 10thmm - up to 320 cm  
  int32_t dist;   // in 10th mm
  int16_t adv_d;  // in 10th mm
  int16_t adv_a;  // in nrads
  //int16_t bearing;
  int16_t bearing_abs;
  int8_t power;
} task;

// cmd
uint32_t lastCommandTime; // =lastTaskTime
uint32_t lastPidTime;
uint16_t last_dur=0;
uint8_t cmd_id=0;
int8_t cmdResult=EnumErrorNone;
uint8_t cmd_power[2]={0,0}; 
uint8_t drv_dir[2]={0,0}; 

// calibration parameters
uint8_t enc_rate_high=0, enc_rate_low=0, enc_rot_rate_low=0 ; // target rate (counts per RATE_SAMPLE_PERIOD) for low power
uint8_t pow_low=0, pow_high=0, pow_rot_low=0;
uint8_t coast_low=0, coast_high=0, coast_rot_low=0; // in cm / deg

// current state 
uint16_t flags=0; 
int32_t dist=0;  // in 10thmm
int16_t diff=0;  // in 10tmm

int32_t x=0, y=0;// in 10thmm
int16_t nx=0, ny=V_NORM;
int32_t angle=0; 

uint16_t us_dist=9999; 

// PID section
uint16_t pid_cnt=0;
int16_t int_err[2]={0,0};

uint8_t pid_to=0;
uint8_t cur_power[2]={0,0}; 
uint8_t trg_rate[2]={0,0}; 
uint8_t enc_cnt[2]={0,0}; 
uint8_t enc_rate[2]={0,0}; 
int8_t  prev_err[2]={0,0};
int8_t d_err[2]={0,0};
int8_t t_err[2]={0,0};

//uint8_t enc_rate_opt[2]={0,0};  // Kalman

#define WALL_LOG_SZ 8
struct __attribute__((__packed__)) WallRec {
  //int8_t adv; //cm
  //int8_t usd; //cm
  int8_t usd_k; //cm - kalman opt
  int8_t adv_k; //cm - kalman opt
  int8_t stall;
} logw[WALL_LOG_SZ]; // candidate for calman filter

CommandReader cmdReader;

// volatile encoder section
volatile uint8_t v_enc_cnt[2]={0,0}; 
volatile uint8_t v_es[2]={0,0};
//volatile uint16_t v_enc_cnt_2[2]={0,0}; // test

//uint16_t enc_cnt_2[2]={0,0}; // test

void setup()
{ 
  uint8_t i;
  int ports[6]={M1_OUT_1,M1_OUT_2,M2_OUT_1,M2_OUT_2, M1_EN, M2_EN };
  for(i=0;i<6;i++) {
    digitalWrite(ports[i], LOW); 
    pinMode(ports[i], OUTPUT);
  }
 
  pinMode(ENC1_IN, INPUT);     
  attachInterrupt(ENC1_IN, encodeInterrupt_1, CHANGE); 
  pinMode(ENC2_IN, INPUT);  
  attachInterrupt(ENC2_IN, encodeInterrupt_2, CHANGE); 
  
  pinMode(US_OUT, OUTPUT);     
  pinMode(US_IN, INPUT);     
   
  analogFrequency(32); 
  Serial.begin(TTY_SPEED);
  
  pinMode(RED_LED, OUTPUT);     
  for(i=0; i<5; i++) {
    digitalWrite(RED_LED, HIGH); delay(100); digitalWrite(RED_LED, LOW); 
  }
  
  // calibration sequence
  pow_low=M_POW_LOWEST_LIM;
  Calibrate(RATE_SAMPLE_TARGET_LOW, &pow_low, &enc_rate_low, &coast_low, false);
  pow_high=pow_low+M_POW_STEP;
  Calibrate(RATE_SAMPLE_TARGET_HIGH, &pow_high, &enc_rate_high, &coast_high, false);

  pow_rot_low=M_POW_LOWEST_LIM;
  Calibrate(RATE_SAMPLE_TARGET_ROT_LOW, &pow_rot_low, &enc_rot_rate_low, &coast_rot_low, true);
  
  for(i=0; i<WALL_LOG_SZ; i++) { /*logw[i].adv=logw[i].usd=*/logw[i].adv_k=logw[i].usd_k=-127;}
  
  cmdReader.Init();
  
  lastCommandTime = lastPidTime = millis();  
  
  InitPos();
}

void loop()
{  
  uint32_t cycleTime = millis();
  if (F_ISDRIVE() && (us_dist<US_WALL_DIST && drv_dir[0]+drv_dir[1]==2)) {
    F_SETWALL();
    if(F_ISTASKANY()) StopTask();
    else StopDrive(); 
  }
  if ( cycleTime < lastPidTime) lastPidTime=0; // wraparound, not correct   
  uint16_t ctime = cycleTime - lastPidTime;
  if ( ctime >= pid_to) { // PID cycle    
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
          
  if(cmdReader.ReadSerialCommand()) {
    cmdResult = Parse(); 
    cmdReader.Reset();
    cmd_id++;  
    if(cmdResult==EnumCmdDrive || (cmdResult==EnumCmdContinueDrive && !F_ISDRIVE())) {
      F_CLEARTASK();
      lastCommandTime = millis();        
      last_dur=0;
      if(!(us_dist<US_WALL_DIST && drv_dir[0]+drv_dir[1]==2)) StartDrive(false);
    } else if(cmdResult==EnumCmdContinueDrive && F_ISDRIVE()) {
      last_dur=millis()-lastCommandTime;
      lastCommandTime = millis();
    } else if(cmdResult==EnumCmdStop) {
      StopDrive();
    } else if(cmdResult==EnumCmdRst) {
      StopDrive();
      InitPos();
    } 
    else if(cmdResult==EnumCmdTest) { ; }
    else if(cmdResult==EnumCmdTaskMove || cmdResult==EnumCmdTaskRotate  || cmdResult==EnumCmdTaskAbs) {
        lastCommandTime = millis();        
        StartTask();
    }
    delay(RESP_TIMEOUT);
    Notify(); 
  } // read serial
}

void InitPos() { 
  x=y=0;
  nx=0; ny=V_NORM;
  dist=diff=0;
  angle=0;
//  v_enc_cnt_2[0]=v_enc_cnt_2[1]=0;
//  enc_cnt_2[0]=enc_cnt_2[1]=0;
}      
      
void StartDrive(boolean rot) 
{
  uint8_t pow_low_d;
  uint8_t enc_rate_low_d;
  if(rot) {
    pow_low_d=pow_rot_low;
    enc_rate_low_d=enc_rot_rate_low;
  } else {
    pow_low_d=pow_low;
    enc_rate_low_d=enc_rate_low;
  }
  for(int i=0; i<2; i++) {
    if(drv_dir[i]) {
       cur_power[i]=cmd_power[i];
       if(cur_power[i]>pow_high) cur_power[i]=pow_high;
       /*
       if(cur_power[i]<pow_low) cur_power[i]=pow_low;
       trg_rate[i]=map(cur_power[i], pow_low, pow_high, enc_rate_low, enc_rate_high);
       */
       if(cur_power[i]<pow_low_d) cur_power[i]=pow_low_d;
       trg_rate[i]=map(cur_power[i], pow_low_d, pow_high, enc_rate_low_d, enc_rate_high);
     } else {
       trg_rate[i]=0;
       cur_power[i]=0;
     }    
    prev_err[i]=0;
    int_err[i]=0; 
    //enc_rate_opt[i]=0;
    enc_rate[i]=0;
  }
  ReadEnc();
  Drive(drv_dir[0], cur_power[0], drv_dir[1], cur_power[1]); // change interface
  if (!F_ISDRIVE())
  {
    F_SETDRIVE();
    digitalWrite(RED_LED, HIGH);
  }          
  pid_cnt=0;
  pid_to = PID_TIMEOUT_HIGH;
  lastPidTime=millis(); 
}

void StopDrive() 
{
  if(!F_ISDRIVE()) return;
  Drive(0, 0, 0, 0);
  F_CLEARDRIVE();
  digitalWrite(RED_LED, LOW);  
  enc_rate[0]=enc_rate[1]=0;
  cur_power[0]=cur_power[1]=0;
  last_dur=millis()-lastCommandTime;
}

void StartTask() 
{
  boolean rot=false;
  //task.nx=0; task.ny=V_NORM;
  //task.x=task.y=0; 
  task.angle=task.dist=task.adv_d=task.adv_a=0;
  
  if(F_ISTASKMOV()) { 
    //cmd_power[0]=cmd_power[1]=(pow_low+pow_high)/2;  
    cmd_power[0]=cmd_power[1]=map(task.power, 0, 100, pow_low, pow_high);
    drv_dir[0]=drv_dir[1]=1;
    task.x_abs=x+(int32_t)nx*(task.target)/V_NORM*10; //10th mm
    task.y_abs=y+(int32_t)ny*(task.target)/V_NORM*10; //10th mm
  } else if(F_ISTASKROT()) { // rot
    rot=true;
    cmd_power[0]=cmd_power[1]=pow_rot_low;
    if(task.target>0) { // clockwise
      drv_dir[0]=1; drv_dir[1]=2; 
    } else { //counterclockwise
      drv_dir[0]=2; drv_dir[1]=1; 
    }
  } else if(F_ISTASKABS()) { // abs
    //task.dist = 1000; // test
    cmd_power[0]=cmd_power[1]=map(task.power, 0, 100, pow_low, pow_high);
    drv_dir[0]=drv_dir[1]=1;
  }
 
  StartDrive(rot);
}

void StopTask() 
{
  StopDrive();
  LogTaskComplete();
  F_CLEARTASK();
}

boolean TrackTask() 
{ 
  bool res=false;
  if(F_ISTASKMOV()) {
    if((task.dist+2*task.adv_d)/10>=task.target) pid_to = PID_TIMEOUT_LOW;
    res = (task.dist+task.adv_d)/10>=task.target; //mm
  }
  else if(F_ISTASKROT()) {
    if(task.target>0) { //clockwise
      if((task.angle+2*task.adv_a)>=task.target) pid_to = PID_TIMEOUT_LOW;
      res = task.angle+task.adv_a>=task.target;
    }
    else { //counterclockwise
      if((task.angle+2*task.adv_a)<=task.target) pid_to = PID_TIMEOUT_LOW;
      res = task.angle+task.adv_a<=task.target;
    }
  } else if(F_ISTASKABS()) {
    uint32_t d2=(uint32_t)((task.x_abs-x)/100)*((task.x_abs-x)/100)+(uint32_t)((task.y_abs-y)/100)*((task.y_abs-y)/100); // in cm2
    if(d2<1600) pid_to = PID_TIMEOUT_LOW; // rad<40cm
    res=d2<100;  // rad<10cm
  }
  if(res) F_SETTFIN();
  return res;
}
  
void ReadEnc()
{
  int16_t s[2];
  int16_t tx, ty;
  int16_t dd, df;
  int16_t tl;

  for(uint8_t i=0; i<2; i++) {
    enc_cnt[i]=v_enc_cnt[i];
    v_enc_cnt[i] = 0;
    if(drv_dir[i]==2) s[i]=-enc_cnt[i];
    else s[i]=enc_cnt[i];
    
//    enc_cnt_2[i]+=enc_cnt[i];
    
  }

  // tracking 
  if(s[0] || s[1]) {  
    dd = CHGST_TO_MM_10(s[0]+s[1]); // in 10th mm
    df = CHGST_TO_MM_10(s[0]-s[1]); // in 10th mm
    dist+=dd/2; // drive distance, 10th mm
    diff+=df; // drive diff, 10th mm 
    
    tx=-ny; ty=nx;     
    tx += (int32_t)nx*df/WHEEL_BASE_MM_10;
    ty += (int32_t)ny*df/WHEEL_BASE_MM_10;
    normalize(&tx, &ty);
    tl = inva16(nx, ny, ty, -tx); //angle from vector product
    nx=ty; ny=-tx;
    x+=(int32_t)nx*dd/(2*V_NORM); // in 10th mm
    y+=(int32_t)ny*dd/(2*V_NORM); // in 10th mm
    angle = (angle+tl)%V_NORM_PI2;
    // task vars
    task.dist += dd/2; // in 10th mm
    task.adv_d = dd/2; // in 10th mm
    task.adv_a = tl;
    task.angle += tl;
    tx=(task.x_abs-x)/10; ty=(task.y_abs-y)/10; // in mm
    normalize(&tx, &ty);
    task.bearing_abs = inva16(nx, ny, tx, ty); //angle from vector product
        /*
    tx=-task.ny; ty=task.nx;     
    tx += (int32_t)task.nx*df/WHEEL_BASE_MM_10;
    ty += (int32_t)task.ny*df/WHEEL_BASE_MM_10;
    normalize(&tx, &ty);
    task.nx=ty; task.ny=-tx;
    task.x+=(int32_t)task.nx*dd/(2*V_NORM); // in 10th mm
    task.y+=(int32_t)task.ny*dd/(2*V_NORM); // in 10th mm
    
    tx=-task.x/10; ty=task.target-task.y/10; // in mm
    normalize(&tx, &ty);
    task.bearing = inva16(task.nx, task.ny, tx, ty); //angle from vector product
    */
    
  }
}

void PID(uint16_t ctime)
{
  if(ctime>0) {
    uint8_t i;
    t_err[0]=t_err[1]=0;   
    if(F_ISTASKANY()) {      
      if(F_ISTASKMOV() || F_ISTASKABS()) {
        int8_t task_err=RADN_TO_GRAD(task.bearing_abs)/10;
        if(task_err>M_PID_TERR_LIM) task_err=M_PID_TERR_LIM;
        if(task_err<-M_PID_TERR_LIM) task_err=-M_PID_TERR_LIM;
        t_err[0]=task_err;
        t_err[1]=-task_err;
      } else { // rotate
      }     
    }
    for(i=0; i<2; i++) {
      int8_t p_err=0;
      enc_rate[i]=(uint8_t)((uint16_t)enc_cnt[i]*RATE_SAMPLE_PERIOD/ctime);    
      //enc_rate[i]=(uint8_t)( ((uint16_t)rate*M_K_K+(uint16_t)enc_rate[i]*(M_K_D-M_K_K))/M_K_D ); // Kalman
      if(pid_cnt>=M_WUP_PID_CNT) { // do not correct for the first cycles - ca 100-200ms(warmup)
        p_err = (trg_rate[i]-enc_rate[i])+t_err[i];
        d_err[i] = p_err-prev_err[i];
        int_err[i]=int_err[i]+p_err;
        int16_t pow=cur_power[i]+((int16_t)p_err*M_PID_KP+(int16_t)int_err[i]*M_PID_KI+(int16_t)d_err[i]*M_PID_KD)/M_PID_DIV;
        if(pow<0) pow=0;
        if(pow>M_POW_MAX) pow=M_POW_MAX;
        if(cur_power[i]!=pow) analogWrite(i==0 ? M1_EN : M2_EN , pow); 
        cur_power[i]=pow;
      }
      prev_err[i]=p_err;
    } 
  PrintLogToSerial(ctime);
  pid_cnt++;
  }
}

void Calibrate(uint8_t targ, uint8_t *ppow, uint8_t *pencr, uint8_t *pcoast, uint8_t rot) {
  uint8_t pow = *ppow;
  F_SETDRIVE();
  digitalWrite(RED_LED, HIGH);  
  while(pow<M_POW_HIGH_LIM) {
   ReadEnc(); 
   Drive(1, pow, rot ? 2 : 1, pow);
   delay(RATE_SAMPLE_PERIOD);
   ReadEnc();   
   if(enc_cnt[0]>=targ && enc_cnt[1]>=targ) break;
   pow+=M_POW_STEP;
  }
  *ppow=pow;
  *pencr = (enc_cnt[0]+enc_cnt[1])/2;
  // coasting
  delay(RATE_SAMPLE_PERIOD);
  digitalWrite(RED_LED, LOW);  
  uint8_t i=0;
  task.dist=0;
  task.angle=0;
  StopDrive();
  while((enc_cnt[0]+enc_cnt[1])>0 && i++<10) {
    delay(RATE_SAMPLE_PERIOD);
    ReadEnc();
  }
  if(rot)
    *pcoast = RADN_TO_GRAD(task.angle);
  else 
    *pcoast = task.dist/100;
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

void PrintLogToSerial(uint16_t ctime) {
  Serial.print("@LP:"); 
  PrintLog(cmd_id); PrintLog(ctime);
  PrintLogPair(enc_cnt[0], enc_cnt[1]); 
  PrintLogPair(trg_rate[0], trg_rate[1]);
  PrintLogPair(enc_rate[0], enc_rate[1]);
  Serial.print(":P:");
  PrintLogPair(trg_rate[0]-enc_rate[0], trg_rate[1]-enc_rate[1]);
  PrintLogPair(t_err[0], t_err[1]);
  PrintLogPair(d_err[0], d_err[1]);
  PrintLogPair(int_err[0], int_err[1]);
  PrintLogPair(cur_power[0], cur_power[1]);
  delay(10);
  Serial.print(":T:");
  PrintLogPair((task.x_abs-x)/100, (task.y_abs-y)/100); //in cm
  PrintLogPair(task.dist/100, task.adv_d/100); //in cm
  PrintLogPair(RADN_TO_GRAD(task.angle), RADN_TO_GRAD(task.adv_a));
  PrintLog(RADN_TO_GRAD(task.bearing_abs));
  /*
  int16_t tx, ty;
  tx=-task.x/10; ty=task.target-task.y/10; // in mm
  normalize(&tx, &ty);
  PrintLogPair(task.nx, task.ny); 
  PrintLogPair(tx, ty); 
  PrintLogPair(RADN_TO_GRAD(task.bearing), RADN_TO_GRAD(task.bearing_abs));
  */
  Serial.print(":S:");
  PrintLogPair(logw[0].adv_k, logw[0].usd_k); 
  PrintLog(logw[0].stall);
  Serial.println(); 
}

void PrintLog(int16_t v) {
  Serial.print(v);Serial.print(","); 
}
void PrintLogPair(int16_t v1, int16_t v2) {
  Serial.print("("); Serial.print(v1);Serial.print(","); Serial.print(v2); Serial.print("),"); 
}

void LogTaskComplete() {
  Serial.print("@LTC:"); 
  PrintLog(last_dur);
  addJsonBin("FLG", flags); 
  Serial.println(); 
}

void Notify() {
  addJson("CQ", cmdResult);
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
          s[i]=enc_rate[i]*WHEEL_RATIO_SMPS_10;
          if(drv_dir[i]==2) s[i]=-s[i];
        } 
      }
      addJson("L", last_dur); 
      addJsonArr16_2("P", cur_power[0], cur_power[1]);
      addJsonArr16_2("T", trg_rate[0], trg_rate[1]);
      addJsonArr16_2("R", enc_rate[0], enc_rate[1]);
      addJsonArr16_2("W", enc_rate[0]*WHEEL_RATIO_RPM, enc_rate[1]*WHEEL_RATIO_RPM);      
      addJson("S", (s[0]+s[1])/2);
      addJson("D", (int16_t)(dist/100)); // in cm
      addJson("F", (int16_t)(diff/100)); // in cm
      addJsonArr16_2("N", (int16_t)nx, (int16_t)ny); // in normval
      addJsonArr16_2("X", (int16_t)(x/100), (int16_t)(y/100)); // in cm
      addJson("U", (int16_t)(us_dist));
      }
      break; 
    case EnumCmdTest:    
 //     addJson("VCC", getVcc());
      addJsonArr16_2("N", (int16_t)nx, (int16_t)ny); // in normval
      addJsonArr16_2("X", (int16_t)(x/100), (int16_t)(y/100)); // in cm
      addJson("A", RADN_TO_GRAD(angle)); 
      addJson("U", (int16_t)(us_dist));      
      delay(10);
      addJson("TG", task.target);
      //addJsonArr16_2("TN", (int16_t)task.nx, (int16_t)task.ny); // in normval
      //addJsonArr16_2("TX", (int16_t)(task.x/100), (int16_t)(task.y/100)); // in cm
      addJsonArr16_2("DX", (int16_t)((task.x_abs-x)/100), (int16_t)((task.y_abs-y)/100)); // in cm
      addJsonArr16_2("TD", (int16_t)(task.dist/100), (int16_t)(task.adv_d/100)); // in cm 
      addJsonArr16_2("TA", RADN_TO_GRAD(task.angle), RADN_TO_GRAD(task.adv_a)); // in deg
      delay(10);
      addJson("L", last_dur);       
      addJsonBin("FLG", flags); 
//      addJsonArr16_2("ECC2", enc_cnt_2[0], enc_cnt_2[1]);
//      addJsonArr16_2("ECCV2", v_enc_cnt_2[0], v_enc_cnt_2[1]);
      break;
   case EnumCmdSetParam:    
      addJsonArr16_2("CLB_P_LH", pow_low, pow_high);   
      addJsonArr16_2("CLB_R_LH", enc_rate_low, enc_rate_high);   
      addJsonArr16_2("CLB_C_LH", coast_low, coast_high);   
      addJson("CLB_P_LRT", pow_rot_low);
      addJson("CLB_R_LRT", enc_rot_rate_low);
      addJson("CLB_C_LRT", coast_rot_low);
      delay(10);
      addJson("KP", M_PID_KP);
      addJson("KD", M_PID_KD);
      addJson("KI", M_PID_KI);
      addJson("KDIV", M_PID_DIV);
      break;      
    case EnumCmdWallLog: {
      Serial.print("\"LOGW\":\""); 
      for(uint8_t i=0; i<WALL_LOG_SZ; i++) { 
        Serial.print(":");
        PrintLogPair(logw[i].adv_k, logw[i].usd_k); 
        PrintLog(logw[i].stall);
        delay(10);
      }       
      Serial.print("\",");
      break;
    }
    case EnumCmdTaskMove:       
    case EnumCmdTaskRotate:           
    case EnumCmdTaskAbs:               
      addJson("FT", flags&R_F_ISTASK);
      addJsonBin("FY", flags&(R_F_ISTASKMOV|R_F_ISTASKROT|R_F_ISTASKABS));
      if(F_ISTASKMOV() || F_ISTASKABS()) {
        addJson("TG", task.target/10);
        addJsonArr16_2("TABSX", task.x_abs/100, task.y_abs/100); // absolute, in cm        
      } else {
        addJson("TG", RADN_TO_GRAD(task.target));
        addJson("TABSA", RADN_TO_GRAD(angle+task.target)); // absolute
      }
      addJson("TPOW", task.power);
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
  if(commandTime > t) {
    F_SETTMO();
    return true;
  }  
  return false;
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
      int8_t adv, usd;
      adv=task.adv_d/100; //cm
      usd=usd_prev-us_dist;      
      for(uint8_t i=WALL_LOG_SZ-1; i>=1; i--) logw[i]=logw[i-1];

      if(logw[1].adv_k!=-127) {
        usd=(uint8_t)( ((uint16_t)usd*SENS_K_K+(uint16_t)logw[1].usd_k*(SENS_K_DIV-SENS_K_K))/SENS_K_DIV ); // Kalman
        adv=(uint8_t)( ((uint16_t)adv*SENS_K_K+(uint16_t)logw[1].adv_k*(SENS_K_DIV-SENS_K_K))/SENS_K_DIV ); // Kalman
      }
      logw[0].usd_k=usd;
      logw[0].adv_k=adv;
      
      // CRITERIA
      // Robot moves fwd means both dir fwd && pows comparable (?)
      // adv > limit1 && usd < limi2 for N last cycles
      
      if(drv_dir[0]+drv_dir[1]==2 && adv>=US_STALL_ADV_LIM && usd<=US_STALL_ADV_LIM) logw[0].stall=1; // todo: add a check tjat the pows[] are comparable
      else logw[0].stall=0;
      
  }
}
//=======================================

int8_t Parse()
{  
  int16_t m;
  cmdReader.StartParse();
  if(cmdReader.Match("D")) {    
    // Expect: "D=100,100"
    boolean chg=false;
    if(!cmdReader.Match("=")) return EnumErrorBadSyntax;
    for(int i=0; i<2; i++) {      
      m=cmdReader.ReadInt();
      if(m<-255 || m>254) return EnumErrorBadParam;
      if(i==0 && !cmdReader.Match(",")) return EnumErrorBadSyntax;
      if(m==0)       { if(cmd_power[i]) { cmd_power[i]=0; chg=true;}} 
      else if (m>0)  { if(m<pow_low) m=pow_low; if(m>M_POW_HIGH_LIM) m=M_POW_HIGH_LIM; if(drv_dir[i]!=1 || cmd_power[i]!=m) { cmd_power[i]=m; drv_dir[i]=1; chg=true;} } 
      else           { m=-m; if(m<pow_low) m=pow_low; if(m>M_POW_HIGH_LIM) m=M_POW_HIGH_LIM; if(drv_dir[i]!=2 || cmd_power[i]!=m) {cmd_power[i]=m; drv_dir[i]=2; chg=true;} }  
    }
    if(!cmd_power[0] && !cmd_power[1]) return EnumCmdStop;
    return chg ? EnumCmdDrive : EnumCmdContinueDrive;
  } 
  else if(cmdReader.Match("L")) {    
    return EnumCmdLog;
  }
  else if(cmdReader.Match("W")) {      
    return EnumCmdWallLog;
  } 
  else if(cmdReader.Match("R")) {    
    return EnumCmdRst;
  }
  else if(cmdReader.Match("TM")) {      
    if(!cmdReader.Match("=")) return EnumErrorBadSyntax;
    m=cmdReader.ReadInt();
    if(!m) return EnumErrorBadParam;
    task.target=m*10; // mm
    task.power=0;
    if(cmdReader.Match(",")) { // optional power
      m=cmdReader.ReadInt();
      if(m<0) m=0;
      if(m>100) m=100;
      task.power=m;
    }
    F_SETTASKMOV();
    return EnumCmdTaskMove;
  }
  else if(cmdReader.Match("TR")) {        
    if(!cmdReader.Match("=")) return EnumErrorBadSyntax;
    m=cmdReader.ReadInt();
    if(!m) return EnumErrorBadParam;
    task.target=GRAD_TO_RADN(m);  // rad norm
    /*
    if(cmdReader.Match(",")) { // optional power
      m=cmdReader.ReadInt();
      if(m<0) m=0;
      if(m>100) m=100;
      task.power=m;
    } 
   */ 
    F_SETTASKROT();
    return EnumCmdTaskRotate;
  }
  else if(cmdReader.Match("TA")) {        
    if(!cmdReader.Match("=")) return EnumErrorBadSyntax;
    task.x_abs=(int32_t)cmdReader.ReadInt()*100; // in 10th mm
    if(!cmdReader.Match(",")) return EnumErrorBadSyntax;
    task.y_abs=(int32_t)cmdReader.ReadInt()*100; // in 10th mm
    task.power=0;
    F_SETTASKABS();
    return EnumCmdTaskAbs;
  }  
  else if(cmdReader.Match("P")) {        
    // P:D=100
    if(!cmdReader.Match(":")) return EnumCmdSetParam; // output
    char param=cmdReader.ReadChar();
    if(!param) return EnumErrorBadSyntax;
    if(!cmdReader.Match("=")) return EnumErrorBadSyntax;
    m=cmdReader.ReadInt();
    switch(param) {
      case 'P' : M_PID_KP=m; break;
      case 'D' : M_PID_KD=m; break;
      case 'I' : M_PID_KI=m; break;
      default:;
    }
    return EnumCmdSetParam;
  }
  else if(cmdReader.Match("T")) {      
    return EnumCmdTest;
  }
  else return EnumErrorUnknown;
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
//  v_enc_cnt_2[i]++; // test
} 


