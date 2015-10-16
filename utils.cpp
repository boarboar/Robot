#include "utils.h"

uint8_t Match(char *buf, uint8_t bytes, const char *cmd) 
{
  uint8_t pos=0;
  while(pos<bytes && *cmd && buf[pos]==*cmd) {
    pos++;
    cmd++;
  }
  if(!*cmd) return pos;
  else return 0;
}

uint8_t bctoi(char *buf, uint8_t index, int16_t *val) 
{
  int16_t i=0;
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
    
int32_t isin32d(int32_t xd)  // xd: -180...180; D=100
{
    const int32_t D = 100;
//    const int32_t PI_D = 314;
    const int32_t PD = 23;
    int32_t xa = xd>0 ? xd : -xd; 
    int32_t yd = (4*D*xd-4*D*xd*xa/180)/180;    
    xa = yd >0 ? yd : -yd;
    yd+=(PD*yd*xa/D-PD*yd)/D;
    return yd;
}

int16_t invsin(int16_t ax, int16_t ay, int16_t bx, int16_t by) 
{
  return -((int32_t)ax*by-(int32_t)ay*bx)/V_NORM;
}
  /*
int16_t asin32(int16_t x) // xd: -norm..norm 
{
  return (int16_t)((int32_t)x+(int32_t)((((int32_t)x*x)/V_NORM)*x)/(6*V_NORM));
}

int16_t asin32x(int16_t s) // xd: -norm..norm 
{
  uint8_t f=0;
  int32_t x=s;
  if(x<-V_NORM || x>V_NORM) return 0;
  if(x<0) { x=-x; f |= 0x01;}
  if(x>V_NORM_PI_4) { // as cos
    x=isqrt32((int32_t)V_NORM*V_NORM-x*x);
    f |= 0x02;
  }		
  x=(x+(((x*x)/V_NORM)*x)/(6*V_NORM));
  if(f & 0x02) x= (int32_t)V_NORM_PI_4-x;		
  if(f & 0x01) x= -x;			
  return (int16_t)x;
}
*/
/*
int16_t asin32(int16_t x) // xd: -norm..norm 
{
  int32_t x1=x;
  x1=x1*x1;
  x1/=(int32_t)V_NORM;
  x1*=(int32_t)x;
  x1/=(int32_t)6L*V_NORM;
  return x1+x;
  //return (int16_t)((int32_t)x+(int32_t)((((int32_t)x*x)/V_NORM)*x)/(6*V_NORM));
}
*/

int16_t asin16(int16_t s) // xd: -norm..norm 
{
  uint8_t f=0;
  int32_t x=s;
  if(x<-V_NORM) return -V_NORM_PI_2;
  if(x>V_NORM) return V_NORM_PI_2;
  if(x<0) { x=-x; f |= 0x01;}
  if(x>V_NORM_PI_4) { // as cos
    x=isqrt32((int32_t)V_NORM*V_NORM-x*x);
    f |= 0x02;
  }		
  //x=(x+(((x*x)/V_NORM)*x)/(6*V_NORM));
  int32_t x1=x;
  x1=x1*x1;
  x1/=(int32_t)V_NORM;
  x1*=(int32_t)x;
  x1/=(int32_t)6L*V_NORM;
  x1+=(int32_t)x;
  if(f & 0x02) x1= (int32_t)V_NORM_PI_2-x1;		
  if(f & 0x01) x1= -x1;			
  return (int16_t)x1;
}

int16_t inva16(int16_t ax, int16_t ay, int16_t bx, int16_t by) 
{
  //int32_t x = ((int32_t)ay*bx-(int32_t)ax*by)/V_NORM;
  //return (int16_t)(x+(((x*x)/V_NORM)*x)/(6*V_NORM));
  int16_t x=(int16_t)(((int32_t)ay*bx-(int32_t)ax*by)/V_NORM);
  return asin16(x);
}

void normalize(int16_t *px, int16_t *py) {  
  int16_t x=*px, y=*py;
  int16_t tl=isqrt32((int32_t)x*x+(int32_t)y*y);
  *px=(int32_t)x*V_NORM/tl;  
  *py=(int32_t)y*V_NORM/tl;
}

void addJson(const char *name, int16_t value) {
  Serial.print("\"");
  Serial.print(name);
  Serial.print("\":");
  Serial.print(value);
  Serial.print(",");
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
 
 /*
 // returns VCC in .01 volts
int16_t getVcc() {
  // start with the 1.5V internal reference
  analogReference(INTERNAL1V5);
  int data = analogRead(11);
  // if overflow, VCC is > 3V, switch to the 2.5V reference
  if (data==0x3ff) {
    analogReference(INTERNAL2V5); 
    data = (int16_t)map(analogRead(11), 0, 1023, 0, 500);
  } else {
    data = (int16_t)map(data, 0, 1023, 0, 300);
  }
  analogReference(DEFAULT);
  return data;  
}
*/
 /*

float sine(float x)
{
    const float B = 4/pi;
    const float C = -4/(pi*pi);

    float y = B * x + C * x * abs(x);

    #ifdef EXTRA_PRECISION
    //  const float Q = 0.775;
        const float P = 0.225;

        y = P * (y * abs(y) - y) + y;   // Q * y + P * y * abs(y)
    #endif
}

                int yt[]={0,0, 0, -100, -100, 71, -71, 50, -50};
		int xt[]={0,314, -314, 157, -157, 79, -79, 52, -52};
                int xtd[]={0,180, -180, 90, -90, 45, -45, 30, -30};
		for(int i=0; i<xt.length; i++) {
			System.out.println(i + " : "+xt[i]+" : "+sin32d(xtd[i])+" : "+yt[i]);
		}

*/
/*
  // tests: 0->0
    // 314->0
    // -314->0
    // 157->100
    // -157->-100
    // 79->71
    // -79->-71
    // 52->50
    // -52->-50
*/  
/*
int32_t sin32(int32_t xd)  // xd: -PI*D...PI*D; D=100
{
    const int32_t D = 100;
    const int32_t PI_D = 314;
    const int32_t PD = 23;
    int32_t xa = xd>0 ? xd : -xd; 
    int32_t yd = (4*D*xd-4*D*xd*xa/PI_D)/PI_D;    
    xa = yd >0 ? yd : -yd;
    yd+=(PD*yd*xa/D-PD*yd)/D;
    return yd;
}
*/

/*
void PrintLogRecs() {
    uint8_t i;
    uint8_t last_cmd_id=0;
    uint8_t first_idx=255;
    uint8_t start_pos=0;
    for(i=0; i<PID_LOG_SZ; i++) {
      if(logr[i].pid_log_idx!=255 && logr[i].cmd_id>last_cmd_id) last_cmd_id=logr[i].cmd_id;
    }
    for(i=0; i<PID_LOG_SZ; i++) {
      if(logr[i].pid_log_idx!=255 && logr[i].cmd_id==last_cmd_id && logr[i].pid_log_idx<first_idx) {first_idx=logr[i].pid_log_idx; start_pos=i; }
    }
    Serial.print("\"LOGR\":\"");             
    i=start_pos;
    if(last_cmd_id) do {
        PrintLogReg(i);
        Serial.print(";"); 
        delay(10);
        if(++i>=PID_LOG_SZ) i=0;
        
    } while(i!=start_pos && logr[i].pid_log_idx!=255 && logr[i].cmd_id==last_cmd_id);
    
   for(i=0; i<PID_LOG_SZ; i++) logr[i].pid_log_idx=255; // cleanup
   Serial.print("\",");
   pid_cnt=0;
   pid_log_ptr=0;
}


void PrintLogReg(uint8_t i) {
        PrintLog(logr[i].pid_log_idx); PrintLog(logr[i].cmd_id); PrintLog(logr[i].ctime);
        PrintLogPair(logr[i].ec[0], logr[i].ec[1]); 
        PrintLogPair(logr[i].pid_t_rate[0], logr[i].pid_t_rate[1]);
        PrintLogPair(logr[i].pid_log_rate[0], logr[i].pid_log_rate[1]);
        PrintLogPair(trg_rate[0]-logr[i].pid_log_rate[0], trg_rate[1]-logr[i].pid_log_rate[1]);
        PrintLogPair(logr[i].pid_log_derr[0], logr[i].pid_log_derr[1]);
        PrintLogPair(logr[i].pid_log_ierr[0], logr[i].pid_log_ierr[1]);
        PrintLogPair(logr[i].pid_log_pow[0], logr[i].pid_log_pow[1]);
        Serial.print(":");
        PrintLogPair(logr[i].t_x, logr[i].t_y);
        PrintLogPair(logr[i].t_dist, logr[i].t_adv_d);
        PrintLogPair(logr[i].t_ang, logr[i].t_adv_a);
        PrintLogPair(logr[i].pid_t_err[0], logr[i].pid_t_err[1]);
 
}
*/

     /*
      // log entry
      logr[pid_log_ptr].ec[i]=last_enc_cnt[i];
      //logr[pid_log_ptr].pid_t_rate[i]=t_rate[i];
      logr[pid_log_ptr].pid_t_rate[i]=trg_rate[i];
      logr[pid_log_ptr].pid_log_rate[i]=last_enc_rate[i];
      logr[pid_log_ptr].pid_log_derr[i]=err_d;
      logr[pid_log_ptr].pid_log_ierr[i]=int_err[i];
      logr[pid_log_ptr].pid_log_pow[i]=cur_power[i];      
      logr[pid_log_ptr].pid_t_err[i]=t_err[i];  // cm
      */


 // log entry  
  /*
  logr[pid_log_ptr].t_ang=RADN_TO_GRAD(t_ang);  
  logr[pid_log_ptr].t_adv_a=RADN_TO_GRAD(t_adv_a);
  logr[pid_log_ptr].t_dist=t_dist/100; //cm  
  logr[pid_log_ptr].t_adv_d=t_adv_d/100; //cm 
  logr[pid_log_ptr].t_x = t_x/100;
  logr[pid_log_ptr].t_y = t_y/100;  
  logr[pid_log_ptr].cmd_id=cmd_id;
  logr[pid_log_ptr].ctime=ctime;
  
  logr[pid_log_ptr].pid_log_idx=pid_cnt;   
  Serial.print("@LP:"); PrintLogReg(pid_log_ptr); Serial.println();  
  // log advance/wrap  
  if(++pid_log_ptr>=PID_LOG_SZ) pid_log_ptr=0;        
  */
  
  /*
#define PID_LOG_SZ 1
uint8_t pid_log_ptr=0;

struct __attribute__((__packed__)) LogRec {
  uint8_t cmd_id;       // ref to cmd id
  uint8_t pid_log_idx;   
  uint8_t ctime;         // pid interval 
  uint8_t ec[2];
  uint8_t pid_t_rate[2];
  uint8_t pid_log_rate[2];
  int8_t pid_log_derr[2];
  int8_t pid_log_ierr[2];
  uint8_t pid_log_pow[2];
  int8_t pid_t_err[2];
  int16_t t_ang;
  int16_t t_dist;
  int8_t t_adv_a;
  int8_t t_adv_d;
  int16_t t_x, t_y;
} logr[PID_LOG_SZ];
*/


//  for(i=0; i<PID_LOG_SZ; i++) logr[i].pid_log_idx=255;
  

        /*
        int8_t task_err=t_x/100; //cm
        if(task_err>1) task_err=1;
        else if (task_err<-1) task_err=-1;
        else task_err=0; 
        t_err[0]=-task_err;
        t_err[1]=task_err;
        */
        /*
        task_incr=calib_enc_rate_high-calib_enc_rate_low;
        task_progress=t_y/10; // mm
        for(i=0; i<2; i++) {
          if(task_progress<task_target/2) t_rate[i] = t_rate[i] + (uint32_t)task_incr*task_progress*2/task_target; //0..1
          else {
            t_rate[i] = t_rate[i] + (uint32_t)task_incr*2*(task_target-task_progress)/task_target; //1..0
          }
        } */

