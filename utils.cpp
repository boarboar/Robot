#include <Energia.h>

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

int16_t invsin(int16_t ax, int16_t ay, int16_t bx, int16_t by, uint16_t norm) 
{
  return -((int32_t)ax*by-(int32_t)ay*bx)/norm;
}
  
int16_t asin32(int16_t x, uint16_t norm) // xd: -norm..norm 
{
  return (int32_t)x+(int32_t)(((x*x)/norm)*x)/(6*norm);
}

int16_t inva16(int16_t ax, int16_t ay, int16_t bx, int16_t by, uint16_t norm) 
{
  int32_t x = ((int32_t)ay*bx-(int32_t)ax*by)/norm;
  return (int32_t)x+(int32_t)(((x*x)/norm)*x)/(6*norm);
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

