#include <Energia.h>

#define V_NORM 10000
#define V_NORM_PI 31416
#define V_NORM_PI2 62832L
#define V_NORM_PI_2 15708
#define V_NORM_PI_4 7854

#define RADN_TO_GRAD(R)      ((int32_t)(R)*180/V_NORM_PI)
#define GRAD_TO_RADN(D)      ((int32_t)(D)*V_NORM_PI/180)

uint8_t Match(char *buf, uint8_t bytes, const char *cmd); 
uint8_t bctoi(char *buf, uint8_t index, int16_t *val); 
int32_t isin32d(int32_t xd);
uint16_t isqrt32(uint32_t n);  
//int16_t invsin(int16_t ax, int16_t ay, int16_t bx, int16_t by, uint16_t norm); 
int16_t asin32(int16_t x);
int16_t asin32x(int16_t x);
int16_t inva16(int16_t ax, int16_t ay, int16_t bx, int16_t by); 
void normalize(int16_t *px, int16_t *py);
void addJson(const char *name, int16_t value);
void addJsonArr16_2(const char *name, int16_t v1, int16_t v2);

