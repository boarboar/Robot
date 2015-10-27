#include <Energia.h>

#define BUF_SIZE 16

class CommandReader {
  public:
    //CommandReader();
    void Init();
    boolean ReadSerialCommand();
    void StartParse(){ pos=0; }
    void Reset(){ bytes=0; buf[0]=0;}
    boolean Match(const char *cmd);
    //boolean ReadInt(int16_t *val) ;
    int16_t ReadInt() ;
    char ReadChar() ;
  protected:
    char buf[BUF_SIZE];
    uint8_t bytes;
    uint8_t pos;
};

