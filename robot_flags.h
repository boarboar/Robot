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

enum EnumCmd { EnumCmdDrive=1, EnumCmdTest=2, EnumCmdStop=3, EnumCmdLog=4, EnumCmdContinueDrive=5, EnumCmdRst=6, EnumCmdTaskMove=7, EnumCmdTaskRotate=8, EnumCmdWallLog=9};  
enum EnumError { EnumErrorUnknown=-1, EnumErrorBadSyntax=-2, EnumErrorBadParam=-3, EnumErrorNone=-100};  
