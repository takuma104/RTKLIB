#include <stdio.h>
#include "rtklib.h"

#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/time.h>
#define __USE_MISC
#include <errno.h>
#include <termios.h>

#ifdef WITHOUT_FILE
int trace_vprintf(const char *format, va_list ap){
    int res;
    res = vfprintf(stderr, format, ap);
    fflush(stderr);
    return res;
}
#endif

#ifdef WITHOUT_SYSTIME
#include <time.h>
extern gtime_t timeget(){
    time_t timer;
    struct tm *tt;
    time(&timer);
    tt = gmtime(&timer);
    double ep[] = {
        tt->tm_year + 1900, tt->tm_mon + 1, tt->tm_mday,
        tt->tm_hour, tt->tm_min, tt->tm_sec};
    return epoch2time(ep);
}
unsigned int tickget(){
    return clock() * 1000 / CLOCKS_PER_SEC;
}
#endif

#define BAUDRATE B115200

static int open_serial(const char* device_name) {
    int fd;
    
    fd = open(device_name, O_RDONLY|O_NOCTTY|O_NONBLOCK);
    if (fd <0) {perror(device_name); exit(-1); }
    
    char *p,parity='N',dev[128],fctr[64]="";
    int bsize=8,stopb=1;
    struct termios ios={0};
    tcgetattr(fd,&ios);
    ios.c_iflag=0;
    ios.c_oflag=0;
    ios.c_lflag=0;     /* non-canonical */
    ios.c_cc[VMIN ]=0; /* non-block-mode */
    ios.c_cc[VTIME]=0;
    cfsetospeed(&ios,BAUDRATE);
    cfsetispeed(&ios,BAUDRATE);
    ios.c_cflag|=bsize==7?CS7:CS8;
    ios.c_cflag|=parity=='O'?(PARENB|PARODD):(parity=='E'?PARENB:0);
    ios.c_cflag|=stopb==2?CSTOPB:0;
    ios.c_cflag|=!strcmp(fctr,"rts")?CRTSCTS:0;
    tcsetattr(fd,TCSANOW,&ios);
    tcflush(fd,TCIOFLUSH);
    return fd;
}

int main(int argc, const char * argv[])
{
    tracelevel(3);
    
    int fd = open_serial("/dev/tty.SLAB_USBtoUART");
    
    char buf[255];
    
    prcopt_t opt = prcopt_default;
    opt.mode = PMODE_SINGLE;
    opt.dynamics = 1;
    opt.nf = 2;
    
    raw_t raw;
    init_raw(&raw);

    while (1) {
        ssize_t s = read(fd, buf, 255);
        if (s <= 0) {
            usleep(1000);
            continue;
        }
        for (int i = 0; i < s; i++) {
            int ret = input_raw(&raw, STRFMT_UBX, buf[i]);
            if (ret > 0) {
                char *pos_mode = NULL;
                double *x = NULL;
                
                sol_t sol;
                
                {
                    // single point positioning
                    char buf[0x100];
                    if(pntpos(raw.obs.data, raw.obs.n, &raw.nav, &opt, &sol, NULL, NULL, buf)){
                        pos_mode = "pntpos";
                        x = sol.rr;
                    }
                }
                
                if(pos_mode){
                    double pos[3];
                    ecef2pos(x, pos);
                    trace(3, "%s (llh): %f, %f, %f\n", pos_mode, pos[0] * R2D, pos[1] * R2D, pos[2]);
                }
                
            }
        }
    }
    
    return 0;
}

