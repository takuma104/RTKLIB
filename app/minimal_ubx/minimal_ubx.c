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
    double ep[] = {
        0, 0, 0,
        0, 0, 0};
    return epoch2time(ep); // dummy
}
unsigned int tickget(){
    return 0; // dummy
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
    tracelevel(1);
    
    int fd = open_serial("/dev/tty.SLAB_USBtoUART");
    
    char buf[255];
/*
    int size = sizeof(raw_t);
    int a = sizeof(obs_t);
    int b = sizeof(nav_t);
    int c = sizeof(sta_t);
*/
    prcopt_t opt = prcopt_default;
    opt.mode = PMODE_SINGLE;
    opt.dynamics = 1;
    opt.nf = 2;
    
    raw_t raw;
    memset(&raw, 0, sizeof(raw_t));
    init_raw(&raw);

    while (1) {
        ssize_t s = read(fd, buf, 255);
        if (s <= 0) {
            usleep(1000);
            continue;
        }
        for (int i = 0; i < s; i++) {
            int ret = input_raw(&raw, STRFMT_UBX, buf[i]);
            if (ret == 1) { // only observe message
                char *pos_mode = NULL;
                double *x = NULL;
                
                sol_t sol;
                memset(&sol, 0, sizeof(sol_t));
                
				ssat_t ssat[MAXSAT];
                memset(ssat, 0, sizeof(ssat_t)*MAXSAT);

                {
                    // single point positioning
                    char buf[0x100];
                    memset(buf, 0, sizeof(buf));
                    if(pntpos(raw.obs.data, raw.obs.n, &raw.nav, &opt, &sol, NULL, ssat, buf)){
                        pos_mode = "pntpos";
                        x = sol.rr;
                    }
                }
                
                if(pos_mode){
                    double pos[3];
                    ecef2pos(x, pos);

					double dop[4]={0};
                    double azel[MAXSAT*2];
                    
					int i, n;
                    for (i=n=0;i<MAXSAT;i++) {
                        if (!ssat[i].vs) continue;
                        azel[  n*2]=ssat[i].azel[0];
                        azel[1+n*2]=ssat[i].azel[1];
                        n++;
                    }
                    dops(n,azel,0.0,dop);
                    
                    printf("UTC:%s SAT:%d/%d POS:%f, %f, %.1f GDOP:%.1f PDOP:%.1f HDOP:%.1f VDOP:%.1f\n", time_str(raw.obs.data[0].time,3), sol.ns, raw.obs.n, pos[0] * R2D, pos[1] * R2D, pos[2], dop[0], dop[1], dop[2], dop[3]);
                } else {
					printf("UTC:%s SAT:0/%d\n", time_str(raw.obs.data[0].time,3), raw.obs.n);
                }
                
            }
        }
    }
    
    return 0;
}

