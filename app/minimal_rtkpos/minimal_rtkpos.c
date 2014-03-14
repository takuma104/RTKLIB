#include "rtklib.h"

#define USE_RINEX_INPUT

#ifdef USE_RINEX_INPUT
#define rcsid rcsid_rinex
#define obscodes obscodes_rinex
#include "../../src/rinex.c"
#undef rcsid
#undef obscodes
#endif

#ifdef WITHOUT_FILE
void rtkprintstat(int level, const char *str, ...){
  if(level > 1){return;}
  va_list arg;
  va_start(arg, str);
  vfprintf(stderr, str, arg);
  va_end(arg);
}

int trace_vprintf(const char *format, va_list ap){
  int res;
  res = vfprintf(stderr, format, ap);
  fflush(stderr);
  return res;
}

#ifdef USE_RINEX_INPUT
int uncompress(const char *file, char *uncfile){
  const char *p;
  trace(3, "uncompress: file=%s\n", file);
  if(!(p = strrchr(file, '.'))){return 0;}
  if(!strcmp(p, ".z")
      || !strcmp(p, ".Z")
      || !strcmp(p,".gz")
      || !strcmp(p,".GZ")
      || !strcmp(p,".zip")
      || !strcmp(p,".ZIP")
      || !strcmp(p,".tar")
      || ((strlen(p) > 3) && ((p[3] == 'd') || (p[3] == 'D')))){
    return -1;
  }
  return 0;
}
#endif
#endif

int main(){

  tracelevel(3); //0xF);

  nav_t nav;

  ///< @see init_raw() of rcvraw.c
  memset(&nav, 0, sizeof(nav_t));

  if(nav.eph = (eph_t *)calloc(MAXSAT * 2, sizeof(eph_t))){
    nav.nmax = MAXSAT * 2;
  }
  if(nav.seph = (seph_t *)calloc(NSATSBS * 2, sizeof(seph_t))){
    nav.ns = nav.nsmax = NSATSBS * 2;
  }

  {
    const double lam_gnss[] = {
        CLIGHT / FREQ1, CLIGHT / FREQ2, CLIGHT / FREQ5};
    const double lam_glo[] = {
        CLIGHT / FREQ1_GLO, CLIGHT / FREQ2_GLO};
    int i;
    for(i = 0; i < sizeof(nav.lam) / sizeof(nav.lam[0]); ++i){
      switch(satsys(i + 1, NULL)){
        case SYS_NONE: continue;
        case SYS_GLO: memcpy(nav.lam[i], lam_glo, sizeof(nav.lam[i])); break;
        default: memcpy(nav.lam[i], lam_gnss, sizeof(nav.lam[i])); break;
      }
    }
  }

  rtk_t rtk;
  prcopt_t opt = prcopt_default;
  opt.mode = PMODE_PPP_KINEMA; // PMODE_SINGLE;
  opt.dynamics = 1;

  rtkinit(&rtk, &opt);

  obsd_t obs_data[MAXOBS * 2];
  memset(obs_data, 0, sizeof(obs_data));
  obs_t obs;
  obs.n = 0;
  obs.nmax = sizeof(obs_data) / sizeof(obs_data[0]);
  obs.data = obs_data;

#ifdef USE_RINEX_INPUT
  char *t_str[] = {
      "05  4  2  2  0  0.0", "05  4  2  2  0 29.0",
      "05  4  2  0  0  0.0000000", "05  4  2  0  0 29.0000000",};
  gtime_t t[sizeof(t_str) / sizeof(t_str[0])];
  {
    int i;
    for(i = 0; i < sizeof(t) / sizeof(t[0]); i++){
      str2time(t_str[i], 0, strlen(t_str[i]), &t[i]);
    }
  }
  if(readrnxt("../../../../test/data/rinex/07590920.05n", 1, t[0], t[1], 0.0, "", NULL, &nav, NULL) == 0){
    exit(-1);
  }
  uniqnav(&nav);
  if(readrnxt("../../../../test/data/rinex/07590920.05o", 1, t[2], t[3], 0.0, "", &obs, NULL, NULL) == 0){
    exit(-1);
  }
  sortobs(&obs);
#endif

  char *pos_mode = NULL;
  double *x = NULL;

  if(0){
    // single point positioning
    char buf[0x100];
    if(pntpos(obs.data, obs.n, &nav, &opt, &(rtk.sol), NULL, NULL, buf)){
      pos_mode = "pntpos";
      x = rtk.sol.rr;
    }
  }else{
    // precise point positioning
    if(rtkpos(&rtk, obs.data, obs.n, &nav)){
      pos_mode = "rtkpos";
      x = rtk.x;
    }
  }

  if(pos_mode){
    double pos[3];
    ecef2pos(x, pos);
    trace(3, "%s (llh): %f, %f, %f\n", pos_mode, pos[0] * R2D, pos[1] * R2D, pos[2]);
  }

  rtkfree(&rtk);

  free(nav.eph);
  free(nav.seph);

  return 0;
}
