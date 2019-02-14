#include <errno.h>
#include <pthread.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/epoll.h>
#include <math.h>
#include <time.h>
#include <semaphore.h>
#include <signal.h>
#include <unistd.h>
#include <stdio.h>
#include <time.h>
#include <sys/inotify.h>
#include <poll.h>

#define  LOG_TAG  "hd8020_bdgps"

#include <cutils/log.h>
#include <cutils/sockets.h>
#include <cutils/properties.h>
#include <hardware/gps.h>

#define HD8020_GPS
#define  HD8020_GPSXtra
#undef  HD8020_GPSNi

#undef  GPS_DEBUG
#define GPS_DEBUG
#undef     GPS_DEBUG_TOKEN    /* print out NMEA tokens */

#define  DFR(...)   ALOGD(__VA_ARGS__)

#ifdef GPS_DEBUG
#  define  D(...)   ALOGD(__VA_ARGS__)
#else
#  define  D(...)   ((void)0)
#endif

#define GPS_STATUS_CB(_cb, _s)    \
  if ((_cb).status_cb) {          \
    GpsStatus gps_status;         \
    gps_status.status = (_s);     \
    (_cb).status_cb(&gps_status); \
    DFR("gps status callback: 0x%x", _s); \
  }

/* Nmea Parser stuff */
#define  NMEA_MAX_SIZE  83
static int power_flag = 0;
enum {
  STATE_QUIT  = 0,
  STATE_INIT  = 1,
  STATE_START = 2
};

enum {
  RNSS_GPS = 0,
  RNSS_BDS = 1
};

typedef struct
{
    int valid;
    double systime;
    GpsUtcTime timestamp;
} UmTimemap_t;

typedef struct {
    int     pos;
    int     overflow;
    int     utc_year;
    int     utc_mon;
    int     utc_day;
    int     utc_diff;
    GpsLocation  fix;
    GpsSvStatus  sv_status[2];
    GpsSvStatus  sv_status_tmp[2];
    int     sv_status_changed;
    char    in[ NMEA_MAX_SIZE+1 ];
    int     gsa_fixed;
    UmTimemap_t timemap;
    int     nmea_mode;
} NmeaReader;

typedef struct {
    int                     init;
    int                     fd;
    GpsCallbacks            callbacks;
    pthread_t               thread;
    pthread_t            nmea_thread;
    pthread_t               tmr_thread;
    int                     control[2];
    int                     fix_freq;
    sem_t                   fix_sem;
    int                     first_fix;
    NmeaReader              reader;
#ifdef HD8020_GPSXtra
    int                        xtra_init;
    GpsXtraCallbacks        xtra_callbacks;
#endif
#ifdef HD8020_GPSNi
    int                     ni_init;
    GpsNiCallbacks          ni_callbacks;
    GpsNiNotification       ni_notification;
#endif
    int     rnss_mode;
} GpsState;

GpsCallbacks* g_gpscallback = 0;
int cold_start(int fd_global);
int hot_start(int fd_global);
int warm_start(int fd_global);

void gps_state_lock_fix(GpsState *state) {
    int ret;
    do {
        ret=sem_wait(&state->fix_sem);
    } while (ret < 0 && errno == EINTR);
    if (ret < 0) {
        D("Error in GPS state lock:%s\n", strerror(errno));
    }
}

void gps_state_unlock_fix(GpsState *state) {
    if (sem_post(&state->fix_sem) == -1)
    {
        if(errno == EAGAIN)
            if(sem_post(&state->fix_sem)== -1)
                D("Error in GPS state unlock:%s\n", strerror(errno));
    }
}

int     gps_opentty(GpsState *state);
void     gps_closetty(GpsState *state);
void     gps_wakeup(GpsState *state);
void     gps_sleep(GpsState *state);
int     gps_checkstate(GpsState *state);
void    power_on(void);
void    power_off(void);

static GpsState  _gps_state[1];
static GpsState *gps_state = _gps_state;
static int sleep_lock = 0;
static int lastcmd = 0;
static int bGetFormalNMEA = 1;
static int started    = 0;
static int continue_thread = 1;
static int bOrionShutdown = 0;

static void gps_nmea_thread( void*  arg );
static void gps_timer_thread( void*  arg );
static void set_rnss_mode(int fd, char mode);
static void check_rnss_mode( NmeaReader* r);
static void init_serial(int fd);

#ifdef HD8020_GPSNi
static void um_send_ni_notification(NmeaReader* r)
{
    //D("um_send_ni_notification is called");
    if (gps_state->ni_init && gps_state->ni_callbacks.notify_cb)
    {
        memset(&gps_state->ni_notification, 0, sizeof(sizeof(gps_state->ni_notification)));
       gps_state->ni_notification.size                     = sizeof(GpsNiNotification);
        gps_state->ni_notification.notification_id          = 0xABBA;
        gps_state->ni_notification.ni_type                  = GPS_NI_TYPE_UMTS_CTRL_PLANE; // AM TBD: is it correct?
        gps_state->ni_notification.notify_flags             = 0; // no notification & no verification is needed
        gps_state->ni_notification.timeout                  = 0; // no timeout limit
        gps_state->ni_notification.default_response         = GPS_NI_RESPONSE_NORESP;
        gps_state->ni_notification.requestor_id[0]          = 0;
        gps_state->ni_notification.text[0]                  = 0;
        gps_state->ni_notification.requestor_id_encoding    = GPS_ENC_NONE;
        gps_state->ni_notification.text_encoding            = GPS_ENC_NONE;
        if (r->timemap.valid && r->timemap.timestamp == r->fix.timestamp)
        {
            sprintf(gps_state->ni_notification.extras, "pps=%10.10lf", r->timemap.systime);
            r->timemap.valid = 0;
        }
        else
        {
            gps_state->ni_notification.extras[0] = 0;
        }

        gps_state->ni_callbacks.notify_cb(&gps_state->ni_notification);
    }
}
#else // #ifdef HD8020_GPSNi
#   define um_send_ni_notification(r)
#endif // #ifdef HD8020_GPSNi

typedef struct {
    const char*  p;
    const char*  end;
} Token;

#define  MAX_NMEA_TOKENS  32

typedef struct {
    int     count;
    Token   tokens[ MAX_NMEA_TOKENS ];
} NmeaTokenizer;

static int
nmea_tokenizer_init( NmeaTokenizer*  t, const char*  p, const char*  end )
{
    int    count = 0;
    char*  q;

    // the initial '$' is optional
    if (p < end && p[0] == '$')
        p += 1;

    // remove trailing newline
    if (end > p && end[-1] == '\n') {
        end -= 1;
        if (end > p && end[-1] == '\r')
            end -= 1;
    }

    // get rid of checksum at the end of the sentecne
    if (end >= p+3 && end[-3] == '*') {
        end -= 3;
    }

    while (p < end) {
        const char*  q = p;

        q = memchr(p, ',', end-p);
        if (q == NULL)
            q = end;

        if (count < MAX_NMEA_TOKENS) {
            t->tokens[count].p   = p;
            t->tokens[count].end = q;
            count += 1;
        }

        if (q < end)
            q += 1;

        p = q;
    }

    t->count = count;
    return count;
}

static Token
nmea_tokenizer_get( NmeaTokenizer*  t, int  index )
{
    Token  tok;
    static const char*  dummy = "";

    if (index < 0 || index >= t->count) {
        tok.p = tok.end = dummy;
    } else
        tok = t->tokens[index];

    return tok;
}


static int
str2int( const char*  p, const char*  end )
{
    int   result = 0;
    int   len    = end - p;

    if (len == 0) {
      return -1;
    }

    for ( ; len > 0; len--, p++ )
    {
        int  c;

        if (p >= end)
            goto Fail;

        c = *p - '0';
        if ((unsigned)c >= 10)
            goto Fail;

        result = result*10 + c;
    }
    return  result;

Fail:
    return -1;
}

static double
str2float( const char*  p, const char*  end )
{
    int   result = 0;
    int   len    = end - p + 1;
    char  temp[32];

    if (len == 0) {
      return -1.0;
    }

    if (len >= (int)sizeof(temp))
        return 0.;

    memcpy( temp, p, len );
    temp[len] = 0;
    return strtod( temp, NULL );
}

static void nmea_reader_update_utc_diff( NmeaReader*  r )
{
    time_t         now = time(NULL);
    struct tm      tm_local;
    struct tm      tm_utc;
    long           time_local, time_utc;

    gmtime_r( &now, &tm_utc );
    localtime_r( &now, &tm_local );

    time_local = tm_local.tm_sec +
                 60*(tm_local.tm_min +
                 60*(tm_local.tm_hour +
                 24*(tm_local.tm_yday +
                 365*tm_local.tm_year)));

    time_utc = tm_utc.tm_sec +
               60*(tm_utc.tm_min +
               60*(tm_utc.tm_hour +
               24*(tm_utc.tm_yday +
               365*tm_utc.tm_year)));

    r->utc_diff = time_utc - time_local;
}


static void nmea_reader_init( NmeaReader*  r )
{
    memset( r, 0, sizeof(*r) );

    r->pos      = 0;
    r->overflow = 0;
    r->utc_year = -1;
    r->utc_mon  = -1;
    r->utc_day  = -1;

    nmea_reader_update_utc_diff( r );
}

static int nmea_reader_get_timestamp(NmeaReader*  r, Token  tok, time_t *timestamp)
{
    int        hour, minute;
    double     seconds;
    struct tm  tm;
    time_t     ttime;

    if (tok.p + 6 > tok.end)
        return -1;

    if (r->utc_year < 0) {
        return -1;
    }

    hour    = str2int(tok.p,   tok.p+2);
    minute  = str2int(tok.p+2, tok.p+4);
    seconds = str2float(tok.p+4, tok.end);

    tm.tm_hour = hour;
    tm.tm_min  = minute;
    tm.tm_sec   = (int) seconds;
    tm.tm_year = r->utc_year - 1900;
    tm.tm_mon  = r->utc_mon - 1;
    tm.tm_mday = r->utc_day;
    tm.tm_isdst = -1;

    // D("h: %d, m: %d, s: %d", tm.tm_hour, tm.tm_min, tm.tm_sec);
    // D("Y: %d, M: %d, D: %d", tm.tm_year, tm.tm_mon, tm.tm_mday);

    nmea_reader_update_utc_diff(r);

    ttime = mktime( &tm );
    *timestamp = ttime - r->utc_diff;

    return 0;
}

static int
nmea_reader_update_time( NmeaReader*  r, Token  tok )
{
    time_t timestamp = 0;
    int ret = nmea_reader_get_timestamp( r, tok, &timestamp);
    if (0 == ret)
        r->fix.timestamp = (long long)timestamp * 1000;
    return ret;
}

static int
nmea_reader_update_cdate( NmeaReader*  r, Token  tok_d, Token tok_m, Token tok_y )
{

    if ( (tok_d.p + 2 > tok_d.end) ||
         (tok_m.p + 2 > tok_m.end) ||
         (tok_y.p + 4 > tok_y.end) )
        return -1;

    r->utc_day = str2int(tok_d.p,   tok_d.p+2);
    r->utc_mon = str2int(tok_m.p, tok_m.p+2);
    r->utc_year = str2int(tok_y.p, tok_y.end+4);

    return 0;
}

static int
nmea_reader_update_date( NmeaReader*  r, Token  date, Token  mtime )
{
    Token  tok = date;
    int    day, mon, year;

    if (tok.p + 6 != tok.end) {

        /* no date info, will use host time in _update_time function
         */
    }
    /* normal case */
    day  = str2int(tok.p, tok.p+2);
    mon  = str2int(tok.p+2, tok.p+4);
    year = str2int(tok.p+4, tok.p+6) + 2000;

    if ((day|mon|year) < 0) {
        return -1;
    }

    r->utc_year  = year;
    r->utc_mon   = mon;
    r->utc_day   = day;

    return nmea_reader_update_time( r, mtime );
}


static double
convert_from_hhmm( Token  tok )
{
    double  val     = str2float(tok.p, tok.end);
    int     degrees = (int)(floor(val) / 100);
    double  minutes = val - degrees*100.;
    double  dcoord  = degrees + minutes / 60.0;
    return dcoord;
}


static int
nmea_reader_update_latlong( NmeaReader*  r,
                            Token        latitude,
                            char         latitudeHemi,
                            Token        longitude,
                            char         longitudeHemi )
{
    double   lat, lon;
    Token    tok;

    tok = latitude;
    if (tok.p + 6 > tok.end) {
        return -1;
    }
    lat = convert_from_hhmm(tok);
    if (latitudeHemi == 'S')
        lat = -lat;

    tok = longitude;
    if (tok.p + 6 > tok.end) {
        return -1;
    }
    lon = convert_from_hhmm(tok);
    if (longitudeHemi == 'W')
        lon = -lon;

    r->fix.flags    |= GPS_LOCATION_HAS_LAT_LONG;
    r->fix.latitude  = lat;
    r->fix.longitude = lon;
    return 0;
}


static int
nmea_reader_update_altitude( NmeaReader*  r,
                             Token        altitude,
                             Token        units )
{
    double  alt;
    Token   tok = altitude;

    if (tok.p >= tok.end)
        return -1;

    r->fix.flags   |= GPS_LOCATION_HAS_ALTITUDE;
    r->fix.altitude = str2float(tok.p, tok.end);
    return 0;
}

static int
nmea_reader_update_accuracy( NmeaReader*  r,
                             Token        accuracy )
{
    double  acc;
    Token   tok = accuracy;

    if (tok.p >= tok.end)
        return -1;

    //tok is cep*cc, we only want cep
    r->fix.accuracy = str2float(tok.p, tok.end);

    if (r->fix.accuracy == 99.99){
      return 0;
    }

    r->fix.flags   |= GPS_LOCATION_HAS_ACCURACY;
    return 0;
}

static int
nmea_reader_update_bearing( NmeaReader*  r,
                            Token        bearing )
{
    double  alt;
    Token   tok = bearing;

    if (tok.p >= tok.end)
        return -1;

    r->fix.flags   |= GPS_LOCATION_HAS_BEARING;
    r->fix.bearing  = str2float(tok.p, tok.end);
    return 0;
}


static int
nmea_reader_update_speed( NmeaReader*  r,
                          Token        speed )
{
    double  alt;
    Token   tok = speed;

    if (tok.p >= tok.end)
        return -1;

    r->fix.flags   |= GPS_LOCATION_HAS_SPEED;
    r->fix.speed    = str2float(tok.p, tok.end);
    r->fix.speed   *= 0.514444;    // fix for Speed Unit form Knots to Meters per Second
    return 0;
}

static int
nmea_reader_update_timemap( NmeaReader* r,
                            Token       systime_tok,
                            Token       timestamp_tok)
{
    int ret;
    time_t timestamp;

    if ( systime_tok.p >= systime_tok.end ||
         timestamp_tok.p >= timestamp_tok.end)
    {
        r->timemap.valid = 0;
        return -1;
    }

    ret = nmea_reader_get_timestamp(r, timestamp_tok, &timestamp);
    if (ret)
    {
        r->timemap.valid = 0;
        return ret;
    }

    r->timemap.valid = 1;
    r->timemap.systime = str2float(systime_tok.p, systime_tok.end);
    r->timemap.timestamp = (GpsUtcTime)((long long)timestamp * 1000);
    return 0;
}

/* commands sent to the gps thread */
enum {
    CMD_QUIT  = 0,
    CMD_START = 1,
    CMD_STOP  = 2,
    CMD_MODE_GPS,
    CMD_MODE_BD,
    CMD_MODE_GN
};


static void
nmea_reader_parse( NmeaReader*  r )
{
   /* we received a complete sentence, now parse it to generate
    * a new GPS fix...
    */
    NmeaTokenizer  tzer[1];
    Token          tok;
    int rtype;

    if (r->pos < 9) {
         return;
    }

    if (gps_state->callbacks.nmea_cb) {
        struct timeval tv;
        unsigned long long mytimems;
        gettimeofday(&tv,NULL);
        mytimems = tv.tv_sec * 1000 + tv.tv_usec / 1000;
        gps_state->callbacks.nmea_cb(mytimems, r->in, r->pos);
        //D("hd8020_reader_parse. %.*s ", r->pos, r->in );
    }

    nmea_tokenizer_init(tzer, r->in, r->in + r->pos);
#ifdef GPS_DEBUG_TOKEN
    {
        int  n;
        D("Found %d tokens", tzer->count);
        for (n = 0; n < tzer->count; n++) {
            Token  tok = nmea_tokenizer_get(tzer,n);
            D("%2d: '%.*s'", n, tok.end-tok.p, tok.p);
        }
    }
#endif

    tok = nmea_tokenizer_get(tzer, 0);

    if (tok.p + 5 > tok.end) {
        /* short sentences */
        return;
    }

    if ( !memcmp(tok.p, "GPG", 3) || !memcmp(tok.p, "GNG", 3) || !memcmp(tok.p, "BDG", 3) ) //GPGSA,GPGGA,GPGSV
        bGetFormalNMEA = 1;

    // check for RNSS type
    if(!memcmp(tok.p, "BD", 2))
        rtype = RNSS_BDS;
    else
        rtype = RNSS_GPS;
    // ignore first two characters.
    tok.p += 2;

    if ( !memcmp(tok.p, "GGA", 3) ) {
        // GPS fix
        Token  tok_fixstaus      = nmea_tokenizer_get(tzer,6);

        if (tok_fixstaus.p[0] > '0') {

          Token  tok_time          = nmea_tokenizer_get(tzer,1);
          Token  tok_latitude      = nmea_tokenizer_get(tzer,2);
          Token  tok_latitudeHemi  = nmea_tokenizer_get(tzer,3);
          Token  tok_longitude     = nmea_tokenizer_get(tzer,4);
          Token  tok_longitudeHemi = nmea_tokenizer_get(tzer,5);
          Token  tok_altitude      = nmea_tokenizer_get(tzer,9);
          Token  tok_altitudeUnits = nmea_tokenizer_get(tzer,10);

          nmea_reader_update_time(r, tok_time);
          nmea_reader_update_latlong(r, tok_latitude,
                                        tok_latitudeHemi.p[0],
                                        tok_longitude,
                                        tok_longitudeHemi.p[0]);
          nmea_reader_update_altitude(r, tok_altitude, tok_altitudeUnits);
        }

    } else if ( !memcmp(tok.p, "GLL", 3) ) {

        Token  tok_fixstaus      = nmea_tokenizer_get(tzer,6);

        if (tok_fixstaus.p[0] == 'A') {

          Token  tok_latitude      = nmea_tokenizer_get(tzer,1);
          Token  tok_latitudeHemi  = nmea_tokenizer_get(tzer,2);
          Token  tok_longitude     = nmea_tokenizer_get(tzer,3);
          Token  tok_longitudeHemi = nmea_tokenizer_get(tzer,4);
          Token  tok_time          = nmea_tokenizer_get(tzer,5);

          nmea_reader_update_time(r, tok_time);
          nmea_reader_update_latlong(r, tok_latitude,
                                        tok_latitudeHemi.p[0],
                                        tok_longitude,
                                        tok_longitudeHemi.p[0]);
        }
    } else if ( !memcmp(tok.p, "GSA", 3) ) {

        Token  tok_fixStatus   = nmea_tokenizer_get(tzer, 2);
        int i;

        if (tok_fixStatus.p[0] != '\0' && tok_fixStatus.p[0] != '1') {

            Token tok_accuracy      = nmea_tokenizer_get(tzer, 15);
            nmea_reader_update_accuracy(r, tok_accuracy);

            /*
               for (i = 0; i < 8; i++) {
               r->sv_status_tmp[rtype].bd_used_in_fix_mask[i] = 0;
               }
               */

            for (i = 3; i <= 14; ++i){
                Token  tok_prn  = nmea_tokenizer_get(tzer, i);
                int prn = str2int(tok_prn.p, tok_prn.end);
                if(rtype == 1 && prn > 0){
                    prn += 200;
                }
                /* available for PRN 1-255 */
                if ((prn > 0) && (prn < 256)){
                    int offset = (prn - 1) / 32;
                    int bit = (prn - 1) % 32;
                    /* ALOGD("type:%d prn:%d, offset:%d, bit:%d\n",rtype, prn, offset, bit); */
                    r->sv_status_tmp[rtype].bd_used_in_fix_mask[offset] |= (1ul << bit);
                    /* mark this parameter to identify the GSA is in fixed state */
                    r->gsa_fixed = 1;
                }
          }

/*
          for (i = 0; i < 8; i++) {
              r->sv_status[rtype].bd_used_in_fix_mask[i] =  r->sv_status_tmp[rtype].bd_used_in_fix_mask[i];
          }
              //r->sv_status_changed = 1;
*/

        }else {
        if (r->gsa_fixed == 1) {
            for (i = 0; i < 8; i++) {
                r->sv_status_tmp[0].bd_used_in_fix_mask[i] = 0;
                r->sv_status_tmp[1].bd_used_in_fix_mask[i] = 0;
                r->sv_status[0].bd_used_in_fix_mask[i] = 0;
                r->sv_status[1].bd_used_in_fix_mask[i] = 0;
            }

            r->gsa_fixed = 0;
            r->sv_status_changed = 1;
        }
        }
    } else if ( !memcmp(tok.p, "GSV", 3) ) {

        //D("The BDGSV data: %s", tok.p);
        //ALOGI("The BDGSV data GSV");
        Token  tok_noSatellites  = nmea_tokenizer_get(tzer, 3);
        int    noSatellites = str2int(tok_noSatellites.p, tok_noSatellites.end);

          Token  tok_noSentences   = nmea_tokenizer_get(tzer, 1);
          Token  tok_sentence      = nmea_tokenizer_get(tzer, 2);

        if (noSatellites > 0) {

          int sentence = str2int(tok_sentence.p, tok_sentence.end);
          int totalSentences = str2int(tok_noSentences.p, tok_noSentences.end);
          int curr;
          int i;
          if (sentence == 1) {
              //r->sv_status_changed = 0;
              r->sv_status_tmp[rtype].num_svs = 0;
          }

          curr = r->sv_status_tmp[rtype].num_svs;

          i = 0;

          while (i < 4 && r->sv_status_tmp[rtype].num_svs < noSatellites) {


                 Token  tok_prn = nmea_tokenizer_get(tzer, i * 4 + 4);
                 Token  tok_elevation = nmea_tokenizer_get(tzer, i * 4 + 5);
                 Token  tok_azimuth = nmea_tokenizer_get(tzer, i * 4 + 6);
                 Token  tok_snr = nmea_tokenizer_get(tzer, i * 4 + 7);
#if 0
                 r->sv_status_tmp[rtype].bd_sv_list[curr].prn = str2int(tok_prn.p, tok_prn.end);
#else
                 if(rtype == 1)
                     r->sv_status_tmp[rtype].bd_sv_list[curr].prn = str2int(tok_prn.p, tok_prn.end) + 200;
                 else
                     r->sv_status_tmp[rtype].bd_sv_list[curr].prn = str2int(tok_prn.p, tok_prn.end);
#endif
                 r->sv_status_tmp[rtype].bd_sv_list[curr].elevation = str2float(tok_elevation.p, tok_elevation.end);
                 r->sv_status_tmp[rtype].bd_sv_list[curr].azimuth = str2float(tok_azimuth.p, tok_azimuth.end);
                 r->sv_status_tmp[rtype].bd_sv_list[curr].snr = str2float(tok_snr.p, tok_snr.end);

                 r->sv_status_tmp[rtype].num_svs += 1;

                 curr += 1;

                 i += 1;
          }
          if (sentence == totalSentences) {
              if(r->nmea_mode != CMD_MODE_GN){
                  memcpy(r->sv_status, r->sv_status_tmp, sizeof(GpsSvStatus)*2);
                  r->sv_status_changed = 1;
                 D("!CMD_MODE_GN : %d %d \n", r->sv_status[0].num_svs, r->sv_status[1].num_svs);
              }else if((r->nmea_mode == CMD_MODE_GN) && (rtype == RNSS_GPS)) {
                 memcpy(r->sv_status, r->sv_status_tmp, sizeof(GpsSvStatus)*1);
                 r->sv_status_changed = 1;
                 D("CMD_MODE_GN && RNSS_GPS: %d %d \n", r->sv_status[0].num_svs, r->sv_status[1].num_svs);
              }else if((r->nmea_mode == CMD_MODE_GN) && (rtype == RNSS_BDS)) {
                 memcpy(r->sv_status + 1, r->sv_status_tmp + 1, sizeof(GpsSvStatus)*1);
                 r->sv_status_changed = 1;
                 D("CMD_MODE_GN && RNSS_BDS: %d %d \n", r->sv_status[0].num_svs, r->sv_status[1].num_svs);
              }
         }
      }
    } else if ( !memcmp(tok.p, "RMC", 3) ) { //NMEA first line
        if(!memcmp(tok.p - 2, "GNR", 3)) {
            r->nmea_mode = CMD_MODE_GN;
        } else if(!memcmp(tok.p - 2, "BDR", 3)) {
            r->nmea_mode = CMD_MODE_BD;
        } else {
            r->nmea_mode = CMD_MODE_GPS;
        }

        memset(gps_state->reader.sv_status_tmp, 0, sizeof(gps_state->reader.sv_status_tmp));

        Token  tok_fixStatus     = nmea_tokenizer_get(tzer,2);

        if (tok_fixStatus.p[0] == 'A')
        {
          Token  tok_time          = nmea_tokenizer_get(tzer,1);
          Token  tok_latitude      = nmea_tokenizer_get(tzer,3);
          Token  tok_latitudeHemi  = nmea_tokenizer_get(tzer,4);
          Token  tok_longitude     = nmea_tokenizer_get(tzer,5);
          Token  tok_longitudeHemi = nmea_tokenizer_get(tzer,6);
          Token  tok_speed         = nmea_tokenizer_get(tzer,7);
          Token  tok_bearing       = nmea_tokenizer_get(tzer,8);
          Token  tok_date          = nmea_tokenizer_get(tzer,9);

            nmea_reader_update_date( r, tok_date, tok_time );

            nmea_reader_update_latlong( r, tok_latitude,
                                           tok_latitudeHemi.p[0],
                                           tok_longitude,
                                           tok_longitudeHemi.p[0] );

            nmea_reader_update_bearing( r, tok_bearing );
            nmea_reader_update_speed  ( r, tok_speed );
        }
        check_rnss_mode(r);

    } else if ( !memcmp(tok.p, "VTG", 3) ) {

        Token  tok_fixStatus     = nmea_tokenizer_get(tzer,9);

        if (tok_fixStatus.p[0] != '\0' && tok_fixStatus.p[0] != 'N')
        {
            Token  tok_bearing       = nmea_tokenizer_get(tzer,1);
            Token  tok_speed         = nmea_tokenizer_get(tzer,5);

            nmea_reader_update_bearing( r, tok_bearing );
            nmea_reader_update_speed  ( r, tok_speed );
        }

    } else if ( !memcmp(tok.p, "ZDA", 3) ) {

        Token  tok_time;
        Token  tok_year  = nmea_tokenizer_get(tzer,4);

        if (tok_year.p[0] != '\0') {

          Token  tok_day   = nmea_tokenizer_get(tzer,2);
          Token  tok_mon   = nmea_tokenizer_get(tzer,3);

          nmea_reader_update_cdate( r, tok_day, tok_mon, tok_year );

        }

        tok_time  = nmea_tokenizer_get(tzer,1);

        if (tok_time.p[0] != '\0') {

          nmea_reader_update_time(r, tok_time);

        }


    } else {
        tok.p -= 2;
    }

    if (!gps_state->first_fix &&
        gps_state->init == STATE_INIT &&
        r->fix.flags & GPS_LOCATION_HAS_LAT_LONG) {

        um_send_ni_notification(r);

        if (gps_state->callbacks.location_cb) {
            gps_state->callbacks.location_cb( &r->fix );
            r->fix.flags = 0;
        }

        gps_state->first_fix = 1;
    }
}

/* parse the commands for HD8020 chipsets */
static void
hd8020_reader_parse( char* buf, int length)
{
    int cnt;

    D("hd8020_reader_parse. %.*s (%d)", length, buf, length );
    if (length < 9) {
        D("HD8020 command too short. discarded.");
        return;
    }

    /* for NMEAListener callback */
    if (gps_state->callbacks.nmea_cb) {
        D("NMEAListener callback for HD8020... ");
        struct timeval tv;
        unsigned long long mytimems;

        gettimeofday(&tv,NULL);
        mytimems = tv.tv_sec * 1000 + tv.tv_usec / 1000;

        gps_state->callbacks.nmea_cb(mytimems, buf, length);

        D("NMEAListener callback for HD8020 sentences ended... ");
    }
}

static void
nmea_reader_addc( NmeaReader*  r, int  c )
{
    int cnt;

    if (r->overflow) {
        r->overflow = (c != '\n');
        return;
    }

    if (r->pos >= (int) sizeof(r->in)-1 ) {
        r->overflow = 1;
        r->pos      = 0;
        return;
    }

    r->in[r->pos] = (char)c;
    r->pos       += 1;

    if (c == '\n') {
        gps_state_lock_fix(gps_state);
        nmea_reader_parse( r );
        gps_state_unlock_fix(gps_state);
        r->pos = 0;
    }
}

static void gps_state_update_fix_freq(GpsState *s, int fix_freq)
{

  s->fix_freq = fix_freq;
  return;

}


static void
gps_state_done( GpsState*  s )
{
    // tell the thread to quit, and wait for it
    char   cmd = CMD_QUIT;
    void*  dummy;
    int ret;

    do { ret=write( s->control[0], &cmd, 1 ); }
    while (ret < 0 && errno == EINTR);

    DFR("gps waiting for command thread to stop");
    gps_sleep(s);

    pthread_join(s->thread, &dummy);

    /* Timer thread depends on this state check */
    s->init = STATE_QUIT;
    s->fix_freq = -1;

    // close the control socket pair
    close( s->control[0] ); s->control[0] = -1;
    close( s->control[1] ); s->control[1] = -1;

    sem_destroy(&s->fix_sem);
    g_gpscallback = 0;

    memset(s, 0, sizeof(*s));

}

void gps_wakeup(GpsState *state)
{
	if( sleep_lock == 0) // it reset by hd8020_gps_start
	{
		gps_state_lock_fix(state);

		gps_opentty(state);

		sleep_lock = time((time_t*)NULL);
		bOrionShutdown = 0;
		bGetFormalNMEA = 0;
		gps_state_unlock_fix(state);
	}
}

void gps_sleep(GpsState *state)
{
	D("%s: gps_sleep", __FUNCTION__);
	gps_state_lock_fix(state);

	if(state->fd == -1)
		gps_opentty(state);

	bOrionShutdown = 0;

	started = 0;
	sleep_lock = 0; // allow next wakeup command

	gps_closetty(state);
	gps_state_unlock_fix(state);
}

static void
gps_state_start( GpsState*  s )
{
    char  cmd = CMD_START;
    int   ret;
    DFR("%s", __FUNCTION__);

    gps_state_lock_fix(s);
    lastcmd = CMD_START;
    gps_state_unlock_fix(s);

    do { ret=write( s->control[0], &cmd, 1 ); }
    while (ret < 0 && errno == EINTR);

    if (ret != 1)
        D("%s: could not send CMD_START command: ret=%d: %s", __FUNCTION__, ret, strerror(errno));
}

static void
gps_state_stop( GpsState*  s )
{
    char  cmd = CMD_STOP;
    int   ret;

    DFR("%s", __FUNCTION__);

    gps_state_lock_fix(s);
    lastcmd = CMD_STOP;
    gps_state_unlock_fix(s);

    do
    {
        DFR("try %s", __FUNCTION__);
        ret=write( s->control[0], &cmd, 1 );
        if(ret < 0)
        {
            ALOGE("write control socket error %s", strerror(errno));
            sleep(1);
        }
    }
    while (ret < 0 && errno == EINTR);

    if (ret != 1)
        D("%s: could not send CMD_STOP command: ret=%d: %s",
          __FUNCTION__, ret, strerror(errno));
}


static int
epoll_register( int  epoll_fd, int  fd )
{
    struct epoll_event  ev;
    int                 ret, flags;

    /* important: make the fd non-blocking */
    flags = fcntl(fd, F_GETFL);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);

    ev.events  = EPOLLIN;
    ev.data.fd = fd;
    do {
        ret = epoll_ctl( epoll_fd, EPOLL_CTL_ADD, fd, &ev );
    } while (ret < 0 && errno == EINTR);
    return ret;
}


static int
epoll_deregister( int  epoll_fd, int  fd )
{
    int  ret;
    do {
        ret = epoll_ctl( epoll_fd, EPOLL_CTL_DEL, fd, NULL );
    } while (ret < 0 && errno == EINTR);
    return ret;
}

/* this is the main thread, it waits for commands from gps_state_start/stop and,
 * when started, messages from the QEMU GPS daemon. these are simple NMEA sentences
 * that must be parsed to be converted into GPS fixes sent to the framework
 */

static int         epoll_ctrlfd ;
static int         epoll_nmeafd ;


static void
gps_state_thread( void*  arg )
{
    GpsState*   state = (GpsState*) arg;
    int         gps_fd     = state->fd;
    int         control_fd = state->control[1];
    epoll_ctrlfd   = epoll_create(1);
    epoll_nmeafd   = epoll_create(1);

    // register control file descriptors for polling
    epoll_register( epoll_ctrlfd, control_fd );

    D("gps thread running");

    state->tmr_thread = state->callbacks.create_thread_cb("hd8020_gps_tmr", gps_timer_thread, state);
    if (!state->tmr_thread)
    {
        ALOGE("could not create gps timer thread: %s", strerror(errno));
        started = 0;
        state->init = STATE_INIT;
        goto Exit;
    }

    state->nmea_thread = state->callbacks.create_thread_cb("hd8020_nmea_thread", gps_nmea_thread, state);
    if (!state->nmea_thread)
    {
        ALOGE("could not create gps nmea thread: %s", strerror(errno));
        started = 0;
        state->init = STATE_INIT;
        goto Exit;
    }

    started = 0;
    state->init = STATE_INIT;

    // now loop
    for (;;) {
        struct epoll_event   events[1];
        int                  ne, nevents;

        nevents = epoll_wait( epoll_ctrlfd, events, 1, -1 );
        if (nevents < 0) {
            if (errno != EINTR)
                ALOGE("epoll_wait() unexpected error: %s", strerror(errno));
            continue;
        }
        D("gps thread received %d events", nevents);
        for (ne = 0; ne < nevents; ne++) {
            if ((events[ne].events & (EPOLLERR|EPOLLHUP)) != 0) {
                ALOGE("EPOLLERR or EPOLLHUP after epoll_wait() !?");
                goto Exit;
            }
            if ((events[ne].events & EPOLLIN) != 0) {
                int  fd = events[ne].data.fd;

                if (fd == control_fd)
                {
                    char  cmd = 255;
                    int   ret;
                    D("gps control fd event");
                    do {
                        ret = read( fd, &cmd, 1 );
                    } while (ret < 0 && errno == EINTR);

                    if (cmd == CMD_QUIT) {
                        D("gps thread quitting on demand");
                        goto Exit;
                    }
                    else if (cmd == CMD_START)
                    {
                        if (!started)
                        {
                            NmeaReader  *reader;
                            reader = &state->reader;
                            nmea_reader_init( reader );
                            D("gps thread starting  location_cb=%p", state->callbacks.location_cb);
                            started = 1;
                            state->init = STATE_START;
                            /* handle wakeup routine*/
                            gps_wakeup(state);
                        }
                        else
                            D("LM already start");
                    }
                    else if (cmd == CMD_STOP) {
                        if (started) {
                            state->init = STATE_INIT;
                       }
                    }
                    else if ((cmd == CMD_MODE_GPS) || (cmd == CMD_MODE_BD) || (cmd == CMD_MODE_GN)) {
                        D("[MODE_SW]: get event cmd:%d, gps_state->fd:%d\n", cmd, gps_state->fd);
                        if(gps_state->fd > 0)
                            set_rnss_mode(gps_state->fd, cmd);
                    }
                }
                else
                {
                    ALOGE("epoll_wait() returned unkown fd %d ?", fd);
                    gps_fd = _gps_state->fd; //resign fd to gps_fd
                }
            }
        }
    }
Exit:
    {
        void *dummy;
        continue_thread = 0;
        close(epoll_ctrlfd);
        close(epoll_nmeafd);
        pthread_join(state->tmr_thread, &dummy);
        pthread_join(state->nmea_thread, &dummy);
        DFR("gps control thread destroyed");
    }
    return;
}

static void
gps_nmea_thread( void*  arg )
{
    GpsState *state = (GpsState *)arg;
    NmeaReader  *reader;
    reader = &state->reader;

    DFR("gps entered nmea thread");
    int versioncnt = 0;

    DFR("%s %d %d", __FUNCTION__, __LINE__, state->fd);
   // now loop
    while (continue_thread)
    {
        char buf[512];
        int  nn, ret;
        struct timeval tv;

        while(sleep_lock == 0 && continue_thread) //don't read while sleep
            sleep(1);

        if(state->fd == -1)
        {
            GPS_STATUS_CB(state->callbacks, GPS_STATUS_SESSION_END);
            gps_opentty(state);
            sleep(1);
            continue;
        }

        if(bOrionShutdown && started) // Orion be shutdown but LM is started, try to wake it up.
        {
            ALOGI("Try to wake orion up after 5 secs");
            sleep_lock = 0;
            sleep(5);
            GPS_STATUS_CB(state->callbacks, GPS_STATUS_SESSION_BEGIN);
            gps_wakeup(state);
            nmea_reader_init( reader );
            bOrionShutdown = 0;
        }

        if(!continue_thread)
            break;

        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(state->fd, &readfds);
        /* Wait up to 100 ms. */
#if 0
        tv.tv_sec = 0;
        tv.tv_usec = 100;
#else
        tv.tv_sec = 0;
        tv.tv_usec = 500000;
#endif
        ret = select(state->fd + 1, &readfds, NULL, NULL, &tv);

        if (state->fd > 0 && FD_ISSET(state->fd, &readfds))
        {
            memset(buf,0,sizeof(buf));
            ret = read( state->fd, buf, sizeof(buf) );
            if (ret > 0)
            {
                for (nn = 0; nn < ret; nn++)
                    nmea_reader_addc( reader, buf[nn] );
            }
            else if((ret == -1) && (errno == EAGAIN)){
                DFR("Try again NMEA read :%s, ret:%d\n",strerror(errno), ret);
                continue;
            }
            else
            {
                DFR("Error on NMEA read :%s, ret:%d\n",strerror(errno), ret);
                gps_closetty(state);
                GPS_STATUS_CB(state->callbacks, GPS_STATUS_SESSION_END);
                sleep(3); //wait Orion shutdown.
                bOrionShutdown = 1;
                continue;
            }
        }

        if(!continue_thread)
            break;
    }
Exit:
    DFR("gps nmea thread destroyed");
    return;
}

static void
gps_timer_thread( void*  arg )
{
    int need_sleep = 0;
    int sleep_val = 0;

  GpsState *state = (GpsState *)arg;

  DFR("gps entered timer thread");

  do {

    while(started != 1 && continue_thread) //
    {
        usleep(500*1000);
    }

    gps_state_lock_fix(state);

      D("%s %d 0x%x %d %d %d", __FUNCTION__, __LINE__, state->reader.fix.flags, state->reader.sv_status_changed, state->reader.sv_status[0].num_svs, state->reader.sv_status[1].num_svs);
    if ((state->reader.fix.flags & GPS_LOCATION_HAS_LAT_LONG) != 0) {

      D("gps fix cb: 0x%x", state->reader.fix.flags);

      if (state->callbacks.location_cb) {
          state->callbacks.location_cb( &state->reader.fix );
          state->reader.fix.flags = 0;
          state->first_fix = 1;
      }

      if (state->fix_freq == 0) {
        state->fix_freq = -1;
      }
    }

    if (state->reader.sv_status_changed != 0) {

      //D("gps sv status callback");

      if (state->callbacks.sv_status_cb) {
      int m = state->reader.sv_status[0].num_svs;
      int n = state->reader.sv_status[1].num_svs;
      char buffer[256*2], *p = buffer;
      int i;
      //D("gps sv status callback: num_svs=%d,%d(fix:%d)", m, n, state->reader.gsa_fixed);
#ifdef GPS_DEBUG
      p = buffer;
      for(i = 0; i < m; i++) {
          p += sprintf(p, "(%d,%.1f)",
              state->reader.sv_status[0].bd_sv_list[i].prn,
              state->reader.sv_status[0].bd_sv_list[i].snr);
      }
      if(m) DFR("gps sv status callback: bd_sv_list[0]=%s", buffer);
      p = buffer;
      for(i = 0; i < n; i++) {
          p += sprintf(p, "(%d,%.1f)",
              state->reader.sv_status[1].bd_sv_list[i].prn ,
              state->reader.sv_status[1].bd_sv_list[i].snr);
      }
      if(n) DFR("gps sv status callback: bd_sv_list[1]=%s", buffer);
#endif


      /* combine BD2&GPS sv in sv_status[0] */
      if((m+n) > BD_GPS_MAX_SVS) n = BD_GPS_MAX_SVS - m;
      if(n) {
          memcpy(&state->reader.sv_status[0].bd_sv_list[m], state->reader.sv_status[1].bd_sv_list, sizeof(GpsSvInfo)*n);
          state->reader.sv_status[0].num_svs = m + n;
      }
#if 0
      for(i=0; i<8; i++)
      {
        D("gps sv status fix mask = %08x ", state->reader.sv_status[0].bd_used_in_fix_mask[i]);
        D("bd_sv_status fix mask = %08x ", state->reader.sv_status[1].bd_used_in_fix_mask[i] );
      }
#endif
      for (i = 0; i < 8; i++) {
          state->reader.sv_status[0].bd_used_in_fix_mask[i] |=  state->reader.sv_status[1].bd_used_in_fix_mask[i];
          /* D("gps + bdsv status fix mask = %08x ", state->reader.sv_status[0].bd_used_in_fix_mask[i]); */
      }
      state->callbacks.sv_status_cb( &state->reader.sv_status[0] );
      //state->reader.sv_status[0].num_svs = 0;
      //state->reader.sv_status[1].num_svs = 0;
    memset(gps_state->reader.sv_status, 0, sizeof(gps_state->reader.sv_status));
    //memset(gps_state->reader.sv_status_tmp, 0, sizeof(gps_state->reader.sv_status_tmp));
          state->reader.sv_status_changed = 0;
      }

    }

    need_sleep = (state->fix_freq != -1 && (state->init != STATE_QUIT) ? 1 : 0);
    sleep_val = state->fix_freq;

    gps_state_unlock_fix(state);

    if (need_sleep) {
        sleep(sleep_val);
    } else {
        D("won't sleep because fix_freq=%d state->init=%d",state->fix_freq, state->init);
    }
    um_send_ni_notification(&state->reader);
    if( state->init == STATE_INIT && lastcmd == CMD_STOP && started == 1)
    {
        int gap = 0;
        D("Wait for NMEA coming,%d,%d,%d", state->init , lastcmd, started);

        while (state->init != STATE_START && bGetFormalNMEA == 0 && continue_thread && !bOrionShutdown)
        {
            usleep(300*1000);
            if (++gap > 100)
                break;
        } ;

        D("Get NMEA %d and state %d",bGetFormalNMEA,state->init);
        // even we don't get nmea after 30 second, still force close it
        bGetFormalNMEA |= (gap >= 100);

        if( state->init == STATE_INIT && lastcmd == CMD_STOP && started == 1)
        {
            gps_sleep(state);
        }
        else
        {
            D("User enter LM before sending sleep, so drop it");
        }
    }

  } while(continue_thread);


  DFR("gps timer thread destroyed");

  return;

}

static char   prop[PROPERTY_VALUE_MAX] = "/dev/ttyHSL1";

int gps_opentty(GpsState *state)
{
    if(state->fd != -1)
        gps_closetty(state);

    do {
        state->fd = open( prop, O_RDWR | O_NOCTTY | O_NONBLOCK);
    } while (state->fd < 0 && errno == EINTR);

	if (state->fd < 0) {
		ALOGE("could not open gps serial device %s: %s", prop, strerror(errno) );
		return -1;
	}

    D("gps will read from %s, state->fd:%d", prop, state->fd);

    // disable echo on serial lines
    if ( isatty( state->fd ) ) {
        init_serial(state->fd);
    }

    return 0;
}

void gps_closetty(GpsState *state)
{
    if(state->fd != -1)
    {
        DFR("%s, state->fd:%d\n", __FUNCTION__, state->fd);
        // close connection to the QEMU GPS daemon
        close( state->fd );
        state->fd = -1;
    }
}

static void
gps_state_init( GpsState*  state )
{
    int    ret;
    int    done = 0;

    struct sigevent tmr_event;

    state->init       = STATE_INIT;
    state->control[0] = -1;
    state->control[1] = -1;
    state->fd         = -1;
    state->fix_freq   = -1;
    state->first_fix  = 0;
    state->rnss_mode = CMD_MODE_GN;
    continue_thread = 1;

    DFR("%s %d ", __FUNCTION__, __LINE__);
    if (sem_init(&state->fix_sem, 0, 1) != 0) {
      D("gps semaphore initialization failed! errno = %d", errno);
      return;
    }
#if 0
    // look for a kernel-provided device name
    if (property_get("hd8020.gps.mode",prop,"hosted") == 0)
    {
      DFR("Running HD8020 GPS driver in hosted mode!");
      if (property_get("hd8020.gps.node",prop,"") == 0)
      {
        DFR("no user specific gps device name... try default name... ");
        if (property_get("ro.kernel.android.gps",prop,"") == 0)
        {
          DFR("no kernel-provided gps device name (hosted)");
          DFR("please set ro.kernel.android.gps property");
          return;
        }
      }
    }else /* not hosted mode */
    {
      if (property_get("ro.kernel.android.gps",prop,"") == 0)
      {
        DFR("no kernel-provided gps device name (not hosted)");
        DFR("please set ro.kernel.android.gps property");
        return;
      }
    }
#endif

    if ( socketpair( AF_LOCAL, SOCK_STREAM, 0, state->control ) < 0 ) {
        ALOGE("could not create thread control socket pair: %s", strerror(errno));
        goto Fail;
    }

    state->thread = state->callbacks.create_thread_cb("hd8020_gps", gps_state_thread, state);
        if (!state->thread)
    {
        ALOGE("could not create gps thread: %s", strerror(errno));
        goto Fail;
    }
        state->callbacks.set_capabilities_cb(GPS_CAPABILITY_SCHEDULING);

    D("gps state initialized");

    return;

Fail:
    gps_state_done( state );
}

static void init_serial(int fd){
        struct termios  ios;
        tcgetattr(fd, &ios );
        ios.c_cflag = 0;
        ios.c_cflag |= (CREAD | CLOCAL);
        ios.c_cflag &= ~CRTSCTS;
        ios.c_iflag = 0;
        ios.c_oflag = 0;
        ios.c_lflag = 0;
        ios.c_cc[VMIN] = 1;
        ios.c_cc[VTIME] = 10;
        ios.c_cflag &= ~CSIZE;
        ios.c_cflag |= CS8;
        ios.c_cflag &= ~CSTOPB;
        ios.c_cflag &= ~PARENB;
        ios.c_iflag &= ~INPCK;
        int ret = tcsetattr( fd, TCSANOW, &ios );
        tcflush(fd, TCIOFLUSH);
        struct termios ios1;
        tcgetattr( fd, &ios1 );
        cfsetispeed(&ios1, B115200);
        cfsetospeed(&ios1, B115200);
        tcsetattr( fd, TCSANOW, &ios1 );
        ret = tcflush(fd, TCIOFLUSH);

}

static int hd8020_gps_init(GpsCallbacks* callbacks)
{
    GpsState*  s = _gps_state;

    D("gps state initializing %d",s->init);

    s->callbacks = *callbacks;
    if (!s->init)
        gps_state_init(s);

    if(!g_gpscallback)
        g_gpscallback = callbacks;

    return 0;
}

static void
hd8020_gps_cleanup(void)
{
    GpsState*  s = _gps_state;

    D("%s: called", __FUNCTION__);

    if (s->init)
        gps_state_done(s);
}

static int hd8020_gps_start()
{
    GpsState*  s = _gps_state;
    D("%s: called", __FUNCTION__ );
    power_on();

    if(gps_checkstate(s) == -1)
    {
        DFR("%s: called with uninitialized state !!", __FUNCTION__);
        return -1;
    }

    gps_state_start(s);
    GPS_STATUS_CB(s->callbacks, GPS_STATUS_SESSION_BEGIN);

    return 0;
}


static int
hd8020_gps_stop()
{
    GpsState*  s = _gps_state;

    D("%s: called", __FUNCTION__ );

    if(gps_checkstate(s) == -1)
    {
        DFR("%s: called with uninitialized state !!", __FUNCTION__);
        return -1;
    }

    //close LM first
    gps_state_stop(s);
    D("Try to change state to init");
    //change state to INIT
    GPS_STATUS_CB(s->callbacks, GPS_STATUS_SESSION_END);

    power_off();
    return 0;
}


static void
hd8020_gps_set_fix_frequency(int freq)
{
    GpsState*  s = _gps_state;

    if(gps_checkstate(s) == -1)
    {
        DFR("%s: called with uninitialized state !!", __FUNCTION__);
        return;
    }

    s->fix_freq = (freq <= 0) ? 1 : freq;

    D("gps fix frquency set to %d secs", freq);
}

static int
hd8020_gps_inject_time(GpsUtcTime time, int64_t timeReference, int uncertainty)
{
    return 0;
}

/*workaround to trigger hot/warm/cold/full start*/
#define FLAG_HOT_START  GPS_DELETE_RTI
#define FLAG_WARM_START GPS_DELETE_EPHEMERIS
#define FLAG_COLD_START (GPS_DELETE_EPHEMERIS | GPS_DELETE_POSITION | GPS_DELETE_TIME | GPS_DELETE_IONO | GPS_DELETE_UTC | GPS_DELETE_HEALTH)
#define FLAG_FULL_START (GPS_DELETE_ALL)
#define FLAG_AGPS_START (GPS_DELETE_EPHEMERIS | GPS_DELETE_ALMANAC | GPS_DELETE_POSITION | GPS_DELETE_TIME | GPS_DELETE_IONO | GPS_DELETE_UTC)

static void
hd8020_gps_delete_aiding_data(GpsAidingData flags)
{
	GpsState*  state = &_gps_state;
	ALOGD("step in %s, gps_fd:%d, started:%d, flags:%x\n", __func__, state->fd, started, flags);
	if(!started){
		power_off();
		power_on();
		gps_opentty(state);
		sleep(1);
	}
	if ((flags == FLAG_FULL_START) || (flags == FLAG_COLD_START))
	{
		ALOGD("in %s, full_start..., state->fd:%d",  __func__, state->fd);
		cold_start(state->fd);
	}
	else if (flags == FLAG_WARM_START)
	{
		ALOGD("in %s, warm_start..., state->fd:%d",  __func__, state->fd);
		warm_start(state->fd);
	}
	/* else if (flags == FLAG_HOT_START) */
	else
	{
		ALOGD("in %s, hot_start..., state->fd:%d",  __func__, state->fd);
		hot_start(state->fd);
	}
	if(!started){
		gps_closetty(state);
		sleep(1);
		power_off();
	}
}

static int
hd8020_gps_inject_location(double latitude, double longitude, float accuracy)
{
    return 0;
}

static int hd8020_gps_set_position_mode(GpsPositionMode mode, GpsPositionRecurrence recurrence,
                                      uint32_t min_interval, uint32_t preferred_accuracy, uint32_t preferred_time)
{
    GpsState*  s = _gps_state;
    char cmd = CMD_MODE_GN;
    int ret;

    // only standalone is supported for now.
    if (mode != GPS_POSITION_MODE_STANDALONE)
    {
        D("%s: set GPS POSITION mode error! (mode:%d) ", __FUNCTION__, mode);
        D("Set as standalone mode currently! ");
        //        return -1;
    }

    if (!s->init) {
        D("%s: called with uninitialized state !!", __FUNCTION__);
        return -1;
    }

    s->fix_freq = min_interval/1000;
    if (s->fix_freq ==0)
    {
        s->fix_freq =1;
    }
    D("gps fix frquency set to %d sec", s->fix_freq);

    if((mode & 0x30) == 0x30)
        cmd = CMD_MODE_GN;
    else if((mode & 0x20) == 0x20)
        cmd = CMD_MODE_BD;
    else if((mode & 0x10) == 0x10)
        cmd = CMD_MODE_GPS;

    s->rnss_mode = cmd;
    D("RNSS mode %s %02x",__FUNCTION__, cmd);
    return 0;
}


int gps_checkstate(GpsState *s)
{
    if (!s->init) {

        if(g_gpscallback)
            hd8020_gps_init(g_gpscallback);

        if(!s->init)
        {
            ALOGE("%s: still called with uninitialized state !!", __FUNCTION__);
            return -1;
        }
    }

    return 0;
}

#ifdef HD8020_GPSXtra
/*
 * no special parameter to be initialized here
 */
static int hd8020_gps_xtra_init(GpsXtraCallbacks* callbacks)
{
    D("%s: entered", __FUNCTION__);

    return 0;
}

/*
 * real nmea command injection function
 */
static int hd8020_gps_xtra_inject_data(char *data, int length)
{
	unsigned int f1_flag = 0, frame_len = 0;
	char *pdata;
	int nwrite=0;
	GpsState*  s =&_gps_state;

	ALOGD("step in %s, gps_fd:%d, started:%d...\n", __func__, s->fd, started);
	if(!started){
		power_off();
		power_on();
		gps_opentty(s);
		sleep(1);
	}
	ALOGD("%s(%d) length : %d", __FUNCTION__, __LINE__, length);
	ALOGD("data length is %d\n",length);
	ALOGD("data:d[0,1,2,3,4] is %x,%x,%x,%x\n",data[0],data[1],data[2],data[3],data[4]);
	if (s->fd == -1){
		ALOGE(" %s: fd fail! ", __FUNCTION__);
		return -1;           //  gps uart not open
	}
	pdata = data;
	while(f1_flag < length-1){
		if((pdata[f1_flag]==0xF1) && (pdata[f1_flag+1]==0xD9)){
			frame_len = (unsigned short)(pdata[f1_flag + 4] | (pdata[f1_flag + 5]<<8));
			nwrite = write(s->fd, pdata+f1_flag, frame_len+8);
			if(nwrite<0)
			{
				ALOGE("hd8020_bdgps_xtra_inject_data Error!\n");
			}
			else
			{
				ALOGD("hd8020_bdgps_xtra_inject_data success!\n");
				ALOGD("inject %d data\n",nwrite);
			}
		}else{
			ALOGE("the pdata is not 0xF1:%x,%x!\n",pdata[f1_flag],pdata[f1_flag+1]);
			break;
		}
		usleep(200000);
		f1_flag = f1_flag + frame_len + 8;
	}
	if(!started){
		gps_closetty(s);
		sleep(1);
		power_off();
	}

	return 0;
}

static const GpsXtraInterface hd8020GpsXtraInterface = {
	sizeof(GpsXtraInterface),
	hd8020_gps_xtra_init,
	hd8020_gps_xtra_inject_data,
};
#endif /* HD8020_GPSXtra */

#ifdef HD8020_GPSNi
/*
 * Registers the callbacks for HAL to use
 */
static void hd8020_gps_ni_init(GpsNiCallbacks* callbacks)
{
    GpsState*  s = _gps_state;

    D("%s: entered", __FUNCTION__);

    if (callbacks)
    {
        s->ni_init = 1;
        s->ni_callbacks = *callbacks;
    }
}

/*
 * Sends a response to HAL
 */
static void hd8020_gps_ni_respond(int notif_id, GpsUserResponseType user_response)
{
    // D("%s: entered", __FUNCTION__);
}

static const GpsNiInterface hd8020GpsNiInterface = {
    sizeof(GpsNiInterface),
    hd8020_gps_ni_init,
    hd8020_gps_ni_respond,
};
#endif // HD8020_GPSNi


static const void* hd8020_gps_get_extension(const char* name)
{
    if (strcmp(name, GPS_XTRA_INTERFACE) == 0)
    {
#ifdef HD8020_GPSXtra
        D("%s: found xtra extension", __FUNCTION__);
        return (&hd8020GpsXtraInterface);
#endif // HD8020_GPSXtra
    }
    else if (strcmp(name, GPS_NI_INTERFACE) == 0)
    {
#ifdef HD8020_GPSNi
        D("%s: found ni extension", __FUNCTION__);
        return (&hd8020GpsNiInterface);
#endif // HD8020_GPSNi
    }

    /* D("%s: no GPS extension for %s is found", __FUNCTION__, name); */
    return NULL;
}

static const GpsInterface  hd8020GpsInterface = {
    .size =sizeof(GpsInterface),
    .init = hd8020_gps_init,
    .start = hd8020_gps_start,
    .stop = hd8020_gps_stop,
    .cleanup = hd8020_gps_cleanup,
    .inject_time = hd8020_gps_inject_time,
    .inject_location = hd8020_gps_inject_location,
    .delete_aiding_data = hd8020_gps_delete_aiding_data,
    .set_position_mode = hd8020_gps_set_position_mode,
    .get_extension = hd8020_gps_get_extension,
};

const GpsInterface* hd_gps_get_gps_interface()
{
    return &hd8020GpsInterface;
}

static unsigned char gps_dev_calc_nmea_csum(char *msg)
{
  unsigned char csum = 0;
  int i;

  for (i = 1; msg[i] != '*'; ++i) {
    csum ^= msg[i];
  }

  return csum;
}
static unsigned char bd_check_eor(char *buf, int len)
{
  unsigned char csum = 0;

  while(len--)
        csum ^= buf[len];

  return csum;
}

static void set_rnss_mode(int fd, char mode)
{
	int v, ret;
	unsigned char sum;
	char cmd[32];
	char gps_mode[12] = {0xf1, 0xd9, 0x06, 0x0c, 0x04, 0x00, 0x01, 0x00, 0x00, 0x00, 0x17, 0xa0};
	char bds_mode[12] = {0xf1, 0xd9, 0x06, 0x0c, 0x04, 0x00, 0x04, 0x00, 0x00, 0x00, 0x1a, 0xac};
	char gnss_mode[12] = {0xf1, 0xd9, 0x06, 0x0c, 0x04, 0x00, 0x05, 0x00, 0x00, 0x00, 0x1b, 0xb0};

	D("[MODE_SW]: mode: %02x, fd: %d\n", mode, fd);
	switch(mode) {
		case CMD_MODE_GPS:
			ret = write(fd, gps_mode, 12);
			DFR("function : %s, ret:%d", __func__, ret);
			break;
		case CMD_MODE_BD:
			ret = write(fd, bds_mode, 12);
			DFR("function : %s, ret:%d", __func__, ret);
			break;
		case CMD_MODE_GN:
			ret = write(fd, gnss_mode, 12);
			DFR("function : %s, ret:%d", __func__, ret);
			break;
		default:
			mode=0;
			break;
	}

	memset(&gps_state->reader.sv_status, 0, sizeof(gps_state->reader.sv_status));
	memset(&gps_state->reader.sv_status_tmp, 0, sizeof(gps_state->reader.sv_status_tmp));
	gps_state->reader.sv_status_changed = 1;

	return;
}

static void check_rnss_mode( NmeaReader* r)
{
	/* D("[MODE_SW]: in %s, rnss_mode:%d, nmea_mode:%d \n", __func__, gps_state->rnss_mode, r->nmea_mode); */
	if(gps_state->rnss_mode != r->nmea_mode) {
		if(gps_state->control[0] > 0) {
			//set_rnss_mode(gps_state->fd, r->nmea_mode);
			int ret;
			char cmd = (char) gps_state->rnss_mode;
			D("[MODE_SW]: will write cmd:%d to control_fd:%d\n", cmd, gps_state->control[0]);
			do
			{
		  ret = write(gps_state->control[0], &cmd, 1);
	  }
			while (ret < 0 && errno == EINTR);
		}
	}
}

int set_message(unsigned char* dst,unsigned char* src,int src_len)
{
	int i;

	//message header
	dst[0]=0xF1;
	dst[1]=0xD9;

	//message ID
	dst[2]=0x06;
	dst[3]=0x40;

	//message len
	dst[4]=src_len;
	dst[5]=00;

	//payload
	for( i = 0 ; i < src_len ; i++)
	{
		dst[6+i] = *src ;
		src++ ;
	}

	unsigned char checksum1 = 0x00;
	unsigned char checksum2 = 0x00;
	for(i=2;i<(6+dst[4]);i++){
		//printf("%2X ",dst[i]);
		checksum1+=dst[i];
		checksum2+=checksum1;
	}

	dst[src_len+6]= checksum1;
	dst[src_len+7]= checksum2;
	dst[src_len+8]=0x0d;
	dst[src_len+9]=0x0a;

	return src_len+10 ;
}

int cold_start(int fd_global){
	unsigned char cmd[1],message[11];
	int len;
	int ret;

	memset(cmd,0,sizeof(cmd));
	cmd[0] = 0x01;
	len = set_message(message,cmd,sizeof(cmd));
	ALOGD("in %s, message: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X", __func__,
			message[0],message[1],message[2],message[3],message[4],message[5],message[6],message[7],
			message[8],message[9],message[10]);
	ret = write(fd_global,message,len);

	ALOGD("in %s, ret:%d, len:%d\n", __func__, ret, len);
	return 1;
}

int warm_start(int fd_global){
	unsigned char cmd[1],message[11];
	int len;
	int ret;

	memset(cmd,0,sizeof(cmd));
	cmd[0] = 0x02;
	len = set_message(message,cmd,sizeof(cmd));
	ALOGD("in %s, message: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X", __func__,
			message[0],message[1],message[2],message[3],message[4],message[5],message[6],message[7],
			message[8],message[9],message[10]);
	ret = write(fd_global,message,len);

	ALOGD("in %s, ret:%d, len:%d\n", __func__, ret, len);
	return 1;
}

int hot_start(int fd_global){
	unsigned char cmd[1],message[11];
	int len;
	int ret;

	memset(cmd,0,sizeof(cmd));
	cmd[0] = 0x03;
	len = set_message(message,cmd,sizeof(cmd));
	ALOGD("in %s, message: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X", __func__,
			message[0],message[1],message[2],message[3],message[4],message[5],message[6],message[7],
			message[8],message[9],message[10]);
	ret = write(fd_global,message,len);
	ALOGD("in %s, ret:%d, len:%d\n", __func__, ret, len);
	return 1;
}

#define BD_GPS_CONTORL_PATH "/sys/class/hd8020_bdgps_power/hd8020_bdgps/ctl_hd8020"
void power_on(void)
{
	int fd = 0;
	char c = '1';

	if(!power_flag){
		fd = open(BD_GPS_CONTORL_PATH, O_WRONLY);
		if(fd < 0){
			ALOGE("[BDGPS]: open %s failed\n", BD_GPS_CONTORL_PATH);
			return;
		}

		if(write(fd, &c, 1) < 0){
			ALOGE("[BDGPS]: write %s failed\n", BD_GPS_CONTORL_PATH);
			close(fd);
			return;
		}
		close(fd);
		power_flag = 1;
		ALOGE("[BDGPS]: ===========Power On============\n");
	}
}

void power_off(void)
{
	int fd = 0;
	char c = '0';

	if(!!power_flag){
		fd = open(BD_GPS_CONTORL_PATH, O_WRONLY);
		if(fd < 0){
			ALOGE("[BDGPS]: Open %s failed\n", BD_GPS_CONTORL_PATH);
			return;
		}

		if(write(fd, &c, 1) < 0){
			ALOGE("[BDGPS]: Write %s failed\n", BD_GPS_CONTORL_PATH);
			close(fd);
			return;
		}
		close(fd);
		power_flag = 0;
		ALOGE("[BDGPS]: ===========Power Off============\n");
	}
}
