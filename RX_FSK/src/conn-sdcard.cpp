#include "../features.h"

#if FEATURE_SDCARD

#include "conn-sdcard.h"

#define TAG "conn-sdcard"
#include "logger.h"

#include "posinfo.h"

#include <ff.h>
#include <dirent.h>
#include <time.h>
#include <sys/stat.h>
#include <stdio.h>
#include <string.h>
#include <utime.h>

extern SemaphoreHandle_t globalLock;
#define SPI_MUTEX_LOCK() \
        do                     \
{                      \
} while (xSemaphoreTake(globalLock, portMAX_DELAY) != pdPASS)
#define SPI_MUTEX_UNLOCK() xSemaphoreGive(globalLock)


static SPIClass sdspi(HSPI);

const char *cardTypeStr(uint8_t cardtype) {
  switch(cardtype) {
     case CARD_NONE: return "No SD card attached";
     case CARD_MMC: return "MMC";
     case CARD_SD: return "SDSC";
     case CARD_SDHC: return "SDHC";
     default: return "unknown";
  }
}

/* Return true if d_name is a CSV file and not mypos.csv */
static int isCsvToMove(const char *d_name) {
  size_t len = strlen(d_name);
  if (len < 4) return 0;
  if (strcmp(d_name + len - 4, ".csv") != 0) return 0;
  if (strcmp(d_name, "mypos.csv") == 0) return 0;
  return 1;
}

/* Add YYMM to dirsTouched if not already present. Returns 1 if added. */
static int addDirTouched(char dirsTouched[][8], int *ndirs, int maxdirs, const char *yymm) {
  for (int i = 0; i < *ndirs; i++)
    if (strcmp(dirsTouched[i], yymm) == 0) return 0;
  if (*ndirs >= maxdirs) return 0;
  strncpy(dirsTouched[*ndirs], yymm, 7);
  dirsTouched[*ndirs][7] = '\0';
  (*ndirs)++;
  return 1;
}

/* After init: move root CSV files (except mypos.csv) into YYMM folders based on file mtime */
static void reorganizeCsvRoot(void) {
  char dirsTouched[48][8];
  int ndirs = 0;
  const int maxdirs = (int)(sizeof(dirsTouched) / sizeof(dirsTouched[0]));

  SPI_MUTEX_LOCK();
  DIR *d = opendir("/sd");
  if (!d) { SPI_MUTEX_UNLOCK(); return; }
  struct dirent *e;
  struct stat st;
  char vfsPath[128];
  char yymm[8];
  while ((e = readdir(d)) != NULL) {
    if (strcmp(e->d_name, ".") == 0 || strcmp(e->d_name, "..") == 0) continue;
    snprintf(vfsPath, sizeof(vfsPath), "/sd/%s", e->d_name);
    if (stat(vfsPath, &st) != 0 || !S_ISREG(st.st_mode)) continue;
    if (!isCsvToMove(e->d_name)) continue;

    struct tm *tm = gmtime(&st.st_mtime);
    if (!tm) continue;
    snprintf(yymm, sizeof(yymm), "/%02d%02d", tm->tm_year % 100, tm->tm_mon + 1);
    if (!SD.exists(yymm) && !SD.mkdir(yymm)) continue;

    addDirTouched(dirsTouched, &ndirs, maxdirs, yymm+1);

    char dstVfs[128];
    snprintf(dstVfs, sizeof(dstVfs), "/sd%s/%s", yymm, e->d_name);
    if (rename(vfsPath, dstVfs) == 0)
      LOG_I(TAG, "Reorganize: moved %s -> %s\n", vfsPath, dstVfs);
  }
  closedir(d);

  /* Set each touched directory mtime to start of that month (1st, 00:00:00) */
  for (int i = 0; i < ndirs; i++) {
    const char *p = dirsTouched[i];
    if (strlen(p) < 4) continue;
    int yy = (p[0] - '0') * 10 + (p[1] - '0');
    int mm = (p[2] - '0') * 10 + (p[3] - '0');
    if (mm < 1 || mm > 12) continue;
    int year = (yy >= 80) ? (1900 + yy) : (2000 + yy);
    struct tm t;
    memset(&t, 0, sizeof(t));
    t.tm_year = year - 1900;
    t.tm_mon = mm - 1;
    t.tm_mday = 1;
    t.tm_isdst = -1;
    time_t startOfMonth = mktime(&t);
    if (startOfMonth == (time_t)-1) continue;
    char dirVfs[32];
    snprintf(dirVfs, sizeof(dirVfs), "/sd/%s", dirsTouched[i]);
    struct utimbuf ub;
    ub.actime = startOfMonth;
    ub.modtime = startOfMonth;
    utime(dirVfs, &ub);
  }
  SPI_MUTEX_UNLOCK();
}

void ConnSDCard::init() {
  if(sonde.config.sd.clk==-1)
    return;

  SPI_MUTEX_LOCK();
  /* Initialize SD card */
  // SPI (==VSPI) is used by SX127x. 
  // On LoRa32, SD-Card is on different pins, so cannot share VSPI
  // Use HSPI (if using a TFT with SPI, you have to make sure that the same pins are used for both (MISO/MOSI/CLK) 
  sdspi.begin(sonde.config.sd.clk, sonde.config.sd.miso, sonde.config.sd.mosi, sonde.config.sd.cs);
  if (sonde.config.sd.speed > 0) {
    initok = SD.begin(sonde.config.sd.cs, sdspi, (uint32_t)sonde.config.sd.speed);
    LOG_I(TAG, "SD card init: %s (SPI %d Hz)\n", initok ? "OK" : "Failed", sonde.config.sd.speed);
  } else {
    initok = SD.begin(sonde.config.sd.cs, sdspi);
    LOG_I(TAG, "SD card init: %s\n", initok ? "OK" : "Failed");
  }
  uint8_t cardType = SD.cardType();
  LOG_I(TAG, "SD Card Type: %s\n", cardTypeStr(cardType));
  if (cardType == CARD_NONE) { SPI_MUTEX_UNLOCK(); return; }

  uint32_t cardSize = SD.cardSize() / (1024 * 1024);
  LOG_I(TAG, "SD Card Size: %luMB\n", cardSize);
  uint32_t usedSize = SD.usedBytes() / (1024 * 1024);
  uint32_t totalSize = SD.totalBytes() / (1024 * 1024);
  LOG_I(TAG, "SD Card used/total: %lu/%lu MB\n", usedSize, totalSize);
  SPI_MUTEX_UNLOCK();

  if (initok && sonde.config.sd.name == 1)
    reorganizeCsvRoot();

#if 0
  file = SD.open("/data.csv", FILE_APPEND);
  if (!file) {
    Serial.println("Cannot open file");
    return;
  }
  file.printf("Hello word... test\n");
  file.close();

  //sdf = SD.open("/data.csv", FILE_READ);

  // Just testcode
  DIR *dir = opendir("/sd/");
  struct dirent *dent;
  struct stat attr;
  char fname[1024];
  strcpy(fname,"/sd/");
  if(dir) {
    while((dent=readdir(dir))!=NULL) {
      strcpy(fname+4, dent->d_name);
      stat(fname, &attr);
      char ftim[50];
      strftime(ftim, 50, "%Y-%m-%d %H:%M:%S", gmtime(&attr.st_mtime)); 
      printf("%s %s %d\n", dent->d_name, ftim, attr.st_size);
      
    }
    closedir(dir);
  }
#endif
}

int ConnSDCard::format() {
    if(sonde.config.sd.clk==-1)
        return -2;  // SD card disabled in config

    SPI_MUTEX_LOCK();
    int retval = 0;
    if(!initok) {
        // In case init() was not successfull on startup, re-run with format_if_empty set to yes
        {
          uint32_t freq = (sonde.config.sd.speed > 0) ? (uint32_t)sonde.config.sd.speed : 4000000u;
          initok = SD.begin(sonde.config.sd.cs, sdspi, freq, "/sd", 5, true);
        }
        if(!initok) retval = -1;
    } else {
        // Force format...
        // f_mkfs assumes that the SD card is registered already (i.e. SD.begin was called successfully before)
        // if not f_mkfs will cause a crash as device 0 is not defined.
        const MKFS_PARM opt = {(BYTE)FM_ANY, 0, 0, 0, 0};
        BYTE *work = (BYTE *)malloc(FF_MAX_SS);
        int res = f_mkfs("0:", &opt, work, FF_MAX_SS);
        free(work);
        if(res != FR_OK) retval = -1;
        else {
            SD.end();
            if (sonde.config.sd.speed > 0)
              initok = SD.begin(sonde.config.sd.cs, sdspi, (uint32_t)sonde.config.sd.speed);
            else
              initok = SD.begin(sonde.config.sd.cs, sdspi);
        }
    }
    LOG_I(TAG, retval==0 ? "Successfully formatted the SD card\n" : "Formatting SD card failed\n");
    SPI_MUTEX_UNLOCK();
    return retval;
}

void ConnSDCard::netsetup() {
  /* empty function, we don't use any network here */
}

void ConnSDCard::netshutdown() {
  /* empty function, we don't use any network here */
}

// Logs: sd.name==0 -> flat /serial.csv; sd.name==1 -> /yymm/serial.csv (create yymm dir). noid -> /noid.csv.
static char logName[32], logOldName[32] = {0};

void ConnSDCard::updateSonde( SondeInfo *si ) {
  if (!initok) return;
  SondeData *sd = &si->d;

  time_t tsec = (time_t)sd->time;
  if (tsec < 0) tsec = 0;
  struct tm t;
  gmtime_r(&tsec, &t);
  int yy = t.tm_year % 100;
  int mm = t.tm_mon + 1;

  if (!sd->validID) {
    snprintf(logName, sizeof(logName), "/noid.csv");
  } else {
    if (sonde.config.sd.name == 0) {
      snprintf(logName, sizeof(logName), "/%s.csv", sd->ser);
    } else {
      snprintf(logName, sizeof(logName), "/%02d%02d/%s.csv", yy, mm, sd->ser);
    }
  }

  SPI_MUTEX_LOCK();
  if (strcmp(logName, logOldName) || !file) {
    if (file) file.close();
    if (sd->validID && sonde.config.sd.name == 1) {
      char folder[8];
      snprintf(folder, sizeof(folder), "/%02d%02d", yy, mm);
      if (!SD.exists(folder)) SD.mkdir(folder);
    }
    file = SD.open(logName, FILE_APPEND);
    strcpy(logOldName, logName);
    LOG_I(TAG, "Logging to file %s\n", logName);
  }
  if (!file) {
    SPI_MUTEX_UNLOCK();
    LOG_E(TAG, "Error opening file %s\n", logName);
    return;
  }
  if (file.size() == 0)
    file.print("validID,ser,typestr,subtype,lat,lon,alt,vs,hs,dir,sats,validPos,time,frame,vframe,validTime\n");
  file.printf("%d,%s,%s,%d,"
              "%f,%f,%f,%f,%f,%f,%d,%d,"
              "%d,%d,%d,%d\n",
              sd->validID, sd->ser, sd->typestr, sd->subtype,
              sd->lat, sd->lon, sd->alt, sd->vs, sd->hs, sd->dir, sd->sats, sd->validPos,
              sd->time, sd->frame, sd->vframe, sd->validTime);

  // TODO: Make this time based, not invocation based (well, should be the same, this is called
  // 1x per second)
  wcount++;
  if (wcount >= sonde.config.sd.sync) {
    file.flush();
    wcount = 0;
  }
  SPI_MUTEX_UNLOCK();
}

static StationPos lastPI={0};

// TODO: This needs some cleanup.
// Code uses global varuable posInfo, not this PosInfo paramter (also in aprs, sondehub)
void ConnSDCard::updateStation( PosInfo *pi ) {
  if (!initok) return;
  if( lastPI.lat == posInfo.lat && lastPI.lon == posInfo.lon) return;

  SPI_MUTEX_LOCK();
  File posfile = SD.open("/mypos.csv", FILE_APPEND);
  if(!posfile) {
    LOG_E(TAG, "Error opening /mypos.csv\n");
    return;
  }
  if (posfile.size() == 0)
    posfile.print("timestamp,lat,lon\n");
  char ftim[50];
  struct tm timeinfo;
  time_t now;
  time(&now);
  gmtime_r(&now, &timeinfo);
  strftime(ftim, 50, "%Y-%m-%dT%H:%M:%SZ", &timeinfo);

  posfile.printf("%s,%.6f,%.6f\n", ftim, posInfo.lat, posInfo.lon);
  posfile.close();
  SPI_MUTEX_UNLOCK();
  lastPI = posInfo;
}

String ConnSDCard::getStatus() {
  if(sonde.config.sd.cs == -1) { return String("disabled"); }
  if(!initok) { return String("SD card init failed"); }

  SPI_MUTEX_LOCK();
  uint8_t cardType = SD.cardType();
  if (cardType == CARD_NONE) { return String(cardTypeStr(cardType)); }

  uint32_t cardSize = SD.cardSize() / (1024 * 1024);
  uint32_t usedSize = SD.usedBytes() / (1024 * 1024);
  uint32_t totalSize = SD.totalBytes() / (1024 * 1024);
  SPI_MUTEX_UNLOCK();
  char buf[256];
  snprintf(buf, 256, "SD card type: %s [size: %lu MB]. File system: %lu / %lu MB used", cardTypeStr(cardType),
    cardSize, usedSize, totalSize);
  strlcat(buf, "; <a href=\\\"/sdfiles.html\\\">SD-Card content</a>", 256);
  return String(buf);
}

String ConnSDCard::getName() {
  return String("SD Card");
}


ConnSDCard connSDCard;

#endif
