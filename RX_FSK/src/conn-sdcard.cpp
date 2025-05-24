#include "../features.h"

#if FEATURE_SDCARD

#include "conn-sdcard.h"

#define TAG "conn-sdcard"
#include "logger.h"

#include "posinfo.h"

#include <ff.h>
#include <dirent.h>
#include <time.h>

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

void ConnSDCard::init() {
  if(sonde.config.sd.clk==-1)
    return;

  SPI_MUTEX_LOCK();
  /* Initialize SD card */
  // SPI (==VSPI) is used by SX127x. 
  // On LoRa32, SD-Card is on different pins, so cannot share VSPI
  // Use HSPI (if using a TFT with SPI, you have to make sure that the same pins are used for both (MISO/MOSI/CLK) 
  sdspi.begin(sonde.config.sd.clk, sonde.config.sd.miso, sonde.config.sd.mosi, sonde.config.sd.cs);
  initok = SD.begin(sonde.config.sd.cs, sdspi);
  LOG_I(TAG, "SD card init: %s\n", initok ? "OK" : "Failed");
  uint8_t cardType = SD.cardType();
  LOG_I(TAG, "SD Card Type: %s\n", cardTypeStr(cardType));
  if (cardType == CARD_NONE) { SPI_MUTEX_UNLOCK(); return; }

  uint32_t cardSize = SD.cardSize() / (1024 * 1024);
  LOG_I(TAG, "SD Card Size: %luMB\n", cardSize);
  uint32_t usedSize = SD.usedBytes() / (1024 * 1024);
  uint32_t totalSize = SD.totalBytes() / (1024 * 1024);
  LOG_I(TAG, "SD Card used/total: %lu/%lu MB\n", usedSize, totalSize);
  SPI_MUTEX_UNLOCK();

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
        initok = SD.begin(sonde.config.sd.cs, sdspi, 4000000, "/sd", 5, true);
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

// Rotation by time or by id.
// only by id for now.
static char logName[20], logOldName[20] = {0};

static void is_to_logname(char *name, int maxlen, SondeData *sd) {
  if(sd->validID) snprintf(name, maxlen, "/%s.csv", sd->ser);
  else strncpy(name, "/noid.csv", maxlen);
}

void ConnSDCard::updateSonde( SondeInfo *si ) {
  if (!initok) return;
  SondeData *sd = &si->d;
  is_to_logname(logName, 20, sd);

  SPI_MUTEX_LOCK();
  if(strcmp(logName, logOldName) || !file) {
    if(file) file.close();
    file = SD.open(logName, FILE_APPEND);
    strcpy(logOldName, logName);
    LOG_I(TAG, "Logging to file %s\n", logName);
  }
  if (!file) {
    LOG_E(TAG, "Error opening file %s\n", logName);
    return;
  }
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
