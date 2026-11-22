#include "../features.h"

#if FEATURE_SDCARD

#include "conn-sdcard.h"

#define TAG "conn-sdcard"
#include "logger.h"

#include "posinfo.h"
#include "Sonde.h"

#include <errno.h>
#include <dirent.h>
#include <time.h>
#include <sys/stat.h>
#include <stdio.h>
#include <string.h>
#include <utime.h>
#include <unistd.h>

#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "sd_protocol_defs.h"

extern SemaphoreHandle_t globalLock;
#define SPI_MUTEX_LOCK() \
        do                     \
{                      \
} while (xSemaphoreTake(globalLock, portMAX_DELAY) != pdPASS)
#define SPI_MUTEX_UNLOCK() xSemaphoreGive(globalLock)

static sdmmc_card_t *s_card = nullptr;

enum {
  CARD_KIND_NONE = 0,
  CARD_KIND_MMC,
  CARD_KIND_SDSC,
  CARD_KIND_SDHC,
};

static const char *cardTypeStr(int kind) {
  switch (kind) {
     case CARD_KIND_NONE: return "No SD card attached";
     case CARD_KIND_MMC: return "MMC";
     case CARD_KIND_SDSC: return "SDSC";
     case CARD_KIND_SDHC: return "SDHC";
     default: return "unknown";
  }
}

static int cardKind(const sdmmc_card_t *card) {
  if (!card) return CARD_KIND_NONE;
  if (card->is_mmc) return CARD_KIND_MMC;
  if (card->ocr & SD_OCR_SDHC_CAP) return CARD_KIND_SDHC;
  return CARD_KIND_SDSC;
}

static uint32_t speed_hz(void) {
  return (sonde.config.sd.speed > 0) ? (uint32_t)sonde.config.sd.speed : 4000000u;
}

/* Dedicated SPI2 bus for SD when TFT did not already initialize it. */
static esp_err_t ensure_sd_spi_bus(void) {
  spi_bus_config_t buscfg = {};
  buscfg.mosi_io_num = sonde.config.sd.mosi;
  buscfg.miso_io_num = sonde.config.sd.miso;
  buscfg.sclk_io_num = sonde.config.sd.clk;
  buscfg.quadwp_io_num = -1;
  buscfg.quadhd_io_num = -1;
  buscfg.max_transfer_sz = 16 * 1024;

  esp_err_t err = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
  if (err == ESP_OK || err == ESP_ERR_INVALID_STATE) {
    /* ESP_ERR_INVALID_STATE: shared TFT (Arduino_ESP32SPIDMA) already initialized SPI2. */
    return ESP_OK;
  }
  return err;
}

static bool mount_sd(bool format_if_empty) {
  if (s_card) {
    return true;
  }

  /* TFT on HSPI with different pins remuxes SPI2; refuse that combo. */
  if (!ISOLED(sonde.config) &&
      sonde.config.oled_scl >= 0 &&
      sonde.config.sd.clk >= 0 &&
      sonde.config.oled_scl != sonde.config.sd.clk) {
    LOG_E(TAG, "SD and TFT both use SPI2 with different clocks (sd.clk=%d oled_scl=%d); refusing SD mount\n",
          sonde.config.sd.clk, sonde.config.oled_scl);
    return false;
  }

  esp_err_t err = ensure_sd_spi_bus();
  if (err != ESP_OK) {
    LOG_E(TAG, "spi_bus_initialize(SPI2) failed: %s\n", esp_err_to_name(err));
    return false;
  }

  sdmmc_host_t host = SDSPI_HOST_DEFAULT();
  host.slot = SPI2_HOST;
  host.max_freq_khz = speed_hz() / 1000;

  sdspi_device_config_t slot = SDSPI_DEVICE_CONFIG_DEFAULT();
  slot.host_id = SPI2_HOST;
  slot.gpio_cs = (gpio_num_t)sonde.config.sd.cs;

  esp_vfs_fat_sdmmc_mount_config_t mnt = {
      .format_if_mount_failed = format_if_empty,
      .max_files = 5,
      .allocation_unit_size = 16 * 1024,
      .disk_status_check_enable = false,
      .use_one_fat = false,
  };

  err = esp_vfs_fat_sdspi_mount("/sd", &host, &slot, &mnt, &s_card);
  if (err != ESP_OK) {
    LOG_E(TAG, "esp_vfs_fat_sdspi_mount failed: %s\n", esp_err_to_name(err));
    s_card = nullptr;
    return false;
  }
  return true;
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
    char dirVfs[32];
    snprintf(dirVfs, sizeof(dirVfs), "/sd%s", yymm);
    if (mkdir(dirVfs, 0755) != 0 && errno != EEXIST) continue;

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
  /* SPI2 / HSPI: SX127x uses VSPI. SD uses SPI2 (same host as shared TFT when pins match). */
  initok = mount_sd(false) ? 1 : 0;
  LOG_I(TAG, "SD card init: %s (SPI %lu Hz)\n", initok ? "OK" : "Failed", (unsigned long)speed_hz());

  int kind = cardKind(s_card);
  LOG_I(TAG, "SD Card Type: %s\n", cardTypeStr(kind));
  if (kind == CARD_KIND_NONE) {
    SPI_MUTEX_UNLOCK();
    return;
  }

  uint64_t cardBytes = (uint64_t)s_card->csd.capacity * s_card->csd.sector_size;
  LOG_I(TAG, "SD Card Size: %luMB\n", (unsigned long)(cardBytes / (1024 * 1024)));
  uint64_t total = 0, freeb = 0;
  if (esp_vfs_fat_info("/sd", &total, &freeb) == ESP_OK) {
    LOG_I(TAG, "SD Card used/total: %lu/%lu MB\n",
          (unsigned long)((total - freeb) / (1024 * 1024)),
          (unsigned long)(total / (1024 * 1024)));
  }
  SPI_MUTEX_UNLOCK();

  if (initok && sonde.config.sd.name == 1)
    reorganizeCsvRoot();
}

int ConnSDCard::format() {
    if(sonde.config.sd.clk==-1)
        return -2;  // SD card disabled in config

    SPI_MUTEX_LOCK();
    int retval = 0;
    if (datafile) {
      fclose(datafile);
      datafile = nullptr;
    }

    if (!initok || !s_card) {
        /* Mount failed earlier: remount and format empty/unreadable media. */
        initok = mount_sd(true) ? 1 : 0;
        if (!initok) retval = -1;
    } else {
        esp_err_t err = esp_vfs_fat_sdcard_format("/sd", s_card);
        if (err != ESP_OK) {
          LOG_E(TAG, "esp_vfs_fat_sdcard_format failed: %s\n", esp_err_to_name(err));
          retval = -1;
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
    snprintf(logName, sizeof(logName), "/sd/noid.csv");
  } else {
    if (sonde.config.sd.name == 0) {
      snprintf(logName, sizeof(logName), "/sd/%s.csv", sd->ser);
    } else {
      snprintf(logName, sizeof(logName), "/sd/%02d%02d/%s.csv", yy, mm, sd->ser);
    }
  }

  SPI_MUTEX_LOCK();
  if (strcmp(logName, logOldName) || !datafile) {
    if (datafile) {
      fclose(datafile);
      datafile = nullptr;
    }
    if (sd->validID && sonde.config.sd.name == 1) {
      char vfsFolder[32];
      snprintf(vfsFolder, sizeof(vfsFolder), "/sd/%02d%02d", yy, mm);
      if (mkdir(vfsFolder, 0755) != 0 && errno != EEXIST)
        LOG_E(TAG, "mkdir %s failed: %s\n", vfsFolder, strerror(errno));
    }
    datafile = fopen(logName, "a");
    strcpy(logOldName, logName);
    LOG_I(TAG, "Logging to file %s\n", logName);
  }
  if (!datafile) {
    SPI_MUTEX_UNLOCK();
    LOG_E(TAG, "Error opening file %s\n", logName);
    return;
  }
  /* Same fallback as display / last_rx: many decoders leave typestr empty (RS41/RS92/MP3H). */
  const char *type = sd->typestr[0] ? sd->typestr : sondeTypeStr[sonde.realType(si)];
  if (ftell(datafile) == 0)
    fprintf(datafile, "validID,ser,typestr,subtype,lat,lon,alt,vs,hs,dir,sats,validPos,time,frame,vframe,validTime\n");
  fprintf(datafile, "%d,%s,%s,%d,"
          "%f,%f,%f,%f,%f,%f,%d,%d,"
          "%d,%d,%d,%d\n",
          sd->validID, sd->ser, type, sd->subtype,
          sd->lat, sd->lon, sd->alt, sd->vs, sd->hs, sd->dir, sd->sats, sd->validPos,
          sd->time, sd->frame, sd->vframe, sd->validTime);

  // TODO: Make this time based, not invocation based (well, should be the same, this is called
  // 1x per second)
  wcount++;
  if (wcount >= sonde.config.sd.sync) {
    fflush(datafile);
    fsync(fileno(datafile));
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
  FILE *posfile = fopen("/sd/mypos.csv", "a");
  if (!posfile) {
    LOG_E(TAG, "Error opening /mypos.csv\n");
    SPI_MUTEX_UNLOCK();
    return;
  }
  if (ftell(posfile) == 0)
    fprintf(posfile, "timestamp,lat,lon\n");
  char ftim[50];
  struct tm timeinfo;
  time_t now;
  time(&now);
  gmtime_r(&now, &timeinfo);
  strftime(ftim, 50, "%Y-%m-%dT%H:%M:%SZ", &timeinfo);

  fprintf(posfile, "%s,%.6f,%.6f\n", ftim, posInfo.lat, posInfo.lon);
  fclose(posfile);
  SPI_MUTEX_UNLOCK();
  lastPI = posInfo;
}

String ConnSDCard::getStatus() {
  char buf[256];
  if(sonde.config.sd.cs == -1) { strncpy(buf, "SD card disabled", 256); }
  else if(!initok || !s_card) { strncpy(buf, "SD card init failed", 256); }
  else {
	  SPI_MUTEX_LOCK();
	  int kind = cardKind(s_card);
	  if (kind == CARD_KIND_NONE) {
	    SPI_MUTEX_UNLOCK();
	    return String(cardTypeStr(kind));
	  }

	  uint64_t cardBytes = (uint64_t)s_card->csd.capacity * s_card->csd.sector_size;
	  uint64_t total = 0, freeb = 0;
	  esp_vfs_fat_info("/sd", &total, &freeb);
	  SPI_MUTEX_UNLOCK();
	  uint32_t cardSize = (uint32_t)(cardBytes / (1024 * 1024));
	  uint32_t usedSize = (uint32_t)((total - freeb) / (1024 * 1024));
	  uint32_t totalSize = (uint32_t)(total / (1024 * 1024));
	  snprintf(buf, 256, "SD card type: %s [size: %lu MB]. File system: %lu / %lu MB used", cardTypeStr(kind),
	    (unsigned long)cardSize, (unsigned long)usedSize, (unsigned long)totalSize);
	  strlcat(buf, "<br><a href=\\\"/sdfiles.html\\\">SD-Card content</a>", 256);
  }
  strlcat(buf, "<br><a href=\\\"/sdfiles.html?dir=.int\\\">LittleFS content</a>", 256);
  return String(buf);
}

String ConnSDCard::getName() {
  return String("Storage");
}


ConnSDCard connSDCard;

#endif
