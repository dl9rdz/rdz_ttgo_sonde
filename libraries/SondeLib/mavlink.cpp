#include <WString.h>
#include <inttypes.h>

#include "mavlink.h"

#define MAV_HEARTBEAT 0
#define MAV_GPS_RAW_INT 24
#define MAV_ATTITUDE 30
#define MAV_VFR_HUD 74

#define X25_INIT_CRC 0xffff

void fmav_crc_accumulate(uint16_t *crc, uint8_t data) {
  uint8_t tmp = data ^ (uint8_t)(*crc & 0xff);
  tmp ^= (tmp << 4);
  *crc = (*crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4);
}

uint16_t fmav_crc_calculate(const uint8_t *buf, uint16_t len) {
  uint16_t crc = X25_INIT_CRC;
  while (len--) {
    fmav_crc_accumulate(&crc, *buf++);
  }
  return crc;
}

struct MavlinkPackage {
  static constexpr size_t MAX_PAYLOAD_LENGTH = 255;
  uint8_t header = 0xfe;
  uint8_t packageLength;
  uint8_t sequenceNumber = 0;
  uint8_t sourceSystem = 2;
  uint8_t sourceComponent = 1;
  uint8_t messageType;
  // also contains the checksum at the end
  uint8_t payload[MAX_PAYLOAD_LENGTH]; // MAX_MAVLINK_PAYLOAD_LENGTH
};

uint8_t getExtraCrc(uint8_t messageType) {
  switch (messageType) {
  case MAV_HEARTBEAT:
    return 50;
  case MAV_ATTITUDE:
    return 39;
  case MAV_GPS_RAW_INT:
    return 24;
  case MAV_VFR_HUD:
    return 20;
  }
  return 0;
}

void addCrcToMavlink(MavlinkPackage *pkg) {
  uint16_t crc =
      fmav_crc_calculate(((uint8_t *)pkg) + 1, 6 + pkg->packageLength - 1);
  fmav_crc_accumulate(&crc, getExtraCrc(pkg->messageType));

  pkg->payload[pkg->packageLength + 1] = crc >> 8;
  pkg->payload[pkg->packageLength + 0] = crc;
}

struct __attribute__((__packed__)) mavlink_heartbeat { // 0
  uint32_t custom_mode = 0;
  uint8_t type = 8;
  uint8_t autopilot = 8;
  uint8_t base_mode = 192;
  uint8_t system_status = 0;
  uint8_t mavlink_version = 3;
};

struct __attribute__((__packed__)) mavlink_attitude { // 30
  uint32_t time_boot_ms = 0;
  float roll = 0;
  float pitch = 0;
  float yaw = 0;
  float rollspeed = 0;
  float pitchspeed = 0;
  float yawspeed = 0;
};

struct __attribute__((packed)) mavlink_gps_raw_int { // 24
  uint32_t time_us = 0;                              // 8
  uint32_t time_us2 = 0;                             // 8
  int32_t lat = 0;                                   // 13
  int32_t lon = 0;                                   // 17
  uint32_t alt = 0;                                  // 21
  uint16_t eph = 0;                                  // 23
  uint16_t epv = 0;                                  // 24
  uint16_t vel = 0;                                  // 26
  uint16_t cog = 0;                                  // 28
  uint8_t fix_type = 3;                              // 9
  uint8_t satellites_visible = 10;                   // 29
  uint32_t alt_elipsoid = 0;                         // 33
  uint32_t h_acc = 0;                                // 37
  uint32_t v_acc = 0;                                // 41
  uint32_t vel_acc = 0;                              // 45
  uint32_t hdg_acc = 0;                              // 49
  uint16_t yaw = 0;                                  // 51
};

struct __attribute__((__packed__)) mavlink_vfr_hud { // 74
  float airspeed;
  float groundspeed;
  float alt;
  float climb;
  int16_t heading;
  uint16_t throttle;
};

MavlinkPackage mavpkg;

uint8_t *mavlink_send_hearhbeat(SondeInfo *s) {
  mavlink_heartbeat hearthbeat;

  mavpkg.sourceSystem = s->freq * 2.0 - 400.0;
  mavpkg.messageType = MAV_HEARTBEAT;
  mavpkg.packageLength = sizeof(mavlink_heartbeat);
  memcpy(mavpkg.payload, &hearthbeat, sizeof(mavlink_heartbeat));
  addCrcToMavlink(&mavpkg);
  return (uint8_t *)&mavpkg;
}

uint8_t *mavlink_send_gps_set_raw(SondeInfo *s) {
  mavlink_gps_raw_int gps_raw_int;

  gps_raw_int.lat = int32_t(s->lat * 10e6);
  gps_raw_int.lon = int32_t(s->lon * 10e6);
  gps_raw_int.fix_type = 3;
  gps_raw_int.satellites_visible = s->sats;
  gps_raw_int.alt = s->alt * 1000;
  // gps_raw_int.vel = s->hs * 100;
  gps_raw_int.time_us = 0;
  // gps_raw_int.yaw = s->dir * 10;

  gps_raw_int.yaw = 0;

  mavpkg.messageType = MAV_GPS_RAW_INT;
  mavpkg.packageLength = sizeof(mavlink_gps_raw_int);
  memcpy(mavpkg.payload, &gps_raw_int, sizeof(mavlink_gps_raw_int));
  mavpkg.sequenceNumber++;
  addCrcToMavlink(&mavpkg);
  return (uint8_t *)&mavpkg;
}

uint8_t *mavlink_send_attitude(SondeInfo *s) {
  mavlink_attitude attitude;

  attitude.yaw = s->dir / 360.0 * 3.1415926 * 2.0;

  mavpkg.messageType = MAV_ATTITUDE;
  mavpkg.packageLength = sizeof(mavlink_attitude);
  memcpy(mavpkg.payload, &attitude, sizeof(mavlink_attitude));
  addCrcToMavlink(&mavpkg);
  return (uint8_t *)&mavpkg;
}

uint8_t *mavlink_send_vfr_hud(SondeInfo *s) {
  mavlink_vfr_hud vfr_hud;

  vfr_hud.groundspeed = s->hs;
  vfr_hud.climb = s->vs;
  vfr_hud.alt = s->alt;

  mavpkg.messageType = MAV_VFR_HUD;
  mavpkg.packageLength = sizeof(mavlink_vfr_hud);
  memcpy(mavpkg.payload, &vfr_hud, sizeof(mavlink_vfr_hud));
  mavpkg.sequenceNumber++;
  addCrcToMavlink(&mavpkg);
  return (uint8_t *)&mavpkg;
}
