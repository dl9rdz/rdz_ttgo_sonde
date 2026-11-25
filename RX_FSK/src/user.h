#pragma once

#include <stdint.h>

#define COOKIE_SIZE 26

/* Permission levels (user.txt "user,level,password"; level is the digit). Ordered:
 * a request needing level L is allowed for any level >= L. */
enum Perm { PERM_NONE = 0, PERM_RO = 1, PERM_ADMIN = 2 };

/* userclass of a one-time login token (not a login).
 * Must be int8_t: plain char is unsigned on ESP32 (Xtensa), so -1 would become 255. */
#define USERCLASS_PREAUTH ((int8_t)-1)

void cleanupExpiredCookies();
/* Store a preauth token; never evicts a live session.
 * Returns 0 on success, else seconds until a slot frees up (all slots hold live sessions). */
int storePreauth(const char *preauth);
void revokeCookie(const char *cookie);
int upgradeCookie(const char *preauth, const char *cookie, int8_t userclass);
void generateRandomCookie(const char *user, char *cookie);
int getCookieAuthLevel(const char *cookie);
int getUserPermissions(const char *user, const char *preauth, const char *auth);
int getDefaultAuthLevel();
