#include "user.h"

#define TAG "user"
#include "logger.h"
#include <LittleFS.h>

#include <mbedtls/md.h>
#include <esp_random.h>

// User management function (simple web authentication)
// As secure as it can be with just http:
// Login form creates a random preauth ticket (nonce)
// Client-side form calculates SHA256(username:preauth:password) as authenticator
// Upon receiving correct authenticator, TTGO generates session ID and replaces preauth ticket with session ID in internal cookie store
// (i.e. preauth ticket can be used only once)
// TODO (optional) restrict session ID to specific client ID
// Session expiry is sliding: COOKIE_EXPIRY_DURATION after the last authenticated use.

#define USERLEN 8
#define RNDLEN 16
//#define COOKIE_SIZE (8+16+2)

struct SessionCookie {
  char value[USERLEN+RNDLEN+2];
  int8_t userclass;  // USERCLASS_PREAUTH (-1) for a preauth token, otherwise a Perm value
  unsigned long expiry;
};

const int MAX_SESSIONS = 4;
SessionCookie authCookies[MAX_SESSIONS];
int cookieCount = 0;

const unsigned long COOKIE_EXPIRY_DURATION = 30 * 60 * 1000; // 30 minutes in milliseconds
const unsigned long PREAUTH_EXPIRY_DURATION = 60 * 1000;  // 1 minute in milliseconds

static const char *getUser(const char *user, char *line, int maxlen, int *level);

// true while 'expiry' (a millis() timestamp) lies in the future.
// Signed difference stays correct across the 49.7-day millis() wrap
// (valid for lifetimes below ~24 days).
static bool notExpired(unsigned long expiry, unsigned long now) {
  return (int32_t)(uint32_t)(expiry - now) > 0;
}

// Remove entry i, keeping the remaining entries in insertion order (oldest first).
static void removeAt(int i) {
  for (int j = i; j < cookieCount - 1; j++) {
    authCookies[j] = authCookies[j + 1];
  }
  cookieCount--;
}

void cleanupExpiredCookies() {
  unsigned long now = millis();
  int i = 0;
  while (i < cookieCount) {
    if (!notExpired(authCookies[i].expiry, now)) {
      removeAt(i);
    } else {
      i++;
    }
  }
}

void revokeCookie(const char *cookie) {
  if (!cookie || !cookie[0]) {
    return;
  }
  for (int i = 0; i < cookieCount; i++) {
    if (strcmp(authCookies[i].value, cookie) == 0) {
      removeAt(i);
      return;
    }
  }
}

// Store a new preauth token. Preauth and sessions share the table, but a preauth
// never evicts a live session (/preauth is unauthenticated):
//   1. expired entries are dropped,
//   2. if still full, the oldest preauth token is evicted,
//   3. if all slots hold live sessions, nothing is stored.
// Sessions expire after COOKIE_EXPIRY_DURATION, so case 3 resolves by itself.
// Returns 0 on success, otherwise the seconds until the oldest session expires (>= 1).
int storePreauth(const char *preauth) {
  cleanupExpiredCookies();
  if (cookieCount >= MAX_SESSIONS) {
    int victim = -1;
    for (int i = 0; i < cookieCount; i++) {
      // Entries are in insertion order and upgraded in place, so the first preauth is the oldest.
      if (authCookies[i].userclass == USERCLASS_PREAUTH) { victim = i; break; }
    }
    if (victim < 0) {
      unsigned long now = millis();
      uint32_t wait = COOKIE_EXPIRY_DURATION;
      for (int i = 0; i < cookieCount; i++) {
        uint32_t left = (uint32_t)(authCookies[i].expiry - now);
        if (left < wait) wait = left;
      }
      LOG_W(TAG, "All %d login slots hold live sessions; refusing new preauth\n", MAX_SESSIONS);
      return (int)(wait / 1000) + 1;
    }
    removeAt(victim);
  }
  strlcpy(authCookies[cookieCount].value, preauth, sizeof(authCookies[cookieCount].value));
  authCookies[cookieCount].userclass = USERCLASS_PREAUTH;
  authCookies[cookieCount].expiry = millis() + PREAUTH_EXPIRY_DURATION;
  cookieCount++;
  return 0;
}

// Replace a valid (unexpired) preauth token by a session cookie. Only preauth entries
// can be upgraded; the session lifetime starts now.
int upgradeCookie(const char *preauth, const char *cookie, int8_t userclass) {
  unsigned long now = millis();
  for (int i = 0; i < cookieCount; i++) {
    if (strcmp(preauth, authCookies[i].value)==0) {
      if (authCookies[i].userclass != USERCLASS_PREAUTH || !notExpired(authCookies[i].expiry, now)) {
        return -1;
      }
      strlcpy(authCookies[i].value, cookie, sizeof(authCookies[i].value));
      authCookies[i].userclass = userclass;
      authCookies[i].expiry = now + COOKIE_EXPIRY_DURATION;
      return 0;
    }
  }
  return -1;
}

char allcn[] = "0123456789abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ";

// Function to generate a random session cookie
void generateRandomCookie(const char *user, char *cookie) {
  int j = 0;
  for (int i = 0; i < USERLEN && user[i]; i++, j++) {
    cookie[j] = user[i];
  }
  cookie[j++] = ':';
  for (int i = 0; i < RNDLEN; ++i) {
    // hardware RNG; modulo bias for 62 symbols out of 2^32 is negligible
    cookie[j++] = allcn[esp_random() % (sizeof(allcn)-1)];  // sizeof includes \0
  }
  cookie[j++] = '\0';
}

// -1: invalid/expired cookie or preauth token; otherwise a Perm value
int getCookieAuthLevel(const char *cookie) {
  unsigned long now = millis();
  for (int i = 0; i < cookieCount; i++) { 
    if (strcmp(authCookies[i].value, cookie) == 0) {
      if (notExpired(authCookies[i].expiry, now)) {
        // A preauth token is a login nonce, never a login.
        if (authCookies[i].userclass == USERCLASS_PREAUTH) return -1;
        // Sliding expiry: a session ends COOKIE_EXPIRY_DURATION after its last use.
        // Only protected requests and page loads get here, not the open polling endpoints.
        authCookies[i].expiry = now + COOKIE_EXPIRY_DURATION;
        return authCookies[i].userclass; // Valid and not expired
      } else {
        removeAt(i);  // expired
        break;
      }
    }
  }
  return -1;
}


// -1: user does not exist or wrong password; otherwise a Perm value
int getUserPermissions(const char *user, const char *preauth, const char *auth) {
    // simple digest authentication:
    // (we want to avoid sending plain text passwords via http)
    // digest is SHA256(user:preauth:password), sent as 64 lowercase hex digits
    if (strlen(auth) != 64) return -1;
    char buf[256];
    char line[128];
    int level;
    const char *pass = getUser(user, line, sizeof(line), &level);
    if (!pass) {  // user not found
      return -1;
    }
    // strlcat bounds everything; an oversized input only yields a wrong digest
    // (preauth validity is checked later by upgradeCookie())
    strlcpy(buf, user, sizeof(buf));
    strlcat(buf, ":", sizeof(buf));
    strlcat(buf, preauth, sizeof(buf));
    strlcat(buf, ":", sizeof(buf));
    strlcat(buf, pass, sizeof(buf));
    unsigned char sharesult[32];
    mbedtls_md_context_t ctx;
    mbedtls_md_init(&ctx);
    mbedtls_md_setup(&ctx, mbedtls_md_info_from_type(MBEDTLS_MD_SHA256), 0);
    mbedtls_md_starts(&ctx);
    mbedtls_md_update(&ctx, (const unsigned char *) buf, strlen(buf));
    mbedtls_md_finish(&ctx, sharesult);
    mbedtls_md_free(&ctx);
    // constant-time compare: no early exit on the first mismatching digit
    static const char hexdig[] = "0123456789abcdef";
    unsigned char diff = 0;
    for (int i = 0; i < 32; i++) {
      diff |= auth[2 * i] ^ hexdig[sharesult[i] >> 4];
      diff |= auth[2 * i + 1] ^ hexdig[sharesult[i] & 0x0f];
    }
    int authres = (diff == 0) ? level : -1;
    LOG_I(TAG, "login: match: %d => auth level %d\n", diff == 0, authres);
    return authres;
}

// Find user intry in password file
extern int readLine(Stream &stream, char *buffer, int maxlen);  // impl in RX_FSK.ino

// Parse "user,level,password" in place: level is a single digit 0..PERM_ADMIN, the password
// is the rest of the line (it may contain ','). Returns false for a malformed line.
static bool parseUserLine(char *line, int *level, const char **pass) {
  char *c1 = strchr(line, ',');
  if (!c1) return false;
  char *c2 = strchr(c1 + 1, ',');
  if (!c2 || c2 != c1 + 2 || c1[1] < '0' || c1[1] > '0' + PERM_ADMIN) return false;
  *c1 = 0;
  *level = c1[1] - '0';
  *pass = c2 + 1;
  return true;
}

// Returns the password of 'user' (pointing into line) and its level, or NULL if not found.
static const char *getUser(const char *user, char *line, int maxlen, int *level) {
  File file = LittleFS.open("/user.txt", "r");
  if (!file) {
    LOG_E(TAG, "Error opening '/user.txt'\n");
    line[0] = 0;
    *level = PERM_ADMIN;  // all permissions for unauthenticated user by default for now...
    return line;
  }
  while (file.available()) {
    int n = readLine(file, line, maxlen);
    if (n >= maxlen - 1) {
      // overlong line: drop it, and its remainder (never parse it as an entry of its own)
      while (file.available() && file.read() != '\n') {}
      LOG_W(TAG, "user.txt: line too long, ignored\n");
      continue;
    }
    if (line[0] == '#' || line[0] == 0) continue;
    const char *pass;
    if (!parseUserLine(line, level, &pass)) {
      LOG_W(TAG, "user.txt: malformed line ignored (expected user,level,password)\n");
      continue;
    }
    if (strcmp(user, line) == 0) {
      LOG_D(TAG, "Found pw entry with user '%s'\n", user);
      return pass;
    }
  }
  return NULL;
}

int getDefaultAuthLevel() {
  char line[128];
  int level;
  if (!getUser("", line, sizeof(line), &level)) return PERM_ADMIN;
  return level;
}
