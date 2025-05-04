
#define COOKIE_SIZE 26

enum { PERM_NONE, PERM_RO, PERM_ADMIN } permissions;

void cleanupExpiredCookies();
void storeCookie(const char *cookie, char userclass);
int upgradeCookie(const char *preauth, const char *cookie, char userclass);
void generateRandomCookie(const char *user, char *cookie);
int getCookieAuthLevel(const char *cookie);
int getUserPermissions(const char *user, const char *preauth, const char *auth);
int getDefaultAuthLevel();
