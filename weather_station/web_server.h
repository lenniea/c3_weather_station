#ifndef __WEB_SERVER_H__
#define __WEB_SERVER_H__

#include <WebServer.h>

extern WebServer server;

// external handlers and string functions
String buildApiNow(void);
String buildApiDays(void);
String buildApiConfig(void);

// external/privileged handlers
void handleApiBuckets(void);
void handleApiBucketsCompact(void);
void handleApiFiles(void);
void handleApiClearData(void);
void handleApiDelete(void);
void handleApiReboot(void);
void handleDownload(void);
void handleDownloadZip(void);
void handleUploadData(void);
void handleUploadComplete(void);

bool initWebServer(void);

#endif // __WEB_SERVER_H__
