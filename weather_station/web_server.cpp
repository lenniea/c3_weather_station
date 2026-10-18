#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WebServer.h>
#include <time.h>
#include <LittleFS.h>
#include "config.h"
#include "web_server.h"
#include "api_help_page.h"
#include "upload_page.h"

extern bool gSdOk;

// Web
WebServer server(80);

// ------------------- WEB UI + API -------------------

void handleRoot() {
  if (gSdOk && LittleFS.exists("/web/index.html")) {
    File f = LittleFS.open("/web/index.html", FILE_READ);
    if (f) {
      server.streamFile(f, "text/html");
      f.close();
      return;
    }
  }

  const char* fallbackHtml = R"HTML(
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8"/>
  <meta name="viewport" content="width=device-width, initial-scale=1"/>
  <title>Weather Station - Setup Required</title>
  <style>
    body { font-family: system-ui, sans-serif; margin: 20px; max-width: 600px; }
    h1 { color: #333; }
    a { color: #0275d8; text-decoration: none; }
    a:hover { text-decoration: underline; }
    ul { line-height: 1.8; }
  </style>
</head>
<body>
  <h1>Weather Station - Setup Required</h1>
  <p>Web interface files not found on SD card.</p>
  <ul>
    <li><a href="/upload">Upload UI files (index.html, app.js)</a></li>
    <li><a href="/api/now">View current sensor data (JSON)</a></li>
    <li><a href="/api/buckets">View bucket data (JSON)</a></li>
    <li><a href="/api/days">View daily summaries (JSON)</a></li>
    <li><a href="/api/config">View configuration (JSON)</a></li>
    <li><a href="/api_help">API documentation</a></li>
  </ul>
</body>
</html>
)HTML";

  server.send(200, "text/html", fallbackHtml);
}

void handleApiHelp() {
  server.send_P(200, "text/html", API_HELP_HTML);
}

void handleRootJs() {
  if (gSdOk && LittleFS.exists("/web/app.js")) {
    File f = LittleFS.open("/web/app.js", FILE_READ);
    if (f) {
      server.streamFile(f, "application/javascript");
      f.close();
      return;
    }
  }

  server.send(200, "application/javascript", "console.log('app.js not found on SD card');");
}

void handleApiNow() {
  String out = buildApiNow();
  server.send(200, "application/json", out);
}

void handleApiDays() {
  String out = buildApiDays();
  server.send(200, "application/json", out);
}

void handleApiConfig() {
  String out = buildApiConfig();
  server.send(200, "application/json", out);
}

#define COUNT_OF(a)   (sizeof(a)/sizeof(a[0]))

void handleApiUiFiles() {
  if (!gSdOk) {
    server.send(503, "application/json", "{\"ok\":false,\"error\":\"sd_not_available\"}");
    return;
  }

  String out = "{\"ok\":true,\"files\":[";
  bool first = true;

  const char* webFiles[] = {
    "/web/index.html",
    "/web/app.js",
    "/web/app_base.js",
    "/web/app_base_clean.js"
  };
  for (int i = 0; i < COUNT_OF(webFiles); i++) {
    const char* path = webFiles[i];
    if (LittleFS.exists(path)) {
      File f = LittleFS.open(path, FILE_READ);
      if (f) {
        if (!first) out += ",";
        first = false;
        out += "{";
        out += "\"path\":\"" + String(path) + "\",";
        out += "\"size\":" + String((uint32_t)f.size()) + ",";
        out += "\"lastModified\":" + String((uint32_t)f.getLastWrite());
        out += "}";
        f.close();
      }
    }
  }

  out += "]}";
  server.send(200, "application/json", out);
}

void handleUploadPage() {
  server.send_P(200, "text/html", UPLOAD_HTML);
}

bool initWebServer(void)
{
  // routes
  server.on("/", handleRoot);
  server.on("/root.js", handleRootJs);
  server.on("/api/now", handleApiNow);
  server.on("/api/buckets", handleApiBuckets);
  server.on("/api/buckets_compact", handleApiBucketsCompact);  // Compact format for internal UI
  server.on("/api/days", handleApiDays);
  server.on("/api/config", handleApiConfig);
  server.on("/api/ui_files", handleApiUiFiles);
  server.on("/api_help", handleApiHelp);
  server.on("/api/clear_data", HTTP_POST, handleApiClearData);
  server.on("/api/delete", HTTP_POST, handleApiDelete);
  server.on("/api/reboot", HTTP_POST, handleApiReboot);

  server.on("/api/files", handleApiFiles);
  server.on("/download", handleDownload);
  server.on("/download_zip", handleDownloadZip);
  server.on("/upload", HTTP_GET, handleUploadPage);
  server.on("/upload", HTTP_POST, handleUploadComplete, handleUploadData);

  server.begin();
  return true;
}
