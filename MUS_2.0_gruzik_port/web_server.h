#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include <Arduino.h>

void WebServer_Init();
void WebServer_BroadcastTelemetry();
void WebServer_BroadcastParams();

#endif // WEB_SERVER_H
