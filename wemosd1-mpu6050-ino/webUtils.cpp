#include <ESP8266WiFi.h>
#include "WebSocketsServer.h"
#include <ESP8266WebServer.h>
// #include <ESP8266mDNS.h>
#include <Hash.h>
#include <FS.h>
#include "webUtils.h"

#define HTTP_PORT 8080
#define WS_PORT 8181

#ifdef WEB_WIFI_CLIENT
#include <ESP8266WiFiMulti.h>
ESP8266WiFiMulti WiFiMulti;
#endif

ESP8266WebServer server(HTTP_PORT);
WebSocketsServer webSocket = WebSocketsServer(WS_PORT);

static uint8_t cNum = 0xFF;

void ws_send_txt(String _str) {
    if(cNum != 0xFF) {
        webSocket.sendTXT(cNum, _str);
    }
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
    switch(type) {
        case WStype_DISCONNECTED:
            Serial.printf("[%u] Disconnected!\n", num);
            cNum = 0xFF;
            break;
        case WStype_CONNECTED: {
            IPAddress ip = webSocket.remoteIP(num);
            Serial.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);

            // send message to client
            webSocket.sendTXT(num, "{\"msg\":\"Connected\"}");
            cNum = num;
        }
            break;
        case WStype_TEXT:
            Serial.printf("[%u] get Text: %s\n", num, payload);

            if(payload[0] == '#') {
                // decode rgb data
                uint32_t rgb = (uint32_t) strtol((const char *) &payload[1], NULL, 16);
            }
            break;
    }
}

bool fileReadHandle(String _path) {
    if (SPIFFS.exists(_path)) {
		File file = SPIFFS.open(_path, "r");
		size_t sent = server.streamFile(file, "text/plain");
		file.close();
		return true;
	} else {
        return false;
    }
}

void web_init(const char *_APID, const char *_APPASS) {
    SPIFFS.begin();

#ifdef WEB_WIFI_CLIENT
    WiFiMulti.addAP(_APID, _APPASS);

    Serial.print(F("\r\nConnecting to Wifi: ")); Serial.println(_APID);
    while(WiFiMulti.run() != WL_CONNECTED) {
        Serial.print(F("."));
        delay(100);
    }
    Serial.println(F("Wifi Connected!"));
    IPAddress myIP = WiFi.localIP();

#else
    /* You can remove the password parameter if you want the AP to be open. */
    WiFi.softAP(_APID, _APPASS);
    IPAddress myIP = WiFi.softAPIP();
    Serial.print("\r\nStart access point...");
#endif
    Serial.print("IP address: ");
    Serial.println(myIP);
    Serial.println();

    // start webSocket server
    // webSocket = WebSocketsServer(_WS_PORT);

    Serial.println(F("Starting Websocket Server!"));
    webSocket.begin();
    webSocket.onEvent(webSocketEvent);

    Serial.println(F("Starting Http server!"));
    // handle index
    server.on("/", []() {
        // send index.html
        File f = SPIFFS.open(F("/index.html"), "r");
        if(!f) Serial.println(F("File index.html failed"));
        else server.send(200, "text/html", f.readString());
    });

    server.onNotFound( []() {
		if (!fileReadHandle(server.uri()))
			server.send(404, "text/plain", "FileNotFound");
	});

    server.begin(HTTP_PORT);

    // Serial.println(F("Starting MDNS Service!"));
    // Add service to MDNS
    // MDNS.addService("http", "tcp", HTTP_PORT);
    // MDNS.addService("ws", "tcp", WS_PORT);

    IPAddress ip = WiFi.localIP();
    Serial.println("\r\nHTTP Address: " + String(ip[0]) + "." + String(ip[1]) + "." + String(ip[2])
                  + "." + String(ip[3]) + ":" + String(HTTP_PORT) + "\r\n");

    // if(MDNS.begin("esp8266")) {
    //     Serial.println("MDNS responder started");
    // }
}

void web_loop(void) {
    webSocket.loop();
    server.handleClient();
}
