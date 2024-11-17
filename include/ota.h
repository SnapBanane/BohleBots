#ifndef ESP_OTA_H
#define ESP_OTA_H

#include <WiFi.h>
#include <WebServer.h>

class ESPOta {
private:
    const char* ssid;
    const char* password;
    WebServer server;
    String receivedData;

public:
    ESPOta(const char* ssid, const char* password) 
        : ssid(ssid), password(password), server(80) {}

    void begin() {
        Serial.begin(115200);
        WiFi.begin(ssid, password);
        
        while (WiFi.status() != WL_CONNECTED) {
            delay(1000);
            Serial.println("Connecting to WiFi...");
        }
        
        Serial.println("Connected to WiFi");
        Serial.print("IP address: ");
        Serial.println(WiFi.localIP());

        server.on("/", [this]() { handleRoot(); });
        server.on("/send", [this]() { handleSend(); });
        server.on("/receive", [this]() { handleReceive(); });
        server.begin();
    }

    void handleClient() {
        server.handleClient();
    }

    void sendSerialData(String data) {
        receivedData = data;
    }

private:
    void handleRoot() {
        server.send(200, "text/html", "<html><body>"
                                      "<h1>ESPOta Control Panel</h1>"
                                      "<form action='/send' method='POST'>"
                                      "Send data to ESPOta: <input type='text' name='data'>"
                                      "<input type='submit' value='Send'>"
                                      "</form>"
                                      "<br><a href='/receive'>Receive data from ESPOta</a>"
                                      "</body></html>");
    }

    void handleSend() {
        if (server.hasArg("data")) {
            String data = server.arg("data");
            Serial.println("Received from client: " + data);
            server.send(200, "text/plain", "Data sent to ESPOta: " + data);
        }
    }

    void handleReceive() {
        server.send(200, "text/plain", "Data from ESPOta: " + receivedData);
    }
};

#endif
