"""
    Serial.println(WiFi.localIP());
    server.on("/tank_drive", HTTP_POST, [](AsyncWebServerRequest* request)
        {
            float l = request->hasParam("l", true) ? request->getParam("l", true)->value().toFloat() : 0.0;
            float r = request->hasParam("r", true) ? request->getParam("r", true)->value().toFloat() : 0.0;

            dt.tankDrive(l, r);

            String message = "l: " + String(l) + ", r: " + String(r);

            request->send(200, "text/plain", "requested tank_drive:" + message);
        }
    );

    server.on("/arcade_drive", HTTP_POST, [](AsyncWebServerRequest* request)
        {
            float speed = request->hasParam("speed", true) ? request->getParam("speed", true)->value().toFloat() : 0.0;
            float rotation = request->hasParam("rotation", true) ? request->getParam("rotation", true)->value().toFloat() : 0.0;

            dt.arcadeDrive(speed, rotation);

            String message = "speed: " + String(speed) + ", rotation: " + String(rotation);

            request->send(200, "text/plain", "requested arcade_drive:" + message);
        }
    );

    server.onNotFound([](AsyncWebServerRequest* request) { request->send(404, "text/plain", "Not found"); });
    server.begin();
"""