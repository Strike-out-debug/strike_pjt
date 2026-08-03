#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

#define DEVICE_NAME "KV260 Trigger"

#define SERVICE_UUID      "12345678-1234-1234-1234-1234567890ab"
#define COMMAND_UUID      "abcdefab-1234-5678-1234-abcdefabcdef"

class MyCallbacks : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *pCharacteristic) override
    {
        String value = pCharacteristic->getValue();

        if (value.length() != 1)
            return;

        uint8_t cmd = (uint8_t)value[0];

        // ★ KV260へは1バイトだけ送る
        Serial.write(&cmd, 1);
        Serial.flush();
    }
};

void setup()
{
    Serial.begin(115200);

    BLEDevice::init(DEVICE_NAME);

    BLEServer *server = BLEDevice::createServer();

    BLEService *service =
        server->createService(SERVICE_UUID);

    BLECharacteristic *ch =
        service->createCharacteristic(
            COMMAND_UUID,
            BLECharacteristic::PROPERTY_WRITE
        );

    ch->setCallbacks(new MyCallbacks());

    service->start();

    BLEAdvertising *adv = BLEDevice::getAdvertising();
    adv->addServiceUUID(SERVICE_UUID);
    adv->start();
}

void loop()
{
    delay(10);
}