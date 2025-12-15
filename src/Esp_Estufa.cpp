#include <Arduino.h>
#include <DHT.h>
#include <WiFi.h>
#include <esp_now.h>

#define DHTPIN 14
#define DHTTYPE DHT11

DHT sensor(DHTPIN, DHTTYPE);

#define Sensor_Solo 5

#define PinSensor_NivelAgua 20
#define PinSensor_TestaAgua 21

#define MIN_VALUE_HUMIDADE_SOLO 0
int DESIRED_VALUE_HUMIDADE_SOLO = 75;
int CURRENT_HUMIDADE_SOLO;
#define MAX_VALUE_HUMIDADE_SOLO 3000

#define MIN_VALUE_HUMIDADE_AR 0
float CURRENT_HUMIDADE_AR;
#define MAX_VALUE_HUMIDADE_AR 100

#define MIN_VALUE_TEMP_AR 0
int DESIRED_VALUE_TEMP_AR = 25;
float CURRENT_TEMP_AR;
#define MAX_VALUE_TEMP_AR 40

bool ENOUGH_AGUA;

#define Lampada 3
#define BombaAgua 4

int lastTempCheck = millis();
int lastSoloCheck = millis();
//==============================================================================================================
// eps now

uint8_t display[] = {0xFC, 0x01, 0x2C, 0xF9, 0x03, 0x5C};

typedef struct
{
    float DESIRED_TEMP;
    int DESIRED_HUMI_SOLO;
} Message_Received;

typedef struct
{
    float CURRENT_TEMP_AR;
    int CURRENT_HUMI_AR;
    int CURRENT_HUMI_SOLO;
    bool EnoughAgua;
} Message_Sent;

Message_Received Data_Received;
Message_Sent Data_Send;
esp_now_peer_info_t peerInfo;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
}

void OnDataRecv(const esp_now_recv_info_t *esp_now_info, const uint8_t *incomingData, int len)
{
    memcpy(&Data_Received, incomingData, sizeof(Data_Received));

    DESIRED_VALUE_HUMIDADE_SOLO = Data_Received.DESIRED_HUMI_SOLO;
    DESIRED_VALUE_TEMP_AR = Data_Received.DESIRED_TEMP;

    lastSoloCheck = 0;
    lastTempCheck = 0;
}
//=========================================================================================================

int measureHumidadeSolo()
{
    int value = analogRead(Sensor_Solo);

    int resultado = map(value, MIN_VALUE_HUMIDADE_SOLO, MAX_VALUE_HUMIDADE_SOLO, 0, 100);

    if (resultado > 100)
    {
        resultado = 100;
    }
    else if (resultado < 0)
    {
        resultado = 0;
    }

    return resultado;
}

float measureTempAr()
{
    int resultado = map(CURRENT_TEMP_AR, MIN_VALUE_TEMP_AR, MAX_VALUE_TEMP_AR, 0, 100);

    return resultado;
}
int measureHumidadeAr()
{
    int resultado = map(CURRENT_HUMIDADE_AR, MIN_VALUE_HUMIDADE_AR, MAX_VALUE_HUMIDADE_AR, 0, 100);

    return resultado;
}

#define SoloCheckIntervalo 2000

#define TempCheckIntervalo 2000
int lastAirCheck = millis();
#define AirCheckIntervalo 2000
int lastWaterLevelCheck = millis();
#define WaterLevelCheckIntervalo 2000

int estadoLampada = LOW;

bool bombaOn = false;
int startedBomba = millis();
int lastBombaOn = millis();

int lastMessage = millis();
bool firstMessage = true;

#define TEMP_HISTERESIS 1.0 // 1°C
#define HUMI_HISTERESIS 5   // 5%

void checkValues()
{
    bool dataChanged = false;

    if (millis() - lastSoloCheck >= SoloCheckIntervalo)
    {
        int novaHumidade = measureHumidadeSolo();
        if (CURRENT_HUMIDADE_SOLO != novaHumidade)
        {
            CURRENT_HUMIDADE_SOLO = novaHumidade;
            Data_Send.CURRENT_HUMI_SOLO = CURRENT_HUMIDADE_SOLO;
            lastSoloCheck = millis();
            dataChanged = true;
        }
    }

    if (millis() - lastAirCheck >= AirCheckIntervalo)
    {
        float newHumi = sensor.readHumidity(false);

        if (!isnan(newHumi))
        {
            if (CURRENT_HUMIDADE_AR != newHumi)
            {
                CURRENT_HUMIDADE_AR = newHumi;
                Data_Send.CURRENT_HUMI_AR = CURRENT_HUMIDADE_AR;
                dataChanged = true;
            }
        }
        lastAirCheck = millis();
    }

    if (millis() - lastTempCheck >= TempCheckIntervalo)
    {
        float newTemp = sensor.readTemperature(false, false);
        if (!isnan(newTemp))
        {
            if (CURRENT_TEMP_AR != newTemp)
            {
                CURRENT_TEMP_AR = newTemp;
                Data_Send.CURRENT_TEMP_AR = CURRENT_TEMP_AR;
                dataChanged = true;
            }
        }
        lastTempCheck = millis();
    }

    if (dataChanged)
    {
        esp_now_send(display, (uint8_t *)&Data_Send, sizeof(Data_Send));
    }

    if (CURRENT_TEMP_AR < DESIRED_VALUE_TEMP_AR - TEMP_HISTERESIS)
    {
        digitalWrite(Lampada, HIGH);
        estadoLampada = HIGH;
    }
    else if (CURRENT_TEMP_AR >= DESIRED_VALUE_TEMP_AR)
    {
        digitalWrite(Lampada, LOW);
        estadoLampada = LOW;
    }

    if (!bombaOn)
    {
        if (CURRENT_HUMIDADE_SOLO < DESIRED_VALUE_HUMIDADE_SOLO - HUMI_HISTERESIS && (millis() - lastBombaOn > 180000))
        {
            bombaOn = true;
            digitalWrite(BombaAgua, HIGH);

            startedBomba = millis();
        }
    }
    else
    {
        if (millis() - startedBomba > 1500)
        {
            bombaOn = false;
            digitalWrite(BombaAgua, LOW);

            lastBombaOn = millis();

            ENOUGH_AGUA = digitalRead(PinSensor_NivelAgua) == LOW;
            Data_Send.EnoughAgua = ENOUGH_AGUA;
        }
    }
}

void setup()
{
    Serial.begin(9600);
    sensor.begin();
    WiFi.mode(WIFI_STA);

    esp_now_init();

    memset(&peerInfo, 0, sizeof(peerInfo));

    memcpy(peerInfo.peer_addr, display, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    esp_now_add_peer(&peerInfo);

    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(OnDataRecv);

    pinMode(Sensor_Solo, INPUT);

    pinMode(Lampada, OUTPUT);
    pinMode(BombaAgua, OUTPUT);

    pinMode(PinSensor_NivelAgua, INPUT_PULLUP);
    pinMode(PinSensor_TestaAgua, OUTPUT);

    digitalWrite(PinSensor_TestaAgua, HIGH);
    delayMicroseconds(10);
    ENOUGH_AGUA = digitalRead(PinSensor_NivelAgua) == LOW;
    Data_Send.EnoughAgua = ENOUGH_AGUA;
}

void loop()
{

    checkValues();
    delay(2);
}