#include <Arduino.h>
#include <DHT.h>
#include <WiFi.h>
#include <esp_now.h>

#define DHTPIN 14
#define DHTTYPE DHT11

DHT sensor(DHTPIN, DHTTYPE);

#define Sensor_Solo 2

#define trigPin 13
#define echoPin 12

#define MIN_VALUE_HUMIDADE_SOLO 300
int DESIRED_VALUE_HUMIDADE_SOLO = 0;
int CURRENT_HUMIDADE_SOLO;
#define MAX_VALUE_HUMIDADE_SOLO 700

#define MIN_VALUE_HUMIDADE_AR 0
int DESIRED_VALUE_HUMIDADE_AR = 0;
float CURRENT_HUMIDADE_AR;
#define MAX_VALUE_HUMIDADE_AR 100

#define MIN_VALUE_TEMP_AR 0
int DESIRED_VALUE_TEMP_AR = 0;
float CURRENT_TEMP_AR;
#define MAX_VALUE_TEMP_AR 40

#define MIN_VALUE_QUANTIDADE_AGUA
int CURRENT_NIVEL_AGUA;
#define MAX_VALUE_QUANTIDADE_AGUA

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
    int CURRENT_NIVEL_AGUA;
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

int measureWater()
{
    long duration, distance;
    int quantidade = 0;

    // Send pulse
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    // Wait for echo and measure time until it happens
    duration = pulseIn(echoPin, HIGH);
    // Compute distance
    distance = duration / 58;

    quantidade = map(distance, 0, 300, 0, 100);
    if (quantidade >= 100)
    {
        quantidade = 100;
    }
    else if (quantidade < 0)
    {
        quantidade = 0;
    }

    return 100 - quantidade;
}

int lastSoloCheck = millis();
#define SoloCheckIntervalo 2000
int lastTempCheck = millis();
#define TempCheckIntervalo 2000
int lastAirCheck = millis();
#define AirCheckIntervalo 2000
int lastWaterLevelCheck = millis();
#define WaterLevelCheckIntervalo 2000

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

    if (millis() - lastWaterLevelCheck >= WaterLevelCheckIntervalo)
    {
        int newWaterLevel = measureWater();
        if (CURRENT_NIVEL_AGUA != newWaterLevel)
        {
            CURRENT_NIVEL_AGUA = newWaterLevel;
            Data_Send.CURRENT_NIVEL_AGUA = CURRENT_NIVEL_AGUA;
            dataChanged = true;
        }
        lastWaterLevelCheck = millis();
    }

    if (dataChanged)
    {
        esp_now_send(display, (uint8_t *)&Data_Send, sizeof(Data_Send));
    }
}

void setup()
{
    Serial.begin(9600);
    sensor.begin();
    WiFi.mode(WIFI_STA);

    esp_now_init();

    memset(&peerInfo, 0, sizeof(peerInfo));

    // Configura o Peer
    memcpy(peerInfo.peer_addr, display, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false; // Sem encriptação para teste inicial

    esp_now_add_peer(&peerInfo);

    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(OnDataRecv);

    pinMode(Sensor_Solo, INPUT);

    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);

    // checkValues();
}

void loop()
{

    checkValues();

    delay(2);
}