#include <Arduino.h>
#include <lvgl.h>
#include <LovyanGFX.hpp>
#include <DHT.h>
#include <WiFi.h>
#include <esp_now.h>
#include "LGFX_config.h"

#include "UI/ui.h"

#define DHTPIN 14
#define DHTTYPE DHT11

DHT sensor(DHTPIN, DHTTYPE);

#define BUTTON_1 19
#define BUTTON_2 9
#define BUTTON_3 15

#define Sensor_Solo 1

#define MIN_VALUE_HUMIDADE_SOLO
int DESIRED_VALUE_HUMIDADE_SOLO = 0; // a medir
#define MAX_VALUE_HUMIDADE_SOLO

#define MIN_VALUE_HUMIDADE_AR
int DESIRED_VALUE_HUMIDADE_AR = 0; // a medir
#define MAX_VALUE_HUMIDADE_AR

#define MIN_VALUE_TEMP_AR
int DESIRED_VALUE_TEMP_AR = 0; // a medir
#define MIN_VALUE_TEMP_AR

#define MIN_VALUE_QUANTIDADE_AGUA
#define MAX_VALUE_QUANTIDADE_AGUA

LGFX display;

// buffer LVGL
static lv_color_t buf1[320 * 40];
static lv_color_t buf2[320 * 40];
static lv_disp_draw_buf_t draw_buf;

// função LVGL → LovyanGFX (desenha no display)
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);

    display.startWrite();
    display.setAddrWindow(area->x1, area->y1, w, h);
    display.pushColors((uint16_t *)&color_p->full, w * h, true);
    display.endWrite();

    lv_disp_flush_ready(disp);
}

/*void indev_read(lv_indev_drv_t *drv, lv_indev_data_t *data)
{
}
*/

unsigned long currentMillis;
unsigned long lastMillis = 0;

void ticks()
{
    currentMillis = millis();

    lv_tick_inc(currentMillis - lastMillis);
    lastMillis = currentMillis;
}

//==============================================================================================================
// eps now
uint8_t broadcastAddress[] = {0xFC, 0x01, 0x2C, 0xF9, 0x2, 0x5C};

typedef struct
{
    float DESIRED_TEMP;
    int DESIRED_HUMI_AR;
    int DESIRED_HUMI_SOLO;
} Message;

Message myData;
esp_now_peer_info_t peerInfo;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    Serial.print("\r\nStatus do envio: ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Sucesso" : "Falha");
}

void OnDataRecv(const esp_now_recv_info_t *esp_now_info, const uint8_t *incomingData, int len)
{
    memcpy(&myData, incomingData, sizeof(myData));
}

//=========================================================================================================
int humidadeSolo;
float tempAr;
int humidadeAr;
int quantidadeAgua;

int lastSoloCheck = millis();
#define SoloCheckIntervalo 16
int lastTempCheck = millis();
#define TempCheckIntervalo 16
int lastAirCheck = millis();
#define AirCheckIntervalo 16
/*

void checkValues()
{
    if (millis() - lastSoloCheck >= SoloCheckIntervalo)
    {
        humidadeAr = measureHumidadeAr();
        lastSoloCheck = millis();
    }
    if (millis() - lastAirCheck >= TempCheckIntervalo)
    {
        humidadeSolo = measureHumidadeSolo();
        lastAirCheck = millis();
    }
    if (millis() - lastTempCheck >= AirCheckIntervalo)
    {
        tempAr = measureTempAr();
        lastTempCheck = millis();
    }
}

int measureHumidadeSolo()
{
    int value;

    int resultado = (value, MIN_VALUE_HUMIDADE_SOLO, MAX_VALUE_HUMIDADE_SOLO, 0, 100);

    return resultado;
}

float measureTempAr()
{
    int value;

    return value;
}
int measureHumidadeAr()
{
    int value;

    int resultado = (value, MIN_VALUE_HUMIDADE_AR, MAX_VALUE_HUMIDADE_AR, 0, 100);

    return resultado;
}

int measureQuantidadeAgua()
{
    int value;

    int resultado = (value, MIN_VALUE_QUANTIDADE_AGUA, MAX_VALUE_QUANTIDADE_AGUA, 0, 100);

    return resultado;
}
*/
typedef enum
{
    Home,
    Solo,
    Ar,
    Def,
} Ecra;

Ecra ecra = Home;

long lastChange = millis();
Ecra lastEcra = Home;

void update_Screen()
{
    if (millis() - lastChange > 500)
    {
        if (digitalRead(BUTTON_1) == LOW)
        {
            lastChange = millis();
            if (ecra == Home)
                ecra = Solo;
            else if (ecra == Ar)
                ecra = Home;
        }
        else if (digitalRead(BUTTON_2) == LOW)
        {
            lastChange = millis();
            if (ecra == Home)
                ecra = Ar;
            else if (ecra == Solo)
                ecra = Home;
        }
        else if (digitalRead(BUTTON_3) == LOW)
        {
            lastChange = millis();
            if (ecra == Home)
                ecra = Def;
            else if (ecra == Def)
                ecra = Home;
        }
    }

    if (lastEcra != ecra)
    {
        switch (ecra)
        {
        case Home:
            if (lastEcra == Def)
            {
                _ui_screen_change(&ui_Home_Screen, LV_SCR_LOAD_ANIM_MOVE_BOTTOM, 200, 0, NULL);
            }
            else if (lastEcra == Solo)
            {
                _ui_screen_change(&ui_Home_Screen, LV_SCR_LOAD_ANIM_MOVE_LEFT, 200, 0, NULL);
            }
            else if (lastEcra == Ar)
            {
                _ui_screen_change(&ui_Home_Screen, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 200, 0, NULL);
            }
            break;
        case Solo:
            _ui_screen_change(&ui_Solo_Screen, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 200, 0, NULL);
            break;

        case Ar:
            _ui_screen_change(&ui_Ar_Screen, LV_SCR_LOAD_ANIM_MOVE_LEFT, 200, 0, NULL);
            break;

        case Def:
            _ui_screen_change(&ui_Def_Screen, LV_SCR_LOAD_ANIM_MOVE_TOP, 200, 0, NULL);
            break;
        }
        lastEcra = ecra;
    }
};

void update_ScreenValues()
{
    lv_label_set_text_fmt(ui_HumidadeSolo, "Humidade do Solo:%g%", humidadeSolo);
    // lv_label_set_text_fmt(ui_Humidade_Ar, "Humidade do Ar:%g%", humidadeAr);
    // lv_label_set_text_fmt(ui_Temp_Ar, "Temperatura:%gº", tempAr);
    lv_arc_set_value(ui_ArcHumidadeSolo, humidadeSolo);
}

void setup()
{
    Serial.begin(115200);
    sensor.begin();
    WiFi.mode(WIFI_STA);

    esp_now_init();

    memset(&peerInfo, 0, sizeof(peerInfo));

    // Configura o Peer
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false; // Sem encriptação para teste inicial

    esp_now_add_peer(&peerInfo);

    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(OnDataRecv);

    // Inicializa o display
    display.init();
    display.setBrightness(255);

    // Inicializa LVGL
    lv_init();

    // buffer de desenho LVGL
    lv_disp_draw_buf_init(&draw_buf, buf1, buf2, 320 * 40);

    // driver LVGL
    static lv_disp_drv_t disp_drv;
    lv_indev_drv_t indev_drv;

    lv_disp_drv_init(&disp_drv);

    disp_drv.hor_res = display.width();
    disp_drv.ver_res = display.height();

    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    indev_drv.type = LV_INDEV_TYPE_KEYPAD;
    // indev_drv.read_cb = indev_read;
    lv_disp_drv_register(&disp_drv);

    ui_init();

    pinMode(BUTTON_1, INPUT_PULLUP);
    pinMode(BUTTON_2, INPUT_PULLUP);
    pinMode(BUTTON_3, INPUT_PULLUP);

    pinMode(Sensor_Solo, INPUT);

    _ui_screen_change(&ui_Home_Screen, LV_SCR_LOAD_ANIM_FADE_IN, 200, 0, NULL);

    // checkValues();
    // update_ScreenValues();
}

long lastPrint;

void loop()
{
    ticks();
    lv_timer_handler();

    if (millis() - lastPrint > 2000)
    {
        float temp = sensor.readTemperature(false, false);
        float humidade = sensor.readHumidity(false);

        Serial.println(temp);
        Serial.println(humidade);
        Serial.println("========================================================================================");
        lastPrint = millis();
    }

    // checkValues();

    // update_ScreenValues();

    // update_Screen();
    delay(5);
}