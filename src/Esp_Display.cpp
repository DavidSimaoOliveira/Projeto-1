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

#define BOMBA 7

#define Sensor_Solo 1

#define MIN_VALUE_HUMIDADE_SOLO
int DESIRED_VALUE_HUMIDADE_SOLO = 0; // a medir
int CURRENT_HUMIDADE_SOLO;
#define MAX_VALUE_HUMIDADE_SOLO

#define MIN_VALUE_HUMIDADE_AR 0
int DESIRED_VALUE_HUMIDADE_AR = 0; // a medir
int CURRENT_HUMIDADE_AR;
#define MAX_VALUE_HUMIDADE_AR 100

#define MIN_VALUE_TEMP_AR 0
int DESIRED_VALUE_TEMP_AR = 0; // a medir
float CURRENT_TEMP_AR;
#define MAX_VALUE_TEMP_AR 40

#define MIN_VALUE_QUANTIDADE_AGUA
int CURRENT_NIVEL_AGUA;
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

/*
int measureHumidadeSolo()
{
    int value=analogRead(Sensor_Solo);

    int resultado = (value, MIN_VALUE_HUMIDADE_SOLO, MAX_VALUE_HUMIDADE_SOLO, 0, 100);

    return resultado;
}
*/
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

void update_ScreenValues()
{
    char mens_Humidade_solo[30] = {};
    String mensagem = "Humidade do Solo:\n" + String(CURRENT_HUMIDADE_SOLO) + "%";
    mensagem.toCharArray(mens_Humidade_solo, sizeof(mens_Humidade_solo));

    lv_label_set_text_fmt(ui_Humidade_Solo, mens_Humidade_solo);
    lv_label_set_text_fmt(ui_Humidade_Ar, "Humidade do Ar:%g%", CURRENT_HUMIDADE_AR);
    lv_arc_set_value(ui_Arc_Humidade_Ar, CURRENT_HUMIDADE_AR);

    lv_label_set_text_fmt(ui_Temp_Ar, "Temperatur: %gºC", CURRENT_TEMP_AR);
    int tempBar = map(CURRENT_TEMP_AR, 0, 40, 0, 100);
    lv_bar_set_value(ui_Bar_Temp, tempBar, LV_ANIM_OFF);
    // lv_label_set_text_fmt(ui_HumidadeAr, "Humidade do Ar:%g%", CURRENT_HUMIDADE_AR);
    // lv_label_set_text_fmt(ui_TempAr, "Temperatura:%gº", CURRENT_TEMPERATURA_AR);
}

typedef enum
{
    Home,
    Solo,
    Ar,
    Def,
} Ecra;

Ecra currentEcra = Home;
Ecra targetEcra = Home;

unsigned long lastButtonPress = 0;
#define DEBOUNCE_DELAY 600

void update_Screen()
{
    if (millis() - lastButtonPress > DEBOUNCE_DELAY)
    {
        if (digitalRead(BUTTON_1) == LOW)
        {
            lastButtonPress = millis();
            if (currentEcra == Home)
                targetEcra = Solo;
            else if (currentEcra == Ar)
                targetEcra = Home;
        }
        else if (digitalRead(BUTTON_2) == LOW)
        {
            lastButtonPress = millis();
            if (currentEcra == Home)
                targetEcra = Ar;
            else if (currentEcra == Solo)
                targetEcra = Home;
        }
        else if (digitalRead(BUTTON_3) == LOW)
        {
            lastButtonPress = millis();
            if (currentEcra == Home)
                targetEcra = Def;
            else if (currentEcra == Def)
                targetEcra = Home;
        }
    }

    if (targetEcra != currentEcra)
    {
        switch (targetEcra)
        {
        case Home:
            if (currentEcra == Def)
            {
                _ui_screen_change(&ui_Home_Screen, LV_SCR_LOAD_ANIM_MOVE_BOTTOM, 600, 0, NULL);
            }
            else if (currentEcra == Solo)
            {
                _ui_screen_change(&ui_Home_Screen, LV_SCR_LOAD_ANIM_MOVE_LEFT, 600, 0, NULL);
            }
            else if (currentEcra == Ar)
            {
                _ui_screen_change(&ui_Home_Screen, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 600, 0, NULL);
            }
            break;

        case Solo:
            _ui_screen_change(&ui_Solo_Screen, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 600, 0, NULL);
            break;

        case Ar:
            _ui_screen_change(&ui_Ar_Screen, LV_SCR_LOAD_ANIM_MOVE_LEFT, 600, 0, NULL);
            break;

        case Def:
            _ui_screen_change(&ui_Def_Screen, LV_SCR_LOAD_ANIM_MOVE_TOP, 600, 0, NULL);
            break;
        }
        currentEcra = targetEcra;
    }
}

//==============================================================================================================
// eps now

uint8_t estufa[] = {0xFC, 0x01, 0x2C, 0xF9, 0x03, 0x5C};

typedef struct
{
    float DESIRED_TEMP;
    int CURRENT_HUMI_AR;
    int CURRENT_HUMI_SOLO;
    int CURRENT_NIVEL_AGUA;
} Message_Sent;

typedef struct
{
    float CURRENT_TEMP_AR;
    int CURRENT_HUMI_AR;
    int CURRENT_HUMI_SOLO;
    int CURRENT_NIVEL_AGUA;
} Message_Received;

Message_Received Data_received;
esp_now_peer_info_t peerInfo;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    Serial.print("\r\nStatus do envio: ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Sucesso" : "Falha");
}

void OnDataRecv(const esp_now_recv_info_t *esp_now_info, const uint8_t *incomingData, int len)
{
    memcpy(&Data_received, incomingData, sizeof(Data_received));

    CURRENT_HUMIDADE_AR = Data_received.CURRENT_HUMI_AR;
    CURRENT_HUMIDADE_SOLO = Data_received.CURRENT_HUMI_SOLO;
    CURRENT_TEMP_AR = Data_received.CURRENT_TEMP_AR;
    CURRENT_NIVEL_AGUA = Data_received.CURRENT_NIVEL_AGUA;

    update_ScreenValues();
}

void setup()
{
    Serial.begin(9600);
    sensor.begin();
    WiFi.mode(WIFI_STA);

    esp_now_init();

    memset(&peerInfo, 0, sizeof(peerInfo));

    memcpy(peerInfo.peer_addr, estufa, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

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
}

void loop()
{
    ticks();
    lv_timer_handler();

    update_Screen();
}