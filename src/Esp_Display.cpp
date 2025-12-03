#include <Arduino.h>
#include <lvgl.h>
#include <LovyanGFX.hpp>
#include "LGFX_config.h"

#include "UI/ui.h"

#define BUTTON_1 19
#define BUTTON_2 20
#define BUTTON_3 21

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

void update_Screen()
{
    switch (ecra)
    {
    case (Home):
    {
        _ui_screen_change(&ui_Home_Screen, LV_SCR_LOAD_ANIM_FADE_IN, 20, 20, NULL);
        if (BUTTON_1 == LOW)
        {
            ecra = Solo;
        }
        else if (BUTTON_2 == LOW)
        {
            ecra = Ar;
        }
        else if (BUTTON_3 == LOW)
        {
            ecra = Def;
        }
    }
    break;
    case (Solo):
    {
        _ui_screen_change(&ui_Humidade_Solo_Screen, LV_SCR_LOAD_ANIM_FADE_IN, 20, 20, NULL);
        if (BUTTON_1 == LOW)
        {
            ecra = Home;
        }
    }
    break;
    case (Ar):
    {
        _ui_screen_change(&ui_Ar_Screen, LV_SCR_LOAD_ANIM_FADE_IN, 20, 20, NULL);
        if (BUTTON_2 == LOW)
        {
            ecra = Home;
        }
    }
    break;
    case (Def):
    {
        _ui_screen_change(&ui_Def_Screen, LV_SCR_LOAD_ANIM_FADE_IN, 20, 20, NULL);
    }
    break;
    }
};

void update_ScreenValues()
{
    lv_label_set_text_fmt(ui_HumidadeSolo, "Humidade do Solo:%g%", humidadeSolo);
    lv_label_set_text_fmt(ui_HumidadeAr, "Humidade do Ar:%g%", humidadeAr);
    lv_label_set_text_fmt(ui_TempAr, "Temperatura:%gº", tempAr);
}

void setup()
{
    Serial.begin(9600);

    // Inicializa o display
    display.init();
    display.setBrightness(255);

    // Inicializa LVGL
    lv_init();

    // buffer de desenho LVGL
    lv_disp_draw_buf_init(&draw_buf, buf1, buf2, 320 * 20);

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
}

void loop()
{
    ticks();
    lv_timer_handler();

    /*checkValues();

    update_ScreenValues();
    */
    update_Screen();
}