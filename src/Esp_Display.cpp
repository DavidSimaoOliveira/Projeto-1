#include <Arduino.h>
#include <lvgl.h>
#include <LovyanGFX.hpp>
#include <WiFi.h>
#include <esp_now.h>
#include "LGFX_config.h"

#include "UI/ui.h"

#define BUTTON_1 19
#define BUTTON_2 9
#define BUTTON_3 15
#define PINO_LITE 12
#define DEBOUNCE_DELAY 400

#define MIN_VALUE_HUMIDADE_SOLO 300
#define MAX_VALUE_HUMIDADE_SOLO 700

#define MIN_VALUE_HUMIDADE_AR 0
#define MAX_VALUE_HUMIDADE_AR 100

#define MIN_VALUE_TEMP_AR 0
#define MAX_VALUE_TEMP_AR 40

int DESIRED_HUMIDADE_SOLO = 70;
int DESIRED_TEMP_AR = 25;
int Luminosidade = 100;

int TARGET_HUMIDADE_SOLO = 0;
int TARGET_HUMIDADE_AR = 0;
float TARGET_TEMP_AR = 0;
int TARGET_NIVEL_AGUA = 0;

float DISP_HUMIDADE_SOLO = 0;
float DISP_HUMIDADE_AR = 0;
float DISP_TEMP_AR = 0;
float DISP_NIVEL_AGUA = 0;

float START_HUMIDADE_SOLO = 0;
float START_HUMIDADE_AR = 0;
float START_TEMP_AR = 0;
float START_NIVEL_AGUA = 0;

unsigned long lastDataTime = 0;
const unsigned long UPDATE_INTERVAL = 2000;
unsigned long lastButtonPress = 0;
unsigned long lastAnimUpdate = 0;
bool isScreenOff = false;

bool Connected;

typedef enum
{
    sec_30,
    min_1,
    min_5,
    min_15,
    min_30,
    NEVER,
} SistemaStates;

SistemaStates ecra_setting = NEVER;

typedef enum
{
    Home,
    Solo,
    Ar,
    Def,
    Brilho,
    Humidade,
    Temperatura,
    Sistema,
} Ecra;

Ecra currentEcra = Home;
Ecra targetEcra = Home;

LGFX display;
static lv_color_t buf1[320 * 40];
static lv_color_t buf2[320 * 40];
static lv_disp_draw_buf_t draw_buf;

uint8_t estufa[] = {0xFC, 0x01, 0x2C, 0xF9, 0x03, 0x5C};

typedef struct
{
    float DESIRED_TEMP;
    int DESIRED_HUMI_SOLO;
} Message_Sent;

typedef struct
{
    float CURRENT_TEMP_AR;
    int CURRENT_HUMI_AR;
    int CURRENT_HUMI_SOLO;
    int CURRENT_NIVEL_AGUA;
} Message_Received;

Message_Received Data_received;
Message_Sent Data_Sent;
esp_now_peer_info_t peerInfo;

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

void ticks()
{
    unsigned long currentMillis = millis();
    static unsigned long lastMillis = 0;
    lv_tick_inc(currentMillis - lastMillis);
    lastMillis = currentMillis;
}

float measureTempAr()
{
    int resultado = map((int)(DISP_TEMP_AR * 10), MIN_VALUE_TEMP_AR * 10, MAX_VALUE_TEMP_AR * 10, 0, 100);
    if (resultado > 100)
        return 100;
    if (resultado < 0)
        return 0;
    return resultado;
}

int measureHumidadeAr()
{
    int resultado = map((int)DISP_HUMIDADE_AR, MIN_VALUE_HUMIDADE_AR, MAX_VALUE_HUMIDADE_AR, 0, 100);
    return resultado;
}

unsigned long getSuspendTime()
{
    switch (ecra_setting)
    {
    case (sec_30):
        return 30000;
    case (min_1):
        return 60000;
    case (min_5):
        return 300000;
    case (min_15):
        return 900000;
    case (min_30):
        return 1800000;
    case (NEVER):
        return 0;
    default:
        return 60000;
    }
}

void update_ScreenValues()
{
    unsigned long timePassed = millis() - lastDataTime;
    float progress = (float)timePassed / (float)UPDATE_INTERVAL;

    if (progress > 1.0)
        progress = 1.0;

    char mensagem_A[30] = {};

    // Solo
    DISP_HUMIDADE_SOLO = START_HUMIDADE_SOLO + ((TARGET_HUMIDADE_SOLO - START_HUMIDADE_SOLO) * progress);
    String msg = String(DISP_HUMIDADE_SOLO, 0) + "%";
    msg.toCharArray(mensagem_A, sizeof(mensagem_A));
    lv_label_set_text(ui_Humidade_Solo, mensagem_A);
    lv_arc_set_value(ui_Arc_Humidade_Solo, (int)DISP_HUMIDADE_SOLO);

    // Ar
    DISP_HUMIDADE_AR = START_HUMIDADE_AR + ((TARGET_HUMIDADE_AR - START_HUMIDADE_AR) * progress);
    msg = String(DISP_HUMIDADE_AR, 0) + "%";
    msg.toCharArray(mensagem_A, sizeof(mensagem_A));
    lv_label_set_text(ui_Humidade_Ar, mensagem_A);
    lv_arc_set_value(ui_Arc_Humidade_Ar, (int)DISP_HUMIDADE_AR);

    // Temp
    DISP_TEMP_AR = START_TEMP_AR + ((TARGET_TEMP_AR - START_TEMP_AR) * progress);
    msg = "Temp: " + String(DISP_TEMP_AR, 1) + "C";
    msg.toCharArray(mensagem_A, sizeof(mensagem_A));
    lv_label_set_text(ui_Temp_Ar, mensagem_A);
    lv_bar_set_value(ui_Bar_Temp, measureTempAr(), LV_ANIM_OFF);

    // Nível Água
    DISP_NIVEL_AGUA = START_NIVEL_AGUA + ((TARGET_NIVEL_AGUA - START_NIVEL_AGUA) * progress);
    msg = String(DISP_NIVEL_AGUA, 0) + "%";
    msg.toCharArray(mensagem_A, sizeof(mensagem_A));
    lv_label_set_text(ui_Nivel_Agua, mensagem_A);

    // Brilho
    lv_slider_set_value(ui_Bar_Brilho, Luminosidade, LV_ANIM_ON);

    // Desired Humidade
    msg = String(DESIRED_HUMIDADE_SOLO) + "%";
    msg.toCharArray(mensagem_A, sizeof(mensagem_A));
    lv_label_set_text(ui_Humidade_Solo_Desired, mensagem_A);
    lv_arc_set_value(ui_Arc_Def_Humidade_Solo, DESIRED_HUMIDADE_SOLO);

    // Desired Temp
    msg = String(DESIRED_TEMP_AR) + "C";
    msg.toCharArray(mensagem_A, sizeof(mensagem_A));
    lv_label_set_text(ui_Temp_Desired, mensagem_A);
    lv_arc_set_value(ui_Arc_Def_Temp, DESIRED_TEMP_AR);

    if (Connected)
    {
        msg = "Connected";
        msg.toCharArray(mensagem_A, sizeof(mensagem_A));
        lv_label_set_text(ui_Connection_Text, mensagem_A);
        lv_obj_clear_flag(ui_Connection_Image, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(ui_No_Connection_Image, LV_OBJ_FLAG_HIDDEN);
    }
    else
    {
        msg = "Connection Failed";
        msg.toCharArray(mensagem_A, sizeof(mensagem_A));
        lv_label_set_text(ui_Connection_Text, mensagem_A);
        lv_obj_add_flag(ui_Connection_Image, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(ui_No_Connection_Image, LV_OBJ_FLAG_HIDDEN);
    }
}

#define CONNECTION_TIMEOUT 15000

void update_Screen()
{
    bool wokeUpNow = false;
    bool desiredChanged = false;

    if (millis() - lastDataTime > CONNECTION_TIMEOUT)
    {
        Connected = false;
    }

    if (digitalRead(BUTTON_1) == LOW || digitalRead(BUTTON_2) == LOW || digitalRead(BUTTON_3) == LOW)
    {
        if (isScreenOff)
        {
            isScreenOff = false;
            lastButtonPress = millis();
            wokeUpNow = true;
            delay(200);
        }
    }
    unsigned long suspend_time = getSuspendTime();
    if (suspend_time > 0 && (millis() - lastButtonPress > suspend_time))
    {
        isScreenOff = true;
    }

    if (isScreenOff)
    {
        analogWrite(PINO_LITE, 0);
        return;
    }
    else
    {
        int brilho = map(Luminosidade, 0, 100, 0, 255);
        analogWrite(PINO_LITE, brilho);
    }

    if (wokeUpNow)
        return;

    uint16_t totalOptsDef = lv_roller_get_option_cnt(ui_Roller_Options);
    uint16_t totalOptsSistema = lv_roller_get_option_cnt(ui_Roller_Sistema);

    if (millis() - lastButtonPress > DEBOUNCE_DELAY)
    {
        if (digitalRead(BUTTON_1) == LOW)
        {
            lastButtonPress = millis();

            if (currentEcra == Def)
            {
                uint16_t currentOpt = lv_roller_get_selected(ui_Roller_Options);
                if (currentOpt > 0)
                    lv_roller_set_selected(ui_Roller_Options, currentOpt - 1, LV_ANIM_ON);
                else
                    lv_roller_set_selected(ui_Roller_Options, totalOptsDef - 1, LV_ANIM_ON);
            }
            else if (currentEcra == Brilho)
            {
                Luminosidade -= 5;
                if (Luminosidade < 10)
                {
                    Luminosidade = 10;
                }
            }
            else if (currentEcra == Temperatura)
            {
                DESIRED_TEMP_AR--;
                if (DESIRED_TEMP_AR < 0)
                {
                    DESIRED_TEMP_AR = 0;
                }
                if (DESIRED_TEMP_AR != Data_Sent.DESIRED_TEMP)
                {
                    desiredChanged = true;
                    Data_Sent.DESIRED_TEMP = DESIRED_TEMP_AR;
                }
            }
            else if (currentEcra == Humidade)
            {
                DESIRED_HUMIDADE_SOLO--;
                if (DESIRED_HUMIDADE_SOLO < 0)
                {
                    DESIRED_HUMIDADE_SOLO = 0;
                }
                if (DESIRED_HUMIDADE_SOLO != Data_Sent.DESIRED_HUMI_SOLO)
                {
                    desiredChanged = true;
                    Data_Sent.DESIRED_HUMI_SOLO = DESIRED_HUMIDADE_SOLO;
                }
            }
            else if (currentEcra == Sistema)
            {
                ecra_setting = (SistemaStates)(ecra_setting - 1);
                if (ecra_setting < 0)
                {
                    ecra_setting = (SistemaStates)(totalOptsSistema - 1);
                }

                lv_roller_set_selected(ui_Roller_Sistema, int(ecra_setting), LV_ANIM_ON);
            }
            else
            {
                if (currentEcra == Home)
                    targetEcra = Solo;
                else if (currentEcra == Ar)
                    targetEcra = Home;
            }
        }
        else if (digitalRead(BUTTON_2) == LOW)
        {
            lastButtonPress = millis();

            if (currentEcra == Def)
            {
                uint16_t currentOpt_Options = lv_roller_get_selected(ui_Roller_Options);
                if (currentOpt_Options < totalOptsDef - 1)
                    lv_roller_set_selected(ui_Roller_Options, currentOpt_Options + 1, LV_ANIM_ON);
                else
                    lv_roller_set_selected(ui_Roller_Options, 0, LV_ANIM_ON);
            }
            else if (currentEcra == Brilho)
            {
                Luminosidade += 5;
                if (Luminosidade > 100)
                {
                    Luminosidade = 100;
                }
            }
            else if (currentEcra == Temperatura)
            {
                DESIRED_TEMP_AR++;
                if (DESIRED_TEMP_AR > 100)
                {
                    DESIRED_TEMP_AR = 100;
                }
                if (DESIRED_TEMP_AR != Data_Sent.DESIRED_TEMP)
                {
                    desiredChanged = true;
                    Data_Sent.DESIRED_TEMP = DESIRED_TEMP_AR;
                }
            }
            else if (currentEcra == Humidade)
            {
                DESIRED_HUMIDADE_SOLO++;
                if (DESIRED_HUMIDADE_SOLO > 100)
                {
                    DESIRED_HUMIDADE_SOLO = 100;
                }
                if (DESIRED_HUMIDADE_SOLO != Data_Sent.DESIRED_HUMI_SOLO)
                {
                    desiredChanged = true;
                    Data_Sent.DESIRED_HUMI_SOLO = DESIRED_HUMIDADE_SOLO;
                }
            }
            else if (currentEcra == Sistema)
            {
                ecra_setting = (SistemaStates)(ecra_setting + 1);
                if (ecra_setting > totalOptsSistema - 1)
                {
                    ecra_setting = (SistemaStates)(0);
                }

                lv_roller_set_selected(ui_Roller_Sistema, int(ecra_setting), LV_ANIM_ON);
            }
            else
            {
                if (currentEcra == Home)
                    targetEcra = Ar;
                else if (currentEcra == Solo)
                    targetEcra = Home;
            }
        }
        else if (digitalRead(BUTTON_3) == LOW)
        {
            lastButtonPress = millis();

            if (currentEcra == Def)
            {
                int opcao = lv_roller_get_selected(ui_Roller_Options);
                switch (opcao)
                {
                case 0:
                    targetEcra = Brilho;
                    break;
                case 1:
                    targetEcra = Humidade;
                    break;
                case 2:
                    targetEcra = Temperatura;
                    break;
                case 3:
                    targetEcra = Sistema;
                    break;
                case 4:
                    targetEcra = Home;
                    break;
                }
            }
            else if (currentEcra == Home)
                targetEcra = Def;
            else if (currentEcra == Brilho)
                targetEcra = Def;
            else if (currentEcra == Temperatura)
                targetEcra = Def;
            else if (currentEcra == Humidade)
                targetEcra = Def;
            else if (currentEcra == Sistema)
                targetEcra = Def;
        }
    }

    if (targetEcra != currentEcra)
    {
        switch (targetEcra)
        {
        case Home:
            if (currentEcra == Def)
                _ui_screen_change(&ui_Home_Screen, LV_SCR_LOAD_ANIM_MOVE_BOTTOM, 400, 0, NULL);
            else if (currentEcra == Solo)
                _ui_screen_change(&ui_Home_Screen, LV_SCR_LOAD_ANIM_MOVE_LEFT, 400, 0, NULL);
            else if (currentEcra == Ar)
                _ui_screen_change(&ui_Home_Screen, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 400, 0, NULL);
            else
                _ui_screen_change(&ui_Home_Screen, LV_SCR_LOAD_ANIM_FADE_IN, 200, 0, NULL);
            break;

        case Solo:
            _ui_screen_change(&ui_Solo_Screen, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 400, 0, NULL);
            break;

        case Ar:
            _ui_screen_change(&ui_Ar_Screen, LV_SCR_LOAD_ANIM_MOVE_LEFT, 400, 0, NULL);
            break;

        case Def:
            if (currentEcra == Home)
                _ui_screen_change(&ui_Def_Screen, LV_SCR_LOAD_ANIM_MOVE_TOP, 400, 0, NULL);
            else
                _ui_screen_change(&ui_Def_Screen, LV_SCR_LOAD_ANIM_FADE_IN, 400, 0, NULL);
            break;

        case Brilho:
            _ui_screen_change(&ui_Brilho_Screen, LV_SCR_LOAD_ANIM_FADE_IN, 400, 0, NULL);
            break;
        case Sistema:
            _ui_screen_change(&ui_Sistema_Screen, LV_SCR_LOAD_ANIM_FADE_IN, 400, 0, NULL);
            break;
        case Humidade:
            _ui_screen_change(&ui_Humidade_Def_Screen, LV_SCR_LOAD_ANIM_FADE_IN, 400, 0, NULL);
            break;
        case Temperatura:
            _ui_screen_change(&ui_Temp_Def_Screen, LV_SCR_LOAD_ANIM_FADE_IN, 400, 0, NULL);
            break;
        }

        currentEcra = targetEcra;
    }
    if (desiredChanged)
    {
        esp_now_send(estufa, (uint8_t *)&Data_Sent, sizeof(Data_Sent));
        desiredChanged = false;
    }
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
}
void OnDataRecv(const esp_now_recv_info_t *esp_now_info, const uint8_t *incomingData, int len)
{
    memcpy(&Data_received, incomingData, sizeof(Data_received));

    START_HUMIDADE_AR = DISP_HUMIDADE_AR;
    START_HUMIDADE_SOLO = DISP_HUMIDADE_SOLO;
    START_TEMP_AR = DISP_TEMP_AR;
    START_NIVEL_AGUA = DISP_NIVEL_AGUA;

    TARGET_HUMIDADE_AR = Data_received.CURRENT_HUMI_AR;
    TARGET_HUMIDADE_SOLO = Data_received.CURRENT_HUMI_SOLO;
    TARGET_TEMP_AR = Data_received.CURRENT_TEMP_AR;
    TARGET_NIVEL_AGUA = Data_received.CURRENT_NIVEL_AGUA;

    lastDataTime = millis();
    Connected = true;
}

void designRollerDef()
{
    lv_obj_set_style_bg_opa(ui_Roller_Options, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(ui_Roller_Options, 0, LV_PART_MAIN);
    lv_obj_set_style_text_color(ui_Roller_Options, lv_color_white(), LV_PART_MAIN);

    lv_obj_set_style_bg_color(ui_Roller_Options, lv_palette_main(LV_PALETTE_BLUE), LV_PART_SELECTED);
    lv_obj_set_style_bg_opa(ui_Roller_Options, LV_OPA_50, LV_PART_SELECTED);
    lv_obj_set_style_text_color(ui_Roller_Options, lv_color_white(), LV_PART_SELECTED);
};

void designRollerSistema()
{
    lv_obj_set_style_bg_opa(ui_Roller_Sistema, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(ui_Roller_Sistema, 0, LV_PART_MAIN);
    lv_obj_set_style_text_color(ui_Roller_Sistema, lv_color_white(), LV_PART_MAIN);

    lv_obj_set_style_bg_color(ui_Roller_Sistema, lv_palette_main(LV_PALETTE_BLUE), LV_PART_SELECTED);
    lv_obj_set_style_bg_opa(ui_Roller_Sistema, LV_OPA_50, LV_PART_SELECTED);
    lv_obj_set_style_text_color(ui_Roller_Sistema, lv_color_white(), LV_PART_SELECTED);
}

void designSliderLuz()
{
    lv_obj_set_style_bg_color(ui_Bar_Brilho, lv_color_hex(0x202020), LV_PART_MAIN);
    lv_obj_set_style_radius(ui_Bar_Brilho, 50, LV_PART_MAIN);

    lv_obj_set_style_bg_color(ui_Bar_Brilho, lv_color_hex(0xFFD700), LV_PART_INDICATOR);
    lv_obj_set_style_bg_grad_color(ui_Bar_Brilho, lv_color_hex(0xFF8C00), LV_PART_INDICATOR);
    lv_obj_set_style_bg_grad_dir(ui_Bar_Brilho, LV_GRAD_DIR_HOR, LV_PART_INDICATOR);
    lv_obj_set_style_radius(ui_Bar_Brilho, 50, LV_PART_INDICATOR);

    lv_obj_set_style_bg_color(ui_Bar_Brilho, lv_color_white(), LV_PART_KNOB);
    lv_obj_set_style_shadow_color(ui_Bar_Brilho, lv_color_hex(0xFFD700), LV_PART_KNOB);
    lv_obj_set_style_shadow_width(ui_Bar_Brilho, 20, LV_PART_KNOB);
    lv_obj_set_style_shadow_spread(ui_Bar_Brilho, 2, LV_PART_KNOB);
    lv_obj_set_style_pad_all(ui_Bar_Brilho, 2, LV_PART_KNOB);
};

void setup()
{
    Serial.begin(9600);
    WiFi.mode(WIFI_STA);

    esp_now_init();

    memset(&peerInfo, 0, sizeof(peerInfo));
    memcpy(peerInfo.peer_addr, estufa, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    esp_now_add_peer(&peerInfo);
    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(OnDataRecv);

    display.init();
    display.setBrightness(255);

    lv_init();
    lv_disp_draw_buf_init(&draw_buf, buf1, buf2, 320 * 40);

    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = display.width();
    disp_drv.ver_res = display.height();
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;

    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_KEYPAD;

    lv_disp_drv_register(&disp_drv);

    ui_init();

    designRollerDef();
    designSliderLuz();
    designRollerSistema();

    pinMode(BUTTON_1, INPUT_PULLUP);
    pinMode(BUTTON_2, INPUT_PULLUP);
    pinMode(BUTTON_3, INPUT_PULLUP);
    pinMode(PINO_LITE, OUTPUT);
}

void loop()
{
    ticks();
    lv_timer_handler();

    update_Screen();

    if (millis() - lastAnimUpdate > 20)
    {
        update_ScreenValues();
        lastAnimUpdate = millis();
    }
}