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

#define MIN_VALUE_HUMIDADE_SOLO 300
int DESIRED_VALUE_HUMIDADE_SOLO = 0; // a medir
#define MAX_VALUE_HUMIDADE_SOLO 700

#define MIN_VALUE_HUMIDADE_AR 0
int DESIRED_VALUE_HUMIDADE_AR = 0; // a medir
#define MAX_VALUE_HUMIDADE_AR 100

#define MIN_VALUE_TEMP_AR 0
#define MAX_VALUE_TEMP_AR 40

#define MIN_VALUE_QUANTIDADE_AGUA
#define MAX_VALUE_QUANTIDADE_AGUA

#define LUMINUSIDADE 100

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

float measureTempAr()
{
    int resultado = map(DISP_TEMP_AR, MIN_VALUE_TEMP_AR, MAX_VALUE_TEMP_AR, 0, 100);

    return resultado;
}

int measureHumidadeAr()
{
    int resultado = map(DISP_HUMIDADE_AR, MIN_VALUE_HUMIDADE_AR, MAX_VALUE_HUMIDADE_AR, 0, 100);

    return resultado;
}

void update_ScreenValues()
{
    unsigned long timePassed = millis() - lastDataTime;

    // Calcula o progresso de 0.0 a 1.0
    float progress = (float)timePassed / (float)UPDATE_INTERVAL;

    // Impede que o progresso ultrapasse 1.0 (se a estufa atrasar)
    if (progress > 1.0)
        progress = 1.0;

    char mensagem_A[30] = {};

    DISP_HUMIDADE_SOLO = START_HUMIDADE_SOLO + ((TARGET_HUMIDADE_SOLO - START_HUMIDADE_SOLO) * progress);

    String msg = String(DISP_HUMIDADE_SOLO, 1) + "%";
    msg.toCharArray(mensagem_A, sizeof(mensagem_A));
    lv_label_set_text(ui_Humidade_Solo, mensagem_A);
    lv_arc_set_value(ui_Arc_Humidade_Solo, (int)DISP_HUMIDADE_SOLO);

    DISP_HUMIDADE_AR = START_HUMIDADE_AR + ((TARGET_HUMIDADE_AR - START_HUMIDADE_AR) * progress);

    msg = String(DISP_HUMIDADE_AR, 1) + "%";
    msg.toCharArray(mensagem_A, sizeof(mensagem_A));
    lv_label_set_text(ui_Humidade_Ar, mensagem_A);
    lv_arc_set_value(ui_Arc_Humidade_Ar, (int)DISP_HUMIDADE_AR);

    DISP_TEMP_AR = START_TEMP_AR + ((TARGET_TEMP_AR - START_TEMP_AR) * progress);

    msg = "Temp: " + String(DISP_TEMP_AR, 1) + "C";
    msg.toCharArray(mensagem_A, sizeof(mensagem_A));
    lv_label_set_text(ui_Temp_Ar, mensagem_A);
    lv_bar_set_value(ui_Bar_Temp, measureTempAr(), LV_ANIM_OFF);

    DISP_NIVEL_AGUA = START_NIVEL_AGUA + ((TARGET_NIVEL_AGUA - START_NIVEL_AGUA) * progress);

    msg = String(DISP_NIVEL_AGUA, 0) + "%";
    msg.toCharArray(mensagem_A, sizeof(mensagem_A));
    lv_label_set_text(ui_Nivel_Agua, mensagem_A);
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
#define DEBOUNCE_DELAY 400

void update_Screen()
{
    int brilho = map(LUMINUSIDADE, 0, 100, 0, 255);
    analogWrite(PINO_LITE, brilho);

    uint16_t totalOpts = lv_roller_get_option_cnt(ui_Options);

    if (millis() - lastButtonPress > DEBOUNCE_DELAY)
    {
        if (digitalRead(BUTTON_1) == LOW)
        {
            lastButtonPress = millis();

            if (currentEcra == Def)
            {
                uint16_t currentOpt = lv_roller_get_selected(ui_Options);

                if (currentOpt > 0)
                {
                    lv_roller_set_selected(ui_Options, currentOpt - 1, LV_ANIM_ON);
                }
                else if (currentOpt == 0)
                {
                    lv_roller_set_selected(ui_Options, totalOpts - 1, LV_ANIM_ON);
                }
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
                uint16_t currentOpt = lv_roller_get_selected(ui_Options);

                if (currentOpt < totalOpts - 1)
                {
                    lv_roller_set_selected(ui_Options, currentOpt + 1, LV_ANIM_ON);
                }
                else if (currentOpt == totalOpts - 1)
                {
                    lv_roller_set_selected(ui_Options, 0, LV_ANIM_ON);
                }
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

                int opcao = lv_roller_get_selected(ui_Options);
                /*switch (opcao)
                {
                case (0):
                }
                */
                targetEcra = Home;
            }
            else if (currentEcra == Home)
            {
                targetEcra = Def;
            }
        }
    }

    if (targetEcra != currentEcra)
    {
        switch (targetEcra)
        {
        case Home:
            if (currentEcra == Def)
            {
                _ui_screen_change(&ui_Home_Screen, LV_SCR_LOAD_ANIM_MOVE_BOTTOM, 400, 0, NULL);
            }
            else if (currentEcra == Solo)
            {
                _ui_screen_change(&ui_Home_Screen, LV_SCR_LOAD_ANIM_MOVE_LEFT, 400, 0, NULL);
            }
            else if (currentEcra == Ar)
            {
                _ui_screen_change(&ui_Home_Screen, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 400, 0, NULL);
            }
            break;

        case Solo:
            _ui_screen_change(&ui_Solo_Screen, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 400, 0, NULL);
            break;

        case Ar:
            _ui_screen_change(&ui_Ar_Screen, LV_SCR_LOAD_ANIM_MOVE_LEFT, 400, 0, NULL);
            break;

        case Def:
            _ui_screen_change(&ui_Def_Screen, LV_SCR_LOAD_ANIM_MOVE_TOP, 400, 0, NULL);
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
esp_now_peer_info_t peerInfo;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    Serial.print("\r\nStatus do envio: ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Sucesso" : "Falha");
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
}

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

    // 1. Fundo principal do Roller transparente
    lv_obj_set_style_bg_opa(ui_Options, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(ui_Options, 0, LV_PART_MAIN); // Remove a borda

    // 2. Cor do texto (Ajusta para Branco ou Preto dependendo do teu fundo)
    lv_obj_set_style_text_color(ui_Options, lv_color_white(), LV_PART_MAIN);

    // 3. Estilo da faixa central selecionada
    // Define uma cor que combine com o teu tema (ex: um azul claro, ou branco)
    // E usa uma opacidade baixa (ex: 50% = LV_OPA_50) para ver o fundo através dela
    lv_obj_set_style_bg_color(ui_Options, lv_palette_main(LV_PALETTE_BLUE), LV_PART_SELECTED);
    lv_obj_set_style_bg_opa(ui_Options, LV_OPA_50, LV_PART_SELECTED);            // Semi-transparente
    lv_obj_set_style_text_color(ui_Options, lv_color_white(), LV_PART_SELECTED); // Texto selecionado

    pinMode(BUTTON_1, INPUT_PULLUP);
    pinMode(BUTTON_2, INPUT_PULLUP);
    pinMode(BUTTON_3, INPUT_PULLUP);

    pinMode(PINO_LITE, OUTPUT);
}

unsigned long lastAnimUpdate = 0;

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