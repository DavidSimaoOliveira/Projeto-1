#include <TFT_eSPI.h>
#include <lvgl.h>

TFT_eSPI tft = TFT_eSPI();

static lv_color_t buf1[128 * 20];
static lv_color_t buf2[128 * 20];
static lv_disp_draw_buf_t draw_buf;

void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
    tft.startWrite();
    tft.setAddrWindow(area->x1, area->y1,
                      area->x2 - area->x1 + 1,
                      area->y2 - area->y1 + 1);

    tft.pushColors((uint16_t *)&color_p->full,
                   lv_area_get_width(area) * lv_area_get_height(area),
                   true);

    tft.endWrite();
    lv_disp_flush_ready(disp);
}

void my_display_driver_init()
{
    tft.init();
    tft.setRotation(1);

    lv_disp_draw_buf_init(&draw_buf, buf1, buf2, 128 * 20);

    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);

    disp_drv.hor_res = 128;
    disp_drv.ver_res = 160;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;

    lv_disp_drv_register(&disp_drv);
}
