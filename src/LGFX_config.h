#pragma once
#include <LovyanGFX.hpp>

class LGFX : public lgfx::LGFX_Device
{
    lgfx::Panel_ST7735S panel;
    lgfx::Bus_SPI bus;

public:
    LGFX(void)
    {

        // --- BUS SPI ---
        {
            auto cfg = bus.config(); // pega a struct de configuração
            cfg.spi_mode = 0;
            cfg.freq_write = 40000000; // 40MHz
            cfg.freq_read = 16000000;
            cfg.pin_sclk = 18; // altere conforme seu setup
            cfg.pin_mosi = 23;
            cfg.pin_miso = -1;
            cfg.pin_dc = 2;
            bus.config(cfg);
            panel.setBus(&bus);
        }

        // --- PANEL ---
        {
            auto cfg = panel.config();
            cfg.pin_cs = 5;
            cfg.pin_rst = 4;
            cfg.pin_busy = -1;

            cfg.memory_width = 128;
            cfg.memory_height = 160;
            cfg.panel_width = 128;
            cfg.panel_height = 160;

            cfg.offset_x = 0; // ajuste se necessário
            cfg.offset_y = 0;
            cfg.offset_rotation = 0;
            cfg.invert = false;
            cfg.rgb_order = true;

            panel.config(cfg);
        }

        setPanel(&panel);
    }
};
