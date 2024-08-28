/* Libary to USE */
#include <lvgl.h>
#include "ui.h"
#include <Wire.h>
#include <SPI.h>

/*******************************************************************************
 ******************************************************************************/
#define TFT_BL 2
#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include <lgfx/v1/platforms/esp32s3/Panel_RGB.hpp>
#include <lgfx/v1/platforms/esp32s3/Bus_RGB.hpp>
class LGFX : public lgfx::LGFX_Device
{
public:

  lgfx::Bus_RGB     _bus_instance;
  lgfx::Panel_RGB   _panel_instance;

  LGFX(void)
  {


    {
      auto cfg = _bus_instance.config();
      cfg.panel = &_panel_instance;

      //Please modify the pins according to the circuit diagram. The pins are not the same for the 4.3-inch, 5-inch, and 7-inch screens.
      cfg.pin_d0  = GPIO_NUM_15; // B0
      cfg.pin_d1  = GPIO_NUM_7;  // B1
      cfg.pin_d2  = GPIO_NUM_6;  // B2
      cfg.pin_d3  = GPIO_NUM_5;  // B3
      cfg.pin_d4  = GPIO_NUM_4;  // B4
      
      cfg.pin_d5  = GPIO_NUM_9;  // G0
      cfg.pin_d6  = GPIO_NUM_46; // G1
      cfg.pin_d7  = GPIO_NUM_3;  // G2
      cfg.pin_d8  = GPIO_NUM_8;  // G3
      cfg.pin_d9  = GPIO_NUM_16; // G4
      cfg.pin_d10 = GPIO_NUM_1;  // G5
      
      cfg.pin_d11 = GPIO_NUM_14; // R0
      cfg.pin_d12 = GPIO_NUM_21; // R1
      cfg.pin_d13 = GPIO_NUM_47; // R2
      cfg.pin_d14 = GPIO_NUM_48; // R3
      cfg.pin_d15 = GPIO_NUM_45; // R4

      cfg.pin_henable = GPIO_NUM_41;
      cfg.pin_vsync   = GPIO_NUM_40;
      cfg.pin_hsync   = GPIO_NUM_39;
      cfg.pin_pclk    = GPIO_NUM_0;
      //------------Pin Config END--------------------
      
      cfg.freq_write  = 16000000;

      cfg.hsync_polarity    = 0;
      cfg.hsync_front_porch = 40;
      cfg.hsync_pulse_width = 48;
      cfg.hsync_back_porch  = 40;
      
      cfg.vsync_polarity    = 0;
      cfg.vsync_front_porch = 1;
      cfg.vsync_pulse_width = 31;
      cfg.vsync_back_porch  = 13;

      cfg.pclk_active_neg   = 1;
      cfg.de_idle_high      = 0;
      cfg.pclk_idle_high    = 0;

      _bus_instance.config(cfg);
    }
            {
      auto cfg = _panel_instance.config();
      //5inch and 7inch: 800*480
      cfg.memory_width  = 800;
      cfg.memory_height = 480;
      cfg.panel_width  = 800;
      cfg.panel_height = 480;
      cfg.offset_x = 0;
      cfg.offset_y = 0;
      _panel_instance.config(cfg);
    }
    _panel_instance.setBus(&_bus_instance);
    setPanel(&_panel_instance);

  }
};


LGFX lcd;
/*******************************************************************************
   End of Arduino_GFX setting
 ******************************************************************************/


/*******************************************************************************
   Config the touch panel in touch.h
 ******************************************************************************/
#include "touch.h"

/* Change to your screen resolution */
static uint32_t screenWidth;
static uint32_t screenHeight;
static lv_disp_draw_buf_t draw_buf;

//7inch: 800*480
static lv_color_t disp_draw_buf1[800 * 480 / 8];
static lv_color_t disp_draw_buf2[800 * 480 / 8];

//static lv_color_t disp_draw_buf;
static lv_disp_drv_t disp_drv;

/* Display flushing */
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
   uint32_t w = ( area->x2 - area->x1 + 1 );
   uint32_t h = ( area->y2 - area->y1 + 1 );

   lcd.pushImageDMA(area->x1, area->y1, w, h,(lgfx::rgb565_t*)&color_p->full);
   lv_disp_flush_ready( disp );

}

void my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data)
{
  if (touch_has_signal())
  {
    if (touch_touched())
    {
      data->state = LV_INDEV_STATE_PR;

      /*Set the coordinates*/
      data->point.x = touch_last_x;
      data->point.y = touch_last_y;
      // Serial.print( "Data x :" );
      // Serial.println( touch_last_x );

      // Serial.print( "Data y :" );
      // Serial.println( touch_last_y );
    }
    else if (touch_released())
    {
      data->state = LV_INDEV_STATE_REL;
    }
  }
  else
  {
    data->state = LV_INDEV_STATE_REL;
  }
  delay(15);
}
/* Structure for receive data from STM32*/
typedef struct
{
  float I1; //Current1
  float I2;
  float I3;
  float I4;
  float I5;

  float ActivePower1;
  float ActivePower2;
  float ActivePower3;
  float ActivePower4;
  float ActivePower5;

  float energy_value1;
  float energy_value2;
  float energy_value3;
  float energy_value4;
  float energy_value5;

  float Voltage;
  float Freq;
} EnergyPackage;

EnergyPackage received_package;

void setup()
{
  Serial.begin(115200);
  //Serial.println("LVGL Widgets Demo");

  //GPIO State Config
  pinMode(38, OUTPUT);
  digitalWrite(38, LOW);
  pinMode(17, OUTPUT);
  digitalWrite(17, LOW);
  pinMode(18, OUTPUT);
  digitalWrite(18, LOW);
  pinMode(42, OUTPUT);
  digitalWrite(42, LOW);


  // Init Display
  lcd.begin();
  lcd.fillScreen(TFT_BLACK);
  lcd.setTextSize(2);
  delay(200);

  lv_init();
  delay(100);
  touch_init();

  screenWidth = lcd.width();
  screenHeight = lcd.height();

  lv_disp_draw_buf_init(&draw_buf, disp_draw_buf1, disp_draw_buf2, screenWidth * screenHeight/8);
  /* Initialize the display */
  lv_disp_drv_init(&disp_drv);
  /* Change the following line to your display resolution */
  disp_drv.hor_res = screenWidth;
  disp_drv.ver_res = screenHeight;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.full_refresh = 1;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register(&disp_drv);

  /* Initialize the (dummy) input device driver */
  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = my_touchpad_read;
  lv_indev_drv_register(&indev_drv);
#ifdef TFT_BL
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);
#endif


  lcd.fillScreen(TFT_BLACK);
  ui_init();            //UI from Squareline Studio
  Serial.println( "Setup done" );

}

void loop()
{
  /* UART communication with STM32 */
  if (Serial.available() >= sizeof(EnergyPackage))
  {
    Serial.readBytes((char *)&received_package, sizeof(EnergyPackage));

    Serial.print("Current1: ");
    Serial.println(received_package.I1);
    Serial.print("Current2: ");
    Serial.println(received_package.I2);
    Serial.print("Current3: ");
    Serial.println(received_package.I3);
    Serial.print("Current4: ");
    Serial.println(received_package.I4);
    Serial.print("Current5: ");
    Serial.println(received_package.I5);

    Serial.print("ActivePower1: ");
    Serial.println(received_package.ActivePower1);
    Serial.print("ActivePower2: ");
    Serial.println(received_package.ActivePower2);
    Serial.print("ActivePower3: ");
    Serial.println(received_package.ActivePower3);
    Serial.print("ActivePower4: ");
    Serial.println(received_package.ActivePower4);
    Serial.print("ActivePower5: ");
    Serial.println(received_package.ActivePower5);

    Serial.print("Received energy 1: ");
    Serial.println(received_package.energy_value1);
    Serial.print("Received energy 2: ");
    Serial.println(received_package.energy_value2);
    Serial.print("Received energy 3: ");
    Serial.println(received_package.energy_value3);
    Serial.print("Received energy 4: ");
    Serial.println(received_package.energy_value4);
    Serial.print("Received energy 5: ");
    Serial.println(received_package.energy_value5);

    Serial.print("Received Voltage: ");
    Serial.println(received_package.Voltage);
    Serial.print("Received Freq: ");
    Serial.println(received_package.Freq);
 
   
float Current1 = received_package.I1;
//float Current1 = 67.89;
float Current2 = received_package.I2;
float Current3 = received_package.I3;
float Current4 = received_package.I4;
float Current5 = received_package.I5;

float P1 = received_package.ActivePower1; //divied by 1000 to covert from  to Watt to Kw
float P2 = received_package.ActivePower2;
float P3 = received_package.ActivePower3;
float P4 = received_package.ActivePower4;
float P5 = received_package.ActivePower5;

float E1 = received_package.energy_value1; 
float E2 = received_package.energy_value2;
float E3 = received_package.energy_value3;
float E4 = received_package.energy_value4;
float E5 = received_package.energy_value5;

float Vol = received_package.Voltage;
float Frequency = received_package.Freq;

 
//Buffer to store
 char buffer1[20];
 char buffer2[20];
 char buffer3[20];
 char buffer4[20];
 char buffer5[20];
 char buffer6[20];
 char buffer7[20];
 char buffer8[20];

 char buffer9[20];
 char buffer10[20];
 char buffer11[20];
 char buffer12[20];
 char buffer13[20];
 char buffer14[20];
 char buffer15[20];
 char buffer16[20];
 char buffer17[20];
 
/* Update BL0910 value receive from STM32 via UART to display */
//Current and Power CH1
 snprintf(buffer1, sizeof(buffer1), "%.3f", Current1);
 lv_label_set_text(ui_Label20, buffer1);
 snprintf(buffer2, sizeof(buffer2), "%.3f", P1);
 lv_label_set_text(ui_Label21, buffer2);
 //Current and Power CH2
 snprintf(buffer3, sizeof(buffer3), "%.3f", Current2);
 lv_label_set_text(ui_Label56, buffer3);
 snprintf(buffer4, sizeof(buffer4), "%.3f", P2);
 lv_label_set_text(ui_Label57, buffer4);
 //Current and Power CH3
 snprintf(buffer5, sizeof(buffer5), "%.3f", Current3);
 lv_label_set_text(ui_Label48, buffer5);
 snprintf(buffer6, sizeof(buffer6), "%.3f", P3);
 lv_label_set_text(ui_Label49, buffer6);
 //Current and Power CH4
 snprintf(buffer7, sizeof(buffer7), "%.3f", Current4);
 lv_label_set_text(ui_Label50, buffer7);
 snprintf(buffer8, sizeof(buffer8), "%.3f", P4);
 lv_label_set_text(ui_Label51, buffer8);
 //Current and Power CH5
 snprintf(buffer9, sizeof(buffer9), "%.3f", Current5);
 lv_label_set_text(ui_Label52, buffer9);
 snprintf(buffer10, sizeof(buffer10), "%.3f", P5);
 lv_label_set_text(ui_Label53, buffer10);

  //Voltage and Freq
 snprintf(buffer11, sizeof(buffer11), "%.2f", Vol);
 lv_label_set_text(ui_Label54, buffer11);
 snprintf(buffer12, sizeof(buffer12), "%.2f", Frequency);
 lv_label_set_text(ui_Label55, buffer12);

  //Energy Kwh
 snprintf(buffer13, sizeof(buffer13), "%.3f", E1); //Energy1
 lv_label_set_text(ui_Label35, buffer13);

 snprintf(buffer14, sizeof(buffer14), "%.3f", E2);
 lv_label_set_text(ui_Label36, buffer14); 

 snprintf(buffer15, sizeof(buffer15), "%.3f", E3);
 lv_label_set_text(ui_Label37, buffer15); 

 snprintf(buffer16, sizeof(buffer16), "%.3f", E4);
 lv_label_set_text(ui_Label38, buffer16); 

 snprintf(buffer17, sizeof(buffer17), "%.3f", E5);
 lv_label_set_text(ui_Label39, buffer17); 
  }
// float Current1 = 67.089;
// char buffer1[20];
//  snprintf(buffer1, sizeof(buffer1), "%.3f", Current1);
//  lv_label_set_text(ui_Label20, buffer1);
//  snprintf(buffer2, sizeof(buffer2), "%.2f", P1);
//  lv_label_set_text(ui_Label21, buffer2);
    lv_timer_handler();
    delay(5);
}
