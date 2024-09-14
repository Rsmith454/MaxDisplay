/*********************
 *      INCLUDES
 *********************/
#include "lv_speedometer.h"
#include "lib/VescUart.h"
#include "lib/SimpleKalmanFilter.h"

 /*********************
  *      DEFINES
  *********************/
#define DISPLAY_WIDTH 320
#define DISPLAY_HEIGHT 170

  /**********************
   *      TYPEDEFS
   **********************/
enum
{
    BEGIN = 0,
    AMP_METER = BEGIN,
    TEMP,
    END,
};

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void lv_create_speedometer(lv_obj_t *parent);
static void lv_create_infopanel(lv_obj_t *parent);
static void lv_create_amp_meter(lv_obj_t *parent);
static void lv_create_temp(lv_obj_t *parent);
static void set_value(lv_obj_t *indic, int32_t v);
static void anim_x_cb(void *var, int32_t v);
static void anim_size_cb(void *var, int32_t v);
static void my_timer(lv_timer_t *timer);

/**********************
 *  STATIC VARIABLES
 **********************/
static lv_style_t style_bg;
static lv_style_t style_label_big;
static lv_style_t style_label_small;
static lv_style_t style_label_extra_small;
static lv_style_t style_bar;

static lv_obj_t *speed_label;
static lv_obj_t *vesc_temp_label;
static lv_obj_t *motor_temp_label;
static lv_obj_t *current_label;
static lv_obj_t *current_bar;

static lv_align_t speed_label_align = LV_ALIGN_TOP_LEFT;
static lv_align_t temp_label_align = LV_ALIGN_RIGHT_MID;
static lv_align_t current_align = LV_ALIGN_BOTTOM_MID;

static lv_anim_t a;

static VescUart UART;
static HardwareSerial SerialPort(2);
static SimpleKalmanFilter filter_current(2, 2, 0.01);
static SimpleKalmanFilter filter_vesc(2, 2, 0.01);
static SimpleKalmanFilter filter_motor(2, 2, 0.01);

static int Poles = 56;                  //Usually 46 for hub motor
static float WheelDia = 0.6278;           //Wheel diameter in m
static float GearReduction = 1;         //reduction ratio. 1 for direct drive. Otherwise motor pulley diameter / Wheel pulley diameter.

float rpm;
float voltage;
float current;
int power;
float amphour;
float tach;
float distance;
float velocity;
float watthour;
float batpercentage;
/**********************
 *      MACROS
 **********************/


 /**********************
  *   GLOBAL FUNCTIONS
  **********************/

void  lv_create_main_screen(void)
{
    // Create a new screen
    lv_obj_t *screen = lv_obj_create(NULL);
    lv_obj_set_size(screen, DISPLAY_WIDTH, DISPLAY_HEIGHT);
    lv_align_t align = LV_ALIGN_CENTER;
    lv_obj_set_align(screen, align);

    lv_style_init(&style_bg);
    lv_style_set_bg_color(&style_bg, lv_color_black());
    lv_obj_add_style(screen, &style_bg, 0);

    lv_obj_set_scrollbar_mode(screen, LV_SCROLLBAR_MODE_OFF);

    lv_scr_load(screen);

    lv_style_init(&style_label_big);
    lv_style_reset(&style_label_big);
    lv_style_set_text_color(&style_label_big, lv_color_white());
    lv_style_set_text_font(&style_label_big, &RedditSans_125);
    lv_style_set_opa(&style_label_big, LV_OPA_COVER);

    speed_label = lv_label_create(screen);
    lv_obj_align(speed_label, speed_label_align,2,0);
    lv_obj_add_style(speed_label, &style_label_big, 0);
    lv_label_set_text(speed_label, "040");

    lv_style_init(&style_label_small);
    lv_style_reset(&style_label_small);
    lv_style_set_text_color(&style_label_small, lv_color_white());
    lv_style_set_text_font(&style_label_small, &RedditSans_60);
    lv_style_set_opa(&style_label_small, LV_OPA_COVER);

    lv_style_init(&style_label_small);
    lv_style_reset(&style_label_small);
    lv_style_set_text_color(&style_label_small, lv_color_white());
    lv_style_set_text_font(&style_label_small, &RedditSans_60);
    lv_style_set_opa(&style_label_small, LV_OPA_COVER);

    lv_style_init(&style_label_extra_small);
    lv_style_reset(&style_label_extra_small);
    lv_style_set_text_color(&style_label_extra_small, lv_color_white());
    lv_style_set_text_font(&style_label_extra_small, &RedditSans_32);
    lv_style_set_opa(&style_label_extra_small, LV_OPA_COVER);

    lv_obj_t *vesc_label = lv_label_create(screen);
    lv_obj_align(vesc_label, LV_ALIGN_TOP_RIGHT, -2, 0);
    lv_obj_add_style(vesc_label, &style_label_extra_small, 0);
    lv_label_set_text(vesc_label, "VESC");

    vesc_temp_label = lv_label_create(screen);
    lv_obj_align(vesc_temp_label, LV_ALIGN_TOP_RIGHT, 0, 15);
    lv_obj_add_style(vesc_temp_label, &style_label_small, 0);
    lv_label_set_text(vesc_temp_label, "000");

    lv_obj_t *motor_label = lv_label_create(screen);
    lv_obj_align(motor_label, LV_ALIGN_RIGHT_MID, -2, -17);
    lv_obj_add_style(motor_label, &style_label_extra_small, 0);
    lv_label_set_text(motor_label, "MOTOR");

    motor_temp_label = lv_label_create(screen);
    lv_obj_align(motor_temp_label, LV_ALIGN_RIGHT_MID, 0, 11);
    lv_obj_add_style(motor_temp_label, &style_label_small, 0);
    lv_label_set_text(motor_temp_label, "000");

    lv_obj_t *current_text_label = lv_label_create(screen);
    lv_obj_align(current_text_label, LV_ALIGN_RIGHT_MID, -2, 39);
    lv_obj_add_style(current_text_label, &style_label_extra_small, 0);
    lv_label_set_text(current_text_label, "CURRENT");

    current_label = lv_label_create(screen);
    lv_obj_align(current_label, LV_ALIGN_BOTTOM_RIGHT, -2, 4);
    lv_obj_add_style(current_label, &style_label_small, 0);
    lv_label_set_text(current_label, "000");

    lv_style_init(&style_bar);
    lv_style_set_radius(&style_bar, 1);

    static lv_style_t style_indic;

    lv_style_init(&style_indic);
    lv_style_set_bg_opa(&style_indic, LV_OPA_COVER);
    lv_style_set_bg_color(&style_indic, lv_color_hex(0x00ff00));
    lv_style_set_bg_grad_color(&style_indic, lv_color_hex(0xff0000));
    lv_style_set_bg_grad_dir(&style_indic, LV_GRAD_DIR_HOR);
    lv_style_set_radius(&style_indic, 1);

    current_bar = lv_bar_create(screen);
    lv_obj_set_size(current_bar, 235, 37);
    lv_obj_align(current_bar, LV_ALIGN_BOTTOM_LEFT, 2, -6);
    lv_bar_set_value(current_bar, 100, LV_ANIM_OFF);
    lv_obj_add_style(current_bar, &style_bar, 0);
    lv_obj_add_style(current_bar, &style_indic, LV_PART_INDICATOR);

    static lv_point_t line_points[] = { {0,0}, {0,40} };

    static lv_style_t style_line;
    lv_style_init(&style_line);
    lv_style_set_line_width(&style_line, 2);
    lv_style_set_line_color(&style_line, lv_color_black());
    lv_style_set_line_rounded(&style_line, true);

    for(size_t i = 0; i < 10; i++)
    {
        lv_obj_t *line = lv_line_create(current_bar);
        lv_line_set_points(line, line_points, 2);     /*Set the points*/
        lv_obj_add_style(line, &style_line, 0);
        lv_obj_align(line, LV_ALIGN_LEFT_MID, 0 + i*23.5, 0);
    }

    lv_anim_init(&a);
    lv_anim_set_var(&a, current_bar);
    lv_anim_set_time(&a, 1500);
    lv_anim_set_playback_delay(&a, 100);
    lv_anim_set_playback_time(&a, 600);
    lv_anim_set_repeat_delay(&a, 500);
    lv_anim_set_repeat_count(&a, 1);
    lv_anim_set_exec_cb(&a, anim_x_cb);
    lv_anim_set_values(&a, 0, 100);
    lv_anim_start(&a);

    lv_timer_t * timer = lv_timer_create(my_timer, 500,  NULL);

    SerialPort.begin(115200, SERIAL_8N1, 18, 17);
    UART.setSerialPort(&SerialPort);
}

/**********************
 *   STATIC FUNCTIONS
 **********************/
static void my_timer(lv_timer_t * timer)
{
    if ( UART.getVescValues() ) 
    {
        rpm = UART.data.rpm / (Poles / 2);                                // UART.data.rpm returns cRPM.  Divide by no of pole pairs in the motor for actual.
        voltage = (UART.data.inpVoltage);                                 //Battery Voltage
        current = (UART.data.avgInputCurrent);                            //Current Draw
        velocity = rpm*3.142*(60.0/1000.0)*WheelDia*GearReduction;
    }

    float current_filtered = filter_current.updateEstimate(current);
    float vesc_filtered = filter_vesc.updateEstimate(UART.data.tempMosfet);
    float motor_filtered = filter_motor.updateEstimate(UART.data.tempMotor);

    lv_label_set_text_fmt(speed_label, "%d", (int)velocity);
    lv_label_set_text_fmt(vesc_temp_label, "%d", (int)vesc_filtered);
    lv_label_set_text_fmt(motor_temp_label, "%d", (int)motor_filtered);
    lv_label_set_text_fmt(current_label, "%d", (int)current_filtered);
    
    int bar_current = lv_map(current_filtered, 0, 700, 0, 100);
    lv_bar_set_value(current_bar, bar_current, LV_ANIM_ON);
}

static void set_value(lv_obj_t *indic, int32_t v)
{

}

static void anim_x_cb(void * var, int32_t v)
{
    lv_bar_set_value((lv_obj_t*)var, v, LV_ANIM_OFF);
    lv_label_set_text_fmt(speed_label, "%d", v);
    lv_label_set_text_fmt(vesc_temp_label, "%d", v);
    lv_label_set_text_fmt(motor_temp_label, "%d", v);
    lv_label_set_text_fmt(current_label, "%d", v);
}


static void anim_size_cb(void * var, int32_t v)
{
    lv_obj_set_size((lv_obj_t *)var, v, v);
}
