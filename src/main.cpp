#define serial_debug   1            //set to '1'  to show debug lines in serial monitor

/**********************
    Include files
 **********************/
#include "FS.h"                     //enable spiff file system on esp 32 (for touch cal data)
#include "lvgl/lvgl.h"                   //This is the graphics library
#include <SPI.h>                    // files for spi
#include <TFT_eSPI.h>               // routines for littlevgl     
#include <Ticker.h>                 //interupt used with littlVgl graphics engine
#include <stdio.h>
#include "lv_ex_conf.h"
#include "lv_conf.h"             //config file to setup lines that go to display and touch screen
#include <EEPROM.h>                 //eeprom utilities
//#include <NMEAGPS.h>                //include for GPS sensor
#include "TouchScreen.h"
//#include "WiFi.h"
#include "Read_User_Setup.h"
/**********************
    Define IO pins
 **********************/
#define YP 12                      //analog pin touch screen
#define XM 13                      //analog pin touch screen
#define YM 14                      //digital pin touch screen
#define XP 27                      //digital pin touch screen
#define status_light 4             //on board led for diagnostic use
#define alarm_light 26              //panel LED alarm light
#define aux_light 27                 //auxillary output line

#define EEPROM_SIZE 500              //must declare how many bytes for eeprom

#define LVGL_TICK_PERIOD 20         //internal timing of graphics module(was 20)
/**********************
    Define Colors
 **********************/
#define BLACK 0x0000                //declare constants for colors
#define NAVY 0x000F
#define DARKGREEN 0x03E0
#define DARKCYAN 0x03EF
#define MAROON 0x7800
#define PURPLE 0x780F
#define OLIVE 0x7BE0
#define LIGHTGREY 0xC618
#define DARKGREY 0x7BEF
#define BLUE 0x001F
#define GREEN 0x07E0
#define CYAN 0x07FF
#define RED 0xF800
#define MAGENTA 0xF81F
#define YELLOW 0xFFE0
#define WHITE 0xFFFF
#define ORANGE 0xFD20
#define GREENYELLOW 0xAFE5
#define PINK 0xF81F

/*********************
   EEPROM varibles
 *********************/
//EEprom address values
#define unit_ee_adr 1                              //location 1, 0= mph 1= kph 2= dis 3= mpm
#define guage_ee_adr 2                             //location 2, 0 = digital readout, 1 = analog meter readout 2= bar graph
#define cal_ee_edr 5                               //calibration number (float number
#define security_ee_adr 10                         //Enable Security check box
#define graph_ee_adr 11                            //status of enable bar graph check box 
#define dis_ee_adr 12                              //distance checkbox status
#define alarm_ee_adr 13                           //alarm status
#define speed_target_ee_adr 15                     //target speed (float number)    
#define fpm_ee_adr 21                              //feet per minute checkbox
#define upw_ee_adr 25                              //user password code  
#define speed_avg_ee_adr 29                        //speed average checkbox status
#define screen_cal_ee_adr 35                       //set to 0 to force screen calibration and 1 to prevent screen calibration
#define speed_input_ee_adr 40                      //0= radar or wheelpulse  input, 1 = gps input
#define speed_target_1_ee_adr 45                   //target speed 1
#define speed_target_2_ee_adr 50                   //target speed 2
#define speed_target_3_ee_adr 55                   //target speed 3
#define speed_target_4_ee_adr 60                  //target speed 4
/**********************
    Graphics engine parameters
 **********************/
Ticker tick;                                        /* timer for interrupt handler */
TFT_eSPI tft = TFT_eSPI();                          /* TFT instance */

static lv_disp_buf_t disp_buf;                      //declare buffer for graphics
static lv_color_t buf[LV_HOR_RES_MAX * 10];         //color depth of display
// static  int x = 90;

/**************************                         // This is the file name used to store the calibration data
   Touch Screen Calibration                         // You can change this to create new calibration files.
 **************************/                        // The SPIFFS file name must start with "/".
TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);          //create instance of touch read program (last number is resistance across touch screen)

#define CALIBRATION_FILE "/TouchCalData2"           //give name to file that holds touch screen cal values


//#define REPEAT_CAL == true                           // Set REPEAT_CAL to true instead of false to run calibration

#if USE_LV_LOG != 0
/* Serial debugging */
void my_print(lv_log_level_t level, const char * file, uint32_t line, const char * dsc)
{
  Serial.printf("%s@%d->%s\r\n", file, line, dsc);
  delay(100);
}
#endif

/**********************
    STATIC PROTOTYPES
 **********************/
// static void btn_event_cb(lv_obj_t * btn, lv_event_t event);
// static void ddlist_event_cb(lv_obj_t * ddlist, lv_event_t event);
// static lv_obj_t * slider;          //create a slider object
/**********************
    varibles
 **********************/
bool REPEAT_CAL = true;                           // Set REPEAT_CAL to true instead of false to run calibration
char software_ver[20] = "Software Ver 1.29";
int pulse_reset;                 //flag to restart timer used for calc speed on start of pulse
int var_REPEAT_CAL ;               //varible use in touch screen calibration routine to enact a recalibration
bool screen_start_flag = 0;
long task_millis = 0;             //timer to run the task
long old_millis = 0;              //used in loop for timing function
long alarm_millis = millis();
char text_buff[20];               //buffer to hold text strings
char temp_buff[30];
int cal_number;                   //(saved in eeprom)
int old_cal_number;               //save old calibration number varible
int run_screen_flag = 1;          //flag to tell loop to calculate and display speed
byte units;                       //saved in eeprom
byte index_sec;
bool bar_flash;                  //flag to flash bar graph when over target speed
const char * txt;
bool title_create_flag;
bool field_cal_flag;
volatile int pulse,old_pulse;   //count of pulses from speed detector
float speed_constant;           //speed_constant = (cal_number * 17.6) / 3600
byte security;                  //checkbox status of "Enable Security"
byte dis;                       //checkbox status of "feet per minute disply
byte graph;                     //checkbox status of "Enable Bar Graph"
byte alarm_enable;                     //flag to indicate option alarm status
byte fpm;                       //feet per minute checkbox
byte avg;                       //average speed checkbox
String hold_text = "";          //used in calculation
String user_passcode = "";           //user password to get into setup
float speed_target;             //value of middle of bar graph
int field_calibration;          // number of pulses read in a 300 ft run
float pulse_distance = 0;       //the distance of one pulse from speed sensing device (3600/pulse)
float distance = 0;             //distance since last reset
float total_pulse;              //distance pulses recieved since last reseet
int hor_pos;                    //temporary varible used when setting up horizontal position parameters
int user_passcode_int;          //user passcode in integerformat
char gps_buffer[75];            //buffer to save gps string in
int gps_pnt;                    //pointer used in gps_buffer
float velocity;                 //varible to  hold current speed reading  
float old_velocity;             // reference to last speed reading
bool status_mode;
bool calc_flag;                     //used to indicate program to calculate speed
bool out_state;
bool diagnostic_flag;               // if set diagnostic values appear in bar graph area.
bool speed_avg;                     //when turned on system does a 4 reading average ( 3 x last reading plus current reading / 4)
byte speed_input;                   //checkboxes for radar or gps input 0 = radar input 1 = gps input
bool field_calibration_flag;        //used to tell interupt timer we are in field calibration
hw_timer_t * timer = NULL;          //create a hardware timer
hw_timer_t * flash_timer = NULL;    //timer used to varible flash the alarm light
int serial_pointer;
char serial_buffer[125]="";
int gps_string_pointer;
static int alarm_var;                      //varible that holds the active alarm value being edited (1-4)
float alarm_speed_1;
float alarm_speed_2;
float alarm_speed_3;
float alarm_speed_4;
/**********************
    Global objects
 **********************/
//declare labels

static lv_style_t  style3;
static lv_style_t  style4;
lv_obj_t * line1;                          //screen line
lv_obj_t * line2;


//Run Screen objects
static lv_obj_t * title_label;           //title at top of screens
lv_obj_t * btn_setup;                    //setup button on run screen
static lv_obj_t * label_units;           //label that displays MPH/KPH/FPM/MPM
static lv_obj_t *label_speed;            //large speed text shown on run screen
static lv_obj_t * bar_speed;             //bar graph for target speed
static lv_obj_t * mbox1;                //3 button box  "Calibrate, Options, Close"
lv_obj_t * label_target_speed;
lv_obj_t * label_bar_max;
lv_obj_t * label_bar_min;
lv_obj_t * btn_reset_dist;              //button to reset distance
lv_obj_t * lab_fpm;
lv_obj_t * unit_line;                   //lines around units and distance per minute
lv_obj_t * label_alarm_icon;
lv_obj_t * messbox_gps_search;           //message box that comes up while searching for satellites
lv_obj_t * label_gps_search_icon;        //icon for 'gps searching'
lv_obj_t * label_gps_lock_icon;        //icon for 'gps searching'
//Target Screen objects
lv_obj_t * btn_set_exit;               //button to exit screen
lv_obj_t * keypad_target;              //keypad to enter number
static lv_obj_t * label_cal;           //large numbers to display number being set
static lv_obj_t * btn_target_exit;     //exit button on "Set Target Speed" screen

// Mulitple Alarm Set objects
lv_obj_t * title_label_set;
lv_obj_t * btn_set_alarm_exit;
lv_obj_t * btn_set_alarm_1;            //alarm 1 button
lv_obj_t * btn_set_alarm_2;            //alarm 2 button
lv_obj_t * btn_set_alarm_3;            //alarm 3 button
lv_obj_t * btn_set_alarm_4;            //alarm 4 button
lv_obj_t * label_set_1;                //label for button 1
lv_obj_t * label_set_2;                //label for button 2
lv_obj_t * label_set_3;                //label for button 3
lv_obj_t * label_set_4;                //label for button 4
lv_obj_t * ma_keypad;                  //keypad to set multi alarm spees
lv_obj_t * label_alarm1;               //label to display preset alarm speeds
lv_obj_t * label_alarm2;               //label to display preset alarm speeds
lv_obj_t * label_alarm3;               //label to display preset alarm speeds
lv_obj_t * label_alarm4;               //label to display preset alarm speeds

//Field Calibration objects
lv_obj_t * field_start_btn;            //start button
lv_obj_t * field_end_btn;              //end button
lv_obj_t * field_save_btn;             //save button
lv_obj_t * field_cancel_btn;           //cancel button
lv_obj_t * label_300_ft;               //label over graphic line between arrows
lv_obj_t * right_arrow;                //graphic arrow
lv_obj_t * left_arrow;                 //graphic arrow
lv_obj_t * line_fld_cal_hor;           //graphic line between arrows
lv_obj_t * label_field_cal;
lv_obj_t * messbox_start_run;
lv_obj_t * label_cal_inst;            //instructions on field calibration

//Option Screen objects
lv_obj_t * cb_mph;                     //checkbox mph
lv_obj_t * cb_kph;                     //checkbox kph
lv_obj_t * cb_distance;                //checkbox 'show distance'
lv_obj_t * cb_security;                //checkbox password enable
lv_obj_t * cb_graph;                   //checkbox 'show bar graph'
lv_obj_t * cb_feet_per_min;
lv_obj_t * cb_speed_avg;
static lv_obj_t * btn_target_set;      //'set target' button on option screen
static lv_obj_t * btn_preAlarm_set;      //button to set the pre alarm value
lv_obj_t * option_descrip_text;         //explanation of the option screen buttons
lv_obj_t * calibrate_descrip_text;        //explanation of the option screen buttons
lv_obj_t * close_descrip_text;
lv_obj_t * cb_alarm;                    //checkbox to flash bar graph on alarm status
lv_obj_t * label_software;                    //label to display software version on option setup screen 
lv_obj_t * cb_gps;
lv_obj_t * cb_radar;

//Speed cal manual entry (screen_calibrate)
lv_obj_t * btn_exit;                   //exit button on calibration screen
lv_obj_t * btn_set_cal;                //set calibration number button
lv_obj_t *btn5_plus;                   //buttons to set calibration number
lv_obj_t *btn5_minus;
lv_obj_t *btn4_plus;
lv_obj_t *btn4_minus;
lv_obj_t *btn3_plus;
lv_obj_t *btn3_minus;
lv_obj_t *btn2_plus;
lv_obj_t *btn2_minus;
lv_obj_t *btn1_plus;
lv_obj_t *btn1_minus;
lv_obj_t *btn0_save;
lv_obj_t *btn0_abort;
lv_obj_t *label_arrow;
lv_obj_t * btn_field_cal;
lv_obj_t * keypad;                         //keypad used to enter new security code
lv_obj_t * tick1;                          //create an object for ruler line below bar graph

//factory screen objects
lv_obj_t * btn_fact_exit;                  //button to exit factory screen
lv_obj_t * btn_reset_pw;                   //button to reset password
lv_obj_t * btn_scr_cal;                    //button to calibrate screen
lv_obj_t * keypad_pw;                      //keypad to enter new passcode
lv_obj_t * pw_but_text;                    //text beside new password button
lv_obj_t * touch_cal_but_text;             //text beside cal touch screen button
lv_obj_t * pw_text;                        //label for password text
lv_obj_t * btn_reset;                      //reset computer

static const char * btnm_map[] = {"1", "2", "3", "4", "5", "Enter", "\n", //text for keys on keypad (last field must be a null)
                                  "6", "7", "8", "9", "0", "Clear", ""
                                 };
/********************
    Interupt routines
 ********************/

//***   speed input interrupt ******** 
void IRAM_ATTR speed_pulse () {                                    //interupt driven pulse counter when using pulse counter mode
   if (pulse == 0){
      timerWrite(timer,0);                                          //restart timer from 0 on first pulse to realign clock with first pulse
      }
   pulse = pulse + 1;                                               //increment the pulse counter if 250 ms have not elapsed
 
   }


//**** TImer interupt  ********
void IRAM_ATTR onTimer(){                                              //interrupt routine for 250 ms timer used to update mph screen
    if (field_calibration_flag == false){                           //if not in field calibration mode
       old_pulse = pulse;                                               //save pulse count
        pulse = 0;                                                      //reset pulse counter
        calc_flag = true;                                               //set flag since 250ms have elapsed
        }
  }


void alarm_Flash_Timer(){
     if (alarm_enable == 1&& (velocity > 1)) {                           //if alarm bit is set turn on alarm light if in range
                                                                         //start flashing light 1 mph below target alarm
                                                                         
            
            if (velocity >= speed_target){
              digitalWrite(alarm_light,true);                    //solid alarm light if over target speed
               }
            else if (velocity >= speed_target  - .1){
              if(millis()- alarm_millis >= 50){
                   digitalWrite(alarm_light,!digitalRead(alarm_light));     //toggle light
                   alarm_millis = millis();                             //reload counter
                   }
              }
            else if (velocity >= speed_target  - .2){
              if(millis()- alarm_millis >= 100){
                   digitalWrite(alarm_light,!digitalRead(alarm_light));     //toggle light
                   alarm_millis = millis();                             //reload counter
                   }
               }
            else if (velocity >= speed_target  - .4){
              if(millis()- alarm_millis >= 175){
                   digitalWrite(alarm_light,!digitalRead(alarm_light));     //toggle light
                   alarm_millis = millis();                             //reload counter
                   }
            }
            else if (velocity >= speed_target  - .6){
              if(millis()- alarm_millis >= 325){
                   digitalWrite(alarm_light,!digitalRead(alarm_light));     //toggle light
                   alarm_millis = millis();                             //reload counter
                   }
            }
            else if (velocity >= speed_target  - .8){
              if(millis()- alarm_millis >= 750){
                   digitalWrite(alarm_light,!digitalRead(alarm_light));     //toggle light
                   alarm_millis = millis();                             //reload counter
                   }
            }
            else if (velocity >= speed_target  - 1){
              if(millis()- alarm_millis >= 1000){
                   digitalWrite(alarm_light,!digitalRead(alarm_light));     //toggle light
                   alarm_millis = millis();                             //reload counter
                   }
            }
            else{                                                //if not in alarm range,turn off alarm light 
              digitalWrite(alarm_light,false);
            }
           
     }
//     else {
//      digitalWrite(alarm_light,false);                           //turn off alarm light if not in alarm mode  (alarm_enable ==0)      
//     }
  }

void create_title_line(void) {

  static lv_point_t line_points[] = { {2, 22}, {480, 22}};              /*Create an array for the points of the line*/
  static lv_style_t style_line;                                        /*Create new style (thick dark blue)*/
  lv_style_copy(&style_line, &lv_style_plain);
  style_line.line.color = LV_COLOR_MAKE(0x00, 0x3b, 0x75);              //set the color of the line to dark blue
  style_line.line.width = 6;                                            //line width in pixels
  style_line.line.rounded = 1;                                          //pixel radius of ends


  line1 = lv_line_create(lv_scr_act(), NULL);
  lv_line_set_points(line1, line_points, 2);                              /*Identify the 2 points  declared in the array*/
  lv_line_set_style(line1, LV_LINE_STYLE_MAIN, &style_line);             //apply a style to the line
  lv_obj_align(line1, NULL, LV_ALIGN_CENTER, 0, -130);                   //set position of line
  lv_obj_set_hidden(line1, true);                                        //hide the line after creating

  static lv_point_t line_points2[] = { {2, 235}, {480, 235}};            //endpoints of line
  line2 = lv_line_create(lv_scr_act(), NULL);                            //create an object
  lv_line_set_points(line2, line_points2, 2);                            /*Identify the 2 points  declared in the array*/
  lv_line_set_style(line2, LV_LINE_STYLE_MAIN, &style_line);             //apply a style to the line
  lv_obj_set_hidden(line2, true);                                        //hide the line

}

//FUNCTION STUBS
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p);
void build_screen_calibrate(void);
void build_option_screen();
void build_screen_target(void);
void build_set_target(void);
bool my_input_read(lv_indev_drv_t * drv, lv_indev_data_t*data);
static void lv_tick_handler(void);
void calculate_speed_constant(void);
void mult_alarm_cb(lv_obj_t * btn, lv_event_t event);
void ma_keyboard_cb(lv_obj_t * obj, lv_event_t event);
void test_flash_alarm(void);
void enable_gps(void);
void enable_radar(void);
void btn_reset_dist_cb(lv_obj_t * btn, lv_event_t event);
void speed_bar(void);
void lv_ex_mbox_1(void);
static void lv_security_code(void);
static void security_event_handler(lv_obj_t * obj, lv_event_t event);
static void mbox1_handler_cb(lv_obj_t * obj, lv_event_t event);
void lv_ex_btnm_1(void);
void target_speed_event_handler(lv_obj_t * obj, lv_event_t event);
static void btn_set_cal_cb(lv_obj_t * obj, lv_event_t event);
static void cal_set_cb(lv_obj_t * btn, lv_event_t event);
static void btn_target_exit_cb(lv_obj_t * btn, lv_event_t event);
// static void cal_btn_exit_cb(lv_obj_t * obj, lv_event_t event);
static void my_start_btn_cb(lv_obj_t * obj, lv_event_t event);
static void unit_event_cb(lv_obj_t * obj, lv_event_t event);
void touch_calibrate();
void build_screen_calibrate(void);
void build_screen_run(void);
void build_field_calibrate(void);
void screen_calibrate_off(void);
void screen_run_off(void);                                               
void screen_target_off(void);
void build_factory_screen(void);                                              //factory settings (new user password, touch screen cal.)
void option_screen_off(void);
void screen_run_on(void); 
void screen_set_multi_target_on(void);
void fact_opt_cb(lv_obj_t * obj, lv_event_t event);
void new_pw_event_handler(lv_obj_t * obj, lv_event_t event);
void screen_calibrate_cb(lv_obj_t * obj, lv_event_t event);
void field_calibrate_cb(lv_obj_t * obj, lv_event_t event);





//=========================  SETUP  ============================================
void setup() {
  task_millis = millis();                               //align task timer to current time
  old_millis = millis();                                //align loop timer to current time
  Serial.begin(115200);                                 //serial debug screen
 // Serial2.begin(19200,SERIAL_8N1,25,22);                //uart 2 being used with gps module,25-RX, 22-TX
  lv_init();                                            //start the littlevgl graphics engine
  pinMode(12, OUTPUT);                                  //diagnostic output
  pinMode(25, INPUT);                                   //Speed input pin
  pinMode(status_light,OUTPUT);                         //on board status led
  pinMode(alarm_light, OUTPUT);                          //diagnostic output
  pinMode(aux_light, OUTPUT);                          //diagnostic output
//  while (Serial2.available() > 0) {
//    Serial.print(char(Serial2.read()));                //send to serial monitor
//  }
  
  EEPROM.begin(50);                                    //reserve 50 bytes or eeprom storage
#if USE_LV_LOG != 0
  lv_log_register_print(my_print);                          /* register print function for debugging */
#endif
  tft.init();                                                  //start the lcd display driver
  tft.setRotation(1);                                         /* Set the  orientation to landscape*/
  lv_disp_buf_init(&disp_buf, buf, NULL, LV_HOR_RES_MAX * 10);   //set the color depth
  /**********************
     Initialize the display
  **********************/
  lv_disp_drv_t disp_drv;                                     //create an instance of the display driver
  lv_disp_drv_init(&disp_drv);                                //initialize the driver
  //set the size of the display below
  disp_drv.hor_res = 480;                                     //width max of display
  disp_drv.ver_res = 320;                                     //height max of display

  disp_drv.flush_cb = my_disp_flush;                          //call back routine to flush screen buffer
  disp_drv.buffer = &disp_buf;
  lv_disp_drv_register(&disp_drv);                            //register the display driver
  /**********************
     Touch Screen Setup
     Initialize the touch pad
  **********************/
  lv_indev_drv_t indev_drv;                                     //create an instance of he driver
  lv_indev_drv_init(&indev_drv);                                //initialize the driver
  indev_drv.type = LV_INDEV_TYPE_POINTER;                       //type of input
  indev_drv.read_cb = my_input_read;                            //call back routine to call
                                         //this only runs the first time unless REPEAT_CAL is set to true
  //*Register the driver in LittlevGL and save the created input device object
  // lv_indev_t * my_indev = lv_indev_drv_register(&indev_drv);    //register the input device
  lv_indev_init();  
   
  //-------------------------------------------------------------------------------
  /**********************
     Initialize the graphics library's tick
  **********************/
  tick.attach_ms(LVGL_TICK_PERIOD, lv_tick_handler);
  old_millis = millis();                                      //save current milli count to varible
  /**********************
      load varibles from eeprom
   **********************/
  EEPROM.get(security_ee_adr, security);                        //Enable Security check box
  EEPROM.get(graph_ee_adr, graph);                              //status of enable bar graph check box
  EEPROM.get(dis_ee_adr, dis);                                  //status enable distance check box
  EEPROM.get(speed_target_ee_adr, speed_target);                //speed target for bar graph float number
  EEPROM.get(cal_ee_edr, cal_number);                           //get calibration number from eeprom
  old_cal_number = cal_number;                                  //set varible the same as cal number
  EEPROM.get(unit_ee_adr, units);                               //get units 1 = MPH  2= KPH
  calculate_speed_constant();                                   //calculate the constant used for speed calculation
  EEPROM.get(fpm_ee_adr, fpm);                                  //feet per minute checkbox status
  EEPROM.get(alarm_ee_adr, alarm_enable);                       //alarm notification 0 = off 1 = on
  EEPROM.get(speed_avg_ee_adr, speed_avg);                      //status of speed average checkbox
  EEPROM.get(speed_input_ee_adr, speed_input);                  //0= radar 1= gps
  
  EEPROM.get(speed_target_1_ee_adr,alarm_speed_1);                //target alarm speeds
  EEPROM.get(speed_target_2_ee_adr,alarm_speed_2);
  EEPROM.get(speed_target_3_ee_adr,alarm_speed_3);
  EEPROM.get(speed_target_4_ee_adr,alarm_speed_4);
  EEPROM.get(screen_cal_ee_adr,var_REPEAT_CAL);                  //screen calibration flag
    if (var_REPEAT_CAL == 55){                                    //if screen is calibrated then 55 is saved toeeprom value
       REPEAT_CAL = false; 
      }
    else{
      REPEAT_CAL = true;                                          //set to true so touch pan calibration will be performed (touch screen routine)  
    }

  speed_target = 4.0;   //***diagnostic (force a value) 


  
  // Calibrate the touch screen and set the scaling factors
  touch_calibrate();    //start the input device
  EEPROM.get(upw_ee_adr, user_passcode_int);                    //user pass code saved as an integer, converted and used as string in program
  
  char data_var[8];                                             //create char array to hold value
  sprintf(data_var, "%dE", user_passcode_int);                   //convert to a string
  user_passcode =  String(data_var);                             //set pass code to saved eeprom value
  if (user_passcode == "-1") {                                    //if not value for passcode then set to "1234"
    Serial.println(user_passcode);
    user_passcode = "1234E";                                      //set default password to 1234
    Serial.println("Default password set to 1234E");
  }
  else {
    Serial.println(user_passcode);                                //***diagnostic line
    Serial.println(String("User password set to " + user_passcode));
  }

  //create an instance of object
  title_label = lv_label_create(lv_scr_act(), NULL);                   //label for page title
  lv_label_set_text(title_label, "");                                  //screen title
  label_units = lv_label_create(lv_scr_act(), NULL);                   //create a label object for the active screen
  lv_obj_set_hidden(label_units, true);                                  //mph/kph label
  bar_speed = lv_bar_create(lv_scr_act(), NULL);                       //create an object for bar graph
  lv_bar_set_style(bar_speed, LV_BAR_STYLE_INDIC, &style3);
  lv_obj_set_hidden(bar_speed, true);
  label_bar_min = lv_label_create(lv_scr_act(), NULL);                 //label for min speed on bar graph
  lv_obj_set_hidden(label_bar_min, true);                              //hide label
  label_bar_max = lv_label_create(lv_scr_act(), NULL);                  //label for max speed on bar graph
  lv_obj_set_hidden(label_bar_max, true);                               //hide label

  
  //setup interrupt timer used for 250ms timer
  timer = timerBegin(0,80,true);                                        //create a timer
  timerAttachInterrupt(timer,&onTimer,true);                             //attach callback function to timer
  timerAlarmWrite(timer,250000,true);                                   //set when to call the callback function (250ms)
  timerAlarmEnable(timer);                                              //start the timer alarm
  //setup interrupt for speed pulse counter

  //setup interrupt timer used for 100ms timer for alarm flash routine
//  flash_timer = timerBegin(1,80,true);                                        //create a timer running a 1 mhz
//  timerAttachInterrupt(flash_timer,&alarm_Flash_Timer,true);                   //attach callback function to timer
//  timerAlarmWrite(flash_timer,100000,true);                                   //set elapsed time when to call the callback function (100ms)
//  timerAlarmEnable(flash_timer);                                             //start the timer               
  
  if (speed_input == 0){
    enable_radar();                                                     //turn off serial2 and turn on pulse interrupt
    }
  else{
    enable_gps();                                                     //turn on serial2 and turn off pulse interrupt                                              
    }
 
  create_title_line();                                                 //create screen seperator lines
  build_screen_run();                                                  //build the different screens
  build_option_screen();                                               //set program options
  build_screen_target();                                               //set target speed for center of bar graph
  build_screen_calibrate();                                            //cal number setup with keypad
  build_field_calibrate();                                             //cal number setup by driving distance
  build_factory_screen();                                              //factory settings (new user password, touch screen cal.)
  build_set_target();                                                  //build screen to set target speeds
  option_screen_off();

  screen_run_on();                                                      //turn on the run screen

  Serial.println(" ");
  Serial.print("File Name - ");                                        //print file name and path to serial monitor
  Serial.println(__FILE__);
  Serial.print("Software installed  - ");
  Serial.println(__DATE__);
//  Serial.print("ESP Mac Address - ");                                   //send mac address to serial monitor
//  Serial.println(WiFi,macAddress());
  printPins(tft);
  
}//end 0f setup()
//======================= Screen ON/OFF Routines  =================================
void field_calibrate_on(void) {
  field_calibration_flag = true;                                  //set flag so timer interupt will not reset pulse count
  screen_calibrate_off();                                         //turn off any other screen that may be on
  screen_run_off();                                               //"
  screen_target_off();                                            //"

  lv_obj_set_pos(title_label, 90, 5);
  lv_label_set_text(title_label, "Field Calibration");              /*Set the text*/
  Serial.print("line 270");
  lv_obj_set_y(label_speed, 100);                                  //reset position of speed readout (changed for field run time routine)
  lv_obj_set_x(label_speed, 55);
  lv_obj_set_pos(title_label, 130, 5);
  lv_obj_set_hidden(title_label, false);
  lv_obj_set_hidden(field_start_btn, false);                                 //show inc/dec buttons that set value
  lv_obj_set_hidden(field_end_btn, false);
  lv_obj_set_hidden(field_save_btn, false);
  lv_obj_set_hidden(field_cancel_btn, false);
  lv_obj_set_hidden(label_field_cal, false);
  lv_obj_set_hidden(line_fld_cal_hor, false);
  lv_obj_set_hidden(label_300_ft, false);
  lv_obj_set_hidden(left_arrow, false);
  lv_obj_set_hidden(right_arrow, false);
  lv_obj_set_hidden(line1, false);
  lv_obj_set_hidden(label_cal_inst, false);
#if serial_debug
  Serial.print("line 288");
#endif
}
void field_calibrate_off(void) {                                           //turn off objects on field cal screen
  lv_obj_set_hidden(label_cal, true);
  lv_obj_set_hidden(field_start_btn, true);
  lv_obj_set_hidden(field_end_btn, true);
  lv_obj_set_hidden(field_save_btn, true);
  lv_obj_set_hidden(field_cancel_btn, true);
  lv_obj_set_hidden(label_field_cal, true);
  lv_obj_set_hidden(line_fld_cal_hor, true);
  lv_obj_set_hidden(label_300_ft, true);
  lv_obj_set_hidden(left_arrow, true);
  lv_obj_set_hidden(right_arrow, true);
  lv_obj_set_hidden(label_cal_inst, true);
  field_calibration_flag = false;                                       //set flag to flase so timer interupt will process timer interrupt
  Serial.println("line 298");
}
void option_screen_off(void) {                                          //hide all objects on option screen

  lv_obj_set_hidden(btn_set_exit, true);                        //hide all screen objects that are on the option screen
  lv_obj_set_hidden(cb_mph, true);
  lv_obj_set_hidden(cb_kph, true);
  lv_obj_set_hidden(cb_distance, true);
  lv_obj_set_hidden(cb_security, true);
  lv_obj_set_hidden(cb_graph, true);
  lv_obj_set_hidden(btn_target_set, true);
  lv_obj_set_hidden(btn_preAlarm_set, true);
  
  lv_obj_set_hidden(btn_target_exit, true);
 // lv_obj_set_hidden(label_target_speed, true);
  lv_label_set_text(title_label, "");                            //clear text in screen title
  lv_obj_set_hidden(cb_feet_per_min, true);
  lv_obj_set_hidden(line2, true);
  lv_obj_set_hidden(cb_alarm, true);
  lv_obj_set_hidden(cb_speed_avg, true);
  lv_obj_set_hidden(label_software, true);
  lv_obj_set_hidden(cb_radar, true);
  lv_obj_set_hidden(cb_gps, true);
}
void option_screen_on(void) {                                           //show all hidden objects on option screen
  screen_calibrate_off();
  screen_run_off();
  screen_target_off();
  field_calibrate_off();
  lv_obj_set_hidden(btn_set_exit, false);                        //show all screen objects that are on the option screen
  lv_obj_set_hidden(cb_mph, false);
  lv_obj_set_hidden(cb_kph, false);
  lv_obj_set_hidden(cb_distance, true);                        //this has been turned off for tractor pulling applications
 // lv_obj_set_hidden(cb_distance, false);
  lv_obj_set_hidden(cb_security, false);
  lv_obj_set_hidden(cb_graph, false);
  lv_obj_set_hidden(line1, false);
  lv_obj_set_hidden(line2, false);
  lv_obj_set_hidden(title_label, false);
  //lv_obj_set_hidden(label_target_speed, false);
  lv_obj_set_hidden(cb_feet_per_min, false);
  lv_obj_set_hidden(cb_alarm, false);
  lv_obj_set_hidden(cb_speed_avg, false);
  lv_obj_set_hidden(label_software, false);
  lv_obj_set_hidden(cb_radar, false);
  lv_obj_set_hidden(cb_gps, false);
  

  lv_style_copy(&style3, &lv_style_plain);
  style3.text.font = &lv_font_roboto_28;                         //12,16,22,28 built in
  style3.text.color = LV_COLOR_BLACK;
  Serial.println("line 345");

  snprintf(text_buff, 7, "(%2.1f)", speed_target);                 //get current speed target and display
  lv_label_set_text(label_target_speed, text_buff);

  lv_obj_set_style(title_label, &style3);                          //assign style to the label
  lv_obj_set_pos(title_label, 160, 5);
  lv_label_set_text(title_label, "    Options");                       //assign text to the label

  if (units == 1) {                                                      //units, set checkbox status
    lv_cb_set_checked(cb_mph, 1);
  }
  else {
    lv_cb_set_checked(cb_kph, 1);
  }

  if (security == 1) {
    Serial.println("line 390");
    lv_cb_set_checked(cb_security, 1);
  }            //Enable Password, set checkbox status
  else {
    Serial.println("line 392");
    lv_cb_set_checked(cb_security, 0);
  }

  if (dis == 1) {                                  // FPM, set checkbox status
    lv_cb_set_checked(cb_distance, 1);
  }
  else {
    lv_cb_set_checked(cb_distance, 0);
  }
  if (units == 1) {
    lv_cb_set_text(cb_feet_per_min, "feet/minute");     //if mph mode set for feet/min
  }
  else {
    lv_cb_set_text(cb_feet_per_min, "meters/minute");  //else metric set for meters/min
  }

  if (graph == 1) {                                // Bar Graph, set checkbox status
    lv_cb_set_checked(cb_graph, 1);
    lv_obj_set_hidden(btn_target_set, false);   //show "set target speed" button
    lv_obj_set_hidden(btn_preAlarm_set, false); //show pre alarm button
  
 //   lv_obj_set_hidden(label_target_speed, false);
    lv_obj_set_hidden(cb_alarm, false);
  }
  else {
    lv_cb_set_checked(cb_graph, 0);              //hide "set target speed" button
    lv_obj_set_hidden(btn_target_set, true);
    lv_obj_set_hidden(btn_preAlarm_set, true);
   // lv_obj_set_hidden(label_target_speed, true);
    lv_obj_set_hidden(cb_alarm, true);
  }

  if (alarm_enable == 1) {                          //set the status of alarm checkbox
    lv_cb_set_checked(cb_alarm, 1);
  }
  else {
    lv_cb_set_checked(cb_alarm, 0);
  }
  Serial.println("line 774 end of option_screen_on()");
}
void screen_run_off(void) {                                            //turn off all objects on run screen
  lv_label_set_text(title_label, "");
  lv_obj_set_hidden(title_label, true);
  lv_obj_set_hidden(btn_setup, true);
  lv_obj_set_hidden(label_units, true);
  lv_obj_set_hidden(label_speed, true);
  lv_obj_set_hidden(bar_speed, true);
  lv_obj_set_hidden(label_arrow, true);
  lv_obj_set_hidden(label_bar_min, true);
  lv_obj_set_hidden(label_bar_max, true);
  lv_obj_set_hidden(btn_reset_dist, true);
  lv_obj_set_hidden(lab_fpm, true);
  lv_obj_set_hidden(tick1, true);
  lv_obj_set_hidden(unit_line, true);
  lv_obj_set_hidden(label_alarm_icon, true);
// lv_mbox_start_auto_close(messbox_gps_search, 50); 
  run_screen_flag = 0;                                          //clear flag to disable mph display routine in loop
  digitalWrite(alarm_light,false);                              //turn off alarm light
}
void screen_run_on(void) {
#if serial_debug
  Serial.println("screen_run_on() start");
#endif  
  run_screen_flag = 1;                                                  //turn flag os so speed will display in run screen
  field_cal_flag = 0;                                                   //turn off flag so calibration will not display

  lv_obj_set_pos(label_speed, 100, 45);                                 //reset X position of speed readout
  lv_obj_set_pos(title_label, 140, 5);                                  //reset x position of title text
  lv_label_set_text(title_label, "ATC Speed Monitor");                  //Set title of page text
  lv_obj_set_hidden(label_units, false);                                //turn on units lable (mph or kph)
  lv_obj_set_hidden(btn_setup, false);                                  //turn on objects to show on run screen
  lv_obj_set_hidden(label_speed, false);
//  lv_obj_set_hidden(label_arrow, false);
//  lv_obj_set_hidden(tick1, false);
  lv_obj_set_hidden(line1, false); //ruler marks below bar graph
  lv_obj_set_hidden(unit_line, false);
  lv_obj_set_hidden(title_label, false);
  if (speed_input == 0) {
      lv_obj_set_hidden(label_gps_lock_icon,true);                   //turn off gps symbol if in radar mode
       lv_obj_set_hidden(label_gps_search_icon,true);
       }
  else{
     lv_obj_set_hidden(label_gps_lock_icon,false);                   //turn on gps symbol if not in radar mode
  }
  //lv_cb_is_checked(obj)
  if (lv_cb_is_checked(cb_feet_per_min)) {                              //if feet per minute checkbox is checked
    lv_obj_set_hidden(lab_fpm, false);                                //show fpm value
    }
  else {
    lv_obj_set_hidden(lab_fpm, true);                                  //hide fpm value
    }
 
  if (dis == 1){                                                         //if distance counter is set on
    lv_obj_set_hidden(btn_reset_dist, false); //turn on distance reset button if "show distance" is selected
    }
  else{
    lv_obj_set_hidden(btn_reset_dist, true); //hide reset button if not showing distance reading
    }
  if (units == 1) {                                                     //(MPH)show the correct units selected
    lv_label_set_text(label_units, "MPH");                            //set label for MPH
    snprintf(text_buff, 7, "%2.1f", (speed_target * 2));             //  double target for max speed on bar graph
    lv_label_set_text(label_bar_max, text_buff);                    //update text of bar max label
    lv_bar_set_range(bar_speed, 0, (int)((2 * speed_target) * 10));  //set range of bar graph (mph mode)
  }
  else {                                                 //(KPH)if in kilometer mode
    lv_label_set_text(label_units, "km/h");                           //set label to kilometers
    snprintf(text_buff, 7, "%2.1f", (speed_target * 2) * 1.609344 ); //  double and convert to metric
    lv_label_set_text(label_bar_max, text_buff);                    //display max value on bar graph in metric
    lv_bar_set_range(bar_speed, 0, (int)((2 * speed_target * 1.609344) * 10)); //set range on bar graph metric mode
  }

  if (graph == 1) {                                                     //if bar graph check box is checked
    lv_obj_set_hidden(bar_speed, false);                                //show bar graph
    lv_obj_set_hidden(label_arrow, false);                              //show center mark arrow on bar
    lv_obj_set_hidden(label_bar_max, false);                            //show lable bar
    lv_obj_set_hidden(label_bar_min, false);                            //show bar label
    lv_obj_set_hidden(title_label, false);                              //show title
    lv_obj_set_hidden(tick1, false);                                    //show ruler marks
    lv_bar_set_range(bar_speed, 0, (int)((2 * speed_target) * 10));     //set the range of the bar
  }
  else {
    lv_obj_set_hidden(bar_speed, true);                                     //hide the bar graph and labels associated with it
    lv_obj_set_hidden(label_arrow, true);
    lv_obj_set_hidden(label_bar_max, true);
    lv_obj_set_hidden(label_bar_min, true);
    lv_obj_set_hidden(tick1, true);                                         //hide ruler line under bar graph
    lv_obj_set_hidden(label_units, false);
  }


  if (alarm_enable == 1 && graph == 1) {                                   //if graph is on and alarm is enabled
    lv_obj_set_hidden(label_alarm_icon, false);
    }
  else {
    lv_obj_set_hidden(label_alarm_icon, true);
    }
  lv_bar_set_value(bar_speed, 0, LV_ANIM_OFF);                              //This refreshes the bar graph

 
#if serial_debug 
  Serial.println("line 421 screen_run_on * End");
#endif
}          //end of screen_run_on()
void screen_calibrate_off(void) {                                       //turn  off all objects on "set calibration number" screen
  lv_label_set_text(title_label, "");
  lv_obj_set_hidden(label_cal, true);
  lv_obj_set_hidden(btn_exit, true);
  lv_obj_set_hidden(btn_set_cal, true);
  lv_obj_set_hidden(btn_field_cal, true);
  // lv_obj_set_hidden(keypad,true);

}
void screen_calibrate_on(void) {                                         //turn  on "set calibration number" screen
  option_screen_off();
  screen_run_off();
  screen_target_off();
  lv_obj_set_pos(title_label, 100, 5);
  lv_obj_set_hidden(title_label, false);                      //turn on all objects for this screen
  lv_label_set_text(title_label, "Speed Calibration Entry");  //set screen title text
  lv_obj_set_hidden(label_cal, false);
  lv_obj_set_hidden(btn_exit, false);
  lv_obj_set_hidden(btn_set_cal, false);
  lv_obj_set_hidden(btn_field_cal, false);
  lv_obj_set_hidden(line1, false);
  snprintf(text_buff, 7, "%05d", cal_number);                        //display current calibration number
  lv_label_set_text(label_cal, text_buff);
}
void screen_set_target_on(void){                                                //turn on target screen
   screen_calibrate_off();
   option_screen_off();
   screen_run_off();
  // lv_obj_set_hidden(title_label3, false); 
   lv_obj_set_hidden(btn_set_alarm_1,false);
   lv_obj_set_hidden(btn_set_alarm_2,false);
   lv_obj_set_hidden(btn_set_alarm_3,false);
   lv_obj_set_hidden(btn_set_alarm_4,false);
   lv_obj_set_hidden(label_alarm1,false);
}
void screen_target_on(void) {                                            //turn on screen to set multi target speed
  screen_calibrate_off();
  option_screen_off();
  screen_run_off();
  lv_obj_set_hidden(btn_target_exit, false);                               //show the exit button
  lv_obj_set_hidden(label_units, false);                                   //display mph or kph when setting target speed
  lv_obj_set_hidden(line1, false);
  lv_obj_set_pos(title_label, 110, 4);                                     //set position of title text
  lv_label_set_text(title_label, "Enter Target Speed");                    //title at top of page
  
  lv_label_set_text(label_alarm1, "1.2");                                    //display current alarm setting for button 1
  lv_obj_set_hidden(label_alarm1,true);                                     //show the alarm speed for alarm 1
 // snprintf(text_buff, 7, "%2.1f", speed_target);                           //get current speed target and display
 // lv_label_set_text(label_cal, text_buff);                                 //display the target speed in large text
  lv_obj_set_hidden(title_label, false);                                   //show title

 // lv_obj_set_hidden(keypad_target, false);                                 //show the keypad
  lv_obj_set_hidden(label_cal, false);
  screen_set_multi_target_on();                                            //turn on buttons to select target speed 1-4
  Serial.println("line 546");                                              //***diagnostic
  
}
void screen_target_off(void) {                                       //turn off multiple target buttons
  lv_label_set_text(title_label, "");
  lv_obj_set_hidden(keypad_target, true);                           //hide keyboard
  lv_obj_set_hidden(label_cal, true);
  lv_obj_set_hidden(btn_set_cal, true);
  lv_obj_set_hidden(btn_target_exit, true);
  lv_obj_set_hidden(label_units, true);                                 //display mph or kph when setting target speed
}
void screen_set_multi_target_on(void){                                  //multiple alarm selection buttons
  option_screen_off();
  lv_obj_set_hidden(btn_set_alarm_1, false);        //alarm set button 1
  lv_obj_set_hidden(btn_set_alarm_2, false);        //alarm set button 2
  lv_obj_set_hidden(btn_set_alarm_3, false);        //alarm set button 3
  lv_obj_set_hidden(btn_set_alarm_4, false);        //alarm set button 4
  lv_obj_set_hidden(btn_set_alarm_exit, false);        //alarm set button 1
  lv_obj_set_hidden(title_label, false);                                   //show title
  lv_label_set_text(title_label, "Enter Alarm Speeds");                    //title at top of page
  lv_label_set_text(label_cal,"");
}
void screen_set_multi_target_off(void){
  lv_obj_set_hidden(btn_set_alarm_1, true);        //alarm set button 1
  lv_obj_set_hidden(btn_set_alarm_2, true);        //alarm set button 2
  lv_obj_set_hidden(btn_set_alarm_3, true);        //alarm set button 3
  lv_obj_set_hidden(btn_set_alarm_4, true);        //alarm set button 4
  lv_obj_set_hidden(btn_set_alarm_exit, true);        //alarm set button 1
}
void screen_factory_on(void) {
  lv_obj_set_hidden(btn_fact_exit, false);          //button to exit factory screen
  lv_obj_set_hidden(btn_reset_pw, false);            //button to reset password
  lv_obj_set_hidden(btn_scr_cal, false);             //button to calibrate screen
  lv_obj_set_hidden(title_label, false);             //show title bar
  lv_obj_set_pos(title_label, 130, 5);
  lv_label_set_text(title_label, "Factory Settings");//change text on title
  lv_obj_set_hidden(touch_cal_but_text, false);      //text beside the screen cal button
  lv_obj_set_hidden(pw_but_text, false);             //text beside the new password button
  lv_obj_set_hidden(pw_text, false);
  lv_obj_set_hidden(touch_cal_but_text, false);
  lv_obj_set_hidden(btn_reset, false);
  lv_obj_set_hidden(line1, false);
  lv_obj_set_hidden(line2, false);                   //lower screen line above bottom two buttons
  run_screen_flag = 0;                               //clear flag to disable mph display routine in loop
  lv_obj_set_pos(pw_text, 180, 90);
  char buff1[6];
  sprintf(buff1, "Current code = %ld", user_passcode.toInt());
  lv_label_set_text(pw_text, buff1 );                /*Show the existing password*/
}
void screen_factory_off(void) {                                         //turn off all objects on factory settings screen
  lv_obj_set_hidden(btn_fact_exit, true);            //button to exit factory screen
  lv_obj_set_hidden(btn_reset_pw, true);             //button to reset password
  lv_obj_set_hidden(btn_scr_cal, true);              //button to calibrate screen
  lv_obj_set_hidden(touch_cal_but_text, true);       //text beside the screen cal button
  lv_obj_set_hidden(pw_but_text, true);              //text beside the new password button
  lv_obj_set_hidden(pw_text, true);                  //text next to password set button
  lv_obj_set_hidden(touch_cal_but_text, true);      //touch calibration button
  lv_obj_set_hidden(btn_reset, true);               //computer reset button
  lv_obj_set_hidden(line2, true);                   //lower screen line above bottom two buttons
  lv_label_set_text(label_speed, "");               //clear this or the password shows up in run screen for 2 seconds
}
void build_screen_target(void) {                                        //create objects for screen to enter target speed
#if serial_debug  
  Serial.println("line 517");
#endif
  btn_target_exit = lv_btn_create(lv_scr_act(), NULL);                       /*Add a screen exit button */
  lv_obj_set_hidden(btn_target_exit, true);                            //set to true to hide button
  lv_obj_set_drag(btn_target_exit, false);                              //do not allow dragging of button
  lv_obj_set_pos(btn_target_exit, 380, 2);                              /*Set its position in upper RIGHT corner*/
  lv_obj_set_size(btn_target_exit, 80, 50);                             /*Set its size*/
  lv_obj_set_event_cb(btn_target_exit, btn_target_exit_cb);             /*Assign an event callback when pressing */
  lv_obj_t * label12 = lv_label_create(btn_target_exit, NULL);               /*Add a label to the button*/
  lv_label_set_text(label12, LV_SYMBOL_CLOSE  " Exit");                  /*Set the labels text*/


  lv_obj_set_hidden(btn_target_exit, true);                           //show  exit button in upper right corner

  keypad_target = lv_btnm_create(lv_scr_act(), NULL);                 // create keypad 6 keys by 2 keys horizontal
  lv_btnm_set_map(keypad_target, btnm_map);                           //bring in key map to create a numeric keypad to enter data
  lv_obj_set_size(keypad_target, 478, 126);                           /*Set outside dimensions of keyboard*/
  lv_obj_align(keypad_target, NULL, LV_ALIGN_CENTER, 0, 98);          //set position of keypad
  lv_obj_set_event_cb(keypad_target, target_speed_event_handler);     //callback for keypad presses
  lv_obj_set_hidden(keypad_target, true);                             //hide the keypad
  
}
void build_factory_screen() {
  Serial.println("Entering build_factory_screen()");
  run_screen_flag = 0;                                                 //set flag so main loop will not process speed
  lv_style_copy(&style3, &lv_style_plain);
  style3.text.font = &lv_font_roboto_28;                               //12,16,22,28 built in
  style3.text.color = LV_COLOR_BLACK;
  lv_obj_set_style(title_label, &style3);                              //assign style to the label
  lv_obj_align(title_label, NULL, LV_ALIGN_CENTER, -95, -140);         //set position of title text
  lv_label_set_text(title_label, "Factory settings");                  //assign text to the label

  btn_reset_pw = lv_btn_create(lv_scr_act(), NULL);                     /*Add a "Reset Password" button */
  lv_obj_set_drag(btn_reset_pw, false);                             //turn drag feature off
  lv_obj_set_pos(btn_reset_pw, 20, 60);                             /*Set its position*/
  lv_obj_set_size(btn_reset_pw, 150, 60);                           /*Set its size*/
  lv_obj_set_event_cb(btn_reset_pw, fact_opt_cb);                   /*Assign an event callback*/
  lv_obj_t * label21 = lv_label_create(btn_reset_pw, NULL);             /*Add a label to the button*/
  lv_label_set_text(label21,  "Reset PW");                          /*Set the labels text*/

  btn_scr_cal = lv_btn_create(lv_scr_act(), NULL);                      /*Add screen callibration button */
  lv_obj_set_drag(btn_scr_cal, false);                              //turn drag feature off
  lv_obj_set_pos(btn_scr_cal, 20, 145);                             /*Set its position*/
  lv_obj_set_size(btn_scr_cal, 150, 60);                            /*Set its size*/
  lv_obj_set_event_cb(btn_scr_cal, fact_opt_cb);                    /*Assign an event callback*/
  lv_obj_t * label22 = lv_label_create(btn_scr_cal, NULL);              /*Add a label to the button*/
  lv_label_set_text(label22,  "Touch Cal.");                        /*Set the labels text*/

  btn_fact_exit = lv_btn_create(lv_scr_act(), NULL);                    /*Screen Exit button*/
  lv_obj_set_drag(btn_fact_exit, false);                            //turn drag feature off
  lv_obj_set_pos(btn_fact_exit, 20, 250);                           /*Set its position*/
  lv_obj_set_size(btn_fact_exit, 150, 60);                          /*Set its size*/
  lv_obj_set_event_cb(btn_fact_exit, fact_opt_cb);                  /*Assign an event callback*/
  lv_obj_t * label20 = lv_label_create(btn_fact_exit, NULL);            /*Add a label to the button*/
  lv_label_set_text(label20, LV_SYMBOL_DOWNLOAD "  SAVE");         /*Set the labels text*/

  btn_reset = lv_btn_create(lv_scr_act(), NULL);                    /*Add a button the active screen*/
  lv_obj_set_drag(btn_reset, false);                            //turn drag feature off
  lv_obj_set_pos(btn_reset, 310, 250);                           /*Set its position*/
  lv_obj_set_size(btn_reset, 150, 60);                          /*Set its size*/
  lv_obj_set_event_cb(btn_reset, fact_opt_cb);                  /*Assign an event callback*/
  lv_obj_t * label24 = lv_label_create(btn_reset, NULL);            /*Add a label to the button*/
  lv_label_set_text(label24, LV_SYMBOL_REFRESH " Reset");         /*Set the labels text*/


  keypad_pw = lv_btnm_create(lv_scr_act(), NULL);                        //create a keypad for entering new security code
  lv_btnm_set_map(keypad_pw, btnm_map);                              //create a numeric keypad to enter data
  lv_obj_set_size(keypad_pw, 478, 126);                              /*Set keypad size*/
  lv_obj_align(keypad_pw, NULL, LV_ALIGN_CENTER, 0, 98);             //set position of keypad
  lv_obj_set_event_cb(keypad_pw, new_pw_event_handler);              //callback for keypad presses
  lv_obj_set_hidden(keypad_pw, true);                                 //hide keypad after creating

  pw_but_text = lv_label_create(lv_scr_act(), NULL);                     /*Add a label to identify current password next to reset pw button*/
  lv_obj_set_pos(pw_but_text, 180, 60);
  lv_label_set_text(pw_but_text,  "Reset password.");                 /*Set the labels text*/

  pw_text = lv_label_create(lv_scr_act(), NULL);                        /*Add a label to the button*/
  lv_obj_set_pos(pw_text, 180, 90);
  char buff1[6];
  sprintf(buff1, "Current code = %ld", user_passcode.toInt());
  lv_label_set_text(pw_text, buff1 );                /*Show the existing password*/

  touch_cal_but_text = lv_label_create(lv_scr_act(), NULL);              /*Add a label to identify current password next to reset pw button*/
  lv_obj_set_pos(touch_cal_but_text, 180, 144);
  lv_label_set_text(touch_cal_but_text, LV_SYMBOL_WARNING  "Calibrate screen.\n   (Factory use only)"); /*Set the labels text*/

  screen_factory_off();                                                   ///turn off all objects after build
}
void fact_opt_cb(lv_obj_t * obj, lv_event_t event) {                      //callback routine for factory option screen
  Serial.println("fact_opt_cb - line 637");
  switch (event) {
    case LV_EVENT_PRESSED:
      if (obj == btn_fact_exit) {                                   //Exit button
        screen_factory_off();                                       //turn off all objects on screen
        screen_run_on();                                             //go back to run screen
      }

      if (obj == btn_reset) {                                      //reset computer
        ESP.restart();
      }

      if (obj == btn_scr_cal) {                                     //touch screen calibration
        screen_factory_off();
        var_REPEAT_CAL = true;                                     //set flag so touch screen routine will run again
        touch_calibrate();                                         //routine to calibrate touch screen
        screen_factory_on();
      }

      if (obj == btn_reset_pw) {                                     //reset the password
        screen_factory_off();                                   //clear screen and bring up keypad
        lv_obj_set_hidden(title_label, false);                  //show the title bar
        lv_obj_set_hidden(label_speed, false);                  //turn on large text label that displays entered numbers
        run_screen_flag = 0;                                    //set flag so mph will not update
        lv_obj_set_hidden(label_units, true);
        lv_obj_set_pos(title_label, 30, 5);                     //set position of text
        lv_label_set_text(title_label, "  Enter New Code [2-4 numbers]");
        lv_label_set_text(label_speed, "----");                  //display ---- on intitial fireup
        Serial.println("line 494");
        lv_obj_set_hidden(keypad_pw, false);                     //show keypad
        strcpy(temp_buff, "");                                 //clear buffer used to store key entry
        Serial.println("line 674");

      }

  }

}
void new_pw_event_handler(lv_obj_t * obj, lv_event_t event) {             //event handler for new password entry
  char hold[2];                                                          //temp array to hold the x value
  if (event == LV_EVENT_VALUE_CHANGED) {
    const char     *txt = lv_btnm_get_active_btn_text(obj);                           //get the text value of pressed button
    Serial.println("line 653");
    if (txt != nullptr) {
      sprintf(hold, "%s", txt);                                         //convert to a string
      strncat(temp_buff, hold, 1);                                      //add charcter to buffer

      lv_label_set_text(label_speed, temp_buff);                        //display code in mph text area with large text
      printf("%s is current string value line 708\n", temp_buff);       //***diagnostic code

      if (*txt == 'C') {                                                //was clear button pressed?
        lv_label_set_text(label_speed, " ---");                       //remove numbers and dispay "-----"
        strcpy(temp_buff, "");                                        //clear the buffer
      }

      if (strlen(temp_buff) >= 6) {                                    //check for overflow

        lv_obj_set_pos(title_label, 30, 5);                           //set position of text
        lv_label_set_text(title_label, "  Enter New Code [2-4 numbers]"); //refresh text at top of screeen
        lv_label_set_text(label_speed, " ---");                        //remove numbers and display "-----"
        strcpy(temp_buff, "");                                          //clear the buffer
      }
      if (*txt == 'E') {                                                //Enter key pressed?
        //check that this is a least 2 charcaters long
        if (String(temp_buff).length() >= 3) {                           //must be at least 2 digits length plus the 'E'
          user_passcode =  String(temp_buff);                          //set passcode to new value
          int xvar = user_passcode.toInt();
          EEPROM.put(upw_ee_adr, xvar);                                 //save new user password to eeprom as integer value
          EEPROM.commit();
          printf("%ld is the new password **********\n", user_passcode.toInt());     //***diagnostic code
          screen_run_off();                                            //turn off objects used from run screen
          lv_obj_set_hidden(keypad_pw, true);                          //hide the keypad
          screen_factory_on();                                         //show the factory screen
        }
        else {
          lv_label_set_text(label_speed, " ---");                        //remove numbers and display "-----"
          strcpy(temp_buff, "");
          //insert message box here about length of code
        }
      }
    }
  }
}
void build_option_screen() {
  Serial.println("Entering build_option_screen()");
  run_screen_flag = 0;                                            //set flag to zero so mph calcultion routines will shut down
  lv_style_copy(&style3, &lv_style_plain);
  style3.text.font = &lv_font_roboto_28;                         //12,16,22,28 built in
  style3.text.color = LV_COLOR_BLACK;                             //set text color to black
  lv_obj_set_style(title_label, &style3);                              //assign style to the label
  // lv_obj_align(title_label, NULL, LV_ALIGN_CENTER, -30,-140);          //set position of title text
  lv_obj_set_pos(title_label, 150, 5);
  lv_label_set_text(title_label, "Options");                           //assign text to the label

  static lv_style_t style_shadow;
  lv_style_copy(&style_shadow,  &lv_style_pretty_color);

  /**********************
      Description text of buttons
   **********************/
  lv_style_copy(&style4, &lv_style_plain);                       //declare style
  style4.text.font = &lv_font_roboto_16;                         //12,16,22,28 built in
  style4.text.color = LV_COLOR_BLACK;


  option_descrip_text = lv_label_create(lv_scr_act(), NULL);          //Description line of OPTIONS button
  lv_obj_set_style(option_descrip_text, &style4);
  lv_obj_set_pos(option_descrip_text, 5, 52);
  lv_label_set_text(option_descrip_text, "CALIBRATE - This button allows you to set how many pulses are\n                          in 300 feet, so speed can be accurately calculated.");
  lv_obj_set_hidden(option_descrip_text, true);

  calibrate_descrip_text = lv_label_create(lv_scr_act(), NULL);          //Description line of CALIBRATE button
  lv_obj_set_style(calibrate_descrip_text, &style4);
  lv_obj_set_pos(calibrate_descrip_text, 5, 112);
  lv_label_set_text(calibrate_descrip_text, "OPTIONS - Select the data to be displayed on the display screen\n                     and set password protection feature. ");
  lv_obj_set_hidden(calibrate_descrip_text, true);

  close_descrip_text = lv_label_create(lv_scr_act(), NULL);          //Description line of CALIBRATE button
  lv_obj_set_style(close_descrip_text, &style4);
  lv_obj_set_pos(close_descrip_text, 5, 172);
  lv_label_set_text(close_descrip_text, "CLOSE - Close this screen and return to display screen.");
  lv_obj_set_hidden(close_descrip_text, true);

  /**********************
      Create exit button
   **********************/  //upper right corner of screen
  btn_set_exit = lv_btn_create(lv_scr_act(), NULL);              /*Add a button the active screen*/
  lv_obj_set_drag(btn_set_exit, false);                       //turn drag feature off
  lv_obj_set_pos(btn_set_exit, 10, 250);                      /*Set its position*/
  lv_obj_set_size(btn_set_exit, 150, 60);                     /*Set its size*/
  lv_obj_set_event_cb(btn_set_exit, unit_event_cb);           /*Assign an event callback*/
  lv_obj_set_style(btn_set_exit, &style_shadow);
  lv_obj_t * label7 = lv_label_create(btn_set_exit, NULL);        /*Add a label to the button*/
  //  lv_obj_set_style(label7, &style3);
  lv_label_set_text(label7, LV_SYMBOL_DOWNLOAD "  SAVE");     /*Set the labels text*/
  lv_obj_set_hidden(btn_set_exit, true);
#if serial_debug
  Serial.println("line 562");
#endif
  /**********************
      Create set_target_btn button
   **********************/  //lower right corner of screen
  btn_target_set = lv_btn_create(lv_scr_act(), NULL);              /*SET Target Button*/
  lv_obj_set_hidden(btn_target_set, true);                      //set to true to hide button
  lv_obj_set_drag(btn_target_set, false);                        //turn drag feature off
  lv_obj_set_pos(btn_target_set, 177, 250);                     /*Set its position*/
  lv_obj_set_size(btn_target_set, 140, 60);                     /*Set its size*/
  lv_obj_set_style(btn_target_set, &style_shadow);
  lv_obj_set_event_cb(btn_target_set, unit_event_cb);           //declare the callback routine
  lv_obj_t * label8 = lv_label_create(btn_target_set, NULL);       /*Add a label to the button*/
  lv_label_set_text(label8, "Set Target");                     /*Set the labels text*/

 /**********************
      Create pre alarm button
   **********************/  //lower right corner of screen
  btn_preAlarm_set = lv_btn_create(lv_scr_act(), NULL);              /*SET Target Button*/
  lv_obj_set_hidden(btn_preAlarm_set, true);                      //set to true to hide button
  lv_obj_set_drag(btn_preAlarm_set, false);                        //turn drag feature off
  lv_obj_set_pos(btn_preAlarm_set, 337, 250);                     /*Set its position*/
  lv_obj_set_size(btn_preAlarm_set, 140, 60);                     /*Set its size*/
  lv_obj_set_style(btn_preAlarm_set, &style_shadow);
  lv_obj_set_event_cb(btn_preAlarm_set, unit_event_cb);           //declare the callback routine
  lv_obj_t * label9 = lv_label_create(btn_preAlarm_set, NULL);       /*Add a label to the button*/
  lv_label_set_text(label9, "Pre-Alarm");                     /*Set the labels text*/







#if serial_debug
  Serial.println("line 603");
#endif
  /**********************
     show target speed
  **********************/  //lower right corner of screen

  label_target_speed = lv_label_create(lv_scr_act(), NULL);      //target speed display on option screen lower right corner
  lv_obj_set_style(label_target_speed, &style3);
  lv_label_set_text(label_target_speed, text_buff);
  lv_obj_set_pos(label_target_speed, 370, 260);
  lv_obj_set_hidden(label_target_speed, true);

/**********************
     Display Software version
  **********************/
  label_software = lv_label_create(lv_scr_act(), NULL);           //label to display software version
  lv_obj_set_style(label_software, &style4);                      //assign style to button
  lv_label_set_text(label_software, software_ver);                //enter software version here
  lv_obj_set_pos(label_software, 10, 18);                         //set position on screen
  lv_obj_set_hidden(label_software, true);                        //hide for now

  /**********************
        Create Check boxes 
     **********************/

  cb_mph = lv_cb_create(lv_scr_act(), NULL);         //checkbox for units MPH
  lv_cb_set_text(cb_mph, "MPH");
  lv_obj_set_pos(cb_mph, 10, 46);                                /*Set its position*/
  lv_obj_set_event_cb(cb_mph, unit_event_cb);
  lv_obj_set_hidden(cb_mph, true);
  if (units == 2) {                                            //if units is kph (2)
    lv_cb_set_checked(cb_mph, false);
  }
  else {
    lv_cb_set_checked(cb_mph, true);
  }

  cb_kph = lv_cb_create(lv_scr_act(), NULL);          //checkbox for units KPH
  lv_cb_set_text(cb_kph, "km/h");
  lv_obj_set_pos(cb_kph, 10, 90);                               /*Set its position*/
  lv_obj_set_event_cb(cb_kph, unit_event_cb);
  lv_obj_set_hidden(cb_kph, true);
  if (units == 1) {                                             //if units is  mph (1)
    lv_cb_set_checked(cb_kph, false);
  }
  else {
    lv_cb_set_checked(cb_kph, true);
  }

  cb_distance = lv_cb_create(lv_scr_act(), NULL);      //checkbox for distance (not enabled )
  lv_cb_set_text(cb_distance, "Distance");
  lv_obj_set_pos(cb_distance, 10, 180);                           /*Set its position*/
  lv_obj_set_event_cb(cb_distance, unit_event_cb);
  lv_obj_set_hidden(cb_distance, true);
  if (cb_distance == 0) {
    lv_cb_set_checked(cb_distance, false);
  }
  else {
    lv_cb_set_checked(cb_distance, true);
  }

  cb_security = lv_cb_create(lv_scr_act(), NULL);      //checkbox for password
  lv_cb_set_text(cb_security, "Password");
  lv_obj_set_pos(cb_security, 190, 46);                           /*Set its position*/
  lv_obj_set_event_cb(cb_security, unit_event_cb);
  lv_obj_set_hidden(cb_security, true);
  if (cb_security == 0) {
    lv_cb_set_checked(cb_security, false);
  }

  else {
    lv_cb_set_checked(cb_security, true);
  }

  cb_graph = lv_cb_create(lv_scr_act(), NULL);        //checkbox for units KPH
  lv_cb_set_text(cb_graph, "Bar Graph");
  lv_obj_set_pos(cb_graph, 190, 90);                           /*Set its position*/
  lv_obj_set_event_cb(cb_graph, unit_event_cb);
  lv_obj_set_hidden(cb_graph, true);
  if (cb_graph == 0) {
    lv_cb_set_checked(cb_graph, false);
  }
  else {
    lv_cb_set_checked(cb_graph, true);
  }

  cb_alarm = lv_cb_create(lv_scr_act(), NULL);        //checkbox for alarm functon
  lv_cb_set_text(cb_alarm, "Alarm");
  lv_obj_set_pos(cb_alarm, 360, 90);                           /*Set its position*/
  lv_obj_set_event_cb(cb_alarm, unit_event_cb);
  lv_obj_set_hidden(cb_alarm, true);
  if (alarm_enable == 0) {
    lv_cb_set_checked(cb_graph, false);
  }                        //set status of checkbox

  else {
    lv_cb_set_checked(cb_graph, true);
  }


  cb_feet_per_min = lv_cb_create(lv_scr_act(), NULL);   //checkbox for distance per minute
  lv_cb_set_text(cb_feet_per_min, "Distance/minute");
  lv_obj_set_pos(cb_feet_per_min, 190, 136);                      /*Set its position*/
  lv_obj_set_event_cb(cb_feet_per_min, unit_event_cb);
  lv_obj_set_hidden(cb_feet_per_min, true);
  if (fpm == 0) {
    lv_cb_set_checked(cb_feet_per_min, false);
  }                                                           //display the checkbox status
  else {
    lv_cb_set_checked(cb_feet_per_min, true);
  }

  cb_speed_avg = lv_cb_create(lv_scr_act(), NULL);   //checkbox for avg/speed
  lv_cb_set_text(cb_speed_avg, "Avg/Speed");
  lv_obj_set_pos(cb_speed_avg, 10, 136);                      /*Set its position*/
  lv_obj_set_event_cb(cb_speed_avg, unit_event_cb);           //callback routine
  lv_obj_set_hidden(cb_speed_avg, true);
  if (speed_avg == 0) {                                             //check for saved check box status
    lv_cb_set_checked(cb_speed_avg, false);
  }                                                           //display the checkbox status
  else {
    lv_cb_set_checked(cb_speed_avg, true);
  }

  cb_radar = lv_cb_create(lv_scr_act(), NULL);   //checkbox for radar input
  lv_cb_set_text(cb_radar, "Radar");
  lv_obj_set_pos(cb_radar, 10, 185);                      /*Set its position*/
  lv_obj_set_event_cb(cb_radar, unit_event_cb);           //callback routine

  cb_gps = lv_cb_create(lv_scr_act(), NULL);   //checkbox for gps input
  lv_cb_set_text(cb_gps, "GPS Sensor");
  lv_obj_set_pos(cb_gps, 190, 185);                      /*Set its position*/
  lv_obj_set_event_cb(cb_gps, unit_event_cb);           //callback routine

  if (speed_input == 0){                               //based on saved eeprom value set the speed input checkboxes
     lv_cb_set_checked(cb_radar, true);
     lv_cb_set_checked(cb_gps, false);
     }
  else{
      lv_cb_set_checked(cb_radar, false);              //if speed_input was 1 then check gps and uncheck radar
      lv_cb_set_checked(cb_gps, true);
    }
  
}
void build_screen_run(void) {
#if serial_debug
  Serial.println("Build Screen run, line 651");
#endif
  /**********************
      Declare a style
   **********************/
  //  lv_theme_t * th = lv_theme_night_init(210,&lv_font_roboto_28);
  //  lv_theme_set_current(th);

  //style 1
  static lv_style_t style1;                                        /*Declare a new style. 28 point white text Should be `static`*/
  lv_style_copy(&style1, &lv_style_plain);
  style1.text.font = &lv_font_roboto_28;                          //12,16,22,28 built in
  style1.text.color = LV_COLOR_WHITE;                            /*label color*/
  //style 2
  static lv_style_t style2;                                        /*Declare a new style. Should be `static`*/
  lv_style_copy(&style2, &lv_style_plain);
  style2.text.font = &Bebasneue;  
  // style2.text.font = &lv_font_roboto_28;                               //12,16,22,28 built in
  style2.text.color = LV_COLOR_BLACK;                            /*label color*/
  //style 3
  static lv_style_t style3;                                        /*Declare a new style. Should be `static`*/
  lv_style_copy(&style3, &lv_style_plain);
  style3.text.font = &lv_font_roboto_28;                         //12,16,22,28 built in
  style3.text.color = LV_COLOR_BLACK;

  //draw_title_line();                                                //Blue line under page title
  //lv_obj_set_hidden(line1,false);
  /**********************
     Create a page title
  **********************/
  /*Create a style and use the new font*/
#if serial_debug
  Serial.println("line 304");
#endif
  lv_obj_set_style(title_label , &style3);                       //assign style to the label
  //lv_obj_set_width(title_label, 450);
  lv_obj_align(title_label, NULL, LV_ALIGN_CENTER, -60, -140);   //set position of text
  lv_obj_set_pos(title_label, 130, 5);
  lv_label_set_text(title_label, "ATC Speed Monitor");          //assign text to the label

  /**********************
     Create a label to display speed
  **********************/
  /*Create a style and use the new font*/
#if serial_debug
  Serial.println("line 319");
#endif
  label_speed = lv_label_create(lv_scr_act(), NULL);                //create a label object for the active screen
  lv_obj_set_style(label_speed, &style2);                       //assign style to the label
  lv_obj_set_width(label_speed, 480);
  lv_obj_set_y(label_speed, 45);
  lv_obj_set_x(label_speed, 100);
  lv_label_set_align(label_speed, LV_LABEL_ALIGN_RIGHT);
  lv_label_set_text(label_speed, "");
  lv_obj_set_drag(label_speed, false);                          //set true to allow text to be dragged


  /************************
     Create label to display Units
   ************************/
  label_units = lv_label_create(lv_scr_act(), NULL);                    //create a label object for the active screen
  lv_obj_set_style(label_units, &style3);                             //assign style to the label
  lv_obj_set_pos(label_units, 408, 75);
  lv_obj_set_size(label_units, 150, 50);
  lv_obj_set_drag(label_units, false);                                //allow text to be dragged
#if serial_debug
  Serial.println("line 337");
#endif
  if (units == 0) {
    lv_label_set_text(label_units, "???");                      //assign text to the label
  }
  if (units == 1) {
    lv_label_set_text(label_units, "MPH");                      //assign text to the label
  }
  if (units == 2) {
    lv_label_set_text(label_units, "km/h");                      //assign text to the label
  }
  /************************
     Alarm Icon
   ************************/

  label_alarm_icon = lv_label_create(lv_scr_act(), NULL);                    //create a label object for the active screen
  lv_obj_set_style(label_alarm_icon, &style3);                             //assign style to the label
  lv_obj_set_pos(label_alarm_icon, 422, 48);
  lv_obj_set_size(label_alarm_icon, 30, 30);
  lv_label_set_text(label_alarm_icon, LV_SYMBOL_BELL);                       //ALARM ICON

 /************************
     GPS Activity logo
   ************************/

  label_gps_search_icon = lv_label_create(lv_scr_act(), NULL);                    //create a label object for the active screen
  lv_obj_set_style(label_gps_search_icon, &style3);                             //assign style to the label
  lv_obj_set_pos(label_gps_search_icon, 430, 7);
  lv_obj_set_size(label_gps_search_icon, 20, 20);
  lv_label_set_text(label_gps_search_icon, LV_SYMBOL_WIFI);                       //Searching 

  label_gps_lock_icon = lv_label_create(lv_scr_act(), NULL);                    //create a label object for the active screen
  lv_obj_set_style(label_gps_lock_icon, &style3);                             //assign style to the label
  lv_obj_set_pos(label_gps_lock_icon, 430, 7);
  lv_obj_set_size(label_gps_lock_icon, 20, 20);
  lv_label_set_text(label_gps_lock_icon, LV_SYMBOL_GPS);                       //GPS locked on signal and ready icon

//  messbox_gps_search = lv_mbox_create(lv_disp_get_scr_act(NULL), NULL);   //create a message box
//        lv_obj_set_width(messbox_gps_search, 350);                             //width of message box
//        lv_mbox_set_text(messbox_gps_search, "\n\nSearching for Satellites\n\n");     //message box to start driving
//       // lv_mbox_start_auto_close(messbox_gps_search, 4000);                 //close box after 4 seconds
//       // lv_mbox_set_anim_time(messbox_gps_search, 1500);
//        lv_obj_align(messbox_gps_search, NULL, LV_ALIGN_CENTER, -30, -30); //position of  box

  /************************
     Create label to display feet per minute
   ************************/
  lab_fpm = lv_label_create(lv_scr_act(), NULL);
  lv_obj_set_style(lab_fpm, &style3);                             //assign style to the label
  lv_obj_set_pos(lab_fpm, 408, 120);
  lv_obj_set_size(lab_fpm, 150, 50);
  lv_obj_set_drag(lab_fpm, false);                                //allow text to be dragged
  lv_label_set_text(lab_fpm, "");

  /************************
     labels for min/max text and target indicator on speed bar
   ************************/
  hor_pos = 290;
  label_bar_min = lv_label_create(lv_scr_act(), NULL);
  lv_obj_set_style(label_bar_min, &style3);                             //assign style to the label
  lv_obj_set_pos(label_bar_min, 1, hor_pos);
  lv_obj_set_size(label_bar_min, 50, 50);
  lv_obj_set_drag(label_bar_min, false);                                //allow text to be dragged
  lv_label_set_text(label_bar_min, "0.0");
  lv_obj_set_hidden(label_bar_min, false);

  label_bar_max = lv_label_create(lv_scr_act(), NULL);
  lv_obj_set_style(label_bar_max, &style3);                             //assign style to the label
  lv_obj_set_pos(label_bar_max, 423 , hor_pos);
  lv_obj_set_size(label_bar_max, 50, 50);
  lv_obj_set_drag(label_bar_max, false);                                //allow text to be dragged
  snprintf(text_buff, 7, "%2.1f", speed_target);                        //convert float to string
  lv_label_set_text(label_bar_max, text_buff);
  lv_obj_set_hidden(label_bar_max, false);

  label_arrow = lv_label_create(lv_scr_act(), NULL);
  lv_obj_set_style(label_arrow, &style3);                             //assign style to the label
  lv_obj_set_pos(label_arrow, 225, hor_pos);
  lv_obj_set_size(label_arrow, 50, 50);
  lv_obj_set_drag(label_arrow, false);                                //allow text to be dragged
  lv_label_set_text(label_arrow, LV_SYMBOL_EJECT);
  lv_obj_set_hidden(label_arrow, false);
  // int hor = 50;
  // int hor_space = 30;

  static lv_point_t line_points[] = { {56, 310}, {56, 290},                       //--------   Ruler grid under bar graph
    {71, 290}, {71, 295},   {71, 295}, {71, 290}, //--
    {86, 290}, {86, 301},   {86, 301}, {86, 290}, //----
    {101, 290}, {101, 295}, {101, 295}, {101, 290}, //--
    {116, 290}, {116, 310}, {116, 310}, {116, 290}, //--------
    {131, 290}, {131, 295}, {131, 295}, {131, 290}, //--
    {146, 290}, {146, 301}, {146, 301}, {146, 290}, //----
    {167, 290}, {167, 295}, {167, 295}, {167, 290}, //--
    {176, 290}, {176, 310}, {176, 310}, {176, 290}, //--------
    {191, 290}, {191, 295}, {191, 295}, {191, 290}, //--
    {206, 290}, {206, 300}, {206, 300}, {206, 290}, //----
    {221, 290}, {221, 295}, {221, 295}, {221, 290}, //--
    {236, 290}, {236, 310}, {236, 310}, {236, 290}, //--------
    {251, 290}, {251, 295}, {251, 295}, {251, 290}, //--
    {266, 290}, {266, 300}, {266, 300}, {266, 290}, //----
    {281, 290}, {281, 295}, {281, 295}, {281, 290}, //--
    {296, 290}, {296, 310}, {296, 310}, {296, 290}, //--------
    {311, 290}, {311, 295}, {311, 295}, {311, 290}, //--
    {326, 290}, {326, 300}, {326, 300}, {326, 290}, //----
    {341, 290}, {341, 295}, {341, 295}, {341, 290}, //--
    {356, 290}, {356, 310}, {356, 310}, {356, 290}, //--------
    {371, 290}, {371, 295}, {371, 295}, {371, 290}, //--
    {386, 290}, {386, 300}, {386, 300}, {386, 290}, //----
    {401, 290}, {401, 295}, {401, 295}, {401, 290}, //--
    {414, 290}, {414, 310}, {414, 310}, {414, 290} //--------
  };    /*Create an array for the points of the line*/
  static lv_style_t style_line1;                                             /*Create new style (thick black)*/
  lv_style_copy(&style_line1, &lv_style_plain);
  style_line1.line.color = LV_COLOR_MAKE(0x00, 0x3b, 0x75);              //set the color of the line to dark blue
  style_line1.line.width = 6;                                             //makelines 6 pix wide

  static lv_style_t style_line;                                             /*Create new style (thin black)*/
  lv_style_copy(&style_line, &lv_style_plain);
  style_line.line.color = LV_COLOR_MAKE(0x00, 0x00, 0x00);              //set the color of the line to dark blue
  style_line.line.width = 2;                                             //make lines 2 pix wide

  tick1 = lv_line_create(lv_scr_act(), NULL);                              //name of ruler marks under bar graph
  lv_line_set_points(tick1, line_points, 98);                              /*Identify the  points  declared in the array*/
  lv_line_set_style(tick1, LV_LINE_STYLE_MAIN, &style_line);             //apply a style to the line
  // lv_obj_align(tick1, NULL, LV_ALIGN_CENTER, 0,-130);                    //set position of line

  static lv_point_t line_points1[] = { {397, 38}, {397, 192}, {395, 192}, {480, 192},
    {480, 192}, {480, 110}, {480, 110}, {397, 110},
    {397, 110}, {480, 110}, {480, 92}, {480, 38}
  };

  //create an object for line
  unit_line = lv_line_create(lv_scr_act(), NULL);
  lv_line_set_style(unit_line, LV_LINE_STYLE_MAIN, &style_line1);             //apply a style to the line 6point wide
  lv_line_set_points(unit_line, line_points1, 12);                           //callt the point array

  /**********************
     Create Setup button
  **********************/  //upper left corner of screen
  static lv_style_t style_shadow;                                     //create a style
  lv_style_copy(&style_shadow,  &lv_style_pretty_color);
  btn_setup = lv_btn_create(lv_scr_act(), NULL);                       /*Add a setup button */
  lv_obj_set_hidden(btn_setup, false);                            //set to true to hide button
  lv_obj_set_drag(btn_setup, false);
  lv_obj_set_pos(btn_setup, 10, 10);                              /*Set its position in upper left corner*/
  lv_obj_set_size(btn_setup, 50, 50);                             /*Set its size*/
  lv_obj_set_event_cb(btn_setup, my_start_btn_cb);                /*Assign an event callback*/
  lv_obj_set_style(btn_setup, &style_shadow);
  lv_obj_t * label1 = lv_label_create(btn_setup, NULL);               /*Add a label to the button*/
  lv_obj_set_style(label1, &style1);
  lv_label_set_text(label1, LV_SYMBOL_SETTINGS "");                  /*Set the labels text*/

  /*****************************
     Create Distance reset button
  *******************************/  //upper right corner of screen
  btn_reset_dist = lv_btn_create(lv_scr_act(), NULL);                       /*Add a setup button */
  lv_obj_set_hidden(btn_reset_dist, false);                            //set to true to hide button
  lv_obj_set_drag(btn_reset_dist, false);
  lv_obj_set_pos(btn_reset_dist, 350, 2);                              /*Set its position in upper right corner*/
  lv_obj_set_size(btn_reset_dist, 120, 33);                             /*Set its size*/
  lv_obj_set_event_cb(btn_reset_dist, btn_reset_dist_cb);                /*Assign an event callback*/
  lv_obj_set_style(btn_reset_dist, &style_shadow);
  lv_obj_t * label13 = lv_label_create(btn_reset_dist, NULL);               /*Add a label to the button*/
  lv_obj_set_style(label13, &style1);
  lv_label_set_text(label13, LV_SYMBOL_REFRESH" Reset");                 /*Set the labels text*/

  /*******************
     speed bar display
  ********************/
  speed_bar();                                                            //routine to create speed bar
  if (graph == 1) {
    lv_obj_set_hidden(bar_speed, false);                             //show bar graph
    lv_obj_set_hidden(label_arrow, false);                           //display arrow in center of bar graph
    lv_obj_set_hidden(label_bar_min, false);                        //show minimum label under bar graph
    lv_obj_set_hidden(label_bar_max, false);                        //show max label under bar graph
  }                            //start the speed bar if selected in option window
  else {                                                                      //hide bar graph objects if not selected to display
    lv_obj_set_hidden(bar_speed, true);
    lv_obj_set_hidden(label_arrow, true);
    lv_obj_set_hidden(label_bar_min, true);                        //show minimum label under bar graph
    lv_obj_set_hidden(label_bar_max, true);                        //show max label under bar graph
  }

}//end of build_screen_run()
void build_screen_calibrate(void) {
  // lv_obj_set_hidden(bar_speed,true);
  old_cal_number = cal_number;
  run_screen_flag = 0;                                          //set flag so mph routine in loop will not run
#if serial_debug
  Serial.println("Entering build_screen_calibrate() - 362");
#endif

  static lv_style_t style3;                                      /*Declare a new style. Should be `static`*/
  lv_style_copy(&style3, &lv_style_plain);
  style3.text.font = &lv_font_roboto_28;                         //12,16,22,28 built in
  style3.text.color = LV_COLOR_WHITE;

  static lv_style_t style4;                                      /*Declare a new style. Should be `static`*/
  lv_style_copy(&style4, &lv_style_plain);
  style4.text.font = &lv_font_roboto_28;                         //12,16,22,28 built in
  style4.text.color = LV_COLOR_BLACK;

  static lv_style_t style5;                                      /*Declare a new style. Should be `static`*/
  lv_style_copy(&style5, &lv_style_plain);
  style5.text.font = &Bebasneue;                                //large font
  //style5.text.font = &lv_font_roboto_28;
  style5.text.color = LV_COLOR_BLACK;                            /*label color*/
  /****************
     ADD A TITLE
   ****************/
#if serial_debug
  Serial.println("line 465");
#endif
  //   title_label = lv_label_create(lv_scr_act(), NULL); /*First parameters (scr) is the parent*/
  //        lv_obj_set_style(title_label, &style4);
  lv_obj_set_pos(title_label, 88, 5);
  lv_label_set_text(title_label, "Speed Calibration Entry");  /*Set the text*/
  lv_obj_set_hidden(title_label, true);                      //hide object on build
  /**********************
      Create a label to display calibration
   **********************/
#if serial_debug
  Serial.println("line 473");
#endif
  label_cal = lv_label_create(lv_scr_act(), NULL);             //large digits that display the cal number
  lv_obj_set_hidden(label_cal, true);
  lv_obj_set_y(label_cal, 45);                                 //y position of label
  lv_obj_set_x(label_cal, 5);                                  //x posiion of label
  lv_obj_set_style(label_cal, &style5);                        //set style for label
  //cal_number = 17896;
  snprintf(text_buff, 6, "%d", cal_number);
  lv_label_set_text(label_cal, text_buff);                     //display text

  /**********************
     Create exit button
  **********************/  //lower left corner of screen
  static lv_style_t style_shadow;                             //create a style
  lv_style_copy(&style_shadow,  &lv_style_pretty_color);
#if serial_debug
  Serial.println("line 404");
#endif
  btn_exit = lv_btn_create(lv_scr_act(), NULL);                   /*Add a button the active screen*/
  lv_obj_set_hidden(btn_exit, true);                          //set to true to hide button
  lv_obj_set_drag(btn_exit, false);                            //turn drag feature off
  lv_obj_set_pos(btn_exit, 10, 250);                           /*Set its position*/
  lv_obj_set_size(btn_exit, 140, 60);                          /*Set its size*/
  // lv_obj_set_event_cb(btn_exit, btn_event_cb1);                 /*Assign a callback to the button*/
  lv_obj_set_event_cb(btn_exit, screen_calibrate_cb);              /*Assign an event callback*/
  lv_obj_set_style(btn_exit, &style_shadow);

  lv_obj_t * label1 = lv_label_create(btn_exit, NULL);          /*Create and Add a label to the button*/
  lv_obj_set_style(label1, &style3);
  lv_label_set_text(label1, LV_SYMBOL_CLOSE "  EXIT");       /*Set the labels text*/

  lv_obj_set_hidden(btn_exit, true);                         //hide object on creation


  /**********************
     Create Field Cal button
  **********************/  //center bottom of screen

  btn_field_cal = lv_btn_create(lv_scr_act(), btn_exit);             //create object for button
  lv_obj_set_pos(btn_field_cal, 167, 250);                           /*Set its position*/
  lv_obj_t * label9 = lv_label_create(btn_field_cal, NULL);          /*Add a label to the button*/
  lv_label_set_text(label9, "Field Cal.");       /*Set the labels text*/
  lv_obj_set_event_cb(btn_field_cal, screen_calibrate_cb);              /*Assign an event callback*/
  lv_obj_set_hidden(btn_field_cal, true);


  /**********************
     Create Set button
  **********************/  //lower right corner of screen
#if serial_debug
  Serial.println("line 543");
#endif
  btn_set_cal = lv_btn_create(lv_scr_act(), btn_exit);             //create object for button
  lv_obj_set_pos(btn_set_cal, 321, 250);                           /*Set its position*/
  lv_obj_t * label2 = lv_label_create(btn_set_cal, NULL);          /*Add a label to the button*/
  lv_label_set_text(label2, LV_SYMBOL_KEYBOARD "  SET");       /*Set the labels text*/
  lv_obj_set_event_cb(btn_set_cal, btn_set_cal_cb);              /*Assign an event callback*/
  lv_obj_set_hidden(btn_set_cal, true);



}
void build_field_calibrate(void) {
#if serial_debug
  Serial.println("Entering build_field_calibrate() - 1603");
#endif
  //create horizontal line for 300 ft
  static lv_style_t style_line;
  lv_style_copy(&style_line, &lv_style_plain);
  // style_line.line.color = LV_COLOR_MAKE(BLACK);              //set the color of the line to dark blue
  style_line.line.width = 6;
  style_line.line.rounded = 1;



  static lv_point_t line_points[] = { {125, 80}, {345, 80}};               /*Create an array for the points of the line*/
  line_fld_cal_hor = lv_line_create(lv_scr_act(), NULL);
  lv_line_set_points(line_fld_cal_hor, line_points, 2);                              /*Identify the 2 points  declared in the array*/
  lv_line_set_style(line_fld_cal_hor, LV_LINE_STYLE_MAIN, &style_line);             //apply a style to the line
  //lv_obj_align(line_fld_cal_hor, NULL, LV_ALIGN_CENTER, 0,-130);                    //set position of line
  lv_obj_set_hidden(line_fld_cal_hor, true);                                       //hide hor line

  right_arrow = lv_label_create(lv_scr_act(), NULL);          /*Right arrow*/
  lv_label_set_text(right_arrow, LV_SYMBOL_NEXT);
  lv_obj_set_pos(right_arrow, 349, 65);
  lv_obj_set_hidden(right_arrow, true);

  left_arrow = lv_label_create(lv_scr_act(), NULL);          /*left arrow*/
  lv_label_set_text(left_arrow, LV_SYMBOL_PREV);
  lv_obj_set_pos(left_arrow, 110, 65);
  lv_obj_set_hidden(left_arrow, true);


  // lv_obj_set_pos(title_label,105,5);
  lv_label_set_text(title_label, "Field Calibration");  /*Set the text*/
  lv_obj_set_hidden(title_label, true);                      //hide object on build

  static lv_style_t style_shadow;                             //create a style
  lv_style_copy(&style_shadow, &lv_style_pretty_color);

  static lv_style_t style5;                                      /*Declare a new style. Should be `static`*/
  lv_style_copy(&style5, &lv_style_plain);
  style5.text.font = &Bebasneue;  
  //style5.text.font = &lv_font_roboto_28;                              //large font used for speed reading 
  style5.text.color = LV_COLOR_BLACK;                            /*label color*/
#if serial_debug
  Serial.println("line 1644");
#endif
  label_field_cal = lv_label_create(lv_scr_act(), NULL);             //create label object
  lv_obj_set_y(label_field_cal, 45);                                 //y position of label
  lv_obj_set_x(label_field_cal, 5);                                  //x posiion of label
  lv_obj_set_style(label_field_cal, &style5);                        //set style for label
  //cal_number = 17896;
  snprintf(text_buff, 6, "%d", field_calibration);
  lv_label_set_text(label_cal, text_buff);                     //display text

  field_save_btn = lv_btn_create(lv_scr_act(), NULL);                   /*Add a button the active screen*/
  lv_obj_set_hidden(field_save_btn, true);                          //set to true to hide button
  lv_obj_set_drag(field_save_btn, false);                            //turn drag feature off
  lv_obj_set_pos(field_save_btn, 370, 255);                           /*Set its position*/
  lv_obj_set_size(field_save_btn, 100, 60);                          /*Set its size*/
  lv_obj_set_event_cb(field_save_btn, field_calibrate_cb);                 /*Assign a callback to the button*/
  lv_obj_set_style(field_save_btn, &style_shadow);
  lv_obj_t * label10 = lv_label_create(field_save_btn, NULL);          /*Add a label to the button*/
  lv_label_set_text(label10, "Save");                               /*Set the labels text*/

  field_start_btn = lv_btn_create(lv_scr_act(), NULL);                   /*Add a button the active screen*/
  lv_obj_set_hidden(field_start_btn, true);                          //set to true to hide button
  lv_obj_set_drag(field_start_btn, false);                            //turn drag feature off
  lv_obj_set_pos(field_start_btn, 5, 50);                           /*Set its position*/
  lv_obj_set_size(field_start_btn, 100, 55);                          /*Set its size*/
  lv_obj_set_event_cb(field_start_btn, field_calibrate_cb);                 /*Assign a callback to the button*/
  lv_obj_set_style(field_start_btn, &style_shadow);
  lv_obj_t * label11 = lv_label_create(field_start_btn, NULL);          /*Add a label to the button*/
  lv_label_set_text(label11, "Start");

  field_end_btn = lv_btn_create(lv_scr_act(), NULL);                   /*Add a button the active screen*/
  lv_obj_set_hidden(field_end_btn, true);                          //set to true to hide button
  lv_obj_set_drag(field_end_btn, false);                            //turn drag feature off
  lv_obj_set_pos(field_end_btn, 370, 50);                           /*Set its position*/
  lv_obj_set_size(field_end_btn, 100, 55);                          /*Set its size*/
  lv_obj_set_event_cb(field_end_btn, field_calibrate_cb);                 /*Assign a callback to the button*/
  lv_obj_set_style(field_end_btn, &style_shadow);
  lv_obj_t * label12 = lv_label_create(field_end_btn, NULL);          /*Add a label to the button*/
  lv_label_set_text(label12, "End");

  field_cancel_btn = lv_btn_create(lv_scr_act(), NULL);                   /*Add a button the active screen*/
  lv_obj_set_hidden(field_cancel_btn, true);                          //set to true to hide button
  lv_obj_set_drag(field_cancel_btn, false);                            //turn drag feature off
  lv_obj_set_pos(field_cancel_btn, 5, 255);                           /*Set its position*/
  lv_obj_set_size(field_cancel_btn, 100, 60);                          /*Set its size*/
  lv_obj_set_event_cb(field_cancel_btn, field_calibrate_cb);                 /*Assign a callback to the button*/
  lv_obj_set_style(field_cancel_btn, &style_shadow);
  lv_obj_t * label13 = lv_label_create(field_cancel_btn, NULL);          /*Add a label to the button*/
  lv_label_set_text(label13, LV_SYMBOL_CLOSE " Exit");
#if serial_debug
  Serial.println("line 1694");
#endif

  label_300_ft = lv_label_create(lv_scr_act(), NULL);                   /*Add a label to screen*/
  lv_obj_set_pos(label_300_ft, 165, 46);                                 //x posiion of label
  lv_label_set_text(label_300_ft, "Drive 300 FT");                 /*Set the text*/
  lv_obj_set_hidden(label_300_ft, true);

  label_cal_inst = lv_label_create(lv_scr_act(), NULL);                   /*Add a label to screen*/
  lv_obj_set_pos(label_cal_inst, 120, 140);                           //set position
  lv_label_set_text(label_cal_inst, "1.  Measure off 300 feet\n2.  Press START\n3.  Drive 300 feet\n4.  Press STOP\n5.  Press SAVE");                 /*Set the text*/
  lv_obj_set_hidden(label_cal_inst, true);                           //hide text
}
void build_set_target(void){                                                //screen to set multiple target speeds
   int hrow = 215;                                                       //locate buttons to bottom of screen
  ma_keypad = lv_btnm_create(lv_scr_act(), NULL);
  lv_btnm_set_map(ma_keypad, btnm_map);                                 //create a numeric keypad to enter data
  lv_obj_set_size(ma_keypad, 478, 126);                                  /*Set keypad size*/
  lv_obj_align(ma_keypad, NULL, LV_ALIGN_CENTER, 0, 98);                 //set position of keypad
  lv_obj_set_event_cb(ma_keypad, ma_keyboard_cb);                       //callback for multi alarm keypad entry
  lv_obj_set_hidden(ma_keypad, true);                                   //hide object on build     

  label_alarm1 = lv_label_create(lv_scr_act(), NULL);                //create a label to display preset alarm speed
 // lv_obj_set_style(label_alarm1, &style2);                            //assign style to the label
  lv_obj_set_width(label_alarm1, 480);
  lv_obj_set_y(label_alarm1, 11);                                      //set position of label
  lv_obj_set_x(label_alarm1, 185);
  lv_label_set_align(label_alarm1, LV_LABEL_ALIGN_CENTER);
  lv_label_set_text(label_alarm1, "");                                 //write text to label                                    
  
  btn_set_alarm_1 = lv_btn_create(lv_scr_act(), NULL);                     /*Add a "Reset Password" button */
  lv_obj_set_drag(btn_set_alarm_1, false);                             //turn drag feature off
  lv_obj_set_pos(btn_set_alarm_1, 5, hrow);                             /*Set its position*/
  lv_obj_set_size(btn_set_alarm_1, 88, 100);                           /*Set its size*/
  lv_obj_set_event_cb(btn_set_alarm_1, mult_alarm_cb);                   /*Assign an event callback*/
  label_set_1 = lv_label_create(btn_set_alarm_1, NULL);             /*Add a label to the button*/
  lv_label_set_text(label_set_1,  "Alarm\n    1");   
  lv_obj_set_hidden(btn_set_alarm_1, true);                           //hide object on build         

 btn_set_alarm_2 = lv_btn_create(lv_scr_act(), NULL);                     /*Add a "Reset Password" button */
  lv_obj_set_drag(btn_set_alarm_2, false);                             //turn drag feature off
  lv_obj_set_pos(btn_set_alarm_2, 100, hrow);                             /*Set its position*/
  lv_obj_set_size(btn_set_alarm_2, 88, 100);                           /*Set its size*/
  lv_obj_set_event_cb(btn_set_alarm_2, mult_alarm_cb);                   /*Assign an event callback*/
  label_set_2 = lv_label_create(btn_set_alarm_2, NULL);             /*Add a label to the button*/
  lv_label_set_text(label_set_2,  "Alarm\n    2");
 lv_obj_set_hidden(btn_set_alarm_2, true);                      //hide object on build                 

  btn_set_alarm_3 = lv_btn_create(lv_scr_act(), NULL);          /*Add a "Reset Password" button */
  lv_obj_set_drag(btn_set_alarm_3, false);                      //turn drag feature off
  lv_obj_set_pos(btn_set_alarm_3, 195, hrow);                   /*Set its position*/
  lv_obj_set_size(btn_set_alarm_3, 88, 100);                    /*Set its size*/
  lv_obj_set_event_cb(btn_set_alarm_3, mult_alarm_cb);            /*Assign an event callback*/
  label_set_3 = lv_label_create(btn_set_alarm_3, NULL);         /*Add a label to the button*/
  lv_label_set_text(label_set_3,  "Alarm\n     3");  
  lv_obj_set_hidden(btn_set_alarm_3, true);                      //hide object on build   

  btn_set_alarm_4 = lv_btn_create(lv_scr_act(), NULL);           /*Add a "Reset Password" button */
  lv_obj_set_drag(btn_set_alarm_4, false);                       //turn drag feature off
  lv_obj_set_pos(btn_set_alarm_4, 290, hrow);                    /*Set its position*/
  lv_obj_set_size(btn_set_alarm_4, 88, 100);                     /*Set its size*/
  lv_obj_set_event_cb(btn_set_alarm_4, mult_alarm_cb);             /*Assign an event callback*/
  label_set_4 = lv_label_create(btn_set_alarm_4, NULL);          /*Add a label to the button*/
  lv_label_set_text(label_set_4,  "Alarm\n    4"); 
   lv_obj_set_hidden(btn_set_alarm_4, true);                      //hide object on build   
   
  btn_set_alarm_exit = lv_btn_create(lv_scr_act(), NULL);         /*Screen Exit button*/
  lv_obj_set_drag(btn_set_alarm_exit, false);                     //turn drag feature off
  lv_obj_set_pos(btn_set_alarm_exit, 385, hrow);                  /*Set its position*/
  lv_obj_set_size(btn_set_alarm_exit, 88, 100);                   /*Set its size*/
  lv_obj_set_event_cb(btn_set_alarm_exit, mult_alarm_cb);           /*Assign an event callback*/
  lv_obj_t * label30 = lv_label_create(btn_set_alarm_exit, NULL); /*Add a label to the button*/
  lv_label_set_text(label30,"    " LV_SYMBOL_DOWNLOAD "\n SAVE"); /*Set the labels text*/
  lv_obj_set_hidden(btn_set_alarm_exit, true);                    //hide object on build   
}
void field_calibrate_cb(lv_obj_t * obj, lv_event_t event) {                    // call back for "Field Calibrate" screen
  switch (event) {
    case LV_EVENT_PRESSED:
      Serial.println("field_calibrate_cb - line 1148");

      if (obj == field_cancel_btn) {                                          //
        field_calibrate_off();                                                   //close field calibration window
        screen_calibrate_on();
        run_screen_flag = 1;                                                   //set flag to enable code in main loop
        field_cal_flag  = 0;                                               //turn off flag to disable code in main loop
      }

      if (obj == field_start_btn) {                                          //code to toggle between mph and kph
        messbox_start_run = lv_mbox_create(lv_disp_get_scr_act(NULL), NULL);   //create a message box
        lv_obj_set_width(messbox_start_run, 350);                             //width of message box
        lv_mbox_set_text(messbox_start_run, "Begin driving 300 feet");     //message box to start driving
        lv_mbox_start_auto_close(messbox_start_run, 4000);                 //close box after 4 seconds
        lv_mbox_set_anim_time(messbox_start_run, 1500);
        lv_obj_align(messbox_start_run, NULL, LV_ALIGN_CENTER, -10, 0); //position of  box
        field_cal_flag = 1;                                                 //set flag so loop will start displaying the pulse count
        run_screen_flag = 0;
        pulse = 0;                                                        //reset the count on start
        lv_obj_set_hidden(label_speed, false);                           //show large text
        lv_obj_set_hidden(label_cal_inst, true);                         //hide instructions
      }

      if (obj == field_save_btn) {                               //if save button was pressed
        if (cal_number != pulse) {                             //new cal number?
          cal_number = pulse;                            //assingn new cal number based on field cal run
          EEPROM.put(0x05, cal_number);                  //save calibrtion value to eeprom if changed
          EEPROM.commit();
          calculate_speed_constant();                    //calculate new speed constant
          messbox_start_run = lv_mbox_create(lv_disp_get_scr_act(NULL), NULL);   //create a message box
          lv_obj_set_width(messbox_start_run, 350);                                         //width of message box
          lv_mbox_set_text(messbox_start_run, "New calibration number saved!");
          lv_mbox_start_auto_close(messbox_start_run, 3000);            //close box after 3 seconds
          lv_mbox_set_anim_time(messbox_start_run, 1500);
          lv_obj_align(messbox_start_run, NULL, LV_ALIGN_CENTER, -10, 0);
        }
        field_calibrate_off();                                                   //close field calibration window
        screen_calibrate_on();                                                    //return to calibration screen
        run_screen_flag = 1;
        field_cal_flag  = 0;
      }
      if (obj == field_end_btn) {
        field_cal_flag = 0;                                                          //turn off flag to stop reading pulses

        if (pulse != 0) {
          if (pulse >= 3000) {
            messbox_start_run = lv_mbox_create(lv_disp_get_scr_act(NULL), NULL);   //create a message box
            lv_obj_set_width(messbox_start_run, 350);                   //width of message box
            lv_mbox_set_text(messbox_start_run, "Press 'Save' to accept new calibration number  or 'Cancel' to keep the old calibration number.");
            lv_mbox_start_auto_close(messbox_start_run, 3000);
            lv_mbox_set_anim_time(messbox_start_run, 1500);
            lv_obj_align(messbox_start_run, NULL, LV_ALIGN_CENTER, -10, 0);
          }
          else {
            messbox_start_run = lv_mbox_create(lv_disp_get_scr_act(NULL), NULL);   //create a message box
            lv_obj_set_width(messbox_start_run, 350);                   //width of message box
            lv_mbox_set_text(messbox_start_run, "Calibration number must be at least 3000.");
            lv_mbox_start_auto_close(messbox_start_run, 3000);
            lv_mbox_set_anim_time(messbox_start_run, 1500);
            lv_obj_align(messbox_start_run, NULL, LV_ALIGN_CENTER, -10, 0);
            pulse  = 0;
          }
        }
      }

  }
}
void screen_calibrate_cb(lv_obj_t *obj, lv_event_t event) {
    // Callback function for "Screen Calibrate" (distance calibration)
    if (event == LV_EVENT_PRESSED) {
        if (obj == btn_exit) {
            // Exit button
            Serial.println("screen_calibrate_cb - line 1118");
            screen_calibrate_off(); // Turn off all objects on screen
            screen_run_on();        // Go back to run screen
        } else if (obj == btn_field_cal) {
            Serial.println("line 900");
            field_calibrate_on();
        }
    }
}

/*=====================  start of loop   =========================================*/
void loop() {

  if (alarm_enable == 1 && run_screen_flag ==1){                   //is alarm feature set?
    alarm_Flash_Timer();                                           //call routine to flash warning light at varible rates
    }
  else{
    digitalWrite(alarm_light,false);                               //turn off light if not in alarm mode
    }
  
  if (millis() - task_millis >= 5) {
    lv_task_handler();                                             //this program executes the graphics
    task_millis = millis();
  }                                       //reset counter
  
  if (run_screen_flag == 1) {                                       //this flag is set to one to display the speed on the run screen
                            
   if(calc_flag == true)                                            //run loop every 250ms (timer set in line 430)
    {  char buf[75];
      
      calc_flag = false;                                            //clear the flag
      status_mode = !status_mode;                                   //toggle value
      digitalWrite(status_light,status_mode);                      //update on board led 
      
      
      if(speed_input != 0){                                         //read serial if GPS is selected
           while (Serial2.available() > 0) {
                //  Serial.print(char(Serial2.peek()));             //send to serial monitor
                 serial_pointer = serial_pointer+1; 
                  if (serial_pointer >= 124){                      //check for overflow of buffer
                    serial_pointer = 0;                             //reset pointer
                    }
//                  if (char(Serial2.peek()) == 'W'){                 //check for a "W" int the string that is just before the speed reading
//                    gps_string_pointer = serial_pointer;            //save the position value of the W
//                  }
                  serial_buffer[serial_pointer] = Serial2.read();    //save character to buffer
//                 Serial.print(serial_buffer[serial_pointer]);      //send to serial monitor

                  if (serial_buffer[serial_pointer] == '*'){         //* symbol is near end of string after speed
                           serial_buffer[serial_pointer] = 0x00;     //reset pointer
                          
                        //     digitalWrite(alarm_light,!digitalRead(alarm_light)); //diagnostic to toggle front panel light
                            serial_pointer = 0;                       //reset pointer
                           sprintf(buf,"%c%c%c%c%c%c%c",serial_buffer[50],serial_buffer[51],serial_buffer[52],  //save speed to buf and add null zero
                           serial_buffer[53],serial_buffer[54],serial_buffer[55],'\0');
//       ****diagnostic line below
      //                     lv_label_set_text(title_label,buf);                    //***diagnostic display at top of screen
//                          
                     velocity = float(atoi(buf));                                    //convert ascaii speed to a float number
                     velocity = 1.1508*(velocity);                                   //convert knots to mph
                     snprintf(buf, 8, "%4.1f", velocity);                       //***diagnostic line
                     Serial.println(buf);                                      //***diagnostic line
                  }
           }
      }
      else{                                                                     //else section is for wheel pulse input

                //***diagnostic (print pulse  count to top of screen)======================
//                    lv_obj_set_hidden(title_label,false);   //show text
//                    snprintf(buf,30,"%d pulses",old_pulse);     //convert to text
//                    lv_label_set_text(title_label, buf);            //update screen
               //=============================================================

 //       total_pulse  = total_pulse + (float)old_pulse;          //add pulses to the total pulse counter
      
 //       distance = (pulse_distance * total_pulse) / 12;         //calculate distance in feet
          velocity = (float(old_pulse) / (speed_constant / 4));   //divide speed constant by 4 for 1/4 second updates
          if(speed_avg == 1){                              //if speed averaging is turned on
              if (velocity <= (1.5 * old_velocity) && (velocity >= (.5 * old_velocity))) //is value within 50% of last value?
              {
                velocity = (velocity + 3*old_velocity) / 4; //average velocity with last reading. 4 reading average with 3x weight given to last reading
              }
         }
      }
      
      old_velocity = velocity;                                //save current velocity for calculation next time through loop
      if (units == 2) {                                       //if in kilometer mode convert to kilometers
          velocity = velocity * 1.6092;                     //convert mph speed to kmh
        }

      if (velocity <= 50) {                                  //check for a speed under 50
          snprintf(buf, 8, "%4.1f", velocity);               //Display speed reading on screen in tenths
          if (velocity ==0){
               digitalWrite(alarm_light,false);}            //turn off alarm light on zero velocity
          }
      else {                                                 //do not display decimal if over 50mph or 50kph
          snprintf(buf, 8, "%4.0f", velocity);
          }
     
      lv_label_set_text(label_speed, buf);                 //write new text to screen in large number font

      //-----

      if (graph == 1) {                                    //if bar graph is turned on

         if (alarm_enable == 1) {                           //if alarm bit is set flash bar on target speed exceeded
            if (velocity > speed_target) {
          //    digitalWrite(alarm_light,true);                //turn on alarm light
              digitalWrite(aux_light,true);                  //turn on aux output  (used when using 12 volt panel mount light)
              style3.body.main_color = LV_COLOR_RED;         //set bar color to red if over target speed
              style3.body.grad_color = LV_COLOR_RED;
              }
            else {
        //      digitalWrite(alarm_light,false); 
              digitalWrite(aux_light,false);                  //turn off aux output
              style3.body.main_color = LV_COLOR_GREEN;       //set bar color to green if below target
              style3.body.grad_color = LV_COLOR_GREEN;
           
            }
         }
        else {
          style3.body.main_color = LV_COLOR_BLUE;         //set bar color to blue if not in alarm show mode
          style3.body.grad_color = LV_COLOR_BLUE;
          }
        lv_bar_set_value(bar_speed, (int)(velocity * 10), LV_ANIM_OFF); //update the bar graph
      }
      else {
        lv_obj_set_hidden(bar_speed, true);                   //hide bar graph if not enabled
        }
      //----------
      if (dis == 1) {                                           //if display distance checkbox is checked then display distance
        if (units == 2) {
          distance = distance * .3048;                    //convert to meters if in metric mode
          snprintf(buf, 30, "%d Meters", (int)distance);  //convert to text
          lv_label_set_text(title_label, buf);            //update screen
        }
        else {
          snprintf(buf, 30, "%d Feet", (int)distance);    //convert to text
          lv_label_set_text(title_label, buf);            //update screen
        }
      }

      if (fpm == 1) {                                                  //if "show fpm" check box is turned on
        //display fpm on run screen under units
        float feet_per_min ;                                         //create varible

        if (units == 1) {                                            //if in english
          feet_per_min = velocity * 5280 / 60;                     //convert to feet/per/minute
          snprintf(buf, 20, "FPM\n%d ", (int)feet_per_min);       //display feet per minute on run screen
        }
        else {                                                       //else if in metric
          feet_per_min = velocity * (5280 / 60) * .3048;          //convert to meters per minute
          snprintf(buf, 20, "MPM\n%d ", (int)feet_per_min);       //display meters per minute on run screen
        }
        lv_label_set_text(lab_fpm, buf);                             //display the feet per minute or meters per minuete to screen
      }
    }
  }
  else if (field_cal_flag == 1) {                                       //if in field calibration mode
    char buf[10];
    if (millis() - old_millis >= 50) {                                 //update pulse count to screen every 50ms (field calibration mode)
      snprintf(buf, 8, "%5d", pulse);                                //convert number to string
      lv_label_set_text(label_speed, buf);                           //display the pulse count during calibration run
      old_millis = millis();                                         //reset 100 ms second counter
    }
  }

}//end of loop()
//=====================  Functions  ==================================
void mult_alarm_cb(lv_obj_t * btn, lv_event_t event){                           //call back for multi alarm buttons
  int h_pos = 390;                                                              //position to relocate button when pressed to show numeric keypad
  int v_pos = 5;
  if (btn == btn_set_alarm_1) {                                                 //if the "Alarm 1" button was pressed
      alarm_var = 1;                                                           //set varible so program knows which button to process
      lv_obj_set_pos(btn_set_alarm_1, h_pos, v_pos);                           //relocate button                                                                          
      lv_obj_set_hidden(ma_keypad, false);                                     //show keypad
      lv_obj_set_hidden(btn_set_alarm_2, true);                                //hide other 4 buttons
      lv_obj_set_hidden(btn_set_alarm_3, true);
      lv_obj_set_hidden(btn_set_alarm_4, true);
      lv_obj_set_hidden(btn_set_alarm_exit, true);
      snprintf(text_buff, 7, "%2.1f", alarm_speed_1);                        //display current alarm 1 speed setting
      speed_target = alarm_speed_1;                                         //assign to varible used in keypad call back routine
      Serial.print("2172 alarm_speed_1 = ");
      Serial.println(alarm_speed_1);   //***diagnostic
      lv_label_set_text(label_cal,text_buff);                              //show the current alarm speed
  }
  if (btn == btn_set_alarm_2) {                                               //if the "Alarm 2" button was pressed
      alarm_var = 2;                                                           //set varible so program knows which button to process
      lv_obj_set_pos(btn_set_alarm_2, h_pos, v_pos);                                                                                 
      lv_obj_set_hidden(ma_keypad, false);                                     //show keypad
      lv_obj_set_hidden(btn_set_alarm_1, true);                                //hide other 4 buttons
      lv_obj_set_hidden(btn_set_alarm_3, true);
      lv_obj_set_hidden(btn_set_alarm_4, true);
      lv_obj_set_hidden(btn_set_alarm_exit, true);
      snprintf(text_buff, 7, "%2.1f", alarm_speed_2);                       //convert float value to text
      speed_target = alarm_speed_2;                                         //assign to varible used in keypad call back routine
      Serial.print("2204 alarm_speed_2 = ");     
      Serial.println(alarm_speed_2);   //***diagnostic
      lv_label_set_text(label_cal,text_buff);                              //show the current alarm speed
      
  }
  if (btn == btn_set_alarm_3) {                                               //if the "Alarm 3" button was pressed
     alarm_var = 3;                                                           //set varible so program knows which button to process
     lv_obj_set_pos(btn_set_alarm_3, h_pos, v_pos);                           ///move button                                                         
     lv_obj_set_hidden(ma_keypad, false);                                     //show keypad
     lv_obj_set_hidden(btn_set_alarm_1, true);                                //hide other 4 buttons
     lv_obj_set_hidden(btn_set_alarm_2, true);
     lv_obj_set_hidden(btn_set_alarm_4, true);
     lv_obj_set_hidden(btn_set_alarm_exit, true);
     snprintf(text_buff, 7, "%2.1f", alarm_speed_3);                       //convert float value to text
     speed_target = alarm_speed_3;                                         //assign to varible used in keypad call back routine
     Serial.print("2203 alarm_speed_3 = ");
     Serial.println(alarm_speed_3);   //***diagnostic
     lv_label_set_text(label_cal,text_buff);                              //show the current alarm speed
             
  }
  if (btn == btn_set_alarm_4) {                                              //if the "Alarm 4" button was pressed
       alarm_var = 4;                                                        //set varible so program knows which button to process
       lv_obj_set_pos(btn_set_alarm_4, h_pos, v_pos);                       //move button                                                          
       lv_obj_set_hidden(ma_keypad, false);                                  //show keypad
       lv_obj_set_hidden(btn_set_alarm_1, true);                                //hide other 4 buttons
       lv_obj_set_hidden(btn_set_alarm_2, true);
       lv_obj_set_hidden(btn_set_alarm_3, true);
       lv_obj_set_hidden(btn_set_alarm_exit, true);
       snprintf(text_buff, 7, "%2.1f", alarm_speed_4);                       //convert float value to text
       speed_target = alarm_speed_4;                                         //assign to varible used in keypad call back routine
      // txt = text_buff;                                                      //load value into varible used in keyboard read routine
       Serial.print("2220 alarm_speed_4 = ");
       Serial.println(alarm_speed_4);   //***diagnostic
       lv_label_set_text(label_cal,text_buff);                              //show the current alarm speed
   }
  if (btn == btn_set_alarm_exit) {                                             //if the "Save" button was pressed
       lv_obj_set_hidden(btn_set_alarm_1, true);                               //hide the 5  pushbuttons
       lv_obj_set_hidden(btn_set_alarm_2, true);
       lv_obj_set_hidden(btn_set_alarm_3, true);
       lv_obj_set_hidden(btn_set_alarm_4, true);
       lv_obj_set_hidden(btn_set_alarm_exit, true);    

      EEPROM.put(speed_target_1_ee_adr, 11);                   //save to eeprom (floating point varibles
      EEPROM.put(speed_target_2_ee_adr, 22); 
      EEPROM.put(speed_target_3_ee_adr, 33); 
      EEPROM.put(speed_target_4_ee_adr, 44); 
      EEPROM.put(speed_target_ee_adr, speed_target);                   //save to eeprom
      EEPROM.commit();
      Serial.println("^^^^^^^^^^^^^^^");
      Serial.println(alarm_speed_1);   //***diagnostic
      Serial.println(alarm_speed_2);
      Serial.println(alarm_speed_3);
      Serial.println(alarm_speed_4);                                                          
       option_screen_on();                                                  //go back to option screen   
         
  }
}

void ma_keyboard_cb(lv_obj_t * obj, lv_event_t event){               //call back from multi alarm keypad to enter alarm speed

  if (event == LV_EVENT_VALUE_CHANGED) {                                   //if a key was pressed
    Serial.println("line 2245");
    txt = lv_btnm_get_active_btn_text(obj);                                //get the text value of pressed button
    if (txt != nullptr) {                                                  //if valid text
      hold_text = hold_text + txt;                                         //add character to buffer
      Serial.print(hold_text);                                             //send button text value to serial monitor
      Serial.println(".....line 2250");
      if (*txt != 'E'){                                                     //if enter key was not pressed
          speed_target = hold_text.toFloat() * .1;                            //convert to a float number
          snprintf(text_buff, 7, "%2.1f", speed_target);                       //convert to text number                   
          lv_label_set_text(label_cal, text_buff);                             //update large numbers on display
          lv_bar_set_range(bar_speed, 0, (int)((2 * speed_target) * 10));      //reset max for 2 times target speed
    
          printf("%2.1f is current string value line 2180\n", speed_target);   //***diagnostic code
          Serial.println("line 2283");
          
        }
      if (*txt == 'C') {                                                    //was clear button pressed?
        hold_text = "";                                                     //clear buffer
        lv_label_set_text(label_cal, "---");                                //remove numbers and dispay "---"
      }

      if (strlen(temp_buff) >= 6) {
        lv_obj_set_pos(title_label, 40, 5);
        lv_label_set_text(title_label, "Enter Target Speed");               //refresh text at top of screeen
        lv_label_set_text(label_cal, "---");                                //remove numbers and dispay "---"
        strcpy(temp_buff, "");                                              //clear the buffer
      }
      Serial.println("line 2267");
      if (*txt == 'E') {                                                    //enter key pressed?
        Serial.print("speed_target = ");
        Serial.println(speed_target);
        if (speed_target <= 30.0 && speed_target >= .5) {                            //
          Serial.println("Successful target speed - line 2301");
         // screen_set_multi_target_off();                                   //turn off 5 keys "1 2 3 4 save"
         // option_screen_on();
          snprintf(text_buff, 7, "%2.1f", speed_target);
        //  lv_label_set_text(label_bar_max, text_buff);
         
         
          
//          lv_obj_t * messbox_invalid = lv_mbox_create(lv_disp_get_scr_act(NULL), NULL);   //create a message box that input is invalid
//          lv_obj_set_width(messbox_invalid, 400);                                         //width of message box
//          lv_mbox_set_text(messbox_invalid, "\nTarget speed set!\n");
//          lv_mbox_start_auto_close(messbox_invalid, 2000);                    //display message box for 2 seconds
//          lv_mbox_set_anim_time(messbox_invalid, 1500);
//          lv_obj_align(messbox_invalid, NULL, LV_ALIGN_CENTER, 0, 0);
          hold_text = "";                                                     //clear buffer
          
          if (alarm_var == 1){                                                 //if alarm 1 was selected to edit
               lv_obj_set_hidden(ma_keypad, true);                             //hide the keypad
               lv_obj_set_hidden(btn_set_alarm_1, false);                       //show 5 pushbuttons
               lv_obj_set_hidden(btn_set_alarm_2, false);
               lv_obj_set_hidden(btn_set_alarm_3, false);
               lv_obj_set_hidden(btn_set_alarm_4, false);
               lv_obj_set_hidden(btn_set_alarm_exit, false);
               lv_obj_set_pos(btn_set_alarm_1, 5, 215 );                       //move button 1 back into normal position 
               alarm_speed_1 = speed_target;                                   //load varible with entered value
               lv_label_set_text(label_cal, "");                                //remove numbers and dispay blank on screen
          }
          if (alarm_var == 2){                                                          //if alarm 2 was selected to edit
               lv_obj_set_hidden(ma_keypad, true);                                     //hide the keypad
               lv_obj_set_hidden(btn_set_alarm_1, false);                               //show 5 pushbuttons
               lv_obj_set_hidden(btn_set_alarm_2, false);
               lv_obj_set_hidden(btn_set_alarm_3, false);
               lv_obj_set_hidden(btn_set_alarm_4, false);
               lv_obj_set_hidden(btn_set_alarm_exit, false);
               lv_obj_set_pos(btn_set_alarm_2, 100, 215 );                             //move button 2 back into normal position 
               alarm_speed_2 = speed_target;                                   //load varible with entered value
               lv_label_set_text(label_cal, "");                                //remove numbers and dispay blank on screen
          }
          if (alarm_var == 3){                                                       //if alarm 3 is selected to edit
              lv_obj_set_hidden(ma_keypad, true);                                     //hide the keypad
               lv_obj_set_hidden(btn_set_alarm_1, false);                               //show 5 pushbuttons
               lv_obj_set_hidden(btn_set_alarm_2, false);
               lv_obj_set_hidden(btn_set_alarm_3, false);
               lv_obj_set_hidden(btn_set_alarm_4, false);
               lv_obj_set_hidden(btn_set_alarm_exit, false);
               lv_obj_set_pos(btn_set_alarm_3, 195, 215 );                         //move button 3 back into position
               alarm_speed_3 = speed_target;                                   //load varible with entered value
               lv_label_set_text(label_cal, "");                                //remove numbers and dispay blank on screen
          }
          if (alarm_var == 4){
              lv_obj_set_hidden(ma_keypad, true);                                     //hide the keypad
               lv_obj_set_hidden(btn_set_alarm_1, false);                               //show 5 pushbuttons
               lv_obj_set_hidden(btn_set_alarm_2, false);
               lv_obj_set_hidden(btn_set_alarm_3, false);
               lv_obj_set_hidden(btn_set_alarm_4, false);
               lv_obj_set_hidden(btn_set_alarm_exit, false);
               lv_obj_set_pos(btn_set_alarm_4, 290, 215 );                         //move button 4 back into position
               alarm_speed_4 = speed_target;                                   //load varible with entered value
               lv_label_set_text(label_cal, "");                                //remove numbers and dispay blank on screen
          }
          hold_text = "";                                                 //erase buffer
        }
      }
    }

  }
  Serial.println("line 2346 - end of ma_keyboard call back");
}
void test_flash_alarm(void){
   digitalWrite(aux_light,false);                  //turn off aux output
   delay(150);
   digitalWrite(aux_light,true);                  //turn off aux output
   delay(150);
   digitalWrite(aux_light,false);                  //turn off aux output
   delay(150);
   digitalWrite(aux_light,true);                  //turn off aux output
   delay(150);
   digitalWrite(aux_light,false);                  //turn off aux output
   delay(150);
   digitalWrite(aux_light,true);                  //turn off aux output
   delay(150);
   digitalWrite(aux_light,false);                  //turn off aux output
   delay(150);
   digitalWrite(aux_light,true);                  //turn off aux output
   delay(150);
   digitalWrite(aux_light,false);                  //turn off aux output
   delay(150);
   digitalWrite(aux_light,true);                  //turn off aux output
   delay(150);
   digitalWrite(aux_light,false);
}
void enable_gps(void){
      detachInterrupt(digitalPinToInterrupt(25));                                   //turn off pulse interrupt                              
      Serial2.end(); 
      Serial2.begin(19200,SERIAL_8N1,25,22);                                       //start serial port to  read gps string
      
}
void enable_radar(void){
     Serial.println("enable wheel pulse interput");
     attachInterrupt(digitalPinToInterrupt(25), speed_pulse, RISING);                //turn on pulse interrupt
}
void btn_reset_dist_cb(lv_obj_t * btn, lv_event_t event) {                          //distance reset button callback for button in upper right corner of  run screen
  Serial.println("line 1180 btn_reset_dist_cb");                                    //***disagnostic
  
}
void calculate_speed_constant(void) {                                               //calculate the speed constant
  speed_constant = (float(cal_number) * 17.6 ) / 3600;                              //divide pulses recieved in one second by this constant to get mph
  pulse_distance = 3600 / (float)cal_number;                                        //calculate the distance of one pulse
  Serial.println("+++++++++++++++++++ Start up ++++++++++++++++++++++++");
  Serial.print("speed_constant =  ");
  Serial.println(speed_constant);
}
void speed_bar(void) {                                                              //speed bar object setup
  lv_bar_set_range(bar_speed, 0, (int)((2 * speed_target) * 10));                   //set max for 2 times target speed
#if serial_debug  
  Serial.printf("line 757 target speed - %d/n", (int)speed_target);
#endif
  lv_obj_set_size(bar_speed, 470, 75);                                              //set size of bar
  lv_obj_align(bar_speed, NULL, LV_ALIGN_CENTER, 0, 90);                            //set location of bar
}
void lv_ex_mbox_1(void) {                                                           //message box with three buttons
  static const char * btns[] = {"Calibrate", "Options", "Close", ""};               //buttons will be created for each name in array
  lv_obj_set_hidden(btn_setup, true);                                               //hide setup button while this box is up
  mbox1 = lv_mbox_create(lv_disp_get_scr_act(NULL), NULL);   //create an object named'mbox1'
  lv_mbox_set_text(mbox1, "Select Function"); //add text to message box
  lv_mbox_add_btns(mbox1, btns);                                                    //add buttons to message box
  lv_obj_set_width(mbox1, 480);                                                     //width of message box
  lv_obj_set_event_cb(mbox1, mbox1_handler_cb);                                     //callback routine to call
  lv_obj_align(mbox1, NULL, LV_ALIGN_IN_BOTTOM_MID, 0, -5);                         /*Align to bottom of screen centered*/
  strcpy(text_buff, "");                                                            //clear buffer used to read keypad
}
static void lv_security_code(void) {                                              //keypad entry for security password

  //create keypad for security code entry
  Serial.println("line 490");
  lv_obj_set_hidden(title_label, false);                                            //show the title bar
  lv_obj_set_hidden(btn_setup, true);                                               //hide the setup button
  lv_obj_set_hidden(label_speed, false);                                            //turn on large text label
  run_screen_flag = 0;                                                              //clear flag so mph value will not update
  lv_obj_set_hidden(label_units, true);
  lv_obj_set_pos(title_label, 100, 5);
  lv_label_set_text(title_label, "Enter Security Code");
  lv_label_set_text(label_speed, "---");                                              //display ---- on initial fireup

  keypad = lv_btnm_create(lv_scr_act(), NULL);
  lv_btnm_set_map(keypad, btnm_map);                                 //create a numeric keypad to enter data
  lv_obj_set_size(keypad, 478, 126);                                  /*Set keypad size*/
  lv_obj_align(keypad, NULL, LV_ALIGN_CENTER, 0, 98);                 //set position of keypad
  lv_obj_set_event_cb(keypad, security_event_handler);                //callback for keypad presses
  strcpy(temp_buff, "");                                              //clear buffer
  Serial.println("line 512");
}
static void security_event_handler(lv_obj_t * obj, lv_event_t event) {            //callback for keypad on password screen
  // char hold[10];                                                                  //temp array to hold the x value
  // char str_temp[] = "";
  if (event == LV_EVENT_VALUE_CHANGED) {
    if (obj != NULL) {
      const char *txt = lv_btnm_get_active_btn_text(obj);                           //get the text value of pressed button
      if (txt != nullptr) {
        strncat(temp_buff, txt, 1);
        lv_label_set_text(label_speed, temp_buff);                                  //display code in mph text area with large text
        if (*txt == 'C') {                                                          //was clear button pressed?
          strcpy(temp_buff, "");                                                    //clear buffer
          lv_label_set_text(label_speed, " ---");                                   //remove numbers and display "-----"
        }

        Serial.println("line 1620");
        if (strlen(temp_buff) >= 6) {
          lv_obj_set_pos(title_label, 80, 5);
          lv_label_set_text(title_label, "Enter Security Code ");                   //refresh text at top of screeen
          lv_label_set_text(label_speed, " ---");                                  //remove numbers and dispay "-----"
          strcpy(temp_buff, "");                                                    //clear the buffer
        }


        if (*txt == 'E') {                                                 //enter key pressed?
          if (String(temp_buff) == "2001E") {                            //backdoor passcode
            screen_run_off();                                            //turn off objects on run screen
            //option_screen_off();
            lv_obj_set_hidden(keypad, true);                             //hide the keypad that is currently being shown
            screen_factory_on();                                         //show the factory screen
          }

          Serial.println("line 1637");
          if (String(temp_buff) == String(user_passcode)) {                            //does string match the security code?
            Serial.println("Successful security code - 676");
            screen_calibrate_off();
            lv_obj_set_pos(title_label, 125, 5);
            lv_label_set_text(title_label, "***  SETUP  ***");            //display text at top of screen
            lv_label_set_text(label_speed, "");
            lv_ex_mbox_1();                                           //show popup with 3 buttons for setup
            lv_obj_set_hidden(option_descrip_text, false);            //show option description text
            lv_obj_set_hidden(calibrate_descrip_text, false);
            lv_obj_set_hidden(close_descrip_text, false);
            //option_screen_on();
            lv_obj_set_hidden(keypad, true);
          }
          else {
            Serial.println("line 686");
            if (String(temp_buff) != "2001E") {                         //check for backdoor passcode
              lv_obj_t * messbox_invalid = lv_mbox_create(lv_disp_get_scr_act(NULL), NULL);   //create a "Invalid security code" message box
              lv_obj_set_width(messbox_invalid, 400);                                         //width of message box
              lv_mbox_set_text(messbox_invalid, "\n     Invalid Security Code!\n");
              lv_mbox_start_auto_close(messbox_invalid, 3000);
              lv_mbox_set_anim_time(messbox_invalid, 1500);
              lv_obj_align(messbox_invalid, NULL, LV_ALIGN_CENTER, -30, 0);
              screen_calibrate_off();                                                        //turn off all items on calibrae screen
              lv_obj_set_hidden(keypad, true);                                               //turn off keypad
              screen_run_on();
            }//goto run screen
          }
        }
      }
    }
  }
}
static void mbox1_handler_cb(lv_obj_t * obj, lv_event_t event) {                  //event handler for message box "Close" "Calibrate" "Option"

  switch (event) {
    case LV_EVENT_PRESSED:

    case LV_EVENT_SHORT_CLICKED:
      Serial.println("Short clicked\n");
      break;

    case LV_EVENT_CLICKED:
      Serial.println("Clicked\n");
      break;

    case LV_EVENT_LONG_PRESSED:
      Serial.println("Long press\n");
      break;

    case LV_EVENT_LONG_PRESSED_REPEAT:
      Serial.println("Long press repeat\n");
      break;

    case LV_EVENT_RELEASED:
      Serial.println("Released\n");
      break;
  }
  if (event == LV_EVENT_VALUE_CHANGED) {
    printf("Button: %s\n", lv_mbox_get_active_btn_text(obj));     //this will display the text of the button that was pressed
    const char * txt = lv_mbox_get_active_btn_text(obj);

    if (strcmp(txt, "Close") == 0) {
      Serial.println("line 1256");
      lv_obj_del(obj);                                         //delete this message box
      lv_obj_set_hidden(option_descrip_text, true);            //hide option description text
      lv_obj_set_hidden(calibrate_descrip_text, true);
      lv_obj_set_hidden(close_descrip_text, true);
      screen_run_on();
    }

    if (strcmp(txt, "Calibrate") == 0) {
      Serial.println("line 1262");
      lv_obj_del(obj);                                         //delete this message box
      lv_obj_set_hidden(option_descrip_text, true);            //hide option description text
      lv_obj_set_hidden(calibrate_descrip_text, true);
      lv_obj_set_hidden(close_descrip_text, true);
      screen_calibrate_on();                                           //goto calibration screen
    }

    if (strcmp(txt, "Options") == 0) {
      Serial.println("line 1268");
      lv_obj_del(obj);                                         //delete this message box
      lv_obj_set_hidden(option_descrip_text, true);            //hide option description text
      lv_obj_set_hidden(calibrate_descrip_text, true);
      lv_obj_set_hidden(close_descrip_text, true);
      option_screen_on();
      Serial.println("line 1272");
    }
  }
  /*Etc.*/
}
void lv_ex_btnm_1(void) {                                                         //numeric keypad 0-9 enter

  static const char * btnm_map[] = {"1", "2", "3", "4", "5", "\n",                  //text for keys on keypad (last field must be a null)
                                    "6", "7", "8", "9", "0", "\n",
                                    "Cancel", "Clear", "Save", ""
                                   };



  lv_obj_t * num_keypad = lv_btnm_create(lv_scr_act(), NULL);                   //create object for keypad
  lv_btnm_set_map(num_keypad, btnm_map);                                        //bring in the text for the keys
  lv_btnm_set_btn_width(num_keypad, 12, 2);                                     /*Make "Save" twice as wide as "Clear"*/
  lv_obj_align(num_keypad, NULL, LV_ALIGN_CENTER, 80, 60);                     //set the position of the keyboard


}
void target_speed_event_handler(lv_obj_t * obj, lv_event_t event) {               //call back from keypad to enter target speed

  if (event == LV_EVENT_VALUE_CHANGED) {
 //   Serial.println("line 1329");
    txt = lv_btnm_get_active_btn_text(obj);                                    //get the text value of pressed button
    if (txt != nullptr) {
      hold_text = hold_text + txt;                                                //add character to buffer
      Serial.println(hold_text);
      Serial.println("line 1333");
      speed_target = hold_text.toFloat() * .1;                                                //convert to a float number

      snprintf(text_buff, 7, "%2.1f", speed_target);
      lv_label_set_text(label_cal, text_buff);
      lv_bar_set_range(bar_speed, 0, (int)((2 * speed_target) * 10));                      //reset max for 2 times target speed

      printf("%2.1f is current string value line 502\n", speed_target);                    //***diagnostic code
 //     Serial.println("line 1341");

      if (*txt == 'C') {                                                    //was clear button pressed?
        hold_text = "";                                                     //clear buffer
        lv_label_set_text(label_cal, "---");                                //remove numbers and dispay "-----"
      }

      if (strlen(temp_buff) >= 6) {
        lv_obj_set_pos(title_label, 40, 5);
        lv_label_set_text(title_label, "Enter Target Speed");               //refresh text at top of screeen
        lv_label_set_text(label_cal, "---");                                //remove numbers and dispay "-----"
        strcpy(temp_buff, "");                                              //clear the buffer
      }
      Serial.println("line 2155");
      if (*txt == 'E') {                                                    //enter key pressed?
        if (speed_target <= 30.0 && speed_target >= 1.0) {                            //
          Serial.println("Successful target speed - line 2158");
          screen_set_multi_target_off();                                   //turn off 5 keys "1 2 3 4 save"
          option_screen_on();
          snprintf(text_buff, 7, "%2.1f", speed_target);
          lv_label_set_text(label_bar_max, text_buff);
          EEPROM.put(speed_target_ee_adr, speed_target);                   //save to eeprom
          EEPROM.commit();

          lv_obj_t * messbox_invalid = lv_mbox_create(lv_disp_get_scr_act(NULL), NULL);           //create a message box that input is invalid
          lv_obj_set_width(messbox_invalid, 400);                                         //width of message box
          lv_mbox_set_text(messbox_invalid, "\nTarget speed set!\n");
          lv_mbox_start_auto_close(messbox_invalid, 2000);                    //display message box for 2 seconds
          lv_mbox_set_anim_time(messbox_invalid, 1500);
          lv_obj_align(messbox_invalid, NULL, LV_ALIGN_CENTER, 0, 0);
          hold_text = "";                                                     //clear buffer
        }
        else {
          Serial.println("line 2174");
          lv_label_set_text(title_label, "");

          lv_obj_t * messbox_invalid = lv_mbox_create(lv_disp_get_scr_act(NULL), NULL);     //create a message box that input is invalid
          lv_obj_set_width(messbox_invalid, 400);                                       //width of message box
          lv_mbox_set_text(messbox_invalid, "Invalid set point, must be between 1.0 and 30.0");
          lv_mbox_start_auto_close(messbox_invalid, 3000);                              //display message box for 2 seconds
          lv_mbox_set_anim_time(messbox_invalid, 1500);
          lv_obj_align(messbox_invalid, NULL, LV_ALIGN_CENTER, 0, 0);
          hold_text = "";                                                               //clear buffer
          lv_label_set_text(label_cal, "-----");

        }

      }
    }

  }
  Serial.println("line 1462");
}
static void btn_set_cal_cb(lv_obj_t * obj, lv_event_t event) {                    //callback for "set with keypad" button in calibration screen


  switch (event) {
    case LV_EVENT_PRESSED:

      Serial.println("line 1465");
      int button_w = 70;                                          //width of plus minus buttons
      int button_h = 50;                                          //height of plus minus buttons
      lv_obj_t * label5P;                                         //create objects for buttons
      lv_obj_t * label5M;
      lv_obj_t * label4P;
      lv_obj_t * label4M;
      lv_obj_t * label3P;
      lv_obj_t * label3M;
      lv_obj_t * label2P;
      lv_obj_t * label2M;
      lv_obj_t * label1P;
      lv_obj_t * label1M;
      lv_obj_t * label_save;                                                //save button
      lv_obj_t * label_abort;                                               //abort button





      //if exit button is pressed
      Serial.println("Btn_set pressed \n");
      lv_obj_set_hidden(btn_exit, true);                              //hide exit button
      lv_obj_set_hidden(btn_set_cal, true);                           //hide the set button
      lv_obj_set_hidden(btn_field_cal, true);                         //hide field cal button

      btn5_plus = lv_btn_create(lv_scr_act(), NULL);                  /*+ button for 1st column*/
      lv_obj_set_pos(btn5_plus, 10, 200);                           /*Set its position*/
      lv_obj_set_size(btn5_plus, button_w, button_h);                          /*Set its size*/
      lv_obj_set_event_cb(btn5_plus, cal_set_cb);                    //callback routine
      label5P = lv_label_create(btn5_plus, NULL);                      /*Add a label to the button*/
      lv_label_set_text(label5P, " +");                             /*Set the labels text*/

      btn5_minus = lv_btn_create(lv_scr_act(), NULL);
      lv_obj_set_pos(btn5_minus, 10, 260);                           /*Set its position*/
      lv_obj_set_size(btn5_minus, button_w, button_h);               /*Set its size*/
      lv_obj_set_event_cb(btn5_minus, cal_set_cb);                   //callback routine
      label5M = lv_label_create(btn5_minus, NULL);                     /*Add a label to the button*/
      lv_label_set_text(label5M, " -");                              /*Set the labels text*/


      btn4_plus = lv_btn_create(lv_scr_act(), NULL);
      lv_obj_set_pos(btn4_plus, 90, 200);                           /*Set its position*/
      lv_obj_set_size(btn4_plus, button_w, button_h);               /*Set its size*/
      lv_obj_set_event_cb(btn4_plus, cal_set_cb);                   //callback routine
      label4P = lv_label_create(btn4_plus, NULL);                     /*Add a label to the button*/
      lv_label_set_text(label4P, " +");                             /*Set the labels text*/

      btn4_minus = lv_btn_create(lv_scr_act(), NULL);
      lv_obj_set_pos(btn4_minus, 90, 260);                           /*Set its position*/
      lv_obj_set_size(btn4_minus, button_w, button_h);               /*Set its size*/
      lv_obj_set_event_cb(btn4_minus, cal_set_cb);                   //callback routine
      label4M = lv_label_create(btn4_minus, NULL);                     /*Add a label to the button*/
      lv_label_set_text(label4M, " -");                              /*Set the labels text*/

      btn3_plus = lv_btn_create(lv_scr_act(), NULL);
      lv_obj_set_pos(btn3_plus, 170, 200);                           /*Set its position*/
      lv_obj_set_size(btn3_plus, button_w, button_h);                /*Set its size*/
      lv_obj_set_event_cb(btn3_plus, cal_set_cb);                    //callback routine
      label3P = lv_label_create(btn3_plus, NULL);                      /*Add a label to the button*/
      lv_label_set_text(label3P, " +");                                 /*Set the labels text*/

      btn3_minus = lv_btn_create(lv_scr_act(), NULL);
      lv_obj_set_pos(btn3_minus, 170, 260);                           /*Set its position*/
      lv_obj_set_size(btn3_minus, button_w, button_h);                /*Set its size*/
      lv_obj_set_event_cb(btn3_minus, cal_set_cb);                     //callback routine
      label3M = lv_label_create(btn3_minus, NULL);                       /*Add a label to the button*/
      lv_label_set_text(label3M, " -");                                 /*Set the labels text*/

      btn2_plus = lv_btn_create(lv_scr_act(), NULL);
      lv_obj_set_pos(btn2_plus, 250, 200);                           /*Set its position*/
      lv_obj_set_size(btn2_plus, button_w, button_h);                /*Set its size*/
      lv_obj_set_event_cb(btn2_plus, cal_set_cb);                    //callback routine
      label2P = lv_label_create(btn2_plus, NULL);                       /*Add a label to the button*/
      lv_label_set_text(label2P, " +");                              /*Set the labels text*/

      btn2_minus = lv_btn_create(lv_scr_act(), NULL);
      lv_obj_set_pos(btn2_minus, 250, 260);                           /*Set its position*/
      lv_obj_set_size(btn2_minus, button_w, button_h);                /*Set its size*/
      lv_obj_set_event_cb(btn2_minus, cal_set_cb);                   //callback routine
      label2M = lv_label_create(btn2_minus, NULL);                    /*Add a label to the button*/
      lv_label_set_text(label2M, " -");                             /*Set the labels text*/

      btn1_plus = lv_btn_create(lv_scr_act(), NULL);
      lv_obj_set_pos(btn1_plus, 330, 200);                           /*Set its position*/
      lv_obj_set_size(btn1_plus, button_w, button_h);                /*Set its size*/
      lv_obj_set_event_cb(btn1_plus, cal_set_cb);                    //callback routine
      label1P = lv_label_create(btn1_plus, NULL);                      /*Add a label to the button*/
      lv_label_set_text(label1P, " +");                              /*Set the labels text*/

      btn1_minus = lv_btn_create(lv_scr_act(), NULL);
      lv_obj_set_pos(btn1_minus, 330, 260);                           /*Set its position*/
      lv_obj_set_size(btn1_minus, button_w, button_h);               /*Set its size*/
      lv_obj_set_event_cb(btn1_minus, cal_set_cb);                   //callback routine
      label1M = lv_label_create(btn1_minus, NULL);                      /*Add a label to the button*/
      lv_label_set_text(label1M, " -");                              /*Set the labels text*/

      btn0_save = lv_btn_create(lv_scr_act(), NULL);
      lv_obj_set_pos(btn0_save, 410, 200);                           /*Set its position*/
      lv_obj_set_size(btn0_save, button_w, button_h);                /*Set its size*/
      lv_obj_set_event_cb(btn0_save, cal_set_cb);                    //callback routine
      label_save = lv_label_create(btn0_save, NULL);                 /*Add a label to the button*/
      lv_label_set_text(label_save, "Save");                         /*Set the labels text*/

      btn0_abort = lv_btn_create(lv_scr_act(), NULL);
      lv_obj_set_pos(btn0_abort, 410, 260);                           /*Set its position*/
      lv_obj_set_size(btn0_abort, button_w, button_h);               /*Set its size*/
      lv_obj_set_event_cb(btn0_abort, cal_set_cb);                   //callback routine
      label_abort = lv_label_create(btn0_abort, NULL);               /*Add a label to the button*/
      lv_label_set_text(label_abort, LV_SYMBOL_CLOSE);               /*Set the labels text*/


      break;


  }
}
static void cal_set_cb(lv_obj_t * btn, lv_event_t event) {                        //calibration keypad callback routine

  switch (event) {
    case LV_EVENT_PRESSED:

      if (btn == btn5_plus) {
        cal_number = cal_number + 10000;                      //increment the 5th column
        if (cal_number > 99999) {
          cal_number = cal_number - 100000;
        }
      }

      if (btn == btn5_minus) {
        if (cal_number > 9999) {
          cal_number = cal_number - 10000;              //deincrement the 5th column
        }
      }

      if (btn == btn4_plus) {
        cal_number = cal_number + 1000;                  //increment the 4th column
      }

      if (btn == btn4_minus) {
        if (cal_number > 999) {
          cal_number = cal_number - 1000;                   //deincrement the 4th column
        }
      }

      if (btn == btn3_plus) {
        cal_number = cal_number + 100;                     //increment the 4th column
      }

      if (btn == btn3_minus) {
        if (cal_number > 99) {
          cal_number = cal_number - 100;                    //deincrement the 4th column
        }
      }

      if (btn == btn2_plus) {
        cal_number = cal_number + 10;                    //increment the 2nd column
      }

      if (btn == btn2_minus) {
        if (cal_number > 9) {
          cal_number = cal_number - 10;                   //deincrement the 2nd column
        }
      }

      if (btn == btn1_plus) {
        cal_number = cal_number + 1;                     //increment the 1st column
      }

      if (btn == btn1_minus) {
        if (cal_number > 1) {
          cal_number = cal_number - 1;                  //deincrement the 1st column
        }
      }

      if (btn == btn0_save) {                              //save new calibration number and check for minumum value of 3000

        lv_obj_del(btn5_plus);                           //buttons to set calibration number
        lv_obj_del(btn5_minus);
        lv_obj_del(btn4_plus);
        lv_obj_del(btn4_minus);
        lv_obj_del(btn3_plus);
        lv_obj_del(btn3_minus);
        lv_obj_del(btn2_plus);
        lv_obj_del(btn2_minus);
        lv_obj_del(btn1_plus);
        lv_obj_del(btn1_minus);
        lv_obj_del(btn0_save);
        lv_obj_del(btn0_abort);
        lv_label_set_text(label_cal, "");
        Serial.println(old_cal_number);
        Serial.println(cal_number);
        if (cal_number <= 2999) {                                               //do not allow number less than 3000
          messbox_start_run = lv_mbox_create(lv_disp_get_scr_act(NULL), NULL);   //create a message box
          lv_obj_set_width(messbox_start_run, 350);                   //width of message box
          lv_mbox_set_text(messbox_start_run, "\n Invalid Entry\n\n Calibration number must be at least 3000.  \n");
          lv_mbox_start_auto_close(messbox_start_run, 4000);
          lv_mbox_set_anim_time(messbox_start_run, 1500);
          lv_obj_align(messbox_start_run, NULL, LV_ALIGN_CENTER, 0, 0);
          cal_number = old_cal_number;                                     //reset to original number

        }

        if (old_cal_number != cal_number)                                    //new cal number?
        { EEPROM.put(0x05, cal_number);                                  //save calibrtion value to eeprom if changed
          EEPROM.commit();
          calculate_speed_constant();                                    //calculate new speed constant
        }

        screen_calibrate_on();                                              //return to calibration screen
      }
      if (btn == btn0_abort) {                                                //save new calibration screen
        lv_obj_del(btn5_plus);                                               //buttons to set calibration number
        lv_obj_del(btn5_minus);
        lv_obj_del(btn4_plus);
        lv_obj_del(btn4_minus);
        lv_obj_del(btn3_plus);
        lv_obj_del(btn3_minus);
        lv_obj_del(btn2_plus);
        lv_obj_del(btn2_minus);
        lv_obj_del(btn1_plus);
        lv_obj_del(btn1_minus);
        lv_obj_del(btn0_save);
        lv_obj_del(btn0_abort);
        lv_label_set_text(label_cal, "");
        EEPROM.get(0x05, cal_number);                                      //get old calibrtion value from eeprom
        screen_calibrate_on();                                              //return to calibration screen
      }

      snprintf(text_buff, 6, "%05d", cal_number);
      lv_label_set_text(label_cal, text_buff);                                //display text
  }
}
static void btn_target_exit_cb(lv_obj_t * btn, lv_event_t event) {
  if (event == LV_EVENT_CLICKED) {
    screen_target_off();
    option_screen_on();
    Serial.println("line 1662 btn_target_exit_cb");
  }
}
// static void cal_btn_exit_cb(lv_obj_t * obj, lv_event_t event) {                   //exit the calibration screen

//   switch (event) {
//     case LV_EVENT_PRESSED:                                                    //if exit button is pressed
//       screen_run_on();                                                      //go back to run screen
//       break;
//     case LV_EVENT_SHORT_CLICKED:
//       //Serial.println("Short clicked\n");
//       break;

//     case LV_EVENT_CLICKED:
//       // Serial.println("Clicked\n");
//       break;

//     case LV_EVENT_LONG_PRESSED:
//       //  Serial.println("Long press\n");
//       break;

//     case LV_EVENT_LONG_PRESSED_REPEAT:
//       //   Serial.println("Long press repeat\n");
//       break;

//     case LV_EVENT_RELEASED:
//       //  Serial.println("Released\n");
//       break;

//   }


//  /*Etc.*/
// }
static void my_start_btn_cb(lv_obj_t * obj, lv_event_t event) {                   //call back for setup button in upper left corner of screen

  switch (event) {
    case LV_EVENT_PRESSED:
   //   Serial.println("Event- my_start_btn_cb - line 983");
      //lv_obj_set_hidden(btn_setup,true);
      //   lv_obj_set_hidden(title_label_scr_run,true);       //hide the run screen lable
      if (security == 1) {
        screen_run_off();                           //turn off run screen objects
        lv_security_code();                        //call password routine
      }
      else {
        lv_ex_mbox_1();                                 //show 3 button popup calibrate,options,close
      }
      break;

    case LV_EVENT_RELEASED:
      //       Serial.println("Released\n");
      break;

  }


  /*Etc.*/
}
static void unit_event_cb(lv_obj_t * obj, lv_event_t event) {                     //cb for checkboxes on option screen
  switch (event) {
    case LV_EVENT_VALUE_CHANGED:
  //    Serial.println("unit_event_cb - line 1148");
      if (obj == cb_kph) {                                          //code to toggle between mph and kph
        lv_cb_set_checked(cb_mph, false);
        lv_cb_set_checked(cb_kph, true);
        units = 2;
        snprintf(text_buff, 7, "%2.1f", (speed_target * 2) * 1.609344 ); //  double and convert to metric
        lv_label_set_text(label_bar_max, text_buff);
      }
      if (obj == cb_mph) {

        lv_cb_set_checked(cb_kph, false);
        lv_cb_set_checked(cb_mph, true);
        units = 1;
        snprintf(text_buff, 7, "%2.1f", (speed_target * 2));
        lv_label_set_text(label_bar_max, text_buff);        //set to mph version in case it is being changed from metric
      }


      if (units == 1) {
        lv_cb_set_text(cb_feet_per_min, "feet/minute");     //if mph mode set for feet/min
      }
      else {
        lv_cb_set_text(cb_feet_per_min, "meters/minute");  //else metric set for meters/min
      }

      if (obj == cb_distance) {

        if (lv_cb_is_checked(cb_distance  )) {
          dis = 1;
        }
        else {
          dis = 0;
        }

      }

      if (obj == cb_security) {                              //if password checkbox was changed
        if (lv_cb_is_checked(cb_security) == true) {
          security = 1;
        }
        else {
          security = 0;
        }
      }

     if (obj == cb_speed_avg){
      if (lv_cb_is_checked(cb_speed_avg) == true) {          //speed averaging checkbox
          speed_avg = 1;
        }
        else {
          speed_avg = 0;
        }
      
     }
      
      if (obj == cb_graph) {                              //if password bar graph was changed
        if (lv_cb_is_checked(cb_graph) == true) {
          graph = 1;
          lv_obj_set_hidden(btn_target_set, false);
          lv_obj_set_hidden(btn_preAlarm_set, false);
        //  lv_obj_set_hidden(label_target_speed, false);
          lv_obj_set_hidden(cb_alarm, false);
        }
        else {
          graph = 0;
          lv_obj_set_hidden(btn_target_set, true);
          lv_obj_set_hidden(btn_preAlarm_set, false);
        //  lv_obj_set_hidden(label_target_speed, true);
          lv_obj_set_hidden(cb_alarm, true);
        }
      }

      if (obj == cb_alarm) {
        if (lv_cb_is_checked(cb_alarm) == true) {
          alarm_enable = 1;
          Serial.println("Alarm on");
        }
        else {
          alarm_enable = 0;
          Serial.println("Alarm off");
        }
      }

      if (obj == cb_radar){                               //if radar input is checked turn off gps checkbox
          lv_cb_set_checked(cb_gps, false);
          lv_cb_set_checked(cb_radar, true);
          speed_input = 0;                                //set speed_input flag to 0 wheel pulse 
      }
      
      if (obj == cb_gps){                                  //if gps input is selected turn off radar checkbox
          lv_cb_set_checked(cb_gps, true);
          lv_cb_set_checked(cb_radar, false);
          speed_input = 1;                                  //set speed input flag to 1 for gps
      }

      if (obj == cb_feet_per_min) {                              //if password feet/per/minute was changed
        if (lv_cb_is_checked(cb_feet_per_min) == true) {
          fpm = 1;
        }
        else {
          fpm = 0;
        }
      }
      break;
    case LV_EVENT_PRESSED:
   //   Serial.println("line 2371");
      if (obj == btn_target_set) {                                            //if set target speed button is pressed
        screen_target_on();                                               //open up screen with keypad  to set target speed
        Serial.println("Set target button pressed");
      }
      
      if (obj == btn_preAlarm_set) {                                            //if set target speed button is pressed
        screen_target_on();                                               //open up screen with keypad  to set target speed
        Serial.println("PreAlarm button pressed");
      }

      
      if (obj == btn_set_exit) {
        Serial.printf("security =%d,graph =%d,distance =%d,speed_input =%d**************\n", security, graph, dis,speed_input);
       if (speed_input == 0){
          enable_radar();                                                     //turn off serial2 and turn on pulse interrupt
          }
       else{
          enable_gps();                                                     //turn on serial2 and turn off pulse interrupt                                              
    }
        
        if (obj != NULL) {
          EEPROM.put(unit_ee_adr, units);                                         ///save units ( 2 kph or 1 mph)
          EEPROM.put(10, security);                                                   //save setting to eeprom
          EEPROM.put(11, graph);
          EEPROM.put(12, dis);
          EEPROM.put(fpm_ee_adr, fpm);
          EEPROM.put(alarm_ee_adr, alarm_enable);
          EEPROM.put(speed_avg_ee_adr,speed_avg);                              //save stautus of speed average checkbox
          EEPROM.put(speed_input_ee_adr,speed_input);                          //save checkbox status of speed input
          EEPROM.commit();                                                     //write values to eeprom
          option_screen_off();                                                 //turn off all objects of option screen
          pulse = 0;                                                             //reset the pulse counter
          old_millis = millis();                                                //reset the timer
          screen_run_on();                                                        //return to run screen
        }

      }

      break;
  }
  Serial.println("line 2480 leaving unit_event_cb()");
}
/*************************
    TouchPad read routine
 ************************/
bool my_input_read(lv_indev_drv_t * drv, lv_indev_data_t*data) {                 //read touch pad function
  uint16_t t_x = 0, t_y = 0;
  static int16_t last_x = 0;
  static int16_t last_y = 0;
  boolean pressed = tft.getTouch(&t_x, &t_y);                                  //read touch screen, set 'pressed' to true if pressed

  char buffer1[20];
  if (pressed == true) {                                                        //***diagnostic***
    sprintf(buffer1, "x= %i :  y= %i", t_x, t_y);                             //convert x-y coridnates to a string to send to serial monitor
    Serial.println(buffer1);                                                  //send xy value to serial monitor
    data->point.x = t_x;                                                      //save x/y coridnates
    data->point.y = t_y;
    last_x = data->point.x;
    last_y = data->point.y;
    data->state = LV_INDEV_STATE_PR;
  }
  else {
    data->point.x = last_x;
    data->point.y = last_y;
    data->state = LV_INDEV_STATE_REL;
  }
  return false;
}

//==================================
/* Display flushing */
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {

  uint16_t c;

  tft.startWrite();           /* Start new TFT transaction */
  tft.setAddrWindow(area->x1, area->y1, (area->x2 - area->x1 + 1), (area->y2 - area->y1 + 1)); /* set the working window */
  for (int y = area->y1; y <= area->y2; y++) {
    for (int x = area->x1; x <= area->x2; x++) {
      c = color_p->full;
      tft.writeColor(c, 1);
      color_p++;
    }
  }
  tft.endWrite();             /* terminate TFT transaction */
  lv_disp_flush_ready(disp); /* tell lvgl that flushing is done */
}

/* Interrupt driven periodic handler */
static void lv_tick_handler(void)
{
  lv_tick_inc(LVGL_TICK_PERIOD);
}

void touch_calibrate()
{
  uint16_t calData[5];
  uint8_t calDataOK = 0;

  // check file system exists
  if (!SPIFFS.begin()) {
    Serial.println("Formating file system");
    SPIFFS.format();
    SPIFFS.begin();
  }

  // check if calibration file exists and size is correct
  if (SPIFFS.exists(CALIBRATION_FILE)) {
    if (REPEAT_CAL)                                                            //if set to true
    {
      // Must Delete calibrateion file if we want to re-calibrate
      SPIFFS.remove(CALIBRATION_FILE);                                            //delete the calibration file
    }
    else
    {
      File f = SPIFFS.open(CALIBRATION_FILE, "r");
      if (f) {
        if (f.readBytes((char *)calData, 14) == 14)
          calDataOK = 1;
        f.close();
      }
    }
  }

  if (calDataOK && !REPEAT_CAL) {
    // calibration data valid
    tft.setTouch(calData);                                                    //enter the calibration numbers
  } else {                                                                    // else run the calibration routine
    // data not valid so recalibrate
    tft.fillScreen(TFT_BLACK);                                                  //set screen to black
    tft.setCursor(20, 0);                                                       //set cursor positon
    tft.setTextFont(2);                                                         //select font
    tft.setTextSize(1);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);

    tft.println("Touch corners as indicated");                                  //user identifies the 4 corners

    tft.setTextFont(1);                                                          //change font
    tft.println();

    if (var_REPEAT_CAL) {
      tft.setTextColor(TFT_RED, TFT_BLACK);
      tft.println("Set REPEAT_CAL to false to stop this running again!");
      var_REPEAT_CAL = REPEAT_CAL;                                          //reset flag so calibration will not run again
    }

    tft.calibrateTouch(calData, TFT_MAGENTA, TFT_BLACK, 15);

    tft.setTextColor(TFT_GREEN, TFT_BLACK);                                    //green text with black background
    tft.println("Calibration complete!");                                      //print to screen  the calibration process is complete

    // store data
    File f = SPIFFS.open(CALIBRATION_FILE, "w");                                //open file and save data
    if (f) {                                                                   //if file opened, save data
      f.write((const unsigned char *)calData, 14);                             //save the calibration data to spiffs
      f.close();                                                               //close the file
    var_REPEAT_CAL = 55;                                                       //set to 55 to indicate touch routine has been performed
    EEPROM.put(screen_cal_ee_adr,var_REPEAT_CAL);
    EEPROM.commit();                                                           //commit to eeprom
    }
  }
}

////--------------------------------------
//void lv_tutorial_objects(void){
//
//
//    /********************
//     * CREATE A SCREEN
//     *******************/
//    /* Create a new screen and load it
//     * Screen can be created from any type object type
//     * Now a Page is used which is an objects with scrollable content*/
//     lv_obj_t * scr1 = lv_obj_create(NULL, NULL);                      //create a new screen and name it 'scr1'
//     lv_disp_load_scr(scr1);                                            //make 'scr1' the active screen
//

//
//    /***********************
//     * ADD A DROP DOWN LIST
//     ************************/
//    lv_obj_t * ddlist = lv_ddlist_create(scr1, NULL);                     /*Create a drop down list*/
//    lv_obj_align(ddlist, slider, LV_ALIGN_OUT_RIGHT_TOP, 50, 0);         /*Align next to the slider*/
//    lv_obj_set_top(ddlist, true);                                        /*Enable to be on the top when clicked*/
//    lv_ddlist_set_fix_width(ddlist, 75);                                 /*set the width*/
//    lv_ddlist_set_draw_arrow(ddlist, true);                               /*add drop down arrow*/
//    lv_ddlist_set_options(ddlist, "None\nLittle\nHalf\nA lot\nAll");     /*Set the options*/
//    lv_obj_set_event_cb(ddlist, ddlist_event_cb);                        /*Set function to call on new option is chosen*/
//   // lv_ddlist_set_action(ddlist, ddlist_event_cb);
//
//
//    /****************
//     * CREATE A CHART
//     ****************/
//    lv_obj_t * chart = lv_chart_create(scr1, NULL);                         /*Create the chart*/
//    lv_obj_set_size(chart, lv_obj_get_width(scr1) / 2, lv_obj_get_width(scr1) / 4);   /*Set the size*/
//    lv_obj_align(chart, slider, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 20);                   /*Align below the slider*/
//    lv_chart_set_series_width(chart, 3);                                            /*Set the line width*/
//
//    /*Add a RED data series and set some points*/
//    lv_chart_series_t * dl1 = lv_chart_add_series(chart, LV_COLOR_RED);
//    lv_chart_set_next(chart, dl1, 10);
//    lv_chart_set_next(chart, dl1, 25);
//    lv_chart_set_next(chart, dl1, 45);
//    lv_chart_set_next(chart, dl1, 60);
//
//    /*Add a BLUE data series and set some points*/
//    lv_chart_series_t * dl2 = lv_chart_add_series(chart, lv_color_make(0x40, 0x70, 0xC0));
//    lv_chart_set_next(chart, dl2, 10);
//    lv_chart_set_next(chart, dl2, 25);
//    lv_chart_set_next(chart, dl2, 45);
//    lv_chart_set_next(chart, dl2, 90);
//    lv_chart_set_next(chart, dl2, 75);
//    lv_chart_set_next(chart, dl2, 505);
//
//}

/**********************
     STATIC FUNCTIONS
 **********************/




//void lv_ex_kb_1(void){                        //this is full keyboard
//
//    /*Create styles for the keyboard*/
//    static lv_style_t rel_style, pr_style;
//
//    lv_style_copy(&rel_style, &lv_style_btn_rel);
//    rel_style.body.radius = 0;
//    rel_style.body.border.width = 1;
//
//    lv_style_copy(&pr_style, &lv_style_btn_pr);
//    pr_style.body.radius = 0;
//    pr_style.body.border.width = 1;
//
//    /*Create a keyboard and apply the styles*/
//    lv_obj_t *kb = lv_kb_create(lv_scr_act(), NULL);
//    lv_kb_set_mode(kb, LV_KB_MODE_NUM);                                        //LV_KB_MODE_TEXT or LV_KB_MODE_NUM / display letters, number, and special characters
//    lv_kb_set_cursor_manage(kb, true);
//    lv_kb_set_style(kb, LV_KB_STYLE_BG, &lv_style_transp_tight);
//    lv_kb_set_style(kb, LV_KB_STYLE_BTN_REL, &rel_style);
//    lv_kb_set_style(kb, LV_KB_STYLE_BTN_PR, &pr_style);
//
//    /*Create a text area. The keyboard will write here*/
//    lv_obj_t *ta = lv_ta_create(lv_scr_act(), NULL);
//    lv_obj_align(ta, NULL, LV_ALIGN_IN_TOP_MID, 0, 10);
//    lv_ta_set_text(ta, "");
//
//    /*Assign the text area to the keyboard*/
//    lv_kb_set_ta(kb, ta);
//}
