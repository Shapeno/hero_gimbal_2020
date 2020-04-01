#ifndef _OLED_TASK_H_
#define _OLED_TASK_H_

#include <stdint.h>

#define MENU_APPID 	0
#define LOGO_APPID 	1
#define TIME_APPID 	2
#define TASK_TIME_APPID 3
#define MEMS_APPID  4
#define MOTOR_APPID 5

#define TITLE_LEN 14
typedef struct _Menu_option
{
	char title[TITLE_LEN+1];
	struct _Menu_option *nextoption;
	struct _Menu_option *submenu;
	struct _Menu_option *parentmenu;
	void (*hook)(void);
	uint8_t seq;
}Menu_option;

void OLED_Prc(void);
uint8_t OLED_Button(void);

void menu_init(void);
Menu_option *menu_option_creat(char title[TITLE_LEN],Menu_option *previous_option,void (*hook)(void));
Menu_option *submenu_creat(char title[TITLE_LEN],Menu_option *parentmenu,void (*hook)(void));
void menu_switch(void);
void menu_display(void);
void show_logo_hook(void);
void show_time_hook(void);
void show_task_runtime_hook(void);
void show_task_runtime(void);
void show_MEMS_hook(void);
void show_MEMS(void);
void show_time(void);
#endif
