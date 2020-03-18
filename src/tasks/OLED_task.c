#include "sys.h"
#include "adc.h"
#include "oled.h"
#include "OLED_task.h"
#include "stdlib.h"
#include "string.h"

#include "imu.h"

uint8_t OLED_Button(void);

//-----------------------Button_Data------------------------//
static uint8_t Button_Val=0;
static uint8_t Button_Val_last=0;
static bool	Button_Change=0;
static portTickType KeyTime=0;
//-----------------------Menu_Data------------------------//
static Menu_option root_menu={"root",NULL,NULL,NULL};
static Menu_option *current_menu=NULL;

static int8_t cursor_pos=0;
static uint8_t cursor_pos_max=0;
//-----------------------APP_Data------------------------//
static uint8_t APP_ID=0;
static uint8_t APP_ID_last=0;
static bool goto_submenu=false;
static bool goto_parentmenu=false;
static bool option_operat=false;


/**
 * @brief OLED进程函数
 * 
 */
void OLED_Prc(void){
	
	OLED_Button();
	
	switch(APP_ID)
	{
		//菜单
		case MENU_APPID:{
			menu_switch();
			menu_display();
			if(Button_Change)
			{
				oled_refresh_gram();
				oled_clear(Pen_Clear);
			}
		}break;
		//logo
		case LOGO_APPID:{
			if(APP_ID!=APP_ID_last){
				oled_LOGO();
				oled_refresh_gram();
				oled_clear(Pen_Clear);
			}
			if(Button_Change&&Button_Val==5)APP_ID=0;
		}break;
		//time
		case TIME_APPID:{
			show_time();
			oled_refresh_gram();
			oled_clear(Pen_Clear);
			if(Button_Change&&Button_Val==5)APP_ID=0;
		}break;
		
		case TASK_TIME_APPID:{
			if(Button_Change){
			show_task_runtime();
			oled_refresh_gram();
			oled_clear(Pen_Clear);
			}
			if(Button_Change&&Button_Val==5)APP_ID=0;
		}break;
		
		case MEMS_APPID:{
			show_MEMS();
			oled_refresh_gram();
			oled_clear(Pen_Clear);
			if(Button_Change&&Button_Val==5)APP_ID=0;
		}break;
	}
	
	if(APP_ID!=APP_ID_last){
	}
	
	
}
/**
 * @brief 按键处理函数
 * 
 * @return uint8_t 按键键值
 */
uint8_t OLED_Button(void)
{
	static uint8_t _Button_Val=0;
	static uint8_t _Button_Val_last=0;
	static bool	_Button_Change=0;
	static portTickType currentTime;
	static portTickType KeyChangeTime;
	uint16_t ad1;
	
	_Button_Change=0;
	_Button_Val_last=_Button_Val;
	Button_Change=0;
	Button_Val_last=Button_Val;
	
	ad1 = get_ADC(ADC_Channel_6);
	if(ad1>3200)//无按键按下
	{
//		printf("NONE\n");按键测试所用
		_Button_Val = 0;
		KeyTime=0;
	}
	else if(ad1>800&&ad1<1000)//左按键按下
	{
//		printf("LEFT\n");
		_Button_Val = 1;
	}
	else if(ad1>1600&&ad1<1800)//右按键按下
	{
//		printf("RIGHT\n");
		_Button_Val = 2;
	}
	else if(ad1>2800&&ad1<3100)//下按键按下
	{
//		printf("DOWN\n");
		_Button_Val = 3;
	}
	else if(ad1>2200&&ad1<2400)//上按键按下
	{
//		printf("UP\n");
		_Button_Val = 4;
	}
	else if(ad1<10)//按键按下
	{
//		printf("HIGH\n");
		_Button_Val = 5;
	}
	
	if(_Button_Val!=_Button_Val_last)
	{
		KeyChangeTime=xTaskGetTickCount();
		_Button_Change=1;
	}
	KeyTime=xTaskGetTickCount()-KeyChangeTime;
	if(KeyTime>50)Button_Val=_Button_Val;
	
	if(Button_Val!=Button_Val_last)
	{
		KeyChangeTime=xTaskGetTickCount();
		Button_Change=1;
	}
	
	return Button_Val;
}

//-----------------------Menu_Functions------------------------//
/**
 * @brief 用于创建菜单（所有菜单从根菜单开始创建，第一个菜单为根菜单的子菜单）
 * 
 */
void menu_init(void){
	current_menu=&root_menu;
	
	Menu_option *_logo=submenu_creat("logo",&root_menu,&show_logo_hook);
	Menu_option *_status=menu_option_creat("status",_logo,NULL);
		Menu_option *_time=submenu_creat("sys runtime",_status,&show_time_hook);
		Menu_option *_tasktime=menu_option_creat("task runtime",_time,&show_task_runtime_hook);
		Menu_option *_MEMS=menu_option_creat("MEMS data",_tasktime,show_MEMS_hook);
	Menu_option *_options=menu_option_creat("options",_status,NULL);
	Menu_option *_test=menu_option_creat("test",_options,NULL);
	Menu_option *kk=menu_option_creat("kk",_test,NULL);
	Menu_option *_print=menu_option_creat("print",kk,NULL);
}
/**
 * @brief 用于创建新的菜单选项，创建的选项将在原选项的后面
 * 
 * @param title 选项标题
 * @param previous_option 上一个选项 
 * @param hook 钩子函数，按键按下时执行
 * @return Menu_option* 菜单选项的指针
 */
Menu_option *menu_option_creat(char title[TITLE_LEN],Menu_option *previous_option,void (*hook)(void))
{
	Menu_option *p=NULL;
	p=(Menu_option*)malloc(sizeof(Menu_option));
	
	memcpy(&p->title,title,sizeof(char)*VAL_MIN(strlen(title),TITLE_LEN));
	p->hook=hook;
	p->seq=0;
	
	p->parentmenu=previous_option->parentmenu;
	
	p->submenu=NULL;
	
	p->nextoption=previous_option->nextoption;
	previous_option->nextoption=p;
	return p;
}
/**
 * @brief 用于创建新的子菜单
 * 
 * @param title 子菜单选项的标题
 * @param parentmenu 父级菜单选项
 * @param hook 钩子函数，按键按下时执行
 * @return Menu_option* 菜单选项的指针
 */
Menu_option *submenu_creat(char title[TITLE_LEN],Menu_option *parentmenu,void (*hook)(void))
{
	Menu_option *p=NULL;
	p=(Menu_option*)malloc(sizeof(Menu_option));
	
	memcpy(&p->title,title,sizeof(char)*VAL_MIN(strlen(title),TITLE_LEN));
	p->hook=hook;
	p->seq=0;
	
	p->submenu=NULL;
	
	p->nextoption=NULL;
	
	p->parentmenu=parentmenu;

	parentmenu->submenu=p;
	return p;
}
/**
 * @brief 菜单操作函数
 * 
 */
void menu_switch(void)
{
//	oled_showchar((cursor_pos>4?4:cursor_pos),0,' '); //消除同级菜单指针拖影效果
	if(Button_Change){
		switch (Button_Val)
		{
			case 0:;break; 
			case 1:goto_parentmenu=true;break; 									////////                    
			case 2:goto_submenu=true;break; 									////////
			case 3:cursor_pos++;break;		//下
			case 4:cursor_pos--;break;		//上
			case 5:option_operat=true;break;
			default:break;
		}
		VAL_LIMIT(cursor_pos,0,cursor_pos_max);
	}
}
/**
 * @brief 菜单的显示
 * 
 */
void menu_display(void)
{
	int n=0;
	Menu_option *p=NULL;
	p=current_menu->submenu;
	for(n=0;p!=NULL;n++){
		if(cursor_pos==n){
			if(p->hook!=NULL) oled_showchar(cursor_pos>4?4:cursor_pos,0,95+' ');
			else oled_showchar(cursor_pos>4?4:cursor_pos,0,'>');
			
			if(goto_submenu==true&&p->submenu!=NULL){
				p->seq=n;
				cursor_pos=0;
				current_menu=p;
				goto_submenu=false;
			}
			if(goto_parentmenu==true&&p->parentmenu->parentmenu!=NULL){
				current_menu=p->parentmenu->parentmenu;
//				oled_showchar((cursor_pos>4?4:cursor_pos),0,' ');
				cursor_pos=p->parentmenu->seq;
				goto_parentmenu=false;
			}
			if(option_operat==true&&p->hook!=NULL){
				p->hook();
				option_operat=false;
			}
		}
		if(cursor_pos>4&&(n-cursor_pos+4)>=0){
			if(p->submenu!=NULL)oled_showstring(n-cursor_pos+4,19,(uint8_t*)"->");
			oled_showstring(n-cursor_pos+4,2,(uint8_t*)p->title);
		}
		else if(cursor_pos<=4){
			if(p->submenu!=NULL)oled_showstring(n,19,(uint8_t*)"->");
			oled_showstring(n,2,(uint8_t*)p->title);
		}
		p=p->nextoption;
	}
	cursor_pos_max=n-1;
	goto_submenu=false;
	goto_parentmenu=false;
	option_operat=false;
}

//-----------------------Hook_Fuctions,APP_Fuctions------------------------//
void show_logo_hook(void)
{
	APP_ID=LOGO_APPID;
}

void show_time_hook(void)
{
	APP_ID=TIME_APPID;
}
void show_time(void)
{
	uint8_t x=2;
	uint8_t y=3;
	portTickType time=xTaskGetTickCount();
	uint32_t second=(uint32_t)(time/1000);
	uint16_t minute=(uint16_t)(second/60);
	uint16_t hours=(uint16_t)(minute/60);
	oled_shownum(x,y+10,time%1000,1,3);
	oled_shownum(x,y+7,second%60,1,2);
	oled_shownum(x,y+4,minute%60,1,2);
	oled_shownum(x,y,hours%60,0,3);
	oled_showchar(x,y+3,':');
	oled_showchar(x,y+6,':');
	oled_showchar(x,y+9,'.');
}

#define TASK_NUM 10
static char name[TASK_NUM][20]={0};
static unsigned int percent[TASK_NUM]={0};
void show_task_runtime_hook(void){
	char RunTimeInfo[400]={0};
	char *pcBuffer=0;
	vTaskGetRunTimeStats(RunTimeInfo);
	pcBuffer=RunTimeInfo;
	for(uint8_t i=0;i<TASK_NUM;i++){
		sscanf(pcBuffer,"%s",name[i]);
		pcBuffer=strchr(pcBuffer,'\t')+1;
		pcBuffer=strchr(pcBuffer,'\t')+2;
		if(*(pcBuffer)=='<'){
			percent[i]=0;
		}
		else sscanf(pcBuffer,"%u",percent+i);
		pcBuffer=strchr(pcBuffer,'\n')+1;
	}
	//冒泡排序
	for(uint8_t i=0;i<TASK_NUM;i++){
		char name_temp[20];
		unsigned int percent_temp;
		for(uint8_t j=i+1;j<TASK_NUM;j++){
			if(percent[i]<percent[j]){
				percent_temp=percent[j];
				percent[j]=percent[i];
				percent[i]=percent_temp;
				strcpy(name_temp,name[j]);
				strcpy(name[j],name[i]);
				strcpy(name[i],name_temp);
			}
		}
	}
	APP_ID=TASK_TIME_APPID;
}
	
void show_task_runtime(){
	static int scroll=0;
	for(uint8_t i=scroll,j=0;i<TASK_NUM;i++,j++){
		oled_showstring(j,0,(uint8_t *)name[i]);
		oled_shownum(j,16,percent[i],0,2);
		oled_showchar(j,18,'%');
	}
	if(Button_Change){
		switch (Button_Val)
		{
			case 0:;break; 
			case 1:show_task_runtime_hook();break; 									////////                    
			case 2:show_task_runtime_hook();break; 									////////
			case 3:scroll++;break;		//下
			case 4:scroll--;break;		//上
			case 5:break;
			default:break;
		}
		VAL_LIMIT(scroll,0,TASK_NUM-5);
	}
}

void show_MEMS_hook(void){
	APP_ID=MEMS_APPID;
};

void show_MEMS(void){
	uint8_t x=2;
	uint8_t y=3;
	oled_shownum(x,y,(uint32_t)(imu_yaw_angle/420.0*90.0),0,5);
	oled_shownum(x+1,y,(uint32_t)imu_temp,0,3);
};

void show_motor_state_hook(void* param)
{
	APP_ID=MOTOR_APPID;
}