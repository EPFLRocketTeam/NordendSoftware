
#include "ui.h"
#include <ncurses.h>
#include <unistd.h>
#include <string.h>
#include "queue.h"
#include <time.h>
#include "serial.h"
#include "sync.h"
#include "../../../CM4/ert/od/data_types.h"


typedef struct ui_data {
	int max_x;
	int max_y;

	int ec_x;
	int ec_y;
	int ec_w;
	int ec_h;

	int rc_x;
	int rc_y;
	int rc_w;
	int rc_h;

	int so_x;
	int so_y;
	int so_w;
	int so_h;

	int fb_x;
	int fb_y;
	int fb_w;
	int fb_h;

	WINDOW * win_ec; //engine control
	WINDOW * win_rc; //recovery cotrol
	WINDOW * win_so; //sensors overview
	WINDOW * win_fb; //feedback



	//other data
	sync_store_t sync_data;

    serial_dev_t cmd_device;
    comunicator_t com;


}ui_data_t;



int ui_init(int argc, char ** argv, ui_data_t * data);
int ui_mainloop(ui_data_t * data);
int ui_make_windows(ui_data_t * data);
int ui_update_content(ui_data_t * data);
int ui_handle_input(ui_data_t * data);


int ui_start(int argc, char ** argv, ui_param_t * param){

	ui_data_t data = {0};

	int nogui = 0;

	if (argc == 2) {
		if(strcmp(argv[1], "nogui") == 0) {
			nogui = 1;
		}
	}

	if(!nogui) {
		//UI initialize
		ui_init(argc, argv, &data);
	}



	//UI mainloop
	for(;;) {

		if(!nogui) {
			sync_copy_data(&data.sync_data);

			ui_update_content(&data);

			ui_handle_input(&data);
		} else {
			sleep(1);
		}



		//


	}



}

void cmd_handle_data(uint8_t opcode, uint16_t len, uint8_t * _data) {
	//do nothing
}

int ui_init(int argc, char ** argv, ui_data_t * data) {



    serial_setup(&data->cmd_device, "/dev/ttyRPMSG3");


    comunicator_init(&data->com, &data->cmd_device, cmd_handle_data);

	initscr();
	noecho();
	curs_set(FALSE);

	ui_make_windows(data);


}

int ui_make_ec(ui_data_t * data){
	data->win_ec = newwin(data->ec_h, data->ec_w, data->ec_y, data->ec_x);
	box(data->win_ec, 0, 0);
	wmove(data->win_ec, 0, 1);
	waddstr(data->win_ec, "Engine Control");


	wrefresh(data->win_ec);
}

int ui_make_rc(ui_data_t * data){
	data->win_rc = newwin(data->rc_h, data->rc_w, data->rc_y, data->rc_x);
	box(data->win_rc, 0, 0);
	wmove(data->win_rc, 0, 1);
	waddstr(data->win_rc, "Recovery Control");


	wrefresh(data->win_rc);
}


int ui_make_so(ui_data_t * data){
	data->win_so = newwin(data->so_h, data->so_w, data->so_y, data->so_x);
	box(data->win_so, 0, 0);
	wmove(data->win_so, 0, 1);
	waddstr(data->win_so, "Sensors Overview");


	wrefresh(data->win_so);
}

int ui_make_fb(ui_data_t * data){
	data->win_fb = newwin(data->fb_h, data->fb_w, data->fb_y, data->fb_x);
	box(data->win_fb, 0, 0);
	wmove(data->win_fb, 0, 1);
	waddstr(data->win_fb, "Debug Feedback");

	wrefresh(data->win_fb);
}





int ui_make_windows(ui_data_t * data) {
	//window size ratios

	//horizontal ratio
	int left = 1;
	int right = 1;

	//vertical ratio
	int upper = 1;
	int middle = 1;
	int lower = 2;

	getmaxyx(stdscr, data->max_y, data->max_x);

	//horizontal calculation
	int left_start = 0;
	data->ec_x = left_start;
	data->rc_x = left_start;
	data->so_x = left_start;

	int right_start = (data->max_x * left) / (right + left);
	data->fb_x = right_start;
	data->fb_w = data->max_x - right_start;

	data->ec_w = right_start;
	data->rc_w = right_start;
	data->so_w = right_start;

	//vertical calculation
	int upper_start = 0;
	data->ec_y = upper_start;
	data->fb_y = upper_start;

	int middle_start = data->max_y * upper / (upper + middle + lower);
	data->rc_y = middle_start;

	data->ec_h = middle_start;

	int lower_start = data->max_y * (upper + middle) / (upper + middle + lower);
	data->so_y = lower_start;

	data->rc_h = lower_start - middle_start;

	data->so_h = data->max_y - lower_start;

	data->fb_h = data->max_y;


	//draw windows

	ui_make_ec(data);
	ui_make_rc(data);
	ui_make_so(data);
	ui_make_fb(data);

	refresh();
}

#define STATE_NAME_LEN 64

int ui_draw_ec(ui_data_t * data){
	wclear(data->win_ec);
	box(data->win_ec, 0, 0);
	wmove(data->win_ec, 0, 1);
	waddstr(data->win_ec, "Engine Control");

	char state_name[STATE_NAME_LEN];

	switch(data->sync_data.engine_control.state) {
	case CONTROL_IDLE:
		snprintf(state_name, STATE_NAME_LEN, "IDLE");
		break;
	case CONTROL_CALIBRATION:
		snprintf(state_name, STATE_NAME_LEN, "CALIBRATION");
		break;
	case CONTROL_ARMED:
		snprintf(state_name, STATE_NAME_LEN, "ARMED");
		break;
	case CONTROL_PRESSURED:
		snprintf(state_name, STATE_NAME_LEN, "PRESSURED");
		break;
	case CONTROL_IGNITER:
		snprintf(state_name, STATE_NAME_LEN, "IGNITER");
		break;
	case CONTROL_IGNITION:
		snprintf(state_name, STATE_NAME_LEN, "IGNITION");
		break;
	case CONTROL_THRUST:
		snprintf(state_name, STATE_NAME_LEN, "THRUST");
		break;
	case CONTROL_SHUTDOWN:
		snprintf(state_name, STATE_NAME_LEN, "SHUTDOWN");
		break;
	case CONTROL_GLIDE:
		snprintf(state_name, STATE_NAME_LEN, "GLIDE");
		break;
	case CONTROL_DESCENT:
		snprintf(state_name, STATE_NAME_LEN, "DESCENT");
		break;
	case CONTROL_SAFE:
		snprintf(state_name, STATE_NAME_LEN, "SAFE");
		break;
	case CONTROL_ERROR:
		snprintf(state_name, STATE_NAME_LEN, "ERROR");
		break;
	case CONTROL_ABORT:
		snprintf(state_name, STATE_NAME_LEN, "ABORT");
		break;
	default:
		snprintf(state_name, STATE_NAME_LEN, "UNKNOWN");
		break;
	}

	mvwprintw(data->win_ec, 1, 1, "Engine state: %s", state_name);
	mvwprintw(data->win_ec, 2, 1, "Engine last cmd: %d", data->sync_data.engine_control.last_cmd);

	mvwprintw(data->win_ec, 3, 1, "eth pressure: %g (%g mV)", data->sync_data.sensor_eng.press_eth, data->sync_data.sensor_eng.adc_2);
	mvwprintw(data->win_ec, 4, 1, "n2o pressure: %g (%g mV)", data->sync_data.sensor_eng.press_n2o, data->sync_data.sensor_eng.adc_1);
	mvwprintw(data->win_ec, 5, 1, "eng pressure: %g (%g mV)", data->sync_data.sensor_eng.press_eng, data->sync_data.sensor_eng.adc_4);
	mvwprintw(data->win_ec, 6, 1, "tank temperature: %g (%g mV)", data->sync_data.sensor_eng.temp_tank, data->sync_data.sensor_eng.adc_3);

	mvwprintw(data->win_ec, data->ec_h-2, 1, "a: ARM | d: DISARM | p: PRESSURE | i: IGNITE");

	wrefresh(data->win_ec);
}

int ui_draw_rc(ui_data_t * data){
	wclear(data->win_rc);
	box(data->win_rc, 0, 0);
	wmove(data->win_rc, 0, 1);
	waddstr(data->win_rc, "Recovery Control");

	mvwprintw(data->win_rc, 1, 1, "Recovery state: LOL", data->sync_data.recovery_control.state);
	mvwprintw(data->win_rc, 2, 1, "[A] GNSS: %gE, %gN, %g, %g, %d", data->sync_data.gnss_data_a.latitude, data->sync_data.gnss_data_a.longitude, data->sync_data.gnss_data_a.altitude, data->sync_data.gnss_data_a.speed, data->sync_data.gnss_data_a.time);
	mvwprintw(data->win_rc, 3, 1, "[B] GNSS: %gE, %gN, %g, %g, %d", data->sync_data.gnss_data_b.latitude, data->sync_data.gnss_data_b.longitude, data->sync_data.gnss_data_b.altitude, data->sync_data.gnss_data_b.speed, data->sync_data.gnss_data_b.time);

	wrefresh(data->win_rc);
}


int ui_draw_so(ui_data_t * data){
	wclear(data->win_so);
	box(data->win_so, 0, 0);
	wmove(data->win_so, 0, 1);
	waddstr(data->win_so, "Sensors Overview");

	mvwprintw(data->win_so, 1, 1, "[A]IMU_ACC: %g %g %g", data->sync_data.imu_a[0].acc[0], data->sync_data.imu_a[0].acc[1], data->sync_data.imu_a[0].acc[2]);
	mvwprintw(data->win_so, 2, 1, "[A]IMU_ACC: %g %g %g", data->sync_data.imu_a[1].acc[0], data->sync_data.imu_a[1].acc[1], data->sync_data.imu_a[1].acc[2]);
	mvwprintw(data->win_so, 3, 1, "[B]IMU_ACC: %g %g %g", data->sync_data.imu_b[0].acc[0], data->sync_data.imu_b[0].acc[1], data->sync_data.imu_b[0].acc[2]);
	mvwprintw(data->win_so, 4, 1, "[B]IMU_ACC: %g %g %g", data->sync_data.imu_b[1].acc[0], data->sync_data.imu_b[1].acc[1], data->sync_data.imu_b[1].acc[2]);

	mvwprintw(data->win_so, 5, 1, "[A]IMU_GYRO: %g %g %g", data->sync_data.imu_a[0].gyro[0], data->sync_data.imu_a[0].gyro[1], data->sync_data.imu_a[0].gyro[2]);
	mvwprintw(data->win_so, 6, 1, "[A]IMU_GYRO: %g %g %g", data->sync_data.imu_a[1].gyro[0], data->sync_data.imu_a[1].gyro[1], data->sync_data.imu_a[1].gyro[2]);
	mvwprintw(data->win_so, 7, 1, "[B]IMU_GYRO: %g %g %g", data->sync_data.imu_b[0].gyro[0], data->sync_data.imu_b[0].gyro[1], data->sync_data.imu_b[0].gyro[2]);
	mvwprintw(data->win_so, 8, 1, "[B]IMU_GYRO: %g %g %g", data->sync_data.imu_b[1].gyro[0], data->sync_data.imu_b[1].gyro[1], data->sync_data.imu_b[1].gyro[2]);

	mvwprintw(data->win_so, 9, 1, "[A]BARO: %g %g (alt: %g)", data->sync_data.baro_a[0].press, data->sync_data.baro_a[0].temp, data->sync_data.baro_a[0].alt);
	mvwprintw(data->win_so,10, 1, "[A]BARO: %g %g (alt: %g)", data->sync_data.baro_a[1].press, data->sync_data.baro_a[1].temp, data->sync_data.baro_a[1].alt);
	mvwprintw(data->win_so,11, 1, "[B]BARO: %g %g (alt: %g)", data->sync_data.baro_b[0].press, data->sync_data.baro_b[0].temp, data->sync_data.baro_b[0].alt);
	mvwprintw(data->win_so,12, 1, "[B]BARO: %g %g (alt: %g)", data->sync_data.baro_b[1].press, data->sync_data.baro_b[1].temp, data->sync_data.baro_b[1].alt);

	mvwprintw(data->win_so,13, 1, "[A]MAG: %g %g %g", data->sync_data.mag_a.mag[0], data->sync_data.mag_a.mag[1], data->sync_data.mag_a.mag[2]);
	mvwprintw(data->win_so,14, 1, "[B]MAG: %g %g %g", data->sync_data.mag_b.mag[0], data->sync_data.mag_b.mag[1], data->sync_data.mag_b.mag[2]);

	mvwprintw(data->win_so,15, 1, "[A]ACC: %g %g %g", data->sync_data.acc_a[0].acc[0], data->sync_data.acc_a[0].acc[1], data->sync_data.acc_a[0].acc[2]);
	mvwprintw(data->win_so,16, 1, "[A]ACC: %g %g %g", data->sync_data.acc_a[1].acc[0], data->sync_data.acc_a[1].acc[1], data->sync_data.acc_a[1].acc[2]);
	mvwprintw(data->win_so,17, 1, "[B]ACC: %g %g %g", data->sync_data.acc_b[0].acc[0], data->sync_data.acc_b[0].acc[1], data->sync_data.acc_b[0].acc[2]);
	mvwprintw(data->win_so,18, 1, "[B]ACC: %g %g %g", data->sync_data.acc_b[1].acc[0], data->sync_data.acc_b[1].acc[1], data->sync_data.acc_b[1].acc[2]);

	wrefresh(data->win_so);
}

int ui_draw_fb(ui_data_t * data){
	wclear(data->win_fb);
	box(data->win_fb, 0, 0);
	wmove(data->win_fb, 0, 1);
	waddstr(data->win_fb, "Debug Feedback");


	wmove(data->win_fb, 1, 1);
	wprintw(data->win_fb, "%ld", time(NULL));
	wrefresh(data->win_fb);
}


int ui_update_content(ui_data_t * data) {

	ui_draw_ec(data);
	ui_draw_rc(data);
	ui_draw_so(data);
	ui_draw_fb(data);

	refresh();

}

void ui_send_engine_command(ui_data_t * data, control_command_t cmd, uint8_t param) {
	uint8_t _data[2];
	_data[0] = cmd;
	_data[1] = param;
	comunicator_send(&data->com, SUBSYSTEM_PROPULSION, 2, _data);
}


int ui_handle_input(ui_data_t * data) {
	timeout(200);

	char c = getch();

	switch(c) {
	case 'q':
		endwin();
		exit(0);
		break;
	case 'a':
		ui_send_engine_command(data, COMMAND_ARM, 0);
		break;
	case 'd':
		ui_send_engine_command(data, COMMAND_DISARM, 0);
		break;
	case 'p':
		ui_send_engine_command(data, COMMAND_PRESSURE, 0);
		break;
	case 'i':
		ui_send_engine_command(data, COMMAND_IGNITE, 0);
		break;
	case 'r':
		ui_send_engine_command(data, COMMAND_RECOVER, 0);
		break;
	case '1':
		ui_send_engine_command(data, COMMAND_VENT_N2O, 1);
		break;
	case '2':
		ui_send_engine_command(data, COMMAND_VENT_N2O, 0);
		break;
	case '3':
		ui_send_engine_command(data, COMMAND_VENT_ETH, 1);
		break;
	case '4':
		ui_send_engine_command(data, COMMAND_VENT_ETH, 0);
		break;
	case '5':
		ui_send_engine_command(data, COMMAND_VALVE_N2O, 180);
		break;
	case '6':
		ui_send_engine_command(data, COMMAND_VALVE_N2O, 0);
		break;
	case '7':
		ui_send_engine_command(data, COMMAND_VALVE_ETH, 180);
		break;
	case '8':
		ui_send_engine_command(data, COMMAND_VALVE_ETH, 0);
		break;
	case '9':
		ui_send_engine_command(data, COMMAND_MAN_PRESS, 1);
		break;
	case '0':
		ui_send_engine_command(data, COMMAND_MAN_PRESS, 0);
		break;
	case 'x':
		ui_send_engine_command(data, COMMAND_ABORT, 0);
		break;
	default:
		break;
	}

}
