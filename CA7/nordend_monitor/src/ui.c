
#include "ui.h"
#include <ncurses.h>
#include <unistd.h>
#include <string.h>
#include "queue.h"



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
	queue_t * q_fb;

}ui_data_t;



int ui_init(int argc, char ** argv, ui_data_t * data);
int ui_mainloop(ui_data_t * data);
int ui_draw_windows(ui_data_t * data);


int ui_start(int argc, char ** argv, ui_param_t * param){

	ui_data_t data = {0};


	//UI initialize
	ui_init(argc, argv, &data);



	char * message = malloc(15);

	//UI mainloop
	for(;;) {


		if(!queue_is_empty(param->q_fb)) {
			queue_pop(param->q_fb, message);
			printf("%s", message);
		}

		ui_draw_windows(&data);




		sleep(1);


	}



}

int ui_init(int argc, char ** argv, ui_data_t * data) {

	initscr();
	noecho();
	curs_set(FALSE);

	ui_draw_windows(data);


}

int ui_draw_windows(ui_data_t * data) {
	//window size ratios

	//horizontal ratio
	int left = 1;
	int right = 1;

	//vertical ratio
	int upper = 1;
	int middle = 1;
	int lower = 1;

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

	data->win_ec = newwin(data->ec_h, data->ec_w, data->ec_y, data->ec_x);
	data->win_rc = newwin(data->rc_h, data->rc_w, data->rc_y, data->rc_x);
	data->win_so = newwin(data->so_h, data->so_w, data->so_y, data->so_x);
	data->win_fb = newwin(data->fb_h, data->fb_w, data->fb_y, data->fb_x);

	box(data->win_ec, 0, 0);
	box(data->win_rc, 0, 0);
	box(data->win_so, 0, 0);
	box(data->win_fb, 0, 0);

	wmove(data->win_ec, 0, 1);
	waddstr(data->win_ec, "Engine Control");
	wmove(data->win_rc, 0, 1);
	waddstr(data->win_rc, "Recovery Control");
	wmove(data->win_so, 0, 1);
	waddstr(data->win_so, "Sensors Overview");
	wmove(data->win_fb, 0, 1);
	waddstr(data->win_fb, "Debug Feedback");


	wrefresh(data->win_ec);
	wrefresh(data->win_rc);
	wrefresh(data->win_so);
	wrefresh(data->win_fb);

	refresh();


}
