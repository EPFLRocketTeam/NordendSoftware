/**
 * @file kalman.cpp
 * @brief Vertical (1D) navigation based around an extended kalman filter
 *
 * @date 18.08.2022
 * @author Iacopo Sprenger
 *
 * @defgroup host_kalman Kalman filter
 * @{
 */

#include "kalman.h"
#include "serial.h"
#include "protocol/structures.h"

#include <stdio.h>
#include <unistd.h>

#include <math.h>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>



#define g 		9.81f
#define M 		0.02897f
#define R		8.3145f
#define T		298.15f

#define sigma_z_gps		5.0f
#define sigma_z_baro 	5.0f
#define sigma_z_acc 	25.0f

#define sigma_a		1.0f
#define sigma_p0	0.1f
#define sigma_k		1e-9f





/*
 * X vector: z v a p0 M h0
 */



typedef struct kalman_rocket_state{

	//Covariance prediction
	Eigen::Matrix<float, 6, 6> P_tilde;

	//State prediction
	Eigen::Matrix<float, 6, 1> X_tilde;

	//Covariance estimation
	Eigen::Matrix<float, 6, 6> P_hat;

	//State estimation
	Eigen::Matrix<float, 6, 1> X_hat;

	//GPS variance
	Eigen::Matrix<float, 1, 1> R_gps;

	//Barometer variance
	Eigen::Matrix<float, 1, 1> R_baro;

    //Accelerometer variance
    Eigen::Matrix<float, 1, 1> R_acc;

	//Prediction variance
	Eigen::Matrix<float, 3, 3> Q;

	//
	Eigen::Matrix<float, 6, 6> F;

	//
	Eigen::Matrix<float, 6, 3> G;

    //check if this is enough
    uint32_t last_time;


}kalman_rocket_state_t;



#define KALMAN_FRAME "Z;V;A;P0;M;Z0;sZ;sV;sA;sP0;sM;sZ0;TIME"



static kalman_rocket_state_t state;

static FILE * fp;

static comunicator_t com;


void kalman_setup(kalman_rocket_state_t * state, float alt0, float p0) {

    state->X_tilde << alt0, 0.0f, 0.0f, p0, M/(R*T), alt0;

    state->P_tilde.diagonal() << 25.0, 0.25, 0.25, 25.0, 1e-12, 25.0;

    state->X_hat << state->X_tilde;

    state->P_hat << state->P_tilde;

    state->R_gps << powf(sigma_z_gps, 2.0f);

    state->R_baro << powf(sigma_z_baro, 2.0f);

    state->R_acc << powf(sigma_z_acc, 2.0f);

    state->Q.diagonal() << powf(sigma_a,2.0f),  powf(sigma_p0, 2.0f), powf(sigma_k, 2.0f);

    state->F <<     0,  1,  0,  0,  0,  0,
                    0,  0,  1,  0,  0,  0,
                    0,  0,  0,  0,  0,  0,
                    0,  0,  0,  0,  0,  0,
                    0,  0,  0,  0,  0,  0,
                    0,  0,  0,  0,  0,  0;

    state->G <<     0,  0,  0,
                    0,  0,  0,
                    1,  0,  0,
                    0,  1,  0,
                    0,  0,  1,
                    0,  0,  0;

    //state->last_time = 0; //init to first value

	//save data to file
	fprintf(fp, "%f;%f;%f;", state->X_hat(0, 0), state->X_hat(1, 0), state->X_hat(2, 0));
	fprintf(fp, "%f;%f;%f;", state->X_hat(3, 0), state->X_hat(4, 0), state->X_hat(5, 0));
	fprintf(fp, "%f;%f;%f;", state->P_hat(0, 0), state->P_hat(1, 1), state->P_hat(2, 2));
	fprintf(fp, "%f;%f;%f;%u;%c;0\n", state->P_hat(3, 3), state->P_hat(4, 4), state->P_hat(5, 5), state->last_time, 'S');
}

//Constant acceleration model
void kalman_predict(kalman_rocket_state_t * state, float dt) {

	//Create the discrete matrix
	Eigen::Matrix<float, 12, 12> A;

	A << -state->F, state->G*state->Q*state->G.transpose(), Eigen::MatrixXf::Zero(6, 6), state->F.transpose();
	A << A*dt;
	Eigen::Matrix<float, 12, 12> B;
	B << A.exp();

	Eigen::Matrix<float, 6, 6> PHI;
	PHI << B.block<6, 6>(6, 6).transpose();
	Eigen::Matrix<float, 6, 6> Q_w;
	Q_w << PHI*B.block<6, 6>(0, 6);

	//Compute step
	state->X_tilde << PHI*state->X_hat;
	state->P_tilde << PHI*state->P_hat*PHI.transpose() + Q_w;

}


void kalman_update_baro(kalman_rocket_state_t * state, float p) {
	float h = state->X_tilde(0, 0);
	float p0 = state->X_tilde(3, 0);
	float k = state->X_tilde(4, 0);
	float h0 = state->X_tilde(5, 0);

	float p_est = p0*expf(k*g*(h0-h));

	Eigen::Matrix<float, 1, 6> H;
	H << -k*g*p_est, 0.0, 0.0, p_est/p0, g*(h0-h)*p_est, k*g*p_est;

	Eigen::Matrix<float, 6, 1> K;
	K << state->P_tilde*H.transpose()*(H*state->P_tilde*H.transpose() + state->R_baro).inverse();

	state->X_hat << state->X_tilde + K*(p - p_est);

	state->P_hat << (Eigen::MatrixXf::Identity(6,6) - K*H)*state->P_tilde;


}

void kalman_update_gnss(kalman_rocket_state_t * state, float z) {
    Eigen::Matrix<float, 1, 6> H;
    H << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    Eigen::Matrix<float, 6, 1> K;
    K << state->P_tilde*H.transpose()*(H*state->P_tilde*H.transpose() + state->R_gps).inverse();

    state->X_hat << state->X_tilde + K*(z - state->X_tilde(0));

    state->P_hat << (Eigen::MatrixXf::Identity(6,6) - K*H)*state->P_tilde;
}

void kalman_update_acc(kalman_rocket_state_t * state, float a) {
    Eigen::Matrix<float, 1, 6> H;
    H << 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;

    Eigen::Matrix<float, 6, 1> K;
    K << state->P_tilde*H.transpose()*(H*state->P_tilde*H.transpose() + state->R_acc).inverse();

    state->X_hat << state->X_tilde + K*(a - state->X_tilde(0));

    state->P_hat << (Eigen::MatrixXf::Identity(6,6) - K*H)*state->P_tilde;
}







void kalman_send_data(comunicator_t * com, kalman_rocket_state * state) {


	transfer_data_res_t data;
	data.alt = (int32_t) (state->X_hat(0, 0));
	data.vel = (int32_t) (state->X_hat(1, 0));


	comunicator_send(com, TRANSFER_DATA_RES, sizeof(transfer_data_res_t), (uint8_t *) &data);

}


void kalman_handle_data(uint8_t opcode, uint16_t len, uint8_t * _data) {

	static uint8_t first_time = 10;
	static float p0 = 92000.0f;
	static float alt0 = 120.0f;

	if(len = sizeof(transfer_data_t)) {
		static uint32_t data_count = 0;
		transfer_data_t data;
		memcpy(&data, _data, len);
		printf("kalman cycle| data: %d, type: %d, time: %d\n",
				data.data, opcode, data.time);
		if(state.last_time == 0) {
			state.last_time = data.time;
			kalman_setup(&state, alt0, p0);
			return;
		}
		uint32_t dt = data.time - state.last_time;
		state.last_time = data.time;
		float delta_t = dt/1000.0;
		printf("delta time: %f\n", delta_t);
		kalman_predict(&state, delta_t);
		float acc, pres, alt;
		char last_val;
		switch(opcode) {
			case TRANSFER_DATA_ACC:
				acc = data.data / 10000.0; //0.1millig
				//printf("acc data: %f\n", acc);
				//kalman_update_acc(&state, acc);
				break;
			case TRANSFER_DATA_BARO:
				last_val = 'B';
				pres = data.data*1.0;  //baro in Pa
				//printf("baro data: %f\n", pres);
				if(first_time) {
					p0 = pres;
				}
				kalman_update_baro(&state, pres);
				break;
			case TRANSFER_DATA_GNSS:
				last_val = 'G';
				alt = data.data; //alt in m
				if(alt != 0) { //avoid zero alt from RMC packets
					if(first_time) {
						alt0 = alt;
						first_time-=1;
						kalman_setup(&state, alt0, p0);
					}
					//printf("alt data: %f\n", alt);
					kalman_update_gnss(&state, alt);
				}
				break;
			default:
				//printf("data error\n");
				break;
		}

		kalman_send_data(&com, &state);

		//save data to file
		fprintf(fp, "%f;%f;%f;", state.X_hat(0, 0), state.X_hat(1, 0), state.X_hat(2, 0));
		fprintf(fp, "%f;%f;%f;", state.X_hat(3, 0), state.X_hat(4, 0), state.X_hat(5, 0));
		fprintf(fp, "%f;%f;%f;", state.P_hat(0, 0), state.P_hat(1, 1), state.P_hat(2, 2));
		fprintf(fp, "%f;%f;%f;%u;%c;%d\n", state.P_hat(3, 3), state.P_hat(4, 4), state.P_hat(5, 5), state.last_time, last_val, data.data);





		printf("compute_result: H=%f | sH=%f\n", state.X_hat(0, 0), state.P_hat(0, 0));
		printf("compute_result: V=%f | sV=%f\n", state.X_hat(1, 0), state.P_hat(1, 1));
		printf("compute_result: A=%f | sA=%f\n", state.X_hat(2, 0), state.P_hat(2, 2));
	}

}






void * kalman_entry(void *) {


	//setup


    static serial_dev_t data_device;

    static transfer_data_t data;


    serial_setup(&data_device, "/dev/ttyRPMSG2");

    uint8_t msg[] = "start";
    serial_send(&data_device, msg, 6);

    printf("setup kalman channel\n");

    static char fname[64];
	static uint16_t num = 0;
	do{
		snprintf(fname, 64, "/home/root/av_kal%d.log", num);
		num++;
	}while((access(fname, F_OK) == 0));
	fp = fopen(fname, "w+");

	fprintf(fp, KALMAN_FRAME"\n");

    state.last_time = 0;

    //kalman_setup(&state);



    comunicator_init(&com, &data_device, kalman_handle_data);


    comunicator_recv(&com);


    return NULL;

}


/**
 * @}
 */

