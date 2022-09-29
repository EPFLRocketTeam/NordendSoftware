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

#include <math.h>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>



#define g 		9.81f
#define M 		0.02897f
#define R		8.3145f
#define T		298.15f

#define sigma_z_gps		5.0f
#define sigma_z_baro 	10.0f
#define sigma_z_acc 	25.0f

#define sigma_a		1.0f
#define sigma_p0	0.01f
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


void kalman_setup(kalman_rocket_state_t * state) {

    state->X_tilde << 400.0f, 0.0f, 0.0f, 96.2*1000, M/(R*T), 400.0;

    state->P_tilde.diagonal() << 25.0, 0.1, 0.1, 25.0, 1e-12, 25.0;

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

    state->last_time = 0; //init to first value
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



static kalman_rocket_state_t state;


void kalman_handle_data(uint8_t opcode, uint16_t len, uint8_t * _data) {

	if(len = sizeof(transfer_data_t)) {
		transfer_data_t data;
		memcpy(&data, _data, len);
		printf("kalman cycle| data: %d, type: %d, time: %d\n",
				data.data, opcode, data.time);
		if(state.last_time == 0) {
			state.last_time = data.time;
			return;
		}
		uint32_t dt = data.time - state.last_time;
		state.last_time = data.time;
		float delta_t = dt/1000.0;
		printf("delta time: %f\n", delta_t);
		kalman_predict(&state, delta_t);
		float acc, pres, alt;
		switch(opcode) {
			case TRANSFER_DATA_ACC:
				acc = data.data / 10000.0; //0.1millig
				printf("acc data: %f\n", acc);
				kalman_update_acc(&state, acc);
				break;
			case TRANSFER_DATA_BARO:
				pres = data.data*10.0;  //baro in 0.1Pa
				printf("baro data: %f\n", pres);
				kalman_update_baro(&state, pres);
				break;
			case TRANSFER_DATA_GNSS:
				alt = data.data/1000.0; //alt in mm
				printf("alt data: %f\n", alt);
				kalman_update_gnss(&state, alt);
				break;
			default:
				printf("data error\n");
		}
		printf("compute_result: H=%f | sH=%f\n", state.X_hat(0, 0), state.P_hat(0, 0));
		printf("compute_result: V=%f | sV=%f\n", state.X_hat(1, 0), state.P_hat(1, 1));
		printf("compute_result: A=%f | sA=%f\n", state.X_hat(2, 0), state.P_hat(2, 2));
	}

}






void * kalman_entry(void *) {


	//setup


    static serial_dev_t data_device;

    static transfer_data_t data;

    kalman_setup(&state);
    serial_setup(&data_device, "/dev/ttyRPMSG2");

    uint8_t msg[] = "start";
    serial_send(&data_device, msg, 6);

    printf("setup kalman channel\n");

    state.last_time = 0;

    comunicator_t com;

    comunicator_init(&com, &data_device, kalman_handle_data);


    comunicator_recv(&com);



}


/**
 * @}
 */

