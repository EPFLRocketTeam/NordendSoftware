//
// Created by iacopo on 8/5/22.
//

#include "kalman.h"
#include "serial.h"
#include "structures.h"

#include <math.h>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>



#define g 		9.81f
#define M 		0.02897f
#define R		8.3145f
#define T		298.15f

#define sigma_z_gps		5.0f
#define sigma_z_baro 	5.0f
#define sigma_z_acc 	9.81f

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

    state->X_tilde << 400.0, 0, 0, 96.2*1000, M/(R*T), 400.0;

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


void * kalman_entry(void *) {


	//setup

    static kalman_rocket_state_t state;
    static serial_dev_t data_device;

    static transfer_data_t data;

    kalman_setup(&state);
    serial_setup(&data_device, "/dev/ttyRPMSG2");

    for(;;) {
        uint32_t len = sizeof(data);
        //ideally a blocking receive
        serial_recv(&data_device, (uint8_t *) &data, &len);
        if(len = sizeof(data)) {
			uint32_t dt = data.time - state.last_time;
			kalman_predict(&state, (float) dt);
			switch(data.type) {
				case TRANSFER_DATA_ACC:
					kalman_update_acc(&state, (float) data.data);
					break;
				case TRANSFER_DATA_BARO:
					kalman_update_baro(&state, (float) data.data);
					break;
				case TRANSFER_DATA_GNSS:
					kalman_update_gnss(&state, (float) data.data);
					break;
			}
        }

    }



}


