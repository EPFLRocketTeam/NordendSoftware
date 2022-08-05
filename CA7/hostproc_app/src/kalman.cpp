//
// Created by iacopo on 8/5/22.
//

#include "kalman.h"
#include <Eigen/Dense>



void * matrix_test(void *) {

    Eigen::Matrix<double, 6, 6> matrix;
    Eigen::Matrix<double, 6, 6> result;

    matrix << 1, 0, 0, 1, 0, 0, 0, 0, 1;

    result = matrix * matrix;


}


