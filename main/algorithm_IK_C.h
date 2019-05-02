/*
 * algorithm_IK_C.h
 *
 *  Created on: 29/10/2018
 *      Author: Igor Koval
 *		Description:
 *
 *      This algorithm is a direct "translation" from algorithm created by Robert Fraczkiewicz,in 01/2018,
 *      algorithm_by_RF.h in C++ to C.
 *
 *
 *	Copyright (C) 2017 Robert Fraczkiewicz, All Rights Reserved.
 */

#ifndef MAIN_ALGORITHM_IK_C_H_
#define MAIN_ALGORITHM_IK_C_H_

#include "stdint.h"
#include "string.h"
//#include "strings.h"
#include "stdbool.h"
#include "math.h"
#include "defines.h"

#define FS 		SPO2_SAMPLE_RATE/SMP_AVE               //  25 sampling frequency
#define BUFFER_SIZE ((FIFO_A_FULL/2)*PACKS_TO_SEND)  //  (FS*4)
#define FS60 (FS*60)        //  Conversion factor for heart rate
#define MAX_HR 230          //  Maximal heart rate
#define MIN_HR 30           //  Minimal heart rate



void heart_rate_and_oxygen_saturation(uint32_t *pun_ir_buffer, int32_t n_ir_buffer_length, uint32_t *pun_red_buffer, float *pn_spo2, int8_t *pch_spo2_valid, int32_t *pn_heart_rate,
                                        int8_t *pch_hr_valid, float *ratio, float *correl);
float linear_regression_beta(float *pn_x, float xmean, float sum_x2);
float autocorrelation(float *pn_x, int32_t n_size, int32_t n_lag);
float rms(float *pn_x, int32_t n_size, float *sumsq);
float Pcorrelation(float *pn_x, float *pn_y, int32_t n_size);
void signal_periodicity(float *pn_x, int32_t n_size, int32_t *p_last_periodicity, int32_t n_min_distance, int32_t n_max_distance, float min_aut_ratio, float aut_lag0, float *ratio);




#endif /* MAIN_ALGORITHM_IK_C_H_ */
