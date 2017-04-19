// gmm.h : interface of the Gaussian Mixture Model
//


#pragma once

int GMM(char *datafile, float means[N_GAUSS][3], float var[N_GAUSS][3], float logw[N_GAUSS]);