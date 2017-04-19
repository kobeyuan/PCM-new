//gmm.cpp : Implementation of Gaussain Mixture Model

#ifndef	 N_GAUSS
#define  N_GAUSS 5
#endif

#include "gmm.h"
#include "Torch\EMTrainer.h"
#include "Torch\MeanVarNorm.h"
#include "Torch\DiagonalGMM.h"
#include "Torch\KFold.h"
#include "Torch\KMeans.h"
#include "Torch\DiskMatDataSet.h"
#include "Torch\MatDataSet.h"
#include "Torch\CmdLine.h"
#include "Torch\NLLMeasurer.h"
#include "Torch\Random.h"
#include "Torch\FileListCmdOption.h"

using namespace Torch;


// create a vector of variance flooring
void initializeThreshold(DataSet* data,real* thresh, real threshold);

int GMM(char *datafile, float means[N_GAUSS][3], float var[N_GAUSS][3], float logw[N_GAUSS])
{
	real accuracy = 0.001; //end accuracy
	real threshold = 0.001; //variance threshold
	int max_iter_kmeans = 50; //max number of iterations of KMeans
	int max_iter_gmm = 50; //max number of iterations of GMM
	int n_gaussians = N_GAUSS; //number of Gaussians
	real prior = 0.001; //prior on the weights

	int max_load = -1; //max number of examples to load for train
	int the_seed = -1; //the random seed

	bool norm = false; //normalize the datas
	char *save_model_file = "model.txt"; //the model file
	int k_fold = -1; //number of folds, if you want to do cross-validation
	bool binary_mode = true; //binary mode for files

	Allocator *allocator = new Allocator;

	//==================================================================== 
	//=================== Create the DataSet ... =========================
	//==================================================================== 
	
	MatDataSet data(datafile, -1, 0, true, max_load, binary_mode);
	MeanVarNorm* mv_norm = NULL;
	if(norm)
		mv_norm = new(allocator)MeanVarNorm (&data);

	//==================================================================== 
	//=================== Training Mode  =================================
	//==================================================================== 

	if(the_seed == -1)
		Random::seed();
	else
		Random::manualSeed((long)the_seed);

	if(norm)
		data.preProcess(mv_norm);

	//=================== Create the GMM... =========================

	// create a KMeans object to initialize the GMM
	KMeans kmeans(data.n_inputs, n_gaussians);
	kmeans.setROption("prior weights",prior);

	// the kmeans trainer
	EMTrainer kmeans_trainer(&kmeans);
	kmeans_trainer.setROption("end accuracy", accuracy);
	kmeans_trainer.setIOption("max iter", max_iter_kmeans);

	// the kmeans measurer
	MeasurerList kmeans_measurers;
	DiskXFile *filektv = new(allocator) DiskXFile("kmeans_train_val", "w");
	NLLMeasurer nll_kmeans_measurer(kmeans.log_probabilities,&data,filektv);
	kmeans_measurers.addNode(&nll_kmeans_measurer);

	// create the GMM
	DiagonalGMM gmm(data.n_inputs,n_gaussians,&kmeans_trainer);
	
	// set the training options
	real* thresh = (real*)allocator->alloc(data.n_inputs*sizeof(real));
	initializeThreshold(&data,thresh,threshold);	
	gmm.setVarThreshold(thresh);
	gmm.setROption("prior weights",prior);
	gmm.setOOption("initial kmeans trainer measurers", &kmeans_measurers);

	//=================== Measurers and Trainer  ===============================

	// Measurers on the training dataset
	MeasurerList measurers;
	DiskXFile *filegtv = new(allocator) DiskXFile("gmm_train_val", "w");
	NLLMeasurer nll_meas(gmm.log_probabilities, &data, filegtv);
	measurers.addNode(&nll_meas);

	// The Gradient Machine Trainer
	EMTrainer trainer(&gmm);
	trainer.setIOption("max iter", max_iter_gmm);
	trainer.setROption("end accuracy", accuracy);
	//trainer.setBOption("viterbi", true);

	//=================== Let's go... ===============================

	if(k_fold <= 0)
	{
		trainer.train(&data, &measurers);

		if(strcmp(save_model_file, ""))
		{
			DiskXFile model_(save_model_file, "w");
			//cmd.saveXFile(&model_);
			if(norm)
				mv_norm->saveXFile(&model_);
			model_.taggedWrite(&n_gaussians, sizeof(int), 1, "n_gaussians");
			model_.taggedWrite(&data.n_inputs, sizeof(int), 1, "n_inputs");
			gmm.saveXFile(&model_);
			for (int i = 0; i < N_GAUSS; i++) {
				logw[i] = gmm.log_weights[i];
				for (int j = 0; j < 3; j++) {
					means[i][j] = gmm.means[i][j];
					var[i][j] = gmm.var[i][j];
				}
			}
		}
	}
	else
	{
		KFold k(&trainer, k_fold);
		k.crossValidate(&data, NULL, &measurers);
	}

	delete allocator;

	return(0);
}



//==================================================================================================== 
//==================================== Functions ===================================================== 
//==================================================================================================== 


void initializeThreshold(DataSet* data,real* thresh, real threshold)
{
	MeanVarNorm norm(data);
	real*	ptr = norm.inputs_stdv;
	real* p_var = thresh;
	for(int i=0;i<data->n_inputs;i++)
		*p_var++ = *ptr * *ptr++ * threshold;
}

