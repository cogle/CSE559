#ifndef FEATURES_H
#define FEATURES_H

#include <algorithm>
#include <iostream>
#include <mutex>
#include <memory>

#include "ImageLib/ImageLib.h"
#include "ImageDatabase.h"

#include "WorkerThread.h"


class Fl_Image;

//5x5 Gaussian
const double gaussian5x5[25] = { 0.003663, 0.014652, 0.025641, 0.014652, 0.003663,
0.014652, 0.0586081, 0.0952381, 0.0586081, 0.014652,
0.025641, 0.0952381, 0.150183, 0.0952381, 0.025641,
0.014652, 0.0586081, 0.0952381, 0.0586081, 0.014652,
0.003663, 0.014652, 0.025641, 0.014652, 0.003663 };


//7x7 Gaussian
const double gaussian7x7[49] = { 0.000896861, 0.003587444, 0.006278027, 0.00896861, 0.006278027, 0.003587444, 0.000896861,
0.003587444, 0.010762332, 0.023318386, 0.029596413, 0.023318386, 0.010762332, 0.003587444,
0.006278027, 0.023318386, 0.049327354, 0.06367713, 0.049327354, 0.023318386, 0.006278027,
0.00896861, 0.029596413, 0.06367713, 0.08161435, 0.06367713, 0.029596413, 0.00896861,
0.006278027, 0.023318386, 0.049327354, 0.06367713, 0.049327354, 0.023318386, 0.006278027,
0.003587444, 0.010762332, 0.023318386, 0.029596413, 0.023318386, 0.010762332, 0.003587444,
0.000896861, 0.003587444, 0.006278027, 0.00896861, 0.006278027, 0.003587444, 0.000896861 };

const int GAUSSIAN_SIZE = 7;

const int MOPS_DESCRIPTOR_OFFEST = 16;
const int SIMPLE_DESCRIPTOR_OFFEST = 2;

struct ROCPoint
{
	double trueRate;
	double falseRate;
};

// Compute harris values of an image.
void computeHarrisValues(CFloatImage &srcImage, CFloatImage &harrisImage, CFloatImage & orientationImage, std::vector<WorkerThread> & wtp, std::vector<double> & max_per_partition);

//  Compute local maximum of Harris values in an image.
void computeLocalMaxima(CFloatImage &srcImage, CByteImage &destImage, std::vector<WorkerThread> & wtp, const double & threshold);

// Compute features of an image.
bool computeFeatures(CFloatImage &image, FeatureSet &features, int featureType);

// Perform a query on the database.
bool performQuery(const FeatureSet &f1, const ImageDatabase &db, int &bestIndex, vector<FeatureMatch> &bestMatches, double &bestScore, int matchType);

// Match one feature set with another.
bool matchFeatures(const FeatureSet &f, const FeatureSet &f2, vector<FeatureMatch> &matches, double &totalScore, int matchType);

// Add ROC curve data to the data vector
void addRocData(const FeatureSet &f1, const FeatureSet &f2, const vector<FeatureMatch> &matches, double h[9], vector<bool> &isMatch, double threshold, double &maxD);

// Evaluate a match using a ground truth homography.
double evaluateMatch(const FeatureSet &f1, const FeatureSet &f2, const vector<FeatureMatch> &matches, double h[9]);

// Compute silly example features.
void dummyComputeFeatures(CFloatImage &image, FeatureSet &features);

// Compute actual feature
void ComputeHarrisFeatures(CFloatImage &image, FeatureSet &features);

// Perform ssd feature matching.
void ssdMatchFeatures(const FeatureSet &f1, const FeatureSet &f2, vector<FeatureMatch> &matches, double &totalScore);

// Perform ratio feature matching.  You must implement this.
void ratioMatchFeatures(const FeatureSet &f1, const FeatureSet &f2, vector<FeatureMatch> &matches, double &totalScore);

// Convert Fl_Image to CFloatImage.
bool convertImage(const Fl_Image *image, CFloatImage &convertedImage);

// Convert CFloatImage to CByteImage.
void convertToByteImage(CFloatImage &floatImage, CByteImage &byteImage);

// Compute SSD distance between two vectors.
double distanceSSD(const vector<double> &v1, const vector<double> &v2);

// Transform point by homography.
void applyHomography(double x, double y, double &xNew, double &yNew, double h[9]);

// Computes points on the Roc curve
vector<ROCPoint> computeRocCurve(vector<FeatureMatch> &matches, vector<bool> &isMatch, vector<double> &thresholds);

// Compute AUC given a ROC curve
double computeAUC(vector<ROCPoint> &results);

// Sets up the work to be done by various worker threads.
void divideUpWork(std::vector<WorkerThread> & workers, int width, int height, int num_threads);

void filterImage(CFloatImage &rsltImg, CFloatImage &origImg, int imgWidth, int imgHeight, const double* kernel, int knlWidth, int knlHeight, double scale, double offset);

void computeSimpleDescriptors(CFloatImage &image, FeatureSet &features);

void computeMOPSDescriptors(CFloatImage & image, FeatureSet & features, std::vector<WorkerThread> & wtp);

void resetIndices(std::vector<WorkerThread> & wtp, int height, int width);

#endif
