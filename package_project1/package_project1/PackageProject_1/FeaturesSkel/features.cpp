#include <assert.h>
#include <math.h>
#include <FL/Fl.H>
#include <FL/Fl_Image.H>
#include "features.h"
#include "ImageLib/FileIO.h"
#include "Spinlock.h"

#include <chrono>
#include <ctime>



#define PI 3.14159265358979323846

// Compute features of an image.
bool computeFeatures(CFloatImage &image, FeatureSet &features, int featureType) {
	// TODO: Instead of calling dummyComputeFeatures, write your own
	// feature computation routines and call them here.
	switch (featureType) {
	case 1:
		std::cout << "Dummy Compute" << std::endl;
		dummyComputeFeatures(image, features);
		break;
	case 2:
		ComputeHarrisFeatures(image, features);
		break;
	default:
		return false;
	}

	// This is just to make sure the IDs are assigned in order, because
	// the ID gets used to index into the feature array.
	for (unsigned int i = 0; i < features.size(); i++) {
		features[i].id = i + 1;
	}

	return true;
}

// Perform a query on the database.  This simply runs matchFeatures on
// each image in the database, and returns the feature set of the best
// matching image.
bool performQuery(const FeatureSet &f, const ImageDatabase &db, int &bestIndex, vector<FeatureMatch> &bestMatches, double &bestScore, int matchType) {
	// Here's a nice low number.
	bestScore = -1e100;

	vector<FeatureMatch> tempMatches;
	double tempScore;

	for (unsigned int i = 0; i < db.size(); i++) {
		if (!matchFeatures(f, db[i].features, tempMatches, tempScore, matchType)) {
			return false;
		}

		if (tempScore > bestScore) {
			bestIndex = i;
			bestScore = tempScore;
			bestMatches = tempMatches;
		}
	}

	return true;
}

// Match one feature set with another.
bool matchFeatures(const FeatureSet &f1, const FeatureSet &f2, vector<FeatureMatch> &matches, double &totalScore, int matchType) {
	// TODO: We have given you the ssd matching function, you must write your own
	// feature matching function for the ratio test.

	printf("\nMatching features.......\n");
	std::cout << f1.size() << std::endl;
	std::cout << f2.size() << std::endl;
	switch (matchType) {
	case 1:
		std::cout << "Matching using SSD Match" << std::endl;
		ssdMatchFeatures(f1, f2, matches, totalScore);
		return true;
	case 2:
		std::cout << "Matching using Ratio Match" << std::endl;
		ratioMatchFeatures(f1, f2, matches, totalScore);
		return true;
	default:
		return false;
	}
}

// Evaluate a match using a ground truth homography.  This computes the
// average SSD distance between the matched feature points and
// the actual transformed positions.
double evaluateMatch(const FeatureSet &f1, const FeatureSet &f2, const vector<FeatureMatch> &matches, double h[9]) {
	double d = 0;
	int n = 0;

	double xNew;
	double yNew;

	unsigned int num_matches = matches.size();
	for (unsigned int i = 0; i < num_matches; i++) {
		int id1 = matches[i].id1;
		int id2 = matches[i].id2;
		applyHomography(f1[id1 - 1].x, f1[id1 - 1].y, xNew, yNew, h);
		d += sqrt(pow(xNew - f2[id2 - 1].x, 2) + pow(yNew - f2[id2 - 1].y, 2));
		n++;
	}

	return d / n;
}

void addRocData(const FeatureSet &f1, const FeatureSet &f2, const vector<FeatureMatch> &matches, double h[9], vector<bool> &isMatch, double threshold, double &maxD) {
	double d = 0;

	double xNew;
	double yNew;

	unsigned int num_matches = matches.size();
	for (unsigned int i = 0; i<num_matches; i++) {
		int id1 = matches[i].id1;
		int id2 = matches[i].id2;
		applyHomography(f1[id1 - 1].x, f1[id1 - 1].y, xNew, yNew, h);

		// Ignore unmatched points.  There might be a better way to
		// handle this.
		d = sqrt(pow(xNew - f2[id2 - 1].x, 2) + pow(yNew - f2[id2 - 1].y, 2));
		if (d <= threshold)
		{
			isMatch.push_back(1);
		}
		else
		{
			isMatch.push_back(0);
		}

		if (matches[i].score>maxD)
			maxD = matches[i].score;
	}
}

vector<ROCPoint> computeRocCurve(vector<FeatureMatch> &matches, vector<bool> &isMatch, vector<double> &thresholds)
{
	vector<ROCPoint> dataPoints;

	for (int i = 0; i < (int)thresholds.size(); i++)
	{
		//printf("Checking threshold: %lf.\r\n",thresholds[i]);
		int tp = 0;
		int actualCorrect = 0;
		int fp = 0;
		int actualError = 0;
		int total = 0;

		int num_matches = (int)matches.size();
		for (int j = 0; j < num_matches; j++)
		{
			if (isMatch[j])
			{
				actualCorrect++;
				if (matches[j].score < thresholds[i])
				{
					tp++;
				}
			}
			else
			{
				actualError++;
				if (matches[j].score < thresholds[i])
				{
					fp++;
				}
			}

			total++;
		}

		ROCPoint newPoint;
		//printf("newPoints: %lf,%lf",newPoint.trueRate,newPoint.falseRate);
		newPoint.trueRate = (double(tp) / actualCorrect);
		newPoint.falseRate = (double(fp) / actualError);
		//printf("newPoints: %lf,%lf",newPoint.trueRate,newPoint.falseRate);

		dataPoints.push_back(newPoint);
	}

	return dataPoints;
}


// Compute silly example features.  This doesn't do anything
// meaningful.
void dummyComputeFeatures(CFloatImage &image, FeatureSet &features) {
	CShape sh = image.Shape();
	Feature f;

	CFloatImage grayImage = ConvertToGray(image);
	CFloatImage temp = ConvertToGray(image);
	Convolve(grayImage, temp, ConvolveKernel_SobelX);

	for (int y = 0; y < sh.height; y++) {
		for (int x = 0; x < sh.width; x++) {
			double r = image.Pixel(x, y, 0);
			double g = image.Pixel(x, y, 1);
			double b = image.Pixel(x, y, 2);

			if ((int)(255 * (r + g + b) + 0.5) % 100 == 1) {
				// If the pixel satisfies this meaningless criterion,
				// make it a feature.

				f.type = 1;
				f.id += 1;
				f.x = x;
				f.y = y;

				f.data.resize(1);
				f.data[0] = r + g + b;

				features.push_back(f);
			}
		}
	}
}

void ComputeHarrisFeatures(CFloatImage &image, FeatureSet &features)
{

	std::chrono::time_point<std::chrono::system_clock> start, end;
	start = std::chrono::system_clock::now();
	std::cout << "Entered the Compute Harris Function\n";

	/*
	Set up a thread pool to be used throughout this functions call.
	*/
	const int max_threads = WorkerThread::max_threads() < 6 ? 6 : WorkerThread::max_threads();
	//const int max_threads = 1;
	const int w = image.Shape().width;
	const int h = image.Shape().height;


	std::vector<WorkerThread> thread_pool;
	divideUpWork(thread_pool, w, h, max_threads);

	std::vector<double> max_per_partition(max_threads);

	//Create grayscale image used for Harris detection
	CFloatImage grayImage = ConvertToGray(image);

	//Create image to store Harris values
	CFloatImage harrisImage(image.Shape().width, image.Shape().height, 1);

	//Create image to store the orientation of each pixel
	CFloatImage orientationImage(image.Shape().width, image.Shape().height, 1);

	//Create image to store local maximum harris values as 1, other pixels 0
	CByteImage harrisMaxImage(image.Shape().width, image.Shape().height, 1);

	//Single Threaded Compute Harris function
	computeHarrisValues(grayImage, harrisImage, orientationImage, thread_pool, max_per_partition);

	double max = 0.0;
	std::for_each(max_per_partition.begin(), max_per_partition.end(), [&max](double & ele)
	{
		if (ele > max){ max = ele; }
	});

	// Threshold the harris image and compute local maxima.  You'll need to implement this function.
	std::cout << max << std::endl;
	std::cout << max*.05 << std::endl;
	computeLocalMaxima(harrisImage, harrisMaxImage, thread_pool, max*.05);

	// Prints out the harris image for debugging purposes
	//CByteImage tmp(harrisImage.Shape());
	//convertToByteImage(harrisImage, tmp);
	//WriteFile(tmp, "harris_yosemite.tga");

	// TO DO--------------------------------------------------------------------
	//Loop through feature points in harrisMaxImage and create feature descriptor 
	//for each point above a threshold

	
	int id = 0;

	const int offset = 20;

	SpinlockMutex spin_lock;
	std::for_each(thread_pool.begin(), thread_pool.end(), [&](WorkerThread & wt)
	{

		wt.assign_work(std::thread([&]()
		{

			int start_x_idx = wt.get_start_x();
			int start_y_idx = wt.get_start_y();


			int end_x_idx = wt.get_end_x();
			int end_y_idx = wt.get_end_y();

			if (wt.get_worker_id() == 0)
			{
				start_x_idx += offset;
				start_y_idx += offset;

				end_x_idx -= offset;
			}
			else if (wt.get_worker_id() == thread_pool.size() - 1)
			{
				start_x_idx += offset;


				end_x_idx -= offset;
				end_y_idx -= offset;
			}
			else
			{
				start_x_idx += offset;
				end_x_idx -= offset;
			}


			for (int y = start_y_idx; y < end_y_idx; y++)
			{
				for (int x = start_x_idx; x < end_x_idx; x++)
				{
					if (harrisMaxImage.Pixel(x, y, 0) == 0){ continue; }

					Feature f;
					f.type = 2;
					f.x = x;
					f.y = y;
					f.angleRadians = orientationImage.Pixel(x, y, 0);

					f.data.resize(1);
					f.data[0] = harrisImage.Pixel(x, y, 0);

					//Custom spin lock implementation. Putting a thread to sleep
					//would be a real waste. Taking advantage of scoped locking.
					{
						std::lock_guard<SpinlockMutex> lock(spin_lock);
						f.id = id;
						features.push_back(f);
						id++;
					}

				}
			}
		}));
	});
	std::for_each(thread_pool.begin(), thread_pool.end(), [](WorkerThread & wt)
	{
		wt.join();
	});
	
	/*
	for (int y = 20; y < h-20; y++)
	{
		for (int x = 20; x < w-20; x++)
		{
			if (harrisMaxImage.Pixel(x, y, 0) == 0){ continue; }

			Feature f;
			f.type = 2;
			f.x = x;
			f.y = y;
			f.angleRadians = orientationImage.Pixel(x, y, 0);

			f.data.resize(1);
			f.data[0] = harrisImage.Pixel(x, y, 0);

				f.id = id;
				features.push_back(f);
				id++;
		}
	}
	*/

	//Threading this method proved to be more work than it was worth. 
	//The overhead from storing shared pointers to elements in the Feature array caused the 
	//run time to spike, beyond the single threaded version. 

	/*SIMPLE DESCRIPTOR*/
	//std::cout << "Simple Descriptor\n";
	computeSimpleDescriptors(grayImage, features);

	/*MOPS DESCRIPTOR*/
	//std::cout << "MOPS Descriptor\n";
	//resetIndices(thread_pool, h, w);
	//computeMOPSDescriptors(grayImage, features, thread_pool);

	end = std::chrono::system_clock::now();
	std::chrono::duration<double> elapsed_seconds = end - start;
	std::cout << "Exited Harris function: " << elapsed_seconds.count() << "sec" << std::endl;
}


//TO DO---------------------------------------------------------------------
//Loop through the image to compute the harris corner values as described in class
// srcImage:  grayscale of original image
// harrisImage:  populate the harris values per pixel in this image
void computeHarrisValues(CFloatImage &srcImage, CFloatImage &harrisImage, CFloatImage & orientationImage, std::vector<WorkerThread> & wtp, std::vector<double> & max_per_partition)
{
	
	const int w = srcImage.Shape().width;
	const int h = srcImage.Shape().height;

	CFloatImage derivative_xx = CFloatImage(w, h, 1);
	CFloatImage derivative_yy = CFloatImage(w, h, 1);
	CFloatImage derivative_xy = CFloatImage(w, h, 1);

	CFloatImage weighted_derivative_xx = CFloatImage(w, h, 1);
	CFloatImage weighted_derivative_yy = CFloatImage(w, h, 1);
	CFloatImage weighted_derivative_xy = CFloatImage(w, h, 1);

	//Part 1
	std::for_each(wtp.begin(), wtp.end(), [&](WorkerThread & wt){
		wt.assign_work(std::thread([&](){
			ConvolveThreaded(srcImage, derivative_xx, ConvolveKernel_SobelX, wt.get_start_y(), wt.get_end_y());
			ConvolveThreaded(srcImage, derivative_yy, ConvolveKernel_SobelY, wt.get_start_y(), wt.get_end_y());
		}));
	});

	std::for_each(wtp.begin(), wtp.end(), [&](WorkerThread & wt){
		wt.join();
	});
	
	//Part 2

	std::for_each(wtp.begin(), wtp.end(), [&](WorkerThread & wt){
		wt.assign_work(std::thread([&](){
			//Take note that y-index refers to the current height.
			//This loop updates each of the arrays from above to be how they are represented in
			//the Harris Matrix.
			for (int y = wt.get_start_y(); y < wt.get_end_y(); y++)
			{
				for (int x = wt.get_start_x(); x < wt.get_end_x(); x++)
				{
					derivative_xy.Pixel(x, y, 0) = derivative_xx.Pixel(x, y, 0) * derivative_yy.Pixel(x, y, 0);
					derivative_xx.Pixel(x, y, 0) = pow(derivative_xx.Pixel(x, y, 0), 2.0);
					derivative_yy.Pixel(x, y, 0) = pow(derivative_yy.Pixel(x, y, 0), 2.0);
				}
			}
		}));
	});
	std::for_each(wtp.begin(), wtp.end(), [&](WorkerThread & wt){
		wt.join();
	});
	
	
	int offset = 2;
	std::for_each(wtp.begin(), wtp.end(), [&](WorkerThread & wt){

		wt.assign_work(std::thread([&](){

			
			int start_x_idx = wt.get_start_x();
			int start_y_idx = wt.get_start_y();


			int end_x_idx = wt.get_end_x();
			int end_y_idx = wt.get_end_y();

			if (wt.get_worker_id() == 0)
			{
				start_x_idx += offset;
				start_y_idx += offset;

				end_x_idx -= offset;
			}
			else if (wt.get_worker_id() == wtp.size() - 1)
			{
				start_x_idx += offset;


				end_x_idx -= offset;
				end_y_idx -= offset;
			}
			else
			{
				start_x_idx += offset;
				end_x_idx -= offset;
			}

			for (int y = start_y_idx; y < end_y_idx; y++)
			{
				for (int x = start_x_idx; x < end_x_idx; x++)
				{

					weighted_derivative_xy.Pixel(x, y, 0) = 0.0;
					weighted_derivative_xx.Pixel(x, y, 0) = 0.0;
					weighted_derivative_yy.Pixel(x, y, 0) = 0.0;

					for (int gy = -offset; gy <= offset; gy++)
					{
						for (int gx = -offset; gx <= offset; gx++)
						{
							//gy*w+gx
							int gaussian_index = (gy + offset)*GAUSSIAN_SIZE + (gx + offset);

							int adjusted_x_idx = x + gx;
							int adjusted_y_idx = y + gy;

							weighted_derivative_xy.Pixel(x, y, 0) += derivative_xy.Pixel(adjusted_x_idx, adjusted_y_idx, 0)*gaussian5x5[gaussian_index];
							weighted_derivative_xx.Pixel(x, y, 0) += derivative_xx.Pixel(adjusted_x_idx, adjusted_y_idx, 0)*gaussian5x5[gaussian_index];
							weighted_derivative_yy.Pixel(x, y, 0) += derivative_yy.Pixel(adjusted_x_idx, adjusted_y_idx, 0)*gaussian5x5[gaussian_index];

						}
					}

				}
			}
			
			double a;
			double b;
			double d;

			double det;
			double trace;
			double eigen_value;

			double angle;
			double root_lhs;
			double root_rhs;
			double root_value;

			double harris_value;

			double max = -1.0;


			for (int j = wt.get_start_y() ; j < wt.get_end_y(); j++)
			{
				
				for (int i = wt.get_start_x(); i < wt.get_end_x(); i++)
				{

					a = weighted_derivative_xx.Pixel(i, j, 0);
					b = weighted_derivative_xy.Pixel(i, j, 0);
					d = weighted_derivative_yy.Pixel(i, j, 0);
					trace = a + d;
					det = a*d - b*b;

					harris_value = (det / trace);

					root_lhs = 4.0*b*b;
					root_rhs = pow(a - d, 2.0);
					root_value = sqrt(root_lhs + root_rhs);
					eigen_value = .5*((trace)-root_value);
					angle = atan2(b, eigen_value - d);

					harrisImage.Pixel(i, j, 0) = harris_value;
					orientationImage.Pixel(i, j, 0) = angle;

					if (harris_value > max){ max = harris_value; }
				}
			}

			max_per_partition[wt.get_worker_id()] = max;

		}));
	});

	std::for_each(wtp.begin(), wtp.end(), [&](WorkerThread & wt){
		wt.join();
	});
}



// TO DO---------------------------------------------------------------------
// Loop through the harrisImage to threshold and compute the local maxima in a neighborhood
// srcImage:  image with Harris values
// destImage: Assign 1 to a pixel if it is above a threshold and is the local maximum in 3x3 window, 0 otherwise.
//    You'll need to find a good threshold to use.
void computeLocalMaxima(CFloatImage &srcImage, CByteImage &destImage, std::vector<WorkerThread> & wtp, const double & threshold)
{
	std::for_each(wtp.begin(), wtp.end(), [&](WorkerThread & wt)
	{
		for (int y = wt.get_start_y(); y < wt.get_end_y(); y++)
		{
			
			for (int x = wt.get_start_x(); x < wt.get_end_x(); x++)
			{
				double cur_pixel = srcImage.Pixel(x, y, 0);
				if (cur_pixel >= threshold)
				{

					bool max = true;
					for (int search_y = y - 1; search_y <= y + 1 && max; search_y++)
					{
						for (int search_x = x - 1; search_x <= x + 1; search_x++)
						{
							if (search_x < 0 || search_y < 0){ continue; }
							if (search_x >= srcImage.Shape().width || search_y >= srcImage.Shape().height){ continue; }
							else if (srcImage.Pixel(search_x, search_y, 0) > cur_pixel)
							{
								destImage.Pixel(x, y, 0) = 0.0;
								max = false;
								break;
							}
						}
					}
					if (max)
					{
						destImage.Pixel(x, y, 0) = 1.0;
					}
				}
				else
				{
					destImage.Pixel(x, y, 0) = 0.0;
				}
			}
		}
	});

}

/*
Basic algoirthm for this procedure comes from the following slides:
http://www.cs.princeton.edu/courses/archive/fall08/cos429/features
*/
void computeMOPSDescriptors(CFloatImage & image, FeatureSet & features, std::vector<WorkerThread> & wtp)
{

	const int window_size = 8;

	// Blur Image
	CFloatImage blur_image{ image.Shape().width, image.Shape().height, 1 };

	//5x5 Gaussian Blur kernel(Copied from the one in the header)
	int blur_window_size = 5;
	CFloatImage blurKernel{ blur_window_size, blur_window_size, 1 };
	blurKernel.origin[0] = blur_window_size / 2;
	blurKernel.origin[1] = blur_window_size / 2;


	for (int j = 0; j < blur_window_size; j++){
		for (int i = 0; i < blur_window_size; i++){
			blurKernel.Pixel(i, j, 0) = gaussian5x5[blur_window_size * j + i];
		}
	}

	std::for_each(wtp.begin(), wtp.end(), [&](WorkerThread & wt){
		wt.assign_work(std::thread([&](){
			ConvolveThreaded(image, blur_image, blurKernel, wt.get_start_y(), wt.get_end_y());
		}));
	});

	std::for_each(wtp.begin(), wtp.end(), [&](WorkerThread & wt){
		wt.join();
	});

	//Convolve(image, blur_image, blurKernel);

	CFloatImage feature_image{ window_size, window_size, 1 };

	for (Feature & f : features)
	{

		CTransform3x3 transformed_matrix;

		CTransform3x3 origin;
		origin = origin.Translation(float(-window_size / 2), float(-window_size / 2));

		CTransform3x3 rotate;
		rotate = rotate.Rotation(f.angleRadians * 180.0 / PI);

		CTransform3x3 trans;
		trans = trans.Translation(f.x, f.y);

		CTransform3x3 scale;
		scale[0][0] = 40 / window_size;
		scale[1][1] = 40 / window_size;

		transformed_matrix = trans * rotate * scale * origin;

		WarpGlobal(blur_image, feature_image, transformed_matrix, eWarpInterpLinear);

		f.data.resize(window_size * window_size);

		double mean = 0.0;
		for (int j = 0; j < window_size; j++)
		{
			for (int i = 0; i < window_size; i++)
			{
				mean += feature_image.Pixel(i, j, 0);
			}
		}
		mean = mean / (window_size * window_size);

		double variance = 0.0;
		for (int j = 0; j < window_size; j++)
		{
			for (int i = 0; i < window_size; i++)
			{
				variance += pow((feature_image.Pixel(i, j, 0) - mean), 2);
			}
		}

		double standard_dev = sqrt(variance / ((window_size * window_size) - 1));

		// Fill in the feature descriptor data for a MOPS descriptor
		for (int j = 0; j < window_size; j++)
		{
			for (int i = 0; i < window_size; i++)
			{
				f.data[j * window_size + i] = (feature_image.Pixel(i, j, 0) - mean) / standard_dev;
			}
		}
	}

}

void computeSimpleDescriptors(CFloatImage &image, FeatureSet &features)
{
	const int window_size = 5;
	const int offset = window_size / 2;
	for (Feature & f : features)
	{

		int x = f.x;
		int y = f.y;

		f.data.resize(window_size * window_size);


		double mean = 0.0;
		double variance = 0.0;
		double standard_dev = 0.0;


		for (int gy = -offset; gy <= offset; gy++)
		{
			for (int gx = -offset; gx <= offset; gx++)
			{
				int adjusted_x_idx = x + gx;
				int adjusted_y_idx = y + gy;
				
				mean += image.Pixel(adjusted_x_idx, adjusted_y_idx, 0);
				
			}
		}

		mean = mean / 25;

		for (int gy = -offset; gy <= offset; gy++)
		{
			for (int gx = -offset; gx <= offset; gx++)
			{
				int adjusted_x_idx = x + gx;
				int adjusted_y_idx = y + gy;

				variance += pow(image.Pixel(adjusted_x_idx, adjusted_y_idx, 0) - mean, 0);

			}
		}

		standard_dev = sqrt(variance / 24);
		for (int gy = -offset; gy <= offset; gy++)
		{
			int data_idx = 0;
			for (int gx = -offset; gx <= offset; gx++)
			{
				int adjusted_x_idx = x + gx;
				int adjusted_y_idx = y + gy;

				f.data[data_idx] = (image.Pixel(x, y, 0) - mean) / standard_dev;
				data_idx++;
			}
		}


	}
}

// Perform simple feature matching.  This just uses the SSD
// distance between two feature vectors, and matches a feature in the
// first image with the closest feature in the second image.  It can
// match multiple features in the first image to the same feature in
// the second image.
void ssdMatchFeatures(const FeatureSet &f1, const FeatureSet &f2, vector<FeatureMatch> &matches, double &totalScore) {
	int m = f1.size();
	int n = f2.size();

	matches.resize(m);
	totalScore = 0;

	double d;
	double dBest;
	int idBest;

	for (int i = 0; i < m; i++) {
		dBest = 1e100;
		idBest = 0;

		for (int j = 0; j < n; j++) {
			d = distanceSSD(f1[i].data, f2[j].data);

			if (d < dBest) {
				dBest = d;
				idBest = f2[j].id;
			}
		}

		matches[i].id1 = f1[i].id;
		matches[i].id2 = idBest;
		matches[i].score = dBest;
		totalScore += matches[i].score;

	}
}

// TODO: Write this function to perform ratio feature matching.  
// This just uses the ratio of the SSD distance of the two best matches as the score
// and matches a feature in the first image with the closest feature in the second image.
// It can match multiple features in the first image to the same feature in
// the second image.  (See class notes for more information, and the sshMatchFeatures function above as a reference)
void ratioMatchFeatures(const FeatureSet &f1, const FeatureSet &f2, vector<FeatureMatch> &matches, double &totalScore)
{

	int m = f1.size();
	int n = f2.size();

	matches.resize(m);

	totalScore = 0;

	double dist;
	double best_dist;
	double second_best_dist;

	int best_idx;
	int second_best_idx;

	for (int i = 0; i < m; i++)
	{
		best_dist = 1e100;
		second_best_dist = 1e100;
		best_idx = 0;
		second_best_idx = 0;

		for (int j = 0; j < n; j++)
		{
			dist = distanceSSD(f1[i].data, f2[j].data);
			if (dist < second_best_dist)
			{
				if (dist < best_dist)
				{
					second_best_dist = best_dist;
					best_dist = dist;

					second_best_idx = best_idx;
					best_idx = f2[j].id;
				}
				else
				{
					second_best_dist = dist;
					second_best_idx = f2[j].id;
				}
			}

		}

		FeatureMatch best_feature;
		best_feature.id1 = f1[i].id;
		best_feature.id2 = best_idx;
		best_feature.score = best_dist / second_best_dist;
		matches[i] = best_feature;

		totalScore += matches[i].score;
	}

}


// Convert Fl_Image to CFloatImage.
bool convertImage(const Fl_Image *image, CFloatImage &convertedImage) {
	if (image == NULL) {
		return false;
	}

	// Let's not handle indexed color images.
	if (image->count() != 1) {
		return false;
	}

	int w = image->w();
	int h = image->h();
	int d = image->d();

	// Get the image data.
	const char *const *data = image->data();

	int index = 0;

	for (int y = 0; y < h; y++) {
		for (int x = 0; x < w; x++) {
			if (d < 3) {
				// If there are fewer than 3 channels, just use the
				// first one for all colors.
				convertedImage.Pixel(x, y, 0) = ((uchar)data[0][index]) / 255.0f;
				convertedImage.Pixel(x, y, 1) = ((uchar)data[0][index]) / 255.0f;
				convertedImage.Pixel(x, y, 2) = ((uchar)data[0][index]) / 255.0f;
			}
			else {
				// Otherwise, use the first 3.
				convertedImage.Pixel(x, y, 0) = ((uchar)data[0][index]) / 255.0f;
				convertedImage.Pixel(x, y, 1) = ((uchar)data[0][index + 1]) / 255.0f;
				convertedImage.Pixel(x, y, 2) = ((uchar)data[0][index + 2]) / 255.0f;
			}

			index += d;
		}
	}

	return true;
}

// Convert CFloatImage to CByteImage.
void convertToByteImage(CFloatImage &floatImage, CByteImage &byteImage) {
	CShape sh = floatImage.Shape();

	assert(floatImage.Shape().nBands == byteImage.Shape().nBands);
	for (int y = 0; y < sh.height; y++) {
		for (int x = 0; x < sh.width; x++) {
			for (int c = 0; c < sh.nBands; c++) {
				float value = floor(255 * floatImage.Pixel(x, y, c) + 0.5f);

				if (value < byteImage.MinVal()) {
					value = byteImage.MinVal();
				}
				else if (value > byteImage.MaxVal()) {
					value = byteImage.MaxVal();
				}

				// We have to flip the image and reverse the color
				// channels to get it to come out right.  How silly!
				byteImage.Pixel(x, sh.height - y - 1, sh.nBands - c - 1) = (uchar)value;
			}
		}
	}
}

// Compute SSD distance between two vectors.
double distanceSSD(const vector<double> &v1, const vector<double> &v2) {
	int m = v1.size();
	int n = v2.size();

	if (m != n) {
		// Here's a big number.
		return 1e100;
	}

	double dist = 0;

	for (int i = 0; i < m; i++) {
		dist += pow(v1[i] - v2[i], 2);
	}


	return sqrt(dist);
}

// Transform point by homography.
void applyHomography(double x, double y, double &xNew, double &yNew, double h[9]) {
	double d = h[6] * x + h[7] * y + h[8];

	xNew = (h[0] * x + h[1] * y + h[2]) / d;
	yNew = (h[3] * x + h[4] * y + h[5]) / d;
}

// Compute AUC given a ROC curve
double computeAUC(vector<ROCPoint> &results)
{
	double auc = 0;
	double xdiff, ydiff;
	for (int i = 1; i < (int)results.size(); i++)
	{
		//fprintf(stream,"%lf\t%lf\t%lf\n",thresholdList[i],results[i].falseRate,results[i].trueRate);
		xdiff = (results[i].falseRate - results[i - 1].falseRate);
		ydiff = (results[i].trueRate - results[i - 1].trueRate);
		auc = auc + xdiff*results[i - 1].trueRate + xdiff*ydiff / 2;

	}
	return auc;
}

void divideUpWork(std::vector<WorkerThread> & workers, int width, int height, int num_threads)
{
	//Divide the blocks up into sections based upon the height of the image.
	int block_size = height / num_threads;
	int start_index = 0;
	int end_index = 0;

	for (int i = 0; i < num_threads - 1; i++)
	{
		end_index += block_size;
		workers.push_back(WorkerThread{ 0, start_index, width, end_index, i });
		start_index = end_index;
	}

	workers.push_back(WorkerThread{ 0, start_index, width, height, num_threads - 1 });

}

void resetIndices(std::vector<WorkerThread> & wtp, int height, int width)
{
	int block_size = height / wtp.size();
	int start_index = 0;
	int end_index = 0;

	for (int i = 0; i < wtp.size() - 1; i++)
	{
		end_index += block_size;
		wtp[i].set_indicies(0, start_index, width, end_index);
		start_index = end_index;
	}
	wtp[wtp.size() - 1].set_indicies(0, start_index, width, height);
}


void filterImage(CFloatImage &rsltImg, CFloatImage &origImg, int imgWidth, int imgHeight, const double* kernel, int knlWidth, int knlHeight, double scale, double offset)
{
	int xMargin = knlWidth / 2 + 2;
	int yMargin = knlHeight / 2 + 2;
	for (int i = xMargin; i < imgWidth - xMargin; i++) {
		for (int j = yMargin; j < imgHeight - yMargin; j++) {
			rsltImg.Pixel(i, j, 0) = 0;
			for (int k = 0; k < knlWidth; k++) {
				for (int l = 0; l < knlHeight; l++) {
					int col = k + i - ((knlHeight - 1) / 2);
					int row = l + j - ((knlWidth - 1) / 2);
					rsltImg.Pixel(i, j, 0) += kernel[l*knlWidth + k] * origImg.Pixel(i + k, j + l, 0);
				}
			}
			rsltImg.Pixel(i, j, 0) /= scale;
			rsltImg.Pixel(i, j, 0) += offset;
		}
	}

}