#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp> 
#include "patterndetector.h"
#include <windows.h> 
 
using namespace std;
using namespace cv;
using namespace AR; 

#define PAT_SIZE 64//igual a la variable del patron
#define SAVE_VIDEO 0 //1 para salvar el video en formato "output.avi" 

char* filename1="pattern1.png";//id=1 cubo
char* filename2="pattern2.png";//id=2 aumento dimensiones
char* filename3="pattern3.png";//id=3 disminuyo dimensiones
char* filename4="pattern4.png";//id=4 reset del cubo
char* filename5="pattern5.png";//id=5 cambio de colores
char* filename6="pattern6.png";//id=6 izquierda
char* filename7="pattern7.png";//id=7 derecha 

static int loadPattern(const char* , std::vector<cv::Mat>& , int& );

int main(int argc, char** argv){

	std::vector<cv::Mat> patternLibrary;
	std::vector<Pattern> detectedPattern;
	int patternCount=0;

	/*create patterns' library using rotated versions of patterns 
	*/
	loadPattern(filename1, patternLibrary, patternCount);
	loadPattern(filename2, patternLibrary, patternCount);
	loadPattern(filename3, patternLibrary, patternCount); 
	loadPattern(filename4, patternLibrary, patternCount); 
	loadPattern(filename5, patternLibrary, patternCount);
	loadPattern(filename6, patternLibrary, patternCount);
	loadPattern(filename7, patternLibrary, patternCount); 


	cout << patternCount << " patrones cargados." << endl;

	int norm_pattern_size = PAT_SIZE;
	double fixed_thresh = 40;
	double adapt_thresh = 5;//non-used with FIXED_THRESHOLD mode
	int adapt_block_size = 45;//non-used with FIXED_THRESHOLD mode
	double confidenceThreshold = 0.35;
	int mode = 2;//1:FIXED_THRESHOLD, 2: ADAPTIVE_THRESHOLD

	PatternDetector myDetector( fixed_thresh, adapt_thresh, adapt_block_size, confidenceThreshold, norm_pattern_size, mode);

	CvCapture* capture = cvCaptureFromCAM(0);

#if (SAVE_VIDEO)
	CvVideoWriter *video_writer = cvCreateVideoWriter( "demo.avi", -1, 5, cvSize(640,480) );
#endif

	Mat imgMat;

	CvMat* intrinsic = (CvMat*)cvLoad("intrinsic.xml");	
	CvMat* distor = (CvMat*)cvLoad("distortion.xml");

	Mat cameraMatrix = cvarrToMat(intrinsic);
	Mat distortions = cvarrToMat(distor);

	while(true){ // infinito hasta presionar ESC o cerrar terminal
		
		IplImage* img = cvQueryFrame(capture);
		Mat imgMat = Mat(img);
		double tic=(double)cvGetTickCount();

		//se corre el detector de patrones
		myDetector.detect(imgMat, cameraMatrix, distortions, patternLibrary, detectedPattern); 

		double toc=(double)cvGetTickCount();
		double detectionTime = (toc-tic)/((double) cvGetTickFrequency()*1000);
		cout << "Detected Patterns: " << detectedPattern.size() << endl;
		cout << "Detection time: " << detectionTime << endl;

		//augment the input frame (and print out the properties of pattern if you want)
		for (unsigned int i =0; i<detectedPattern.size(); i++){
			//detectedPattern.at(i).showPattern();
			detectedPattern.at(i).draw( imgMat, cameraMatrix, distortions);
		}

#if (SAVE_VIDEO)
		cvWriteFrame(video_writer, &((IplImage) imgMat));
#endif
		imshow("Realidad Aumentada   (Pulse ESC para salir)", imgMat);
		cvWaitKey(1);
	 
		detectedPattern.clear();

		if( cvWaitKey(100) == 27 ) break;
	}

#if (SAVE_VIDEO)
	cvReleaseVideoWriter(&video_writer);
#endif
	cvReleaseCapture(&capture);

	return 0;


}

int loadPattern(const char* filename, std::vector<cv::Mat>& library, int& patternCount){
	Mat img = imread(filename,0);
	
	if(img.cols!=img.rows){
		return -1;
		printf("Patron no detectado");
	}

	int msize = PAT_SIZE; 

	Mat src(msize, msize, CV_8UC1);
	Point2f center((msize-1)/2.0f,(msize-1)/2.0f);
	Mat rot_mat(2,3,CV_32F);
	
	resize(img, src, Size(msize,msize));
	Mat subImg = src(Range(msize/4,3*msize/4), Range(msize/4,3*msize/4));
	library.push_back(subImg);

	rot_mat = getRotationMatrix2D( center, 90, 1.0);

	for (int i=1; i<4; i++){
		Mat dst= Mat(msize, msize, CV_8UC1);
		rot_mat = getRotationMatrix2D( center, -i*90, 1.0);
		warpAffine( src, dst , rot_mat, Size(msize,msize));
		Mat subImg = dst(Range(msize/4,3*msize/4), Range(msize/4,3*msize/4));
		library.push_back(subImg);	
	}

	patternCount++;
	return 1;
}

