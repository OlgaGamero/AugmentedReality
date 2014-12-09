#ifndef _ARMA_PATTERN_
#define _ARMA_PATTERN_
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
using namespace cv;

namespace AR {

class Pattern
{
	public:
		vector<Point2f> vertices;
		int id;
		int orientation;//{0,1,2,3}
		float size; //in milimeters
		double confidence;//min: -1, max: 1
		Mat rotVec, transVec, rotMat;

		Pattern(double param1=80);

		~Pattern(){};
		
		void linea(int id,int patternSize);
		void cuadrado(int id,int patternSize);

		 
		// resuelve el problema con la orientacion externa entre patron y camara 
		void getExtrinsics(vector<Pattern> patternsFound,int patternSize, const Mat& cameraMatrix, const Mat& distortions);

		//Pinta la imagen en 3d cubo
		void Pattern::draw(Mat& frame, const Mat& camMatrix, const Mat& distMatrix);

		//Cambia el vector de rotacion por el vector de matriz usando Rodrigues 
		void Pattern::rotationMatrix(const Mat& rotation_vector, Mat& rotation_matrix); 
		
		// imprime las propiedades del patron y sus transformaciones 
		void showPattern(void);

		// define los puntos del cuadrado (cubo)
		void Pattern::drawCuadrado(Mat& frame,std::vector<cv::Point2f> model2ImagePts); 

		// define los puntos de las lineas para la resta, suma, cuadrado del reset
		void Pattern::drawLinea(Mat& frame,int id,CvScalar color,std::vector<cv::Point2f> model2ImagePts);

		// muestra texto
		void Pattern::drawText(string label,Mat& frame,int id,std::vector<cv::Point2f> model2ImagePts, int marginX,int marginY); 


};


}

#endif
