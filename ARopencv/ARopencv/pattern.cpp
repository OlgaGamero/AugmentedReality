#include "pattern.h" 
#include <iostream>
#include <string>
using namespace cv;
using namespace std;

namespace AR {
	 
	CvPoint3D32f puntos[10][4];
	float diferencial =0.0;
	float traslacionX =0.0;
	float traslacionY =0.0; 
	CvScalar colorCubo= cvScalar(255,0,255); 

	Pattern::Pattern(double param1){
		id =-1;
		size = param1;
		orientation = -1;
		confidence = -1; 
		rotVec = (Mat_<float>(3,1) << 0, 0, 0);
		transVec = (Mat_<float>(3,1) << 0, 0, 0);
		rotMat = Mat::eye(3, 3, CV_32F);
	}

	//convierte el vector de rotacion a matriz de rotacion 
	void Pattern::rotationMatrix(const Mat& rotation_vector, Mat& rotation_matrix)
	{
		Rodrigues(rotation_vector, rotation_matrix);		
	}

	void Pattern::showPattern()
	{
		cout << "Pattern ID: " << id << endl;
		cout << "Pattern Size: " << size << endl;
		cout << "Pattern Confedince Value: " << confidence << endl;
		cout << "Pattern Orientation: " << orientation << endl;
		rotationMatrix(rotVec, rotMat);
		cout << "Exterior Matrix (from pattern to camera): " << endl;
		for (int i = 0; i<3; i++){
		cout << rotMat.at<float>(i,0) << "\t" << rotMat.at<float>(i,1) << "\t" << rotMat.at<float>(i,2) << " |\t"<< transVec.at<float>(i,0) << endl;
		}
	}

	bool existePatron(vector<Pattern> patternsFound, int id)
	{
		// 1 : cubo
		// 2 : Aumentar
		// 3 : Disminuir
		// 4 : Reset posicion inicial
		// 5 : Intercambiador de colores
		// 6 : Traslacion izquierda
		// 7 : Traslacion derecha
		// 8 : Traslacion arriba
		// 9 : Traslacion abajo

		for (int i = 0; i<patternsFound.size(); i++){  
			if((patternsFound[i]).id==id)
				return true;
		}
		return false;
	}

	void Pattern::getExtrinsics(vector<Pattern> patternsFound , int patternSize, const Mat& cameraMatrix, const Mat& distortions)
	{

		CvMat objectPts;//header for 3D points of pat3Dpts
		CvMat imagePts;//header for 2D image points of pat2Dpts 
		CvMat intrinsics = cameraMatrix;
		CvMat distCoeff = distortions;
		CvMat rot = rotVec;
		CvMat tra = transVec;
		//CvMat rotationMatrix = rotMat; // projectionMatrix = [rotMat tra];

		CvPoint2D32f pat2DPts[4];
		for (int i = 0; i<4; i++){
			pat2DPts[i].x = this->vertices.at(i).x;
			pat2DPts[i].y = this->vertices.at(i).y;
		}
		 

		if(!existePatron(patternsFound,2) && existePatron(patternsFound,3))  //disminuyo el tamanio del cubo
			diferencial=diferencial+0.5;
	    if(!existePatron(patternsFound,3) && existePatron(patternsFound,2))  //aumento el tamanio del cubo
			diferencial=diferencial-0.5;
 
		if(id==6)// traslacion izquirda
			traslacionX = traslacionX + 3;
		if(id==7)//traslacion derecha
			traslacionX = traslacionX - 3;
		if(id==8)//traslacion arriba
			traslacionY = traslacionY + 1;
		if(id==9)//traslacion abajo
			traslacionY = traslacionY - 1;


		if (id==1) // definir puntos del cubo
			cuadrado(id, patternSize); 
		if (id==2 || id==3 || id==4 || id==5 || id==6 || id==7) // definir puntos de la linea (recta, cruz, cuadrado)
			linea(id, patternSize);
		
		cvInitMatHeader(&objectPts, 4, 3, CV_32FC1, puntos[id-1]);
		cvInitMatHeader(&imagePts, 4, 2, CV_32FC1, pat2DPts);
		
		//find extrinsic parameters
		cvFindExtrinsicCameraParams2(&objectPts, &imagePts, &intrinsics, &distCoeff, &rot, &tra);
	}

 


	void Pattern::linea(int id,int patternSize)
	{

		CvPoint3D32f pat3DPts[4];
		CvPoint2D32f pat2DPts[4];
		if(id==2) //linea horizontal
		{
			pat3DPts[0].x = -patternSize;
			pat3DPts[0].y = -0;
			pat3DPts[0].z = -0;
			pat3DPts[1].x = patternSize;
			pat3DPts[1].y = -patternSize;
			pat3DPts[1].z = -0;
			pat3DPts[2].x = patternSize+patternSize;
			pat3DPts[2].y = patternSize;
			pat3DPts[2].z = -0;
			pat3DPts[3].x = -0;
			pat3DPts[3].y = patternSize+patternSize;
			pat3DPts[3].z = -0; 

		}
		if (id==3) //cruz
		{
			pat3DPts[0].x = -patternSize;
			pat3DPts[0].y = -patternSize;
			pat3DPts[0].z = -0;
			pat3DPts[1].x = patternSize+patternSize;
			pat3DPts[1].y = -patternSize;
			pat3DPts[1].z = -0;
			pat3DPts[2].x = patternSize;
			pat3DPts[2].y = patternSize;
			pat3DPts[2].z = -0;
			pat3DPts[3].x = -0;
			pat3DPts[3].y = patternSize;
			pat3DPts[3].z = -0;
			 
		}

		if(id==4 ||id==5 || id==6 || id==7) //cuadrado de reset 
		{
			pat3DPts[0].x = -0;
			pat3DPts[0].y = -0;
			pat3DPts[0].z = -0;
			pat3DPts[1].x = patternSize;
			pat3DPts[1].y = -0;
			pat3DPts[1].z = -0;
			pat3DPts[2].x = patternSize;
			pat3DPts[2].y = patternSize;
			pat3DPts[2].z = -0;
			pat3DPts[3].x = -0;
			pat3DPts[3].y = patternSize;
			pat3DPts[3].z = -0; 

			if(id==4)
			{
				diferencial = 0;
				traslacionX=0;
				traslacionY=0;
				colorCubo= cvScalar(255,0,255); 
			}
		}

	 
		 	puntos[id-1][0]=pat3DPts[0];
			puntos[id-1][1]=pat3DPts[1];
			puntos[id-1][2]=pat3DPts[2];
			puntos[id-1][3]=pat3DPts[3];
	} 

	void Pattern::cuadrado(int id,int patternSize)
	{
		//puntos del cubo
		CvPoint3D32f pat3DPts[4];
		CvPoint2D32f pat2DPts[4];

 	
		pat3DPts[0].x = -diferencial+traslacionX;
		pat3DPts[0].y = -diferencial+traslacionY;
		pat3DPts[0].z = -diferencial;
		pat3DPts[1].x = patternSize+diferencial+traslacionX;
		pat3DPts[1].y = -diferencial+traslacionY;
		pat3DPts[1].z = -diferencial;
		pat3DPts[2].x = patternSize+diferencial+traslacionX;
		pat3DPts[2].y = patternSize+diferencial;
		pat3DPts[2].z = -diferencial;
		pat3DPts[3].x = -diferencial+traslacionX;
		pat3DPts[3].y = patternSize+diferencial;
		pat3DPts[3].z = -diferencial; 
 
		puntos[id-1][0]=pat3DPts[0];
		puntos[id-1][1]=pat3DPts[1];
		puntos[id-1][2]=pat3DPts[2];
		puntos[id-1][3]=pat3DPts[3];
  
	} 

 

	void Pattern::draw(Mat& frame, const Mat& camMatrix, const Mat& distMatrix)
	{

		CvScalar color = cvScalar(255,255,255);
		
		switch (id){
			case 1:
				 color = colorCubo;
				break;
			case 2:
				 color = cvScalar(51,255,51);
				break;
			case 3:
				 color = cvScalar(0,0,255);
				break;
			case 4:
				 color = cvScalar(255,0,0);
				break;
			case 5:
				//busca de manera aleatoria numeros entre 0 a 255 
				 color = colorCubo = cvScalar(rand()%256,rand()%256,rand()%256);
				break;
		}


		//model 3D points: they must be projected to the image plane
		 Mat modelPts = (Mat_<float>(8,3) << 0, 0, 0, size, 0, 0, size, size, 0, 0, size, 0,0, 0, -size, size, 0, -size, size, size, -size, 0, size, -size );
		 
		std::vector<cv::Point2f> model2ImagePts;
		/* project model 3D points to the image. Points through the transformation matrix 
		(defined by rotVec and transVec) "are transfered" from the pattern CS to the 
		camera CS, and then, points are projected using camera parameters 
		(camera matrix, distortion matrix) from the camera 3D CS to its image plane
		*/
		projectPoints(modelPts, rotVec, transVec, camMatrix, distMatrix, model2ImagePts); 

		if(id==1) //pinta el cubo
		{
			drawCuadrado(frame,model2ImagePts);
			drawText("Realidad Aumentada" , frame,id,model2ImagePts,10,60); 
		}
		if(id==2 || id==3 || id==4) // linea (linea de resta) o cruz (lineas de suma) 
			drawLinea(frame,id,color,model2ImagePts);
		if(id==4) //reset
			drawText("Reset" , frame,id,model2ImagePts,0,0); 
		if(id==5) //intercambaidor de color
			drawText("COLOR" , frame,id,model2ImagePts,-10,-70);
		if(id==6) //intercambaidor de color
			drawText("Izquierda" , frame,id,model2ImagePts,0,0);
		if(id==7) //intercambaidor de color
			drawText("Derecha" , frame,id,model2ImagePts,0,0);

		model2ImagePts.clear();

	}

	//pinta el texto en pantalla
	void Pattern::drawText(string label,Mat& frame,int id,std::vector<cv::Point2f> model2ImagePts,int marginX,int marginY)
	{
		int fontface = cv::FONT_HERSHEY_SIMPLEX;
		double scale = 0.5;
		int thickness = 1;
		int baseline = 0;	

		cv::Size text = cv::getTextSize(label, fontface, scale, thickness, &baseline);
		Point margen=model2ImagePts.at((4)%4);
		margen.x=margen.x-marginX; 
		margen.y=margen.y-marginY;

		
		cv::Scalar colorAux;
		if(id!=5) 
			cv::putText(frame, label, margen, fontface, scale,  CV_RGB(255,255,0) , thickness, 8); 
		else
			cv::putText(frame, label, margen, fontface, 1, colorCubo , 2, 8); 
		

		if(id==1)
		{
			std::ostringstream ss;
			ss << -1*diferencial;
			margen.y=margen.y+20; 
			cv::putText(frame, "Zoom: "+ ss.str() , margen, fontface, scale, CV_RGB(255,255,0), thickness, 8); 
		}
	}

	// pinta lineas (resta, cruz,cuadrado)
	void Pattern::drawLinea(Mat& frame,int id,CvScalar color,std::vector<cv::Point2f> model2ImagePts)
	{

	 	if(id==2)
		{
			cv::line(frame, model2ImagePts.at(0), model2ImagePts.at(2), color, 3);
			cv::line(frame, model2ImagePts.at(1), model2ImagePts.at(3), color, 3);

		}
		if (id==3)
		{
			int i;
			for (i =0; i<2; i++){
				cv::line(frame, model2ImagePts.at(i%2), model2ImagePts.at((i+1)%2), color, 3);
			} 
 
		}
		if (id==4 || id==6 || id==7)
		{
			int i;
			for (i =0; i<4; i++){
				cv::line(frame, model2ImagePts.at(i%4), model2ImagePts.at((i+1)%4), color, 3);
			} 

		}
 
		 
		
	}

	// pinta el cubo
	void Pattern::drawCuadrado(Mat& frame,std::vector<cv::Point2f> model2ImagePts)
	{

		//draw cube, or whatever
		int i;
		for (i =0; i<4; i++){
			cv::line(frame, model2ImagePts.at(i%4), model2ImagePts.at((i+1)%4), colorCubo, 3);
		} 
		for (i =4; i<7; i++){
			cv::line(frame, model2ImagePts.at(i%8), model2ImagePts.at((i+1)%8), colorCubo, 3);
		}
		cv::line(frame, model2ImagePts.at(7), model2ImagePts.at(4), colorCubo, 3);
		for (i =0; i<4; i++){
			cv::line(frame, model2ImagePts.at(i), model2ImagePts.at(i+4), colorCubo, 3);
		}	
		// dibuja la linea que refleja la orientacion. esto indica el lado inferior del patron 
		cv::line(frame, model2ImagePts.at(2), model2ImagePts.at(3), cvScalar(80,255,80), 3);
		
	}

}