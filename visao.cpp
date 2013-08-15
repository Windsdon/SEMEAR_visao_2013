#include "cv.h"
#include "highgui.h"

#include <iostream>
#include <stdio.h>
#include "ros/ros.h"
#include "std_msgs/UInt16.h"
#include <sstream>
#include <math.h>
#include "ieee/topic.h"

#include<time.h>


using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
	//Configurando ROs
  	ros::init(argc, argv, "visao");
  	ros::NodeHandle n;
  	ros::Publisher chatter_pub = n.advertise<ieee::topic>("topic", 1000);
  	ros::Rate loop_rate(10);
	ieee::topic posLata;

	//Configurando do OpenCV
	VideoCapture cap(0);
	    	if(!cap.isOpened()) return -1;

	//Variaveis auxiliares
	Mat frame, hsv, binario;
	vector<vector<Point> > contours;
	int posicao=0, maior;
	//vector<Mat> hsv_planos;

	//Rotina
	while (ros::ok())	
	{
		cap >> frame;
		cvtColor(frame, hsv, CV_BGR2HSV);
		GaussianBlur(hsv, hsv, Size(11,11), 0, 0);
		inRange(hsv, Scalar(110, 70, 70), Scalar(130, 255, 255), binario);
		erode(binario, binario, Mat());
		findContours(binario.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
//findContours(binario, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

		Mat contornos = Mat::zeros(frame.size(), frame.type());
		
		drawContours(contornos, contours, -1, Scalar::all(255), CV_FILLED);

		/// Get the moments
  		vector<Moments> mu(contours.size() );

  		for (int i=0; i<contours.size(); i++)
     		{
			mu[i] = moments(contours[i], false);
		}
		
		///  Get the mass centers:
  		vector<Point2f> mc(contours.size());
  		
		for (int i=0; i<contours.size(); i++)
     		{
			mc[i] = Point2f(mu[i].m10/mu[i].m00, mu[i].m01/mu[i].m00);
		}
		

		maior=0;

		for( int i = 0; i<contours.size(); i++ )
     		{
			if (contourArea(contours[i])>100)       			
				circle (frame, mc[i],(unsigned int) 1.3*sqrt(contourArea(contours[i])/3.14), Scalar(0, 0, 255), 4, 8, 0 );
			if (contourArea(contours[i])>contourArea(contours[maior])) maior=i;
     		}
	
		if(contours.size()!= 0 && contourArea(contours[maior])>100)
			posicao=mc[maior].x;
		else
			posicao=700;

		if(posicao > 700) posicao = 700;
		else if(posicao < 0) posicao = -1;
    			posLata.pos = posicao;
    		//std::stringstream ss;

			
    		//ROS_INFO("centro %d T: %d", pos.data, contours.size()/*.c_str()*/);
		chatter_pub.publish( posLata);

    		ros::spinOnce();

    		loop_rate.sleep();

		imshow("webcam", frame);
		imshow("tratado", binario);
		imshow("contornos", contornos);
        	if(waitKey(30) >= 0) break;
  	}


	
	return 0;
}
