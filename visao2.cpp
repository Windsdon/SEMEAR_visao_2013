#include "cv.h"
#include "highgui.h"

#include <iostream>
#include <stdio.h>
#include "ros/ros.h"
#include <sstream>
#include <math.h>
#include "ieee/topic.h"


using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
	//Inicialização de variáveis	
	int agua,i,j, n_pontos;
	ieee::topic temAgua;	

	std::stringstream ss;

	//Configuracao ROS
  	ros::init(argc, argv, "visao2");							//Incia o no com "visao2" 
  	ros::NodeHandle visao2;									//Cria um controlador de nó (visao2)
  	ros::Publisher chatter_pub = visao2.advertise<ieee::topic>("topic", 1000);	//Cria um publicador de mensagens (chatter_pub)
  	ros::Rate loop_rate(10);								//Frequencia de atualização do rOS (EM Hz)
		

	//Configuracoes OpenCv - Inicialização de variáveis
	VideoCapture cap(1);
    		if(!cap.isOpened()) return -1;
	

	//Variáveis auxiliares no qual trataremos a imagem obtida pela webcam
	Mat  frame, aux, binario;
					/*frame   - é o frame atual  pego da webcam
         				 aux 	  - variável no qual será tratada a imagem obtida para conseguir contornos
	  				 binario - variável no qual tem o resultado do processamento de imagem
					*/

	vector<vector<Point> > contours; //Variável que possui os contonos encontrados

	//Rotina de execução do robo
	while (ros::ok()){									//Enquanto ros estiver funcionando
		//Captura e tratamento de imagem		

		cap >> frame;
		cvtColor(frame, aux, CV_BGR2HSV);

		GaussianBlur(aux, aux, Size(11,11), 0, 0);			//aux recebe: Borra a imagem (diminuir ruido)
		erode(aux,aux, Mat());						//Tambem diminuir ruido		
		inRange(aux, Scalar(90,0,0), Scalar(120,255,255), binario);	//binário recebe: todos os pontos que estao entre o range de cores
//Scalar para HSV funciona   H: 0-180, S:0-255, V:0-255



		findContours(binario.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
		agua = 0;		
		for(int y = 0; y < binario.rows/2; y++)
		{
		    for(int x = 0; x < binario.cols/3; x++)
		    {
				//circle(binario, cvPoint(x+binario.cols/3,y), 1 ,Scalar(255,255,255), 1,1,0);
			if (binario.at<uchar>(x+binario.cols/3,y+binario.rows/2) == 255)
			   agua+=1;
		    }
		}



//ROS_INFO("total: %d, agua: %d", ((binario.cols/3)*(binario.rows/2)), agua);


    		//temAgua.isTrue = agua;							    //agua é uma variavel que esta na MENSAGEM water.msg
		if(agua >= ((binario.cols/3)*(binario.rows/2))*0.7)
			agua = 0;			
		else
			agua = 1;
		temAgua.isTrue = agua;
    		chatter_pub.publish(temAgua);							//Publica mensagem para water.msg

		//Função do rOS para enviar mensagem e esperar um pouco
    		ros::spinOnce();
    		loop_rate.sleep();

		imshow("webcam", frame);
		imshow("tratado", binario);
		imshow("aux", aux);
        	if(waitKey(30) >= 0) break;



  	}





	
	return 0;
}
