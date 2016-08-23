#include <stdio.h>
#include <iostream>
#include <wiringSerial.h>
#include <wiringPi.h>
#include <bitset>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include <errno.h>
#include <math.h>


using namespace std;
using namespace cv;

float rad_degree = M_PI*2/360;
int fd  = serialOpen("/dev/ttyAMA0", 115200);

int getDist() {
	bitset<14> distance;

	int readingChar = serialGetchar(fd);
	bitset<8> readingBit = bitset<8>(readingChar);
        distance[0] = readingBit[0];
        distance[1] = readingBit[1];
        distance[2] = readingBit[2];
        distance[3] = readingBit[3];
        distance[4] = readingBit[4];
        distance[5] = readingBit[5];
        distance[6] = readingBit[6];
        distance[7] = readingBit[7];

        readingChar = serialGetchar(fd);
        readingBit = bitset<8>(readingChar);
        distance[8] = readingBit[0];
        distance[9] = readingBit[1];
        distance[10] = readingBit[2];
        distance[11] = readingBit[3];
        distance[12] = readingBit[4];
        distance[13] = readingBit[5];

	//signalStrength
        serialGetchar(fd);
        serialGetchar(fd);

	return (int)(distance.to_ulong())/10;
}

float getAngle(int indexChar) {
	int angle = indexChar - 159;
	float rad = rad_degree * angle * 4;
	return rad;
}

void addPoint(Mat image, float rad, int dist) {
	if (dist == -1) {
		return;
	}
	float x = 250 - (dist * cos(rad));
        float y = 250 + (dist * sin(rad));
        if(x<500 && x>0 && y<500 && y>0){
        	image.at<Vec3b>(y,x)[1] = 0;
        }
}

void readPackage(Mat image) {
	int indexChar = serialGetchar(fd);
	float startRad = getAngle(indexChar);

        int speedLChar = serialGetchar(fd);
        int speedHChar = serialGetchar(fd);

	int dist0 = getDist();
	int dist1 = getDist();
	int dist2 = getDist();
	int dist3 = getDist();

	int checkSumL = serialGetchar(fd);
	int checkSumH = serialGetchar(fd);

	addPoint(image, startRad, dist0);
	addPoint(image, startRad + rad_degree, dist1);
	addPoint(image, startRad + rad_degree*2, dist2);
	addPoint(image, startRad + rad_degree*3, dist3);
}

int  main(){
	cout << "running ..." << endl;

	//variable til tegning
	Mat image (500,500, CV_8UC3, Scalar(200,200,200));
   	namedWindow( "win");
	int y;
    	int x;
    	float rad;
    	float rad_degree = 6.29/360;
    	int dist = 1;

	//variable til behandling af signal fra lidar
	int readingChar;
	int angle = 0;
	int tempOutput; 
	bitset<8> readingBit;
	bitset<14> distance;

	//start seriel forbindelse
	if (fd <0){
		fprintf(stderr, "Kan ikke aabne serial");
		return 1;
	}
	if(wiringPiSetup() == -1){
		fprintf(stdout, "Kan ikke starte WiPi");
		return 1;
	}

	int i = 0;
	//loop:  if char== 250 -> loop 22 gange
	for(;;){
		//read from stream
		readingChar = serialGetchar(fd);
                readingBit = bitset<8>(readingChar);
		//looking for starting byte value
		if(readingChar == 250){
			readPackage(image);
			i++;
                }

		//efter en omgang: tegn billede igen
		if (i>90){
			imshow( "win", image);
			waitKey(1);
			i = 0;
			image = Scalar(200,200,200);
		}
	}
	return 0;
}

