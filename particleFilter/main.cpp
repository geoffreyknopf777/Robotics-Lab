#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include "Util_Map.h"
#include <opencv/highgui.h>
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/legacy/legacy.hpp>
#include "Util.h"
#include "Particle.h"
#include "drive.h"
#include "sonar.h"
#include <math.h>

using namespace cv;
using namespace std;

const int BOXSIZE = 301;
const int ROBOTSIZE = 50;
int angleRot = 0;
int angle = 90;
int NUM_PARTICLES = 30;
int VARIANCE = 5;
int DISTANCE = 5;
std::vector<Particle> vParticles;
std::vector<Point> particlesShoots;

#define drawCross( center, color, d )                  \
line( image, cv::Point( center.x - d, center.y - d ),           \
cv::Point( center.x + d, center.y + d ), color, 2, CV_AA, 0);   \
line( image, cv::Point( center.x + d, center.y - d ),           \
cv::Point( center.x - d, center.y + d ), color, 2, CV_AA, 0 )

void forward (cv::Point& pos) {	
	pos.x -= DISTANCE*cos(((float)angle)*M_PI/180);
	pos.y -= DISTANCE*sin(((float)angle)*M_PI/180);
	for(int i=0; i<NUM_PARTICLES; i++){		
		vParticles[i].moveParticle(angle, -DISTANCE, VARIANCE);
	}
	//cout << "Pos x" << pos.x << endl;
	//cout << "Pos y" << pos.y << endl;
}

Point getIntersectionWithWall(Point pos){
	Point intersection;
	
	//Up
	if(angle == 90){
		intersection.x = pos.x;
		intersection.y = 0;
	}

	//Down
	if(angle == 270){
		intersection.x = pos.x;
		intersection.y = BOXSIZE;
	}

	//Left
	if(angle == 0){
		intersection.x = 0;
		intersection.y = pos.y;
	}

	//Right
	if(angle == 180){
		intersection.x = BOXSIZE;
		intersection.y = pos.y;
	}
	return intersection;
}

void rotate (cv::Mat& src, cv::Mat& dst) {

    angle += angleRot;
    angle = angle % 360;

    if(angleRot >= 360) {
        angleRot -= 360;
    }
    cout << "angleRot = " << angleRot << endl;    
    cout << "angle = " << angle << endl; 

    if(angleRot == 90) {
        flip(src.t(), dst, 1);
    }
    else if(angleRot == 180) {
        flip(src, dst, -1); 
    }
    else if(angleRot == 270) {
        flip(src.t(), dst, 0);
    }
    
}

void updateProbability(std::vector<Particle> &particles, std::vector<cv::Point> &particlesShoot, double distance);
std::vector<Particle> resampleParticles(std::vector<Particle>& oldParticles);

int main (int argc, char * const argv[]) {
    char code;
    bool draw = true;
    Mat image;
    Mat image2;
    image = Mat::zeros(BOXSIZE, BOXSIZE, CV_8UC3);
    image = cv::Scalar::all(0);

    std::vector<Point > edges;
    std::vector<Point > doors;
    std::vector<Point > docks;
    Point arcReactorLoc;
    parseFile(edges,doors,docks,arcReactorLoc);
    drawMap(image, edges, doors, docks, arcReactorLoc);
    cv :: Mat logo = cv :: imread ("ironman_icon.jpg");
    cv::Point pos(ROBOTSIZE, ROBOTSIZE);
    cv :: Mat imageROI;
    imageROI = image (cv :: Rect (pos.x, pos.y, logo.cols, logo.rows));
    logo.copyTo (imageROI);
    cv :: namedWindow("result");
    cv :: imshow ("result", image);
    cv :: waitKey (0);


	//create particleShoots
	for(int i=0; i<NUM_PARTICLES; i++){
		particlesShoots.push_back(Point(0, 0));
	}

	//create particles
	for(int i=0; i<NUM_PARTICLES; i++){
		vParticles.push_back(Particle(pos));
	}

	//initialize sonar
	SonarInit();

	//initialize drive
	DriveController uDrive;

    while(1) {
       
        code = (char)cv::waitKey(100);
               
        if (code == 'f') {
	    forward(pos);   
	    uDrive.DriveDistance(5);    
	}
	else{
	    uDrive.DriveLinear(0);
	}
        
        if (code == 's') {
	    Point robotShoot = getIntersectionWithWall(pos);
	    //int distance = distToEdge(pos, robotShoot);
	    int distance = SonarGetCM();
		printf("\nSonar:%f",distance);
		fflush(stdout);
		for(int i=0; i<NUM_PARTICLES; i++){
                particlesShoots[i] = getIntersectionWithWall(vParticles[i].getPosition());
		cout << "x: " << particlesShoots[i].x << endl;
		cout << "y: " << particlesShoots[i].y << endl;
	    }
	    updateProbability(vParticles, particlesShoots, distance);
            vParticles = resampleParticles(vParticles); 
        }
        
        if (code == 'r') {
            angleRot = 90;
            rotate(logo, logo);
	    uDrive.DriveTurn(90);
        }
        
        if (draw) {
	    image = Mat::zeros(BOXSIZE, BOXSIZE, CV_8UC3); 
            drawMap(image, edges, doors, docks, arcReactorLoc);
	    image2 = image;
            imageROI = image2 (cv :: Rect (pos.x, pos.y, logo.cols, logo.rows));
            logo.copyTo (imageROI);
            //cv :: namedWindow("result");
		for(int i=0; i<=NUM_PARTICLES; i++){	
			drawCross(vParticles[i].getPosition(), 					Scalar(100, 0, 0), 5);
		//cout << "particle x " << vParticles[i].getPosition().x << endl;
		//cout << "particle y " << vParticles[i].getPosition().y << endl;
		}
            cv :: imshow ("result", image2);
        }
        
    }
    
    return 0;
}

void updateProbability(std::vector<Particle> &particles, std::vector<cv::Point> &particlesShoot, double distance) {
    
    float total_probabilities = 0.0;
    float new_weight = 0.0;
    float old_probabilities = 0.0;
    float new_probability = 0.0;
    double map_distance = 0.0;
    double sonar_variance = 10.0;
    
    // update all the particle probabilities
    for (int i=0; i<particles.size(); i++){
        cv::Point pos = particles[i].getPosition();
        
        // use heading to calculate the map distance from particle to wall.
        //map_distance =  distToEdge(direction,cv::Point(pos));
        map_distance = distToEdge(particles[i].getPosition(), particlesShoot[i]);
	//cout << "X" << i << " " << particlesShoot[i].x << endl;
	//cout << "Y" << i << " " << particlesShoot[i].y << endl;
	cout << "DIS" << i << " " << map_distance << endl;
        
        // Compute new probability using measured distance , map distance and sonar variance
        new_probability =  getProbability(distance, sonar_variance, map_distance); //distance by sonar report, sonar variance, given loaction
        // update each probability and compute total probabilities
        old_probabilities = particles[i].getWeight(); //P(robot@location)
        new_weight = old_probabilities * new_probability; //P(Sensor Reading| Robot @ Location) * P(robot@location)
        particles[i].setWeight(new_weight);
        total_probabilities += new_weight; //Ex: 0.25 + 0.25 + 0.3 = 0.8, so N = 1/0.8
    }
    
    
    // Normalize all probabilities
    for (int i=0; i<particles.size(); i++){
        //normalized probability = probability / total probabilities
        particles[i].setWeight(particles[i].getWeight()/total_probabilities); //0.25/0.8 + 0.25/0.8 + 0.3/0.8 = 1
    }
    
}


std::vector<Particle> resampleParticles(std::vector<Particle>& oldParticles) {
    std::vector<Particle> newParticles;
    
    //Calculate a Cumulative Distribution Function for our particle weights
    std::vector<double> CDF;
    CDF.resize(oldParticles.size());
    CDF[0] = ((Particle)oldParticles[0]).getWeight();
    
    for(int i=1; i<CDF.size(); i++)
        CDF[i] = CDF[i-1] + oldParticles[i].getWeight();
    //Loop through our particles as if spinning a roulette wheel.
    //The random u that we start with determines the initial offset
    //and each particle will have a probability of getting landed on and
    //saved proportional to its posterior probability. If a particle has a very large
    //posterior, then it may get landed on many times. If a particle has a very low
    //posterior, then it may not get landed on at all. By incrementing by
    // 1/(numParticles) we ensure that we don't change the number of particles in our
    // returned set.
    
    int i = 0;
    double u = randomDouble()* 1.0/double(oldParticles.size());
    //double u = 1.0/double(oldParticles.size());
    
    
    for(int j=0; j < oldParticles.size(); j++){
        while(u > CDF[i]) //if i particle is very small, we don't want to let it in newparticle, so i++
            i++;
        //cout << "i = " << i << endl;
        
        Particle p = oldParticles[i]; //kill ridiculos particles, and leave possible particles
        p.setWeight(1.0/double(oldParticles.size()));
        newParticles.push_back(p);
        //cout << " particles[" << i << "].x_2 = " << newParticles[i].getPosition().x << endl;
        
        u += 1.0/double(oldParticles.size());
        //cout << "u = " << u << endl;
    }

    
    return newParticles;
}

