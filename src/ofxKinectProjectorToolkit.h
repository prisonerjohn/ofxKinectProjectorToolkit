#pragma once

#include "ofMain.h"
#include "matrix.h"
#include "matrix_qr.h"


class ofxKinectProjectorToolkit
{
public:
    ofxKinectProjectorToolkit();
    
    void calibrate(const vector<glm::vec3>& pairsKinect,
                   const vector<glm::vec2>& pairsProjector);
    
    const glm::vec2& getProjectedPoint(const glm::vec3& worldPoint) const;
    
    vector<double> getCalibration();

    void loadCalibration(string path);
    void saveCalibration(string path);
    
    bool isCalibrated() {return calibrated;}
    
private:
    
    dlib::matrix<double, 0, 11> A;
    dlib::matrix<double, 0, 1> y;
    dlib::matrix<double, 11, 1> x;
    
    bool calibrated;
};