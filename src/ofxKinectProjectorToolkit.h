#pragma once

#include "ofMain.h"
#include <matrix/matrix.h>
#include <matrix/matrix_qr.h>


class ofxKinectProjectorToolkit
{
public:
    ofxKinectProjectorToolkit();
    
    void calibrate(const std::vector<glm::vec3>& pairsKinect,
                   const std::vector<glm::vec2>& pairsProjector);
    
    const glm::vec2& getProjectedPoint(const glm::vec3& worldPoint) const;
    
    std::vector<double> getCalibration();

    bool serialize(nlohmann::json& json, const std::string& name = "calibration") const;
	bool deserialize(const nlohmann::json& json, const std::string& name = "calibration");

	bool saveCalibration(const std::filesystem::path& path) const;
	bool loadCalibration(const std::filesystem::path& path);
    
    bool isCalibrated() {return calibrated;}

private:
    dlib::matrix<double, 0, 11> A;
    dlib::matrix<double, 0, 1> y;
    dlib::matrix<double, 11, 1> x;
    
    bool calibrated;
};