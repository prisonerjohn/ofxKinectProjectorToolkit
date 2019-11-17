#include "ofxKinectProjectorToolkit.h"


ofxKinectProjectorToolkit::ofxKinectProjectorToolkit() {
    calibrated = false;
}

void ofxKinectProjectorToolkit::calibrate(const vector<glm::vec3>& pairsKinect,
                                          const vector<glm::vec2>& pairsProjector) {
    int nPairs = pairsKinect.size();
    A.set_size(nPairs*2, 11);
    y.set_size(nPairs*2, 1);
    
    for (int i=0; i<nPairs; i++) {
        A(2*i, 0) = pairsKinect[i].x;
        A(2*i, 1) = pairsKinect[i].y;
        A(2*i, 2) = pairsKinect[i].z;
        A(2*i, 3) = 1;
        A(2*i, 4) = 0;
        A(2*i, 5) = 0;
        A(2*i, 6) = 0;
        A(2*i, 7) = 0;
        A(2*i, 8) = -pairsKinect[i].x * pairsProjector[i].x;
        A(2*i, 9) = -pairsKinect[i].y * pairsProjector[i].x;
        A(2*i, 10) = -pairsKinect[i].z * pairsProjector[i].x;
        
        A(2*i+1, 0) = 0;
        A(2*i+1, 1) = 0;
        A(2*i+1, 2) = 0;
        A(2*i+1, 3) = 0;
        A(2*i+1, 4) = pairsKinect[i].x;
        A(2*i+1, 5) = pairsKinect[i].y;
        A(2*i+1, 6) = pairsKinect[i].z;
        A(2*i+1, 7) = 1;
        A(2*i+1, 8) = -pairsKinect[i].x * pairsProjector[i].y;
        A(2*i+1, 9) = -pairsKinect[i].y * pairsProjector[i].y;
        A(2*i+1, 10) = -pairsKinect[i].z * pairsProjector[i].y;
        
        y(2*i, 0) = pairsProjector[i].x;
        y(2*i+1, 0) = pairsProjector[i].y;
    }
    
    dlib::qr_decomposition<dlib::matrix<double, 0, 11> > qrd(A);
    x = qrd.solve(y);
    calibrated = true;
}

const glm::vec2& ofxKinectProjectorToolkit::getProjectedPoint(const glm::vec3& worldPoint) const {
    float a = x(0, 0)*worldPoint.x + x(1, 0)*worldPoint.y + x(2, 0)*worldPoint.z + x(3,0);
    float b = x(4, 0)*worldPoint.x + x(5, 0)*worldPoint.y + x(6, 0)*worldPoint.z + x(7,0);
    float c = x(8, 0)*worldPoint.x + x(9, 0)*worldPoint.y + x(10, 0)*worldPoint.z + 1;
    glm::vec2 projectedPoint = glm::vec2(a/c, b/c);
    return projectedPoint;
}

vector<double> ofxKinectProjectorToolkit::getCalibration()
{
    vector<double> coefficients;
    for (int i=0; i<11; i++) {
        coefficients.push_back(x(i, 0));
    }
    return coefficients;
}

bool ofxKinectProjectorToolkit::serialize(nlohmann::json& json, const std::string& name) const {
    nlohmann::json jsonCoefficients;
    for (int i = 0; i < 11; ++i)
    {
        jsonCoefficients.push_back(this->x(i, 0));
    }

    nlohmann::json jsonCalibration;
    jsonCalibration["coefficients"] = jsonCoefficients;

    json[name] = jsonCalibration;

    return true;
}

bool ofxKinectProjectorToolkit::deserialize(const nlohmann::json& json, const std::string& name) {
    if (json.count(name) == 0)
    {
        return false;
    }

    const auto jsonCalibration = json[name];
    const auto jsonCoefficients = jsonCalibration["coefficients"];
    for (int i = 0; i < 11; ++i)
    {
        this->x(i, 0) = jsonCoefficients[i];
    }
}

bool ofxKinectProjectorToolkit::saveCalibration(const std::filesystem::path& path) const{
    nlohmann::json json;
    serialize(json);
    return ofSavePrettyJson(path, json);
}

bool ofxKinectProjectorToolkit::loadCalibration(const std::filesystem::path& path) {
    const auto json = ofLoadJson(path);
    if (deserialize(json))
    {
        this->calibrated = true;
        return true;
    }

    return false;
}


