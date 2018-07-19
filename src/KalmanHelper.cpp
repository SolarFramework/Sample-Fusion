/**
 * @copyright Copyright 1998-2018, Yost Labs Corporation
 *
 * The Yost 3-Space API is released under the Yost 3-Space Open Source License, which allows for both
 * non-commercial use and commercial use with certain restrictions.
 *
 * For Non-Commercial Use, your use of Covered Works is governed by the GNU GPL v.3, subject to the Yost 3-Space Open
 * Source Licensing Overview and Definitions.
 *
 * For Commercial Use, a Yost Commercial/Redistribution License is required, pursuant to the Yost 3-Space Open Source
 * Licensing Overview and Definitions. Commercial Use, for the purposes of this License, means the use, reproduction
 * and/or Distribution, either directly or indirectly, of the Covered Works or any portion thereof, or a Compilation,
 * Improvement, or Modification, for Pecuniary Gain. A Yost Commercial/Redistribution License may or may not require
 * payment, depending upon the intended use.
 *
 * Full details of the Yost 3-Space Open Source License can be found in LICENSE
 * License also available online at https://yostlabs.com/support/open-source-license/
 */

#include "../include/KalmanHelper.h"

std::chrono::nanoseconds KalmanHelper::m_offset = std::chrono::nanoseconds(0);

std::vector<std::string> KalmanHelper::getNextLineAndSplitIntoTokens(std::istream& str)
{
    std::vector<std::string>   result;
    std::string                line;
    std::getline(str, line);

    std::stringstream          lineStream(line);
    std::string                cell;

    while (std::getline(lineStream, cell, ','))
    {
        result.push_back(cell);
    }
    // This checks for a trailing comma with no data after it.
    if (!lineStream && cell.empty())
    {
        // If there was a trailing comma then add an empty element.
        result.push_back("");
    }
    return result;
}

void KalmanHelper::poseToPositionAndOrientation(Transform3Df tr, Vector<T,3>& position, Quaternion<T>& orientation)
{
    position = Vector<T,3>(tr(0, 3), tr(1, 3), tr(2, 3));

    orientation = Quaternion<T>();
    orientation.w() = std::sqrt(1 + tr(0, 0) + tr(1, 1) + tr(2, 2)) / 2;
    orientation.x() = (tr(2, 1) - tr(1, 2)) / (4 * orientation.w());
    orientation.y() = (tr(0, 2) - tr(2, 0)) / (4 * orientation.w());
    orientation.z() = (tr(1, 0) - tr(0, 1)) / (4 * orientation.w());
}

void KalmanHelper::kalmanToOpenCVBase(Vector<T, 3>& position, Quaternion<T>& orientation)
{
    position(1) = -position(1);

    orientation.w() = -orientation.w();
    orientation.y() = -orientation.y();
}

Transform3Df KalmanHelper::constructPose(const  Transform3Df &r, const  Vector3f & t){

    Transform3Df m_poseTransform;
    m_poseTransform(0,0) =  r(0,0);  m_poseTransform(0,1) = r(0,1);   m_poseTransform(0,2) = r(0,2);
    m_poseTransform(1,0) =  r(1,0);  m_poseTransform(1,1) = r(1,1);   m_poseTransform(1,2) = r(1,2);
    m_poseTransform(2,0) =  r(2,0);  m_poseTransform(2,1) = r(2,1);   m_poseTransform(2,2) = r(2,2);

    m_poseTransform(0,3) =   t[0];
    m_poseTransform(1,3) =   t[1];
    m_poseTransform(2,3) =   t[2];

    m_poseTransform(3,0) = 0; m_poseTransform(3,1) = 0; m_poseTransform(3,2) = 0; m_poseTransform(3,3) = 1;

    return m_poseTransform;
}

void KalmanHelper::parseVideoLine(string& line, std::chrono::high_resolution_clock::time_point& timestampVision, Transform3Df& pose, bool& poseValid)
{
    istringstream str(line);
    vector<string> lineV = getNextLineAndSplitIntoTokens(str);

    long long t;
    Vector<T, 3> position;
    Quaternion<T> orientation;

    t = atoll(lineV[0].c_str());
    position(0) = atof(lineV[1].c_str());
    position(1) = atof(lineV[2].c_str());
    position(2) = atof(lineV[3].c_str());
    orientation.w() = atof(lineV[4].c_str());
    orientation.x() = atof(lineV[5].c_str());
    orientation.y() = atof(lineV[6].c_str());
    orientation.z() = atof(lineV[7].c_str());
    poseValid = (atoi(lineV[8].c_str()) != 0);
    timestampVision = std::chrono::high_resolution_clock::time_point(std::chrono::nanoseconds(t));

    kalmanToOpenCVBase(position, orientation);
    pose = constructPose(Transform3Df(orientation.toRotationMatrix().cast<float>()), position.cast<float>());
    pose = pose.inverse();
}

void KalmanHelper::parseIMULine(string& line, std::chrono::high_resolution_clock::duration& timestampIMU, Vector3f & gyro, Vector3f & accel, Vector3f & compass)
{
    istringstream str(line);
    vector<string> lineV = KalmanHelper::getNextLineAndSplitIntoTokens(str);

    long long t;

    t = atoll(lineV[0].c_str());
    gyro[0] = atof(lineV[1].c_str());
    gyro[1] = atof(lineV[2].c_str());
    gyro[2] = atof(lineV[3].c_str());
    accel[0] = atof(lineV[4].c_str());
    accel[1] = atof(lineV[5].c_str());
    accel[2] = atof(lineV[6].c_str());
    compass[0] = atof(lineV[7].c_str());
    compass[1] = atof(lineV[8].c_str());
    compass[2] = atof(lineV[9].c_str());

    timestampIMU = std::chrono::high_resolution_clock::duration(std::chrono::nanoseconds(t));
}

std::chrono::high_resolution_clock::time_point KalmanHelper::getTimestamp()
{
    return std::chrono::high_resolution_clock::now() + m_offset;
}

void KalmanHelper::setOffset(std::chrono::nanoseconds offset)
{
    m_offset = offset;
}

std::chrono::nanoseconds KalmanHelper::getOffset()
{
    return m_offset;
}
