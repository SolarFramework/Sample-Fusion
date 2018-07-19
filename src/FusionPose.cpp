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

#include "../include/FusionPose.h"

FusionPose::FusionPose(SRef<fusion::IVisualInertialFusion> kalman,
                       SharedFifo<std::tuple<std::chrono::high_resolution_clock::time_point, Vector3f, Vector3f, Vector3f>> & fifoInertial,
                       SharedFifo<std::tuple<std::chrono::high_resolution_clock::time_point, SRef<Image>, Transform3Df, bool>> & fifoVision,
                       SharedFifo<std::tuple<std::chrono::high_resolution_clock::time_point, SRef<Image>, Transform3Df, bool>> & fifoPose):m_kalman(kalman), m_fifoInertial(fifoInertial), m_fifoVision(fifoVision), m_fifoPose(fifoPose)
{

}

void FusionPose::fusion(std::chrono::high_resolution_clock::time_point& start)
{
    SRef<Image> image;

    std::chrono::high_resolution_clock::time_point timestampVision;
    std::chrono::high_resolution_clock::time_point timestampIMU;
    std::chrono::high_resolution_clock::time_point lastKalman = start;

    m_process = true;

    bool poseValid;

    while (m_process)
    {
        Vector3f gyro, accel, compass;
        Transform3Df pose;

        if(!m_fifoInertial.empty())
        {
            std::tie(timestampIMU, gyro, accel, compass) = m_fifoInertial.pop();

            api::fusion::InertialData id;
            id.gyroData = gyro;
            id.accelData = accel;
            id.magData = compass;
            id.timestamp = timestampIMU;

            m_kalman->addInertialData(id);
        }

        if(!m_fifoVision.empty())
        {
            std::tie(timestampVision, image, pose, poseValid) = m_fifoVision.pop();

            api::fusion::VisionData vd;
            vd.pose = pose;
            vd.isPoseValid = poseValid;
            vd.timestamp = timestampVision;

            m_kalman->addVisionData(vd);
            Transform3Df fusionPose;
            if(m_kalman->process(fusionPose) == FrameworkReturnCode::_SUCCESS)
            {
                m_fifoPose.push(std::tuple<std::chrono::high_resolution_clock::time_point, SRef<Image>, Transform3Df, bool>(timestampVision, image, fusionPose, true));

                //log estimated pose
                Vector<T,3> position;
                Quaternion<T> orientation;
                Transform3Df poseInverse = fusionPose.inverse();
                KalmanHelper::poseToPositionAndOrientation(poseInverse, position, orientation);
                KalmanHelper::kalmanToOpenCVBase(position, orientation);
                m_osKalman << timestampVision.time_since_epoch().count() << ", "
                           << position(0) << ", " << position(1) << ", " << position(2) << ", "
                           << orientation.w() << ", " << orientation.x() << ", " << orientation.y() << ", " << orientation.z() << endl;
            }
        }
    }
}

void FusionPose::stop()
{
    m_process = false;
}

void FusionPose::saveLogs(string file)
{
    ofstream kalmanFile;

    kalmanFile.open(file);
    kalmanFile << m_osKalman.str();
    kalmanFile.close();
}
