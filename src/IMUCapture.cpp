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

#include "../include/IMUCapture.h"

IMUCapture::IMUCapture(SRef<input::devices::IIMU> imu,
                       SharedFifo<std::tuple<std::chrono::high_resolution_clock::time_point, Vector3f, Vector3f, Vector3f>> & fifoInertial):m_imu(imu), m_fifoInertial(fifoInertial)
{

}

void IMUCapture::imuLogging()
{
    std::chrono::high_resolution_clock::time_point timestamp;

    m_process = true;

    while (m_process) {
        Vector3f gyro;
        Vector3f accel;
        Vector3f compass;

        if (m_imu->getAllSensorsData(gyro, accel, compass) != FrameworkReturnCode::_SUCCESS) {
            m_process = false;
            return;
        }

        timestamp = KalmanHelper::getTimestamp();

        m_osIMU << timestamp.time_since_epoch().count() << ", " << gyro[0] << ", " << gyro[1] << ", " << gyro[2] << ", " << accel[0] << ", " << accel[1] << ", " << accel[2] << ", " << compass[0] << ", " << compass[1] << ", " << compass[2] << endl;

        m_fifoInertial.push(std::tuple<std::chrono::high_resolution_clock::time_point, Vector3f, Vector3f, Vector3f>(timestamp, gyro, accel, compass));
    }
}

void IMUCapture::imuReplay(string file, std::chrono::high_resolution_clock::time_point& start)
{
    ifstream infile(file);
    string sLine;

    std::chrono::high_resolution_clock::time_point lastTimestamp = start;

    if (!infile.good())
    {
        LOG_ERROR("File with url {} does not exist", file);
        return;
    }

    m_process = true;

    while (!infile.eof() && m_process)
    {
        Vector3f gyro;
        Vector3f accel;
        Vector3f compass;

        getline(infile, sLine);
        if (sLine.empty())
            break;

        istringstream str(sLine);
        vector<string> line = KalmanHelper::getNextLineAndSplitIntoTokens(str);

        std::chrono::high_resolution_clock::time_point timestamp;
        long long t;

        t = atoll(line[0].c_str());
        gyro[0] = atof(line[1].c_str());
        gyro[1] = atof(line[2].c_str());
        gyro[2] = atof(line[3].c_str());
        accel[0] = atof(line[4].c_str());
        accel[1] = atof(line[5].c_str());
        accel[2] = atof(line[6].c_str());
        compass[0] = atof(line[7].c_str());
        compass[1] = atof(line[8].c_str());
        compass[2] = atof(line[9].c_str());

        timestamp = std::chrono::high_resolution_clock::time_point(std::chrono::nanoseconds(t));

        //std::this_thread::sleep_for(timestamp - lastTimestamp);

        m_fifoInertial.push(std::tuple<std::chrono::high_resolution_clock::time_point, Vector3f, Vector3f, Vector3f>(timestamp, gyro, accel, compass));

        lastTimestamp = timestamp;
    }

    infile.close();
}

void IMUCapture::saveLogs(string file)
{
    ofstream imuFile;

    imuFile.open(file);
    imuFile << m_osIMU.str();
    imuFile.close();
}

void IMUCapture::stop()
{
    m_process = false;
}
