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

#ifndef IMUCAPTURE_HPP
#define IMUCAPTURE_HPP

#include <string>
#include <chrono>

#include "api/input/devices/IIMU.h"

#include "SharedFifo.hpp"

#include "../include/KalmanHelper.h"

using namespace std;
using namespace SolAR;
using namespace SolAR::api;
using namespace SolAR::datastructure;
namespace xpcf  = org::bcom::xpcf;

class IMUCapture
{
public:
    IMUCapture(SRef<input::devices::IIMU> imu,
               SharedFifo<std::tuple<std::chrono::high_resolution_clock::time_point, Vector3f, Vector3f, Vector3f>> & fifoInertial);

    void imuLogging();

    void imuReplay(string file, std::chrono::high_resolution_clock::time_point& start);
    void saveLogs(string file);

    void stop();

private:

    SRef<input::devices::IIMU> m_imu;

    SharedFifo<std::tuple<std::chrono::high_resolution_clock::time_point, Vector3f, Vector3f, Vector3f>> & m_fifoInertial;

    ostringstream m_osIMU;

    bool m_process;
};

#endif // IMUCAPTURE_HPP
