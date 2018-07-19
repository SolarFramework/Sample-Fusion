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

#ifndef FUSIONPOSE_HPP
#define FUSIONPOSE_HPP

#include <string>
#include <chrono>

#include "api/fusion/IVisualInertialFusion.h"
#include "datastructure/Image.h"

#include "SharedFifo.hpp"

#include "../include/KalmanHelper.h"

using namespace std;
using namespace SolAR;
using namespace SolAR::api;
using namespace SolAR::datastructure;
namespace xpcf  = org::bcom::xpcf;

class FusionPose
{
public:
    FusionPose(SRef<fusion::IVisualInertialFusion> kalman,
               SharedFifo<std::tuple<std::chrono::high_resolution_clock::time_point, Vector3f, Vector3f, Vector3f>> & fifoInertial,
               SharedFifo<std::tuple<std::chrono::high_resolution_clock::time_point, SRef<Image>, Transform3Df, bool>> & fifoVision,
               SharedFifo<std::tuple<std::chrono::high_resolution_clock::time_point, SRef<Image>, Transform3Df, bool>> & fifoPose
    );

    void fusion(std::chrono::high_resolution_clock::time_point& start);
    void stop();
    void saveLogs(string file);

private:
    SRef<fusion::IVisualInertialFusion> m_kalman;

    SharedFifo<std::tuple<std::chrono::high_resolution_clock::time_point, Vector3f, Vector3f, Vector3f>> & m_fifoInertial;
    SharedFifo<std::tuple<std::chrono::high_resolution_clock::time_point, SRef<Image>, Transform3Df, bool>> & m_fifoVision;
    SharedFifo<std::tuple<std::chrono::high_resolution_clock::time_point, SRef<Image>, Transform3Df, bool>> & m_fifoPose;

    ostringstream m_osKalman;

    bool m_process;
};

#endif // FUSIONPOSE_HPP
