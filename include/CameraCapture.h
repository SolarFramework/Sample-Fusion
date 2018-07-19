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

#ifndef CAMERACAPTURE_HPP
#define CAMERACAPTURE_HPP

#include <string>
#include <chrono>

#include "api/input/devices/ICamera.h"

#include "SharedFifo.hpp"

#include <opencv2/videoio.hpp> // VideoWriter
#include <SolAROpenCVHelper.h> //helper

#include "../include/KalmanHelper.h"

using namespace std;
using namespace SolAR;
using namespace SolAR::MODULES::OPENCV;
using namespace SolAR::api;
using namespace SolAR::datastructure;
namespace xpcf  = org::bcom::xpcf;

class CameraCapture
{

public:
    CameraCapture(SRef<input::devices::ICamera> camera,
                  SharedFifo<std::tuple<std::chrono::high_resolution_clock::time_point, SRef<Image>>>& fifoCamera,
                  SharedFifo<SRef<Image>>& fifoVideo);
    ~CameraCapture() = default;

    void saveVideo(string file);

    void videoLogging();

    void videoReplay(string file, std::chrono::high_resolution_clock::time_point& start);

    void stop();

private:
    SRef<input::devices::ICamera> m_camera;

    SharedFifo<std::tuple<std::chrono::high_resolution_clock::time_point, SRef<Image>>> & m_fifoCamera;
    SharedFifo<SRef<Image>> & m_fifoVideo;

    bool m_process;
};

#endif // CAMERACAPTURE_HPP
