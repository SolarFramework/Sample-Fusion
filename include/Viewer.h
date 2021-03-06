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

#ifndef VIEWER_H
#define VIEWER_H

#include <chrono>

#include "api/input/files/IMarker2DSquaredBinary.h"
#include "api/display/IImageViewer.h"
#include "api/display/I3DOverlay.h"

#include "SharedBuffer.hpp"

#include "../include/KalmanHelper.h"

using namespace std;
using namespace SolAR;
using namespace SolAR::api;
using namespace SolAR::datastructure;
namespace xpcf  = org::bcom::xpcf;

class Viewer
{
public:
    Viewer(SRef<display::IImageViewer> imageViewer,
           SRef<display::I3DOverlay> overlay3D,
           SRef<input::files::IMarker2DSquaredBinary> binaryMarker,
           SharedFifo<std::tuple<std::chrono::high_resolution_clock::time_point, SRef<Image>, Transform3Df, bool>> & fifoPose);

    void display(const char* exitKey);

private:
    SRef<display::IImageViewer> m_imageViewer;
    SRef<display::I3DOverlay> m_overlay3D;
    SRef<input::files::IMarker2DSquaredBinary> m_binaryMarker;

    SharedFifo<std::tuple<std::chrono::high_resolution_clock::time_point, SRef<Image>, Transform3Df, bool>> & m_fifoPose;

    bool m_process;
};

#endif // VIEWER_H
