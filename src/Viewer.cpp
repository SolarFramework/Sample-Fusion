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

#include "../include/Viewer.h"

Viewer::Viewer(SRef<display::IImageViewer> imageViewer,
               SRef<display::I3DOverlay> overlay3D,
               SRef<input::files::IMarker2DSquaredBinary> binaryMarker,
               SharedFifo<std::tuple<std::chrono::high_resolution_clock::time_point, SRef<Image>, Transform3Df, bool>> & fifoPose):m_imageViewer(imageViewer), m_overlay3D(overlay3D), m_binaryMarker(binaryMarker), m_fifoPose(fifoPose)
{

}

void Viewer::display(const char *exitKey)
{
    SRef<Image> image;
    Transform3Df pose;
    std::chrono::high_resolution_clock::time_point timestamp;
    bool success;

    m_process = true;

    while(m_process)
    {
        if(!m_fifoPose.empty())
        {
            std::tie(timestamp, image, pose, success) = m_fifoPose.pop();

            if(success)
            {
                Transform3Df cubeTransform = Transform3Df::Identity();
                m_overlay3D->drawBox(pose, m_binaryMarker->getWidth(), m_binaryMarker->getHeight(), m_binaryMarker->getHeight(), cubeTransform, image);
            }
        }
        if (image != NULL)
        {
            if (m_imageViewer->display("original image", image, exitKey) == FrameworkReturnCode::_STOP)
            {
                m_process = false;
                return;
            }
        }
    }
}
