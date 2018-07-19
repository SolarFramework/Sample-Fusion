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

#ifndef KALMANHELPER_HPP
#define KALMANHELPER_HPP

#include <string>
#include <chrono>

#include "datastructure/MathDefinitions.h"

typedef double T;

using namespace std;
using namespace SolAR;
using namespace SolAR::datastructure;

class KalmanHelper
{
public:
    static void poseToPositionAndOrientation(Transform3Df tr, Vector<T,3>& position, Quaternion<T>& orientation);

    static void kalmanToOpenCVBase(Vector<T, 3>& position, Quaternion<T>& orientation);

    static std::vector<std::string> getNextLineAndSplitIntoTokens(std::istream& str);

    static Transform3Df constructPose(const  Transform3Df &r, const  Vector3f & t);

    static void parseVideoLine(string& line, std::chrono::high_resolution_clock::time_point& timestampVision, Transform3Df& pose, bool& poseValid);

    static void parseIMULine(string& line, std::chrono::high_resolution_clock::duration& timestampIMU, Vector3f & gyro, Vector3f & accel, Vector3f & compass);

    static std::chrono::high_resolution_clock::time_point getTimestamp();

    static void setOffset(std::chrono::nanoseconds offset);

    static std::chrono::nanoseconds getOffset();

private:
    static std::chrono::nanoseconds m_offset;
};

#endif // KALMANHELPER_HPP
