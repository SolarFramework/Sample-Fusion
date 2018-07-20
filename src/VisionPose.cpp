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

#include "../include/VisionPose.h"

VisionPose::VisionPose(const VisionPoseStruct& visionPoseStruct,
                       SharedFifo<std::tuple<std::chrono::high_resolution_clock::time_point, SRef<Image>>> & fifoCamera,
                       SharedFifo<std::tuple<std::chrono::high_resolution_clock::time_point, SRef<Image>, Transform3Df, bool>> & fifoVision): m_fifoCamera(fifoCamera), m_fifoVision(fifoVision)
{
    m_vps = VisionPoseStruct(visionPoseStruct);
}

std::vector<std::string> VisionPose::getNextLineAndSplitIntoTokens(std::istream& str)
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

bool VisionPose::processImage(SRef<Image>& image, Transform3Df& pose) {
    SRef<Image> greyImage;
    SRef<Image> binaryImage;
    SRef<Image> contoursImage;
    SRef<Image> filteredContoursImage;

    std::vector<SRef<Contour2Df>>              contours;
    std::vector<SRef<Contour2Df>>              filtered_contours;
    std::vector<SRef<Image>>                   patches;
    std::vector<SRef<Contour2Df>>              recognizedContours;
    SRef<DescriptorBuffer>                     recognizedPatternsDescriptors;
    std::vector<DescriptorMatch>               patternMatches;
    std::vector<SRef<Point2Df>>                pattern2DPoints;
    std::vector<SRef<Point2Df>>                img2DPoints;
    std::vector<SRef<Point3Df>>                pattern3DPoints;

    bool success = false;

    // Convert Image from RGB to grey
    m_vps.imageConvertor->convert(image, greyImage, Image::ImageLayout::LAYOUT_GREY);

    // Convert Image from grey to black and white
    auto imageFilterBinary_property=m_vps.rIConfigurable_imageFilterBinary->getProperty("min");
    imageFilterBinary_property->setIntegerValue(-1);
    imageFilterBinary_property=m_vps.rIConfigurable_imageFilterBinary->getProperty("max");
    imageFilterBinary_property->setIntegerValue(255);
    m_vps.imageFilter->filter(greyImage, binaryImage);

    // Extract contours from binary image
    m_vps.contoursExtractor->extract(binaryImage, contours);

    // Filter 4 edges contours to find those candidate for marker contours
    m_vps.contoursFilter->filter(contours, filtered_contours);

    // Create one warpped and cropped image by contour
    m_vps.perspectiveController->correct(binaryImage, filtered_contours, patches);

    // test if this last image is really a squared binary marker, and if it is the case, extract its descriptor
    if (m_vps.patternDescriptorExtractor->extract(patches, filtered_contours, recognizedPatternsDescriptors, recognizedContours) != FrameworkReturnCode::_ERROR_)
    {
        // From extracted squared binary pattern, match the one corresponding to the squared binary marker
        if (m_vps.patternMatcher->match(m_vps.markerPatternDescriptor, recognizedPatternsDescriptors, patternMatches) == features::DescriptorMatcher::DESCRIPTORS_MATCHER_OK)
        {
            // Reindex the pattern to create two vector of points, the first one corresponding to marker corner, the second one corresponding to the poitsn of the contour
            m_vps.patternReIndexer->reindex(recognizedContours, patternMatches, pattern2DPoints, img2DPoints);

            // Compute the 3D position of each corner of the marker
            m_vps.img2worldMapper->map(pattern2DPoints, pattern3DPoints);

            // Compute the pose of the camera using a Perspective n Points algorithm using only the 4 corners of the marker
            if (m_vps.PnP->estimate(img2DPoints, pattern3DPoints, pose) == FrameworkReturnCode::_SUCCESS)
            {
                success = true;
                m_lastPose = Transform3Df(pose);
            }
        }
    }

    return success;
}

void VisionPose::estimatePose()
{
    std::chrono::high_resolution_clock::time_point timestamp;

    m_process = true;

    while (m_process)
    {
        if (!m_fifoCamera.empty())
        {
            SRef<Image> image;
            bool success;

            std::tie(timestamp, image) = m_fifoCamera.pop();

            Transform3Df pose;
            success = processImage(image, pose);

            m_fifoVision.push(std::tuple<std::chrono::high_resolution_clock::time_point, SRef<Image>, Transform3Df, bool>(timestamp, image, pose, success));

            Vector<T, 3> position;
            Quaternion<T> orientation;

            Transform3Df poseInverse = pose.inverse();
            KalmanHelper::poseToPositionAndOrientation(poseInverse, position, orientation);
            KalmanHelper::kalmanToOpenCVBase(position, orientation);
            m_osVideo << timestamp.time_since_epoch().count() << ", " << position(0) << ", " << position(1) << ", " << position(2) << ", " <<
                orientation.w() << ", " << orientation.x() << ", " << orientation.y() << ", " << orientation.z() << ", " << success << endl;
        }
   }
}

void VisionPose::saveLogs(string file)
{
    ofstream videoFile;

    videoFile.open(file);
    videoFile << m_osVideo.str();
    videoFile.close();
}

void VisionPose::stop()
{
    m_process = false;
}

Transform3Df VisionPose::getLastPose()
{
    return m_lastPose;
}
