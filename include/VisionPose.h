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

#ifndef VISIONPOSE_HPP
#define VISIONPOSE_HPP

#include "api/input/files/IMarker2DSquaredBinary.h"
#include "api/display/IImageViewer.h"
#include "api/image/IImageFilter.h"
#include "api/image/IImageConvertor.h"
#include "api/features/IContoursExtractor.h"
#include "api/features/IContoursFilter.h"
#include "api/image/IPerspectiveController.h"
#include "api/features/IDescriptorsExtractorSBPattern.h"
#include "api/features/IDescriptorMatcher.h"
#include "api/features/ISBPatternReIndexer.h"
#include "api/geom/IImage2WorldMapper.h"
#include "api/solver/pose/I3DTransformFinder.h"
#include "api/display/I3DOverlay.h"
#include "api/display/I2DOverlay.h"

#include "xpcf/api/IConfigurable.h"

#include "SharedFifo.hpp"

using namespace std;
using namespace SolAR;
using namespace SolAR::api;
using namespace SolAR::datastructure;
namespace xpcf  = org::bcom::xpcf;

#include "../include/KalmanHelper.h"

using namespace SolAR::datastructure;

struct VisionPoseStruct
{
    SRef<input::files::IMarker2DSquaredBinary>      binaryMarker;
    SRef<display::IImageViewer>                     imageViewer;
    SRef<display::IImageViewer>                     imageViewerGrey;
    SRef<display::IImageViewer>                     imageViewerBinary;
    SRef<display::IImageViewer>                     imageViewerContours;
    SRef<display::IImageViewer>                     imageViewerFilteredContours;
    SRef<image::IImageFilter>                       imageFilter;
    SRef<xpcf::IConfigurable>                       rIConfigurable_imageFilterBinary;
    SRef<image::IImageConvertor>                    imageConvertor;
    SRef<features::IContoursExtractor>              contoursExtractor;
    SRef<features::IContoursFilter>                 contoursFilter;
    SRef<image::IPerspectiveController>             perspectiveController;
    SRef<features::IDescriptorsExtractorSBPattern>  patternDescriptorExtractor;
    SRef<features::IDescriptorMatcher>              patternMatcher;
    SRef<features::ISBPatternReIndexer>             patternReIndexer;
    SRef<geom::IImage2WorldMapper>                  img2worldMapper;
    SRef<solver::pose::I3DTransformFinder>          PnP;
    SRef<display::I3DOverlay>                       overlay3D;
    SRef<display::I2DOverlay>                       overlay2D;
    SRef<DescriptorBuffer>                          markerPatternDescriptor;

    VisionPoseStruct(){}

    VisionPoseStruct(const VisionPoseStruct& other){
        binaryMarker = other.binaryMarker;
        imageViewer = other.imageViewer;
        imageViewerGrey = other.imageViewerGrey;
        imageViewerBinary = other.imageViewerBinary;
        imageViewerContours = other.imageViewerContours;
        imageViewerFilteredContours = other.imageViewerFilteredContours;
        imageFilter = other.imageFilter;
        rIConfigurable_imageFilterBinary = other.rIConfigurable_imageFilterBinary;
        imageConvertor = other.imageConvertor;
        contoursExtractor = other.contoursExtractor;
        contoursFilter = other.contoursFilter;
        perspectiveController = other.perspectiveController;
        patternDescriptorExtractor = other.patternDescriptorExtractor;
        patternMatcher = other.patternMatcher;
        patternReIndexer = other.patternReIndexer;
        img2worldMapper = other.img2worldMapper;
        PnP = other.PnP;
        overlay3D = other.overlay3D;
        overlay2D = other.overlay2D;
        markerPatternDescriptor = other.markerPatternDescriptor;
    }
};

class VisionPose
{
public:
    VisionPose(const VisionPoseStruct & visionPoseStruct,
               SharedFifo<std::tuple<std::chrono::high_resolution_clock::time_point, SRef<Image>>> & fifoCamera,
               SharedFifo<std::tuple<std::chrono::high_resolution_clock::time_point, SRef<Image>, Transform3Df, bool>> & fifoVision);

    void estimatePose();

    void saveLogs(string file);

    void stop();

    Transform3Df getLastPose();

private:
    std::vector<std::string> getNextLineAndSplitIntoTokens(std::istream& str);
    bool processImage(SRef<Image>& image, Transform3Df& pose);

    VisionPoseStruct m_vps;

    Transform3Df m_lastPose;

    SharedFifo<std::tuple<std::chrono::high_resolution_clock::time_point, SRef<Image>>> & m_fifoCamera;
    SharedFifo<std::tuple<std::chrono::high_resolution_clock::time_point, SRef<Image>, Transform3Df, bool>> & m_fifoVision;

    ostringstream m_osVideo;

    bool m_process;
};

#endif // VISIONPOSE_HPP
