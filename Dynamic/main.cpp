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

#include "datastructure/MathDefinitions.h"

#include <boost/log/core.hpp>
#include <boost/timer/timer.hpp>
#include <boost/chrono.hpp>
#include <boost/thread.hpp>

#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <map>

#include <chrono>
#include <ratio>
#include "SharedFifo.hpp"
#include <tuple> // std::tuple

// ADD COMPONENTS HEADERS HERE, e.g #include "SolarComponent.h"

#include "SolARModuleOpencv_traits.h"

#include "SolARModuleTools_traits.h"

#include "ModuleIMUYostLabs_traits.h"

#include "SolARModuleFusion_traits.h"

#include "xpcf/api/IComponentManager.h"


#include "api/input/devices/ICamera.h"
#include "api/input/devices/IIMU.h"
#include "api/input/files/IMarker2DSquaredBinary.h"
#include "api/display/IImageViewer.h"
#include "api/image/IImageFilter.h"
#include "api/image/IImageConvertor.h"
#include "api/features/IContoursExtractor.h"
#include "api/features/IContoursFilter.h"
#include "api/fusion/IVisualInertialFusion.h"
#include "api/image/IPerspectiveController.h"
#include "api/features/IDescriptorsExtractorSBPattern.h"
#include "api/features/IDescriptorMatcher.h"
#include "api/features/ISBPatternReIndexer.h"
#include "api/geom/IImage2WorldMapper.h"
#include "api/solver/pose/I3DTransformFinder.h"
#include "api/display/I3DOverlay.h"
#include "api/display/I2DOverlay.h"

using namespace std;
using namespace SolAR;
using namespace SolAR::MODULES::OPENCV;
using namespace SolAR::MODULES::TOOLS;
using namespace SolAR::MODULES::IMUYOSTLABS;
using namespace SolAR::MODULES::FUSION;
using namespace SolAR::api;
using namespace SolAR::datastructure;
namespace xpcf  = org::bcom::xpcf;

#include "../include/KalmanHelper.h"
#include "../include/CameraCapture.h"
#include "../include/IMUCapture.h"
#include "../include/VisionPose.h"
#include "../include/FusionPose.h"
#include "../include/Viewer.h"

int printHelp() {
    printf(" usage :\n");
    printf(" exe exe FiducialMarkerFilename CameraCalibrationFile VideoFile|cameraId\n\n");
    printf(" Escape key to exit");
    return 1;
}

int main(int argc, char *argv[])
{

#if NDEBUG
    boost::log::core::get()->set_logging_enabled(false);
#endif

    /* instantiate component manager*/
    /* this is needed in dynamic mode */
    SRef<xpcf::IComponentManager> xpcfComponentManager = xpcf::getComponentManagerInstance();

    if(xpcfComponentManager->load("$BCOMDEVROOT/.xpcf/SolAR/", true)!=org::bcom::xpcf::_SUCCESS){
        LOG_ERROR("XPCF library load has failed")
        return -1;
    }

    /* create a file logger*/
    //LOG_ADD_LOG_TO_FILE("SolarLog.log","r");

    LOG_ADD_LOG_TO_CONSOLE();

    if (argc != 3 && argc != 4) {
        printHelp();
        getchar();
        return -1;
    }

    LOG_INFO("Program is running");

    SRef<Image> inputImage;
    SRef<Image> greyImage;
    SRef<Image> binaryImage;
    SRef<Image> contoursImage;
    SRef<Image> filteredContoursImage;

    std::vector<SRef<Contour2Df>>              contours;
    std::vector<SRef<Contour2Df>>              filtered_contours;
    std::vector<SRef<Image>>                   patches;
    std::vector<SRef<Contour2Df>>              recognizedContours;
    SRef<DescriptorBuffer>                     recognizedPatternsDescriptors;
    SRef<DescriptorBuffer>                     markerPatternDescriptor;
    std::vector<DescriptorMatch>               patternMatches;
    std::vector<SRef<Point2Df>>                pattern2DPoints;
    std::vector<SRef<Point2Df>>                img2DPoints;
    std::vector<SRef<Point3Df>>                pattern3DPoints;
    Transform3Df							   pose;
    // The Intrinsic parameters of the camera
    CamCalibration K;
    // The escape key to exit the sample
    char escape_key = 27;
    char enter_key = 13;
    // color used to draw contours
    std::vector<unsigned int> bgr{ 128, 128, 128 };

    bool isReplay = false;

    SharedFifo<std::tuple<std::chrono::high_resolution_clock::time_point, Vector3f, Vector3f, Vector3f>> fifoInertial; //contains inertial measurements from IMU
    SharedFifo<std::tuple<std::chrono::high_resolution_clock::time_point, SRef<Image>>> fifoCamera; //contains images from camera
    SharedFifo<std::tuple<std::chrono::high_resolution_clock::time_point, SRef<Image>, Transform3Df, bool>> fifoVision; //contains vision-based pose estimation
    SharedFifo<std::tuple<std::chrono::high_resolution_clock::time_point, SRef<Image>, Transform3Df, bool>> fifoPose; //contains fusion-based pose estimation
    SharedFifo<SRef<Image>> fifoVideo; //contains copied images for video-saving

    std::chrono::high_resolution_clock::time_point start;

    ostringstream osLogging;

    /* create components  */
    auto camera =xpcfComponentManager->create<SolARCameraOpencv>()->bindTo<input::devices::ICamera>();
    auto binaryMarker =xpcfComponentManager->create<SolARMarker2DSquaredBinaryOpencv>()->bindTo<input::files::IMarker2DSquaredBinary>();
    auto imageViewer =xpcfComponentManager->create<SolARImageViewerOpencv>()->bindTo<display::IImageViewer>();
    auto imageViewerGrey =xpcfComponentManager->create<SolARImageViewerOpencv>()->bindTo<display::IImageViewer>();
    auto imageViewerBinary =xpcfComponentManager->create<SolARImageViewerOpencv>()->bindTo<display::IImageViewer>();
    auto imageViewerContours =xpcfComponentManager->create<SolARImageViewerOpencv>()->bindTo<display::IImageViewer>();
    auto imageViewerFilteredContours =xpcfComponentManager->create<SolARImageViewerOpencv>()->bindTo<display::IImageViewer>();
    auto imageFilter =xpcfComponentManager->create<SolARImageFilterBinaryOpencv>()->bindTo<image::IImageFilter>();
    auto rIConfigurable_imageFilterBinary = imageFilter->bindTo<xpcf::IConfigurable>();
    auto imageConvertor =xpcfComponentManager->create<SolARImageConvertorOpencv>()->bindTo<image::IImageConvertor>();
    auto contoursExtractor =xpcfComponentManager->create<SolARContoursExtractorOpencv>()->bindTo<features::IContoursExtractor>();
    auto contoursFilter =xpcfComponentManager->create<SolARContoursFilterBinaryMarkerOpencv>()->bindTo<features::IContoursFilter>();
    auto perspectiveController =xpcfComponentManager->create<SolARPerspectiveControllerOpencv>()->bindTo<image::IPerspectiveController>();
    auto patternDescriptorExtractor =xpcfComponentManager->create<SolARDescriptorsExtractorSBPatternOpencv>()->bindTo<features::IDescriptorsExtractorSBPattern>();
    auto patternMatcher =xpcfComponentManager->create<SolARDescriptorMatcherRadiusOpencv>()->bindTo<features::IDescriptorMatcher>();
    auto patternReIndexer = xpcfComponentManager->create<SolARSBPatternReIndexer>()->bindTo<features::ISBPatternReIndexer>();
    auto img2worldMapper = xpcfComponentManager->create<SolARImage2WorldMapper4Marker2D>()->bindTo<geom::IImage2WorldMapper>();
    auto PnP =xpcfComponentManager->create<SolARPoseEstimationPnpOpencv>()->bindTo<solver::pose::I3DTransformFinder>();
    auto overlay3D =xpcfComponentManager->create<SolAR3DOverlayOpencv>()->bindTo<display::I3DOverlay>();
    auto overlay2D =xpcfComponentManager->create<SolAR2DOverlayOpencv>()->bindTo<display::I2DOverlay>();
    auto imu =xpcfComponentManager->create<IMUYostLabsStream>()->bindTo<input::devices::IIMU>();
    auto kalman =xpcfComponentManager->create<SolARVisualInertialEKF>()->bindTo<api::fusion::IVisualInertialFusion>();

    LOG_INFO("All components have been created");

    binaryMarker->loadMarker(argv[1]);
    patternDescriptorExtractor->extract(binaryMarker->getPattern(), markerPatternDescriptor);

    int minContourSize = 4;
    contoursExtractor->setParameters(minContourSize);

    int minContourLength = 20;
    contoursFilter->setParameters(minContourLength);

    Sizei CorrectedImagesSize = { 640, 480 };
    perspectiveController->setParameters(CorrectedImagesSize);

    int patternSize = binaryMarker->getPattern()->getSize();
    patternDescriptorExtractor->setParameters(patternSize);

    patternReIndexer->setParameters(patternSize);

    Sizei sbPatternSize;
    sbPatternSize.width = patternSize;
    sbPatternSize.height = patternSize;
    img2worldMapper->setParameters(sbPatternSize, binaryMarker->getSize());

    //int maximalDistanceToMatch = 0;
    //patternMatcher->setParameters(maximalDistanceToMatch);

    std::string cameraArg = std::string(argv[3]);
    if (cameraArg.find("mp4") != std::string::npos || cameraArg.find("wmv") != std::string::npos || cameraArg.find("avi") != std::string::npos)
    {
        if (camera->start(argv[3]) != FrameworkReturnCode::_SUCCESS) // videoFile
        {
            LOG_ERROR("Video with url {} does not exist", argv[3]);
            getchar();
            return -1;
        }

        isReplay = true; // TODO here init IMU with a file to read
    }
    else
    {
        if (camera->start(atoi(argv[3])) != FrameworkReturnCode::_SUCCESS) // Camera
        {
            LOG_ERROR("Camera with id {} does not exist", argv[3]);
            getchar();
            return -1;
        }

        if (imu->start() != FrameworkReturnCode::_SUCCESS) { // IMU
            getchar();
            return -1;
        }
    }

    //Load camera parameters and start it
    if (camera->loadCameraParameters(argv[2]) != FrameworkReturnCode::_SUCCESS){
        {
            LOG_ERROR ("camera calibration file {} does not exist", argv[2]);
            return -1;
        }
    }

    PnP->setCameraParameters(camera->getIntrinsicsParameters(), camera->getDistorsionParameters());
    overlay3D->setCameraParameters(camera->getIntrinsicsParameters(), camera->getDistorsionParameters());


    //Prepare structure for vision pose Task
    VisionPoseStruct vps;
    vps.binaryMarker = binaryMarker;
    vps.imageViewer = imageViewer;
    vps.imageViewerGrey = imageViewerGrey;
    vps.imageViewerBinary = imageViewerBinary;
    vps.imageViewerContours = imageViewerContours;
    vps.imageViewerFilteredContours = imageViewerFilteredContours;
    vps.imageFilter = imageFilter;
    vps.rIConfigurable_imageFilterBinary = rIConfigurable_imageFilterBinary;
    vps.imageConvertor = imageConvertor;
    vps.contoursExtractor = contoursExtractor;
    vps.contoursFilter = contoursFilter;
    vps.perspectiveController = perspectiveController;
    vps.patternDescriptorExtractor = patternDescriptorExtractor;
    vps.patternMatcher = patternMatcher;
    vps.patternReIndexer = patternReIndexer;
    vps.img2worldMapper = img2worldMapper;
    vps.PnP = PnP;
    vps.overlay3D = overlay3D;
    vps.overlay2D = overlay2D;
    vps.markerPatternDescriptor = markerPatternDescriptor;

    if(!isReplay)
    {
        LOG_INFO("Initialization... press enter to capture first pose");

        KalmanHelper::setOffset(std::chrono::system_clock::now().time_since_epoch() - std::chrono::steady_clock::now().time_since_epoch());

        SharedFifo<std::tuple<std::chrono::high_resolution_clock::time_point, SRef<Image>>> fifoCameraInit;
        SharedFifo<std::tuple<std::chrono::high_resolution_clock::time_point, SRef<Image>, Transform3Df, bool>> fifoVisionInit;
        SharedFifo<SRef<Image>> fifoVideoInit;

        CameraCapture camCap(camera, fifoCameraInit, fifoVideoInit);
        VisionPose visPos(vps, fifoCameraInit, fifoVisionInit);
        Viewer viewer(imageViewer, overlay3D, binaryMarker, fifoVisionInit);

        thread t1 = thread(&CameraCapture::videoLogging, &camCap);
        thread t2 = thread(&VisionPose::estimatePose, &visPos);

        viewer.display(&enter_key);

        pose = visPos.getLastPose();

        visPos.stop();
        camCap.stop();

        t2.join();
        t1.join();
    }else{
        //here read the first pose from log file

        ifstream infile("logs/log.txt");

        bool poseValid;

        if (infile.good())
        {
            string sLine;
            getline(infile, sLine);
            if (sLine.empty())
            {
                getchar();
                return -1;
            }

            KalmanHelper::parseVideoLine(sLine, start, pose, poseValid);
        }
        else
        {
            LOG_ERROR("File with url {} does not exist", "log.txt");
            getchar();
            return -1;
        }

        infile.close();
    }

    SolAR::api::fusion::VisionData initialPose;
    initialPose.pose = pose;

    IMUCapture imuCap(imu, fifoInertial);
    CameraCapture camCap(camera, fifoCamera, fifoVideo);
    VisionPose visPos(vps, fifoCamera, fifoVision);
    FusionPose fusPos(kalman, fifoInertial, fifoVision, fifoPose);
    Viewer viewer(imageViewer, overlay3D, binaryMarker, fifoPose);

    LOG_INFO("Fusion started... press Esc to stop");

    thread t1, t2, t3, t4;

    if(!isReplay)
    {
        t1 = thread([&](CameraCapture* camCap){camCap->videoLogging();}, &camCap);
        t2 = thread([&](IMUCapture* imuCap){imuCap->imuLogging();}, &imuCap);

        Vector<T, 3> position;
        Quaternion<T> orientation;
        Transform3Df poseInverse = initialPose.pose.inverse();
        KalmanHelper::poseToPositionAndOrientation(poseInverse, position, orientation);
        KalmanHelper::kalmanToOpenCVBase(position, orientation);

        start = KalmanHelper::getTimestamp();
        osLogging << start.time_since_epoch().count() << ", " << position(0) << ", " << position(1) << ", " << position(2) << ", " <<
                    orientation.w() << ", " << orientation.x() << ", " << orientation.y() << ", " << orientation.z() << ", " << 1 << endl;
    }
    else
    {
        t1 = thread([&](CameraCapture* camCap){camCap->videoReplay("logs/logVideo.txt", start);}, &camCap);
        t2 = thread([&](IMUCapture* imuCap){imuCap->imuReplay("logs/logIMU.txt", start);}, &imuCap);
    }

    initialPose.timestamp = start;

    kalman->init(initialPose);

    t3 = thread([&](VisionPose* visPos){visPos->estimatePose();}, &visPos);
    t4 = thread([&](FusionPose* fusPos){fusPos->fusion(start);}, &fusPos);

    viewer.display(&escape_key);

    fusPos.stop();
    visPos.stop();
    imuCap.stop();
    camCap.stop();

    t4.join();
    t3.join();
    t2.join();
    t1.join();

    if(!isReplay)
    {
        camCap.saveVideo("logs/videoOutput.avi");
        imuCap.saveLogs("logs/logIMU.txt");
        visPos.saveLogs("logs/logVideo.txt");
        fusPos.saveLogs("logs/logKalman.txt");

        ofstream logFile;

        logFile.open("logs/log.txt");
        logFile << osLogging.str();
        logFile.close();

        LOG_INFO("Logs saved");
    }

    LOG_INFO("End of program");
    return 0;
}
