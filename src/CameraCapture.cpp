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

#include "../include/CameraCapture.h"

CameraCapture::CameraCapture(SRef<input::devices::ICamera> camera,
                             SharedFifo<std::tuple<std::chrono::high_resolution_clock::time_point, SRef<Image>>> & fifoCamera,
                             SharedFifo<SRef<Image>> & fifoVideo):m_camera(camera), m_fifoCamera(fifoCamera), m_fifoVideo(fifoVideo)
{

}

void CameraCapture::saveVideo(string file)
{
    cv::VideoWriter outputVideo;
    SRef<Image> image;
    cv::Mat imgDest;

    double fps = 30.0;

    CvSize size = cvSize((int)640,
        (int)480);

    outputVideo.open(file, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), fps, size, 1);

    while (!m_fifoVideo.empty()) {
        image = m_fifoVideo.pop();
        SolAROpenCVHelper::mapToOpenCV(image, imgDest);
        outputVideo << imgDest;
    }

    outputVideo.release();
}

void CameraCapture::videoLogging()
{
    std::chrono::high_resolution_clock::time_point timestamp;

    m_process = true;

    while (m_process) {
        SRef<Image> inputImage;

        if (m_camera->getNextImage(inputImage) == SolAR::FrameworkReturnCode::_ERROR_)
        {
            m_process = false;
            return;
        }

        timestamp = KalmanHelper::getTimestamp();

        m_fifoCamera.push(std::pair<std::chrono::high_resolution_clock::time_point, SRef<Image>>(timestamp, inputImage));
        m_fifoVideo.push(inputImage->copy());
    }
}

void CameraCapture::videoReplay(string file, std::chrono::high_resolution_clock::time_point& start)
{
    ifstream infile(file);
    string sLine;

    std::chrono::high_resolution_clock::time_point lastTimestamp = start;

    if (!infile.good())
    {
        LOG_ERROR("File with url {} does not exist", file);
        return;
    }

    double fps = 30;

    m_process = true;

    while (!infile.eof() && m_process)
    {
        SRef<Image> inputImage;

        getline(infile, sLine);
        if (sLine.empty())
            break;

        std::chrono::high_resolution_clock::time_point timestamp;
        Transform3Df pose;
        bool poseValid;

        KalmanHelper::parseVideoLine(sLine, timestamp, pose, poseValid);

        if (m_camera->getNextImage(inputImage) == SolAR::FrameworkReturnCode::_ERROR_)
        {
            m_process = false;
            return;
        }

        //std::this_thread::sleep_for(timestamp - lastTimestamp);

        m_fifoCamera.push(std::pair<std::chrono::high_resolution_clock::time_point, SRef<Image>>(timestamp, inputImage));

        lastTimestamp = timestamp;
         //  cv::waitKey(0) : 0 // To control video input (Esc to go to the next frame)
    }

    infile.close();
    m_process = false;
}

void CameraCapture::stop()
{
    m_process = false;
}
