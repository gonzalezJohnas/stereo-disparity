//
// Created by jonas on 19/11/2019.
//

#ifndef $KEYWORD_DISPARITYMODULE_H
#define $KEYWORD_DISPARITYMODULE_H

#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/ximgproc.hpp"
#include <iostream>
#include <string>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Network.h>
#include <yarp/sig/Image.h>
#include <yarp/cv/Cv.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Log.h>
#include <yarp/os/Mutex.h>

using namespace cv;
using namespace cv::ximgproc;
using namespace std;
using namespace yarp::os;
using namespace yarp::sig;


class DisparityModule : public yarp::os::RFModule {

    enum {

        STATUS_NONE,
        VOCAB_OK = yarp::os::createVocab('o', 'k'),
        VOCAB_SET = yarp::os::createVocab('s', 'e', 't'),
        VOCAB_GET= yarp::os::createVocab('g', 'e', 't'),
        VOCAB_DISP = yarp::os::createVocab('d', 'i', 's', 'p'),
        VOCAB_WIN = yarp::os::createVocab('w', 'i', 'n'),
        VOCAB_QUIT = yarp::os::createVocab('q', 'u', 'i', 't'),
        VOCAB_FAILED = yarp::os::createVocab('f', 'a', 'i', 'l'),
        VOCAB_HELP = yarp::os::createVocab('h', 'e', 'l', 'p'),

    };

    yarp::os::Mutex mutex;

    BufferedPort<ImageOf<PixelBgr>> imageLeftPortIn;
    BufferedPort<ImageOf<PixelBgr>> imageRightPortIn;

    BufferedPort<ImageOf<PixelMono>> imageDisparityPortOut;

    ImageOf<PixelBgr> *left_img, *right_img;
    int sourceWidth, sourceHeight;

    Mat left_mat, right_mat;

    yarp::os::Port handlerPort; // a port to handle messages

    // stereo disparity parameters
    int numDisparities, windowSize;
    double lambda, sigma;
    string moduleName;
    Ptr<StereoSGBM> left_matcher;
    Ptr<StereoMatcher> right_matcher;
    Ptr<DisparityWLSFilter> wls_filter;
    Mat left_for_matcher, right_for_matcher;
    Mat left_disp, right_disp;
    Mat filtered_disp;
    Mat conf_map;


public:
    double getPeriod();

    // This is our main function. Will be called periodically every getPeriod() seconds
    bool updateModule();

    // Message handler. Just echo all received messages.
    bool respond(const yarp::os::Bottle &command, yarp::os::Bottle &reply);

    // Configure function. Receive a previously initialized
    // resource finder object. Use it to configure your module.
    // If you are migrating from the old module, this is the function
    // equivalent to the "open" method.
    bool configure(yarp::os::ResourceFinder &rf);

    // Interrupt function.
    bool interruptModule();

    // Close function, to perform cleanup.
    bool close();


};


#endif //$KEYWORD_DISPARITYMODULE_H
