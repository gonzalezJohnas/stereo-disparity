//
// Created by jonas on 19/11/2019.
//

#include "../include/iCub/disparityModule.h"





    double DisparityModule::getPeriod()
    {
        // module periodicity (seconds), called implicitly by the module.
        return 0.1;
    }

    bool DisparityModule::updateModule()
    {
            if(imageLeftPortIn.getInputCount() && imageRightPortIn.getInputCount()){
                left_img = imageLeftPortIn.read(true);
                right_img = imageRightPortIn.read(true);



                left_mat = yarp::cv::toCvMat(*left_img);
                right_mat = yarp::cv::toCvMat(*right_img);

                left_for_matcher  = left_mat.clone();
                right_for_matcher = right_mat.clone();




                left_matcher-> compute(left_for_matcher, right_for_matcher,left_disp);
                right_matcher->compute(right_for_matcher,left_for_matcher, right_disp);

                //! [filtering]

                wls_filter->filter(left_disp,left_mat,filtered_disp,right_disp);
                //! [filtering]
                conf_map = wls_filter->getConfidenceMap();

                Mat left_disp_resized;
                resize(left_disp,left_disp_resized,left_mat.size());


                Mat filtered_disp_vis;
                getDisparityVis(filtered_disp,filtered_disp_vis,1);



                if(imageDisparityPortOut.getOutputCount()){
                    ImageOf<PixelMono > &out_disparity = imageDisparityPortOut.prepare();
                    out_disparity = yarp::cv::fromCvMat<PixelMono>(filtered_disp_vis);
                    imageDisparityPortOut.write();
                }


        }

        return true;

    }

    bool DisparityModule::respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply)
    {
        bool ok = false;
        bool rec = false; // is the command recognized?


        reply.clear();

        if (command.get(0).asString()=="quit") {
            reply.addString("quitting");
            return false;
        }


        mutex.lock();
        switch (command.get(0).asVocab()) {
            case VOCAB_HELP:
                rec = true;
                {

                    reply.addVocab(Vocab::encode("many"));
                    reply.addString("(set | get) disp \t: set/get the number of disparities ");
                    reply.addString("(set | get) win \t: set/get the window size ");

                    ok = true;
                }
                break;
            case VOCAB_GET:
                rec = true;
                {
                    if(command.get(1).asVocab() == VOCAB_DISP){
                        reply.addInt(this->numDisparities);
                        ok = true;
                    }

                    else if(command.get(1).asVocab() == VOCAB_WIN){
                        reply.addInt(this->windowSize);
                        ok = true;
                    }

                    else{
                        ok = false;
                    }
                }
                break;
            case VOCAB_SET:
                rec = true;
                {
                    if(command.get(1).asVocab() == VOCAB_DISP){
                        const int received_disp = command.get(2).isNull() ? this->numDisparities : command.get(2).asInt();
                        if( received_disp%16 == 0 && received_disp < sourceWidth){
                            left_matcher->setNumDisparities(received_disp);
                            numDisparities = received_disp;
                            right_matcher = createRightMatcher(left_matcher);
                            wls_filter = createDisparityWLSFilter(left_matcher);
                            wls_filter->setLambda(lambda);
                            wls_filter->setSigmaColor(sigma);


                        }

                        else{
                            reply.addString("You must provided a multiple of 16");

                        }

                        ok = true;
                    }

                    else if(command.get(1).asVocab() == VOCAB_WIN){
                        this->windowSize = command.get(2).isNull() ? this->windowSize : command.get(2).asInt();
                        left_matcher->setBlockSize(windowSize);
                        wls_filter = createDisparityWLSFilter(left_matcher);
                        wls_filter->setLambda(lambda);
                        wls_filter->setSigmaColor(sigma);

                        right_matcher = createRightMatcher(left_matcher);
                        ok = true;
                    }

                    else{
                        ok = false;
                    }
                }
                break;
            default:
                break;
        }
        mutex.unlock();

        if (!rec)
            ok = RFModule::respond(command,reply);

        if (!ok) {
            reply.clear();
            reply.addVocab(VOCAB_FAILED);
        }
        else{
            reply.addVocab(VOCAB_OK);
        }


        return ok;
    }


    bool DisparityModule::configure(yarp::os::ResourceFinder &rf)
    {
        moduleName = rf.check("name", Value("/stereoDisparity"), "module Name (string)").asString();


        sourceWidth = rf.check("width", Value(320), "source width (int)").asInt();
        sourceHeight = rf.check("height", Value(240), "source height(int)").asInt();

        numDisparities = rf.check("numDisparities", Value(16), "number of disparities (int)").asInt();
        windowSize = rf.check("windowSize", Value(7), "window size (int)").asInt();

        sigma = rf.check("sigma", Value(1.5), "sigma value  wls filtering (double)").asDouble();
        lambda = rf.check("lamdba", Value(8000), "lambda value for wls filtering (double)").asDouble();


        if (!handlerPort.open(moduleName))
            return false;

        // optional, attach a port to the module
        // so that messages received from the port are redirected
        // to the respond method
        attach(handlerPort);

        if(!imageLeftPortIn.open(moduleName + "/imageLeft:i")){
            yError("Unable to open port %s%s", moduleName.c_str(), "/imageLeft:i");
            return false;
        }

        if(!imageRightPortIn.open(moduleName + "/imageRight:i")){
            yError("Unable to open port %s%s", moduleName.c_str(), "/imageRight:i");
            return false;
        }

        if(!imageDisparityPortOut.open(moduleName + "/disparity:o")){
            yError("Unable to open port %s%s", moduleName.c_str(), "/disparity:o");
            return false;
        }

        left_matcher= StereoSGBM::create(0,numDisparities,windowSize);
        left_matcher->setP1(24*windowSize*windowSize);
        left_matcher->setP2(96*windowSize*windowSize);
        left_matcher->setPreFilterCap(63);
        left_matcher->setMode(StereoSGBM::MODE_SGBM_3WAY);

        wls_filter = createDisparityWLSFilter(left_matcher);
        wls_filter->setLambda(lambda);
        wls_filter->setSigmaColor(sigma);

        right_matcher = createRightMatcher(left_matcher);

        conf_map = Mat(sourceHeight, sourceWidth,CV_8U);
        conf_map = Scalar(255);

        return true;
    }

    bool DisparityModule::interruptModule()
    {
        yInfo("Interrupting your module, for port cleanup");
        imageLeftPortIn.interrupt();
        imageRightPortIn.interrupt();
        handlerPort.interrupt();

        return true;
    }

    bool DisparityModule::close()
    {
        // optional, close port explicitly
        yInfo("Closing module, Ports");
        imageDisparityPortOut.close();
        imageLeftPortIn.close();
        imageRightPortIn.close();
        handlerPort.close();
        return true;
    }

