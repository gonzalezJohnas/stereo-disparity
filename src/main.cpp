#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>

#include "../include/iCub/disparityModule.h"





int main(int argc, char** argv) {

    yarp::os::Network::init();

    DisparityModule module;

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile("disparity.ini"); //overridden by --from parameter
    rf.setDefaultContext("stereo-disparity");    //overridden by --context parameter
    rf.configure(argc, argv);

    module.runModule(rf);

    return 0;

}