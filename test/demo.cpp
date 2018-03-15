#include "common_include.h"

#include "system.h"
#include "config.h"

using namespace std;
using namespace SOFT;

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps);

int main(int argc, char **argv)
{
	//*************************//
	 Eigen::Vector3d a,b,t;
	Eigen::Matrix3d R;
	R<<1,2,3,2,1,2,3,2,1;
	a<<3,9,8;
	t<<10,-3,7;
	b=R.inverse()*a+t;
	std::cout<<R<<endl<<endl;
	std::cout<<a<<endl<<endl;
	std::cout<<b<<endl<<endl;
	//**************************//

    if(argc < 1)
    {
        cerr << endl << "Usage: ./stereo_kitti path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }

    SOFT::Config::setParameterFile("../test/KITTI00-02.yaml");
    // Retrieve paths to images
    vector<string> vstrImageLeft;
    vector<string> vstrImageRight;
    vector<double> vTimestamps;
    LoadImages(string(argv[1]), vstrImageLeft, vstrImageRight, vTimestamps);
    
    const int nImages = vstrImageLeft.size();
    
        // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);
    
    SOFT::System SLAM(1);

    cv::Mat imLeft, imRight;
    for ( int ni=0; ni<nImages; ni++ ) {
        // Read left and right images from file
        imLeft = cv::imread ( vstrImageLeft[ni],0 );
        imRight = cv::imread ( vstrImageRight[ni],0 );
        double tframe = vTimestamps[ni];

        if ( imLeft.empty() ) {
            cerr << endl << "Failed to load image at: "
                 << string ( vstrImageLeft[ni] ) << endl;
            return 1;
        }

        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

		cv::imshow("left",imLeft);
		cv::waitKey(200);
        // Pass the images to the SLAM system
        SLAM.TrackStereo ( imLeft,imRight,tframe );
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> > ( t2 - t1 ).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if ( ni<nImages-1 ) {
            T = vTimestamps[ni+1]-tframe;
        } else if ( ni>0 ) {
            T = tframe-vTimestamps[ni-1];
        }

        if ( ttrack<T ) {
            this_thread::sleep_for ( std::chrono::microseconds ( ( int ) ( ( T-ttrack ) *1e6 ) ) );
        }
    }

}

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps)
{
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);
        }
    }

    string strPrefixLeft = strPathToSequence + "/image_0/";
    string strPrefixRight = strPathToSequence + "/image_1/";

    const int nTimes = vTimestamps.size();
    vstrImageLeft.resize(nTimes);
    vstrImageRight.resize(nTimes);

    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrImageLeft[i] = strPrefixLeft + ss.str() + ".png";
        vstrImageRight[i] = strPrefixRight + ss.str() + ".png";
    }
}