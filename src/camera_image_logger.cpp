#include <ros/ros.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cvwimage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <signal.h>

using std::cout;
using std::endl;

class CameraListener
{
    public:
        CameraListener(ros::NodeHandle nh, std::string folder, std::string topic);

        void setFolder(const std::string& foldername) { _folder = foldername; }
        std::string getFolder() { return _folder; }
        int getNumMisses() { return nb_consecutive_misses; }
        
        void imageConverter(const sensor_msgs::ImageConstPtr& msg);
        bool takeSnapshot(ros::Time time);

        void main_loop();

    private:
        double SPIN_RATE_;
         ros::NodeHandle nh_;
        int _file;
        cv_bridge::CvImagePtr _current_img;
        bool _is_recorded;
        int nb_consecutive_misses;
        int _id;
        std::string _folder;
        image_transport::Subscriber _sub;
        image_transport::Publisher _pub;
};

CameraListener::CameraListener(ros::NodeHandle nh, std::string folder, std::string topic) : nh_(nh)
{
    cout << "Enter constructer for camera" << endl;

    image_transport::ImageTransport it(nh);
    _is_recorded = false;
    _file = 0;
    nb_consecutive_misses = 0;
    _folder = folder;
    _current_img.reset();
    
    SPIN_RATE_ = 40;

    cout << "start subscriber" << endl;
    _sub = it.subscribe( topic, 1, &CameraListener::imageConverter, this);
}

void CameraListener::imageConverter(const sensor_msgs::ImageConstPtr& msg)
{
    _current_img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    _is_recorded = false;
}

bool CameraListener::takeSnapshot(ros::Time time)
{
    if( !_is_recorded && _current_img.get() != NULL )
    {
        try
        {
            std::stringstream s;
            s << _folder << time.sec << "_" << time.nsec << ".png";
            // s << _folder << _id << "_" << _file++ << ".png";
            cv::imwrite(s.str().c_str(), _current_img->image);
            _is_recorded = true;
            cout.flush();
            cout << "save file : " << _file++ << '\r';
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return false;
        }
    }

    return true;
}

void CameraListener::main_loop()
{
    // Spins until killed
    while (ros::ok())
    {
        // Wait long enough before sending the next one and receiving the running signal
        ros::spinOnce();
        ros::Rate looprate( SPIN_RATE_ );
        takeSnapshot( ros::Time::now() );
        looprate.sleep();
    }
}

//! This needs to be global so the signal handler can use it
static CameraListener* logger=NULL;

//! Signal handler to catch SIGINT (the shutdown command) and attempt to safely shutdown the trajectory interface
void shutdown(int signum)
{
    ROS_WARN("Attempting to shutdown node...");
    if (logger != NULL)
    {
        delete logger;
    }
    else
    {
        ROS_WARN("CameraListener not yet loaded, aborting the load process and shutting down");
    }
    ros::shutdown();
}

//! Main function that spins the ros node
int main(int argc, char** argv)
{
    ROS_INFO("Starting CameraListener action server node...");
    ros::init( argc, argv, "camera_image_logger_node",  ros::init_options::NoSigintHandler);
    ros::NodeHandle node;
    ros::NodeHandle nhp("~");
    std::string folder;
    std::string topic;
    nhp.param(std::string("camera_logger_folder"), folder, std::string("./images/"));
    nhp.param(std::string("camera_logger_topic"), topic, std::string("/usb_cam/image_raw"));

    cout << "use folder : " << folder << endl;
    cout << "use topic : " << topic << endl;

    // ros::ServiceServer service = n.advertiseService("take_snapshot", add);

    // Register a signal handler to safely shutdown the node
    signal(SIGINT, shutdown);
    ROS_INFO("Attempting to start CameraListener node...");
    logger = new CameraListener( node, folder + "/" , topic );
    logger->main_loop();
    return 0;
}


