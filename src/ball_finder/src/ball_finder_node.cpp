#include "ros/ros.h"
#include "tf/tf.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point32.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"


ros::Publisher vis_pub;
ros::Publisher pose_pub;
ros::Publisher ball_pub;

double computeXVal(double x1, double x2, double y1, double y2, double R, bool type) {
  if (type) {
    return (x1 + x2 + (x1*pow(y1, 2)) /
      pow(x1 - x2, 2) +
      (x2*pow(y1, 2)) / pow(x1 - x2, 2) -
      (2 * x1*y1*y2) /
      pow(x1 - x2, 2) -
      (2 * x2*y1*y2) /
      pow(x1 - x2, 2) +
      (x1*pow(y2, 2)) / pow(x1 - x2, 2) +
      (x2*pow(y2, 2)) / pow(x1 - x2, 2) -
      sqrt(pow(-x1 - x2 -
      (x1*pow(y1, 2)) /
      pow(x1 - x2, 2) -
      (x2*pow(y1, 2)) /
      pow(x1 - x2, 2) +
      (2 * x1*y1*y2) /
      pow(x1 - x2, 2) +
      (2 * x2*y1*y2) /
      pow(x1 - x2, 2) -
      (x1*pow(y2, 2)) /
      pow(x1 - x2, 2) -
      (x2*pow(y2, 2)) /
      pow(x1 - x2, 2), 2) -
      4 * (1 + pow(y1, 2) / pow(x1 - x2, 2) -
      (2 * y1*y2) / pow(x1 - x2, 2) +
      pow(y2, 2) / pow(x1 - x2, 2))*
      (-pow(R, 2) + pow(x1, 2) / 2. + pow(x2, 2) / 2. +
      pow(y1, 2) / 4. +
      (pow(x1, 2)*pow(y1, 2)) /
      (4.*pow(x1 - x2, 2)) +
      (x1*x2*pow(y1, 2)) /
      (2.*pow(x1 - x2, 2)) +
      (pow(x2, 2)*pow(y1, 2)) /
      (4.*pow(x1 - x2, 2)) -
      (y1*y2) / 2. -
      (pow(x1, 2)*y1*y2) /
      (2.*pow(x1 - x2, 2)) -
      (x1*x2*y1*y2) /
      pow(x1 - x2, 2) -
      (pow(x2, 2)*y1*y2) /
      (2.*pow(x1 - x2, 2)) + pow(y2, 2) / 4. +
      (pow(x1, 2)*pow(y2, 2)) /
      (4.*pow(x1 - x2, 2)) +
      (x1*x2*pow(y2, 2)) /
      (2.*pow(x1 - x2, 2)) +
      (pow(x2, 2)*pow(y2, 2)) /
      (4.*pow(x1 - x2, 2))))) /
      (2.*(1 + pow(y1, 2) / pow(x1 - x2, 2) -
      (2 * y1*y2) / pow(x1 - x2, 2) +
      pow(y2, 2) / pow(x1 - x2, 2)));
  }
  else {
    return (x1 + x2 + (x1*pow(y1, 2)) /
      pow(x1 - x2, 2) +
      (x2*pow(y1, 2)) / pow(x1 - x2, 2) -
      (2 * x1*y1*y2) /
      pow(x1 - x2, 2) -
      (2 * x2*y1*y2) /
      pow(x1 - x2, 2) +
      (x1*pow(y2, 2)) / pow(x1 - x2, 2) +
      (x2*pow(y2, 2)) / pow(x1 - x2, 2) +
      sqrt(pow(-x1 - x2 -
      (x1*pow(y1, 2)) /
      pow(x1 - x2, 2) -
      (x2*pow(y1, 2)) /
      pow(x1 - x2, 2) +
      (2 * x1*y1*y2) /
      pow(x1 - x2, 2) +
      (2 * x2*y1*y2) /
      pow(x1 - x2, 2) -
      (x1*pow(y2, 2)) /
      pow(x1 - x2, 2) -
      (x2*pow(y2, 2)) /
      pow(x1 - x2, 2), 2) -
      4 * (1 + pow(y1, 2) / pow(x1 - x2, 2) -
      (2 * y1*y2) / pow(x1 - x2, 2) +
      pow(y2, 2) / pow(x1 - x2, 2))*
      (-pow(R, 2) + pow(x1, 2) / 2. + pow(x2, 2) / 2. +
      pow(y1, 2) / 4. +
      (pow(x1, 2)*pow(y1, 2)) /
      (4.*pow(x1 - x2, 2)) +
      (x1*x2*pow(y1, 2)) /
      (2.*pow(x1 - x2, 2)) +
      (pow(x2, 2)*pow(y1, 2)) /
      (4.*pow(x1 - x2, 2)) -
      (y1*y2) / 2. -
      (pow(x1, 2)*y1*y2) /
      (2.*pow(x1 - x2, 2)) -
      (x1*x2*y1*y2) /
      pow(x1 - x2, 2) -
      (pow(x2, 2)*y1*y2) /
      (2.*pow(x1 - x2, 2)) + pow(y2, 2) / 4. +
      (pow(x1, 2)*pow(y2, 2)) /
      (4.*pow(x1 - x2, 2)) +
      (x1*x2*pow(y2, 2)) /
      (2.*pow(x1 - x2, 2)) +
      (pow(x2, 2)*pow(y2, 2)) /
      (4.*pow(x1 - x2, 2))))) /
      (2.*(1 + pow(y1, 2) / pow(x1 - x2, 2) -
      (2 * y1*y2) / pow(x1 - x2, 2) +
      pow(y2, 2) / pow(x1 - x2, 2)));
  }
}

double computeYVal(double x, double x1, double x2, double y1, double y2, double R) {
  return (x*(-y1 + y2)) / (x1 - x2) - ((x1 + x2)*(-y1 + y2)) / (2.0*(x1 - x2)) + (y1 + y2) / 2.0;
}

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void chatterCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  // std::cout << msg->header.frame_id << std::endl;
  double minScore = 10000000000;
  double minX = 0;
  double minY = 0;
  double minIntensity = 0;
  int minPFD = 0;

  visualization_msgs::MarkerArray markerArray;

  // for (int i = 0; i < 720; i++) {
  //       float theta = msg->angle_min + i * msg->angle_increment;
  //       int x = range[i] * cos(theta);
  //       int y = range[i] * sin(theta);        
  //     }


double rOrig = .28/2;
      for (int a = 0; a < 720; a += 2) {
        bool equation = a > 360;
        int i = a % 360;
        
        if (i < 5 || i > 355) continue;

        float theta = msg->angle_min + i * msg->angle_increment;
        double b = msg->ranges[i] * cos(theta); // x1
        double e = msg->ranges[i] * sin(theta); // y1
        double c = msg->ranges[i + 3] * cos(theta + 3*msg->angle_increment); // x2
        double f = msg->ranges[i + 3] * sin(theta + 3*msg->angle_increment); // y2

        double  r = rOrig;

        if (b == c || e  == f) continue;

        // std::cout << "T: " << theta << " R: " << msg->ranges[i] << " X1: " << b << " Y1: " << e << std::endl;

        double x0 = computeXVal(b, c, e, f, r, true);
        double y0 = computeYVal(x0, b, c, e, f, r);

        if (msg->ranges[i] < .1) continue;
        if (std::isnan(x0) || std::isnan(y0)) continue;
        
        double score = 0;

        int pointsForDistance = 0.8 * (720*asin(r/(2*msg->ranges[i])))/(2*3.1415);

        if (pointsForDistance < 8) continue;

        double deltaTheta = 2 * asin(sqrt((b - c) * (b - c) + (e - f) * (e - f)) / (2 * r));
        double theoreticalTheta = deltaTheta;

        for (int j = 0; j <= pointsForDistance; j += 1) {
          double dist = msg->ranges[i + j];

          double cx = dist * cos(theta + msg->angle_increment * (j));
          double cy = dist * sin(theta + msg->angle_increment * (j));

          double cx0 = computeXVal(b, cx, e, cy, r, equation);
          double cy0 = computeYVal(cx0, b, cx, e, cy, r);
          
          double actualPoint[2] = {cx, cy};
          
          double rotationalMatrix[2][2] = {{cos(theoreticalTheta), -sin(theoreticalTheta)}, {sin(theoreticalTheta), cos(theoreticalTheta)}};
          double center[2] = {x0, y0};
          double point[2] = {b, e};
          
          double transformedPoint[2] = {point[0] - center[0], point[1] - center[1]};
          
          double centeredTransformedPoint[2] = {rotationalMatrix[0][0] * transformedPoint[0] + rotationalMatrix[0][1] * transformedPoint[1], rotationalMatrix[1][0] * transformedPoint[0] + rotationalMatrix[1][1] * transformedPoint[1]};
          
          double fullyTransformedPoint[2] = {centeredTransformedPoint[0] + center[0], centeredTransformedPoint[1] + center[1]};
          
          double deltaX = fullyTransformedPoint[0] - actualPoint[0];
          double deltaY = fullyTransformedPoint[1] - actualPoint[1];
          
          double distance = sqrt(deltaX * deltaX + deltaY * deltaY);
          
          theoreticalTheta += deltaTheta;
          
          score += distance;

        // std::cout << cx0 << std::endl;
        }
        
        if (fabs(score) < minScore && r - rOrig <= .2) {

          minScore = fabs(score);
          minX = x0;
          minY = y0;
          minPFD = pointsForDistance;
          minIntensity = msg->intensities[i];
        }
        
        visualization_msgs::Marker marker;
        marker.header.frame_id = "xv11_front";
        marker.header.stamp = ros::Time();
        marker.ns = "my_namespace";
        marker.id = i + 1;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = x0;
        marker.pose.position.y = y0;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = r * 2;
        marker.scale.y = r * 2;
        marker.scale.z = r * 2;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0;
        marker.color.g = 0.0;
        marker.color.b = score / 10.0;

        if (score < 10) markerArray.markers.push_back(marker);
      }

      std::cout << minScore << ", " << minPFD << " => " << " " << minX << " : " << minY << std::endl;

      if (minScore < 10) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "xv11_front";
        marker.header.stamp = ros::Time();
        marker.ns = "pick";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = minX;
        marker.pose.position.y = minY;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = rOrig*2;
        marker.scale.y = rOrig*2;
        marker.scale.z = rOrig*2;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        markerArray.markers.push_back(marker);

        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "xv11_front";
        pose.pose.position.x = 0;
        pose.pose.position.y = 0;
        pose.pose.position.z = 0;
        tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, atan(minY / minX));
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();
        pose_pub.publish(pose);

        geometry_msgs::Point32 ballPoint;
        ballPoint.x = minX;
        ballPoint.y = minY;
        ballPoint.z = 0;

        ball_pub.publish(ballPoint);
    }
    
    vis_pub.publish( markerArray );
}

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "listener");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  ros::Subscriber sub = n.subscribe("/scan", 1000, chatterCallback);

  vis_pub = n.advertise<visualization_msgs::MarkerArray>( "markerArray", 0 );
  pose_pub = n.advertise<geometry_msgs::PoseStamped>( "ballPose", 0 );
  ball_pub = n.advertise<geometry_msgs::Point32>( "ballPoint", 0 );


  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}
