#!/usr/bin/env python

""" Parses LaserScan and looks for circular features """

# We always import roslib, and load the manifest (i.e., package name) to handle dependencies
import rospy

# import relevant modules
import math
import numpy
 
# import relevant message types
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker


# define variables
targets = [];


def scan2coords( start_ang, end_ang, ang_inc, ranges ):
    '''Converts range measurements (in robot frame) to Cartesian coordinates (still in robot frame)'''
    coords = numpy.zeros( (ranges.shape[0],2) );
    
    for i in range( ranges.shape[0] ):
        coords[i][0] = ranges[i] * math.cos( start_ang + i*ang_inc );   # compute x coordinate
        coords[i][1] = ranges[i] * math.sin( start_ang + i*ang_inc );   # compute x coordinate
#        print ranges[i], coords[i][0], coords[i][1]
        
    return coords

def computeIntervals(ranges, range_max ):
    '''Compute intervals of continuity based on distance thresholds'''

    # Determine intervals of continuity
    #  - determined by sharp changes in range
    threshold = 0.5;       # 0.05 m

    # Compute intervals of continuity
    idx_s = []; idx_e = [];
    inside = True;
    lastRange = 0
    for i, curr_range in enumerate(ranges):
        if inside:
            idx_s.append(i)
            inside = False
        elif ( math.fabs(curr_range - lastRange) > threshold  ) :
            idx_e.append(i - 1)
            idx_s.append(i)
        lastRange = curr_range
    return idx_s, idx_e


def sendPoint(pub):
    '''Defines Circle and publishes to topic'''

    for i in range(len(targets)):
        target = Point();
        target.x = targets[i][0];
        target.y = targets[i][1];
        target.z = targets[i][2];
        
        # Publish the command
        pub.publish(target);

def sendMarker(pub):
    '''Defines target markers and publishes to visualization_marker topic'''
    
    for i in range( len(targets) ):

        # Container for the command that will be published
        marker = Marker();
    
        marker.header.frame_id = "/base_link";
        marker.header.stamp = rospy.Time.now();
        
        marker.ns = "basic_shapes";
        marker.id = i;
    
        marker.type = Marker.CYLINDER;
        marker.action = Marker.ADD;
        
        
        # laser located at x=0.3937/2-0.08 = 0.19685-0.08 = 0.11685
        marker.pose.position.x = targets[i][0]+0.11685;
        marker.pose.position.y = targets[i][1]        ;
        marker.pose.position.z = 0.2;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
    
        # Set the scale of the marker -- 1x1x1 here means 1m on a side
        marker.scale.x = 2*targets[i][2];
        marker.scale.y = 2*targets[i][2];
        marker.scale.z = 0.5;
    
        # Set the color -- be sure to set alpha to something non-zero!
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 0.4;
        
        marker.lifetime = rospy.Duration(0.2);
#        marker.lifetime = rospy.Duration();         # last as long as ROS is running
            
        # Publish the command
        pub.publish(marker);


def circleFinder( coords ):
    '''
    Determines circle geometry from a set of points in (x,y) plane
        Ref: http://paulbourke.net/geometry/circlefrom3/
    '''
    # Recall all scans are relative to sensor (vs. robot) position

    # Define three points (x,y) within this interval
    pt1 = coords[0,:]                                   # first point in interval
    pt2 = coords[math.floor(coords.shape[0]/2),:]       # mid point (approx) in interval
    pt3 = coords[-1,:]                                  # last point in interval
#    print pt1
#    print pt2
#    print pt3

    # Determine the equations of the two lines
    #  Line a passes through Points pt1 and pt2
    #  Line b passes through Points pt2 and pt3
    m_a = (pt2[1] - pt1[1]) / (pt2[0] - pt1[0]) 
    m_b = (pt3[1] - pt2[1]) / (pt3[0] - pt2[0]) 
 
    # Computer center coordinates of circle
    target_loc_x = ( (m_a * m_b * (pt1[1] - pt3[1]) 
                         + m_b * (pt1[0] + pt2[0]) 
                         - m_a * (pt2[0] + pt3[0])) 
                        / (2 * (m_b - m_a)) );
    target_loc_y = m_a * (target_loc_x - pt2[0]) + pt2[1]; # using line equation
    
    # Compute radius of circle
    #  Distance between center and any point
    radius = math.sqrt(math.pow(pt2[0] - target_loc_x, 2) + math.pow(pt2[1] - target_loc_y, 2));

#    print "Target center (%f,%f) with radius %fm" % (target_loc_x, target_loc_y, radius);

    return target_loc_x, target_loc_y, radius
#    
#      # Determine whether this circle is within the tolerance
#      double radius_tol = 0.05; // 5cm
#      if (abs(target.radius - 0.50) <= radius_tol) :
#          target_found = true;
#      else :
#          target_found = false;
#    
#      return target_found;
#    

def circleFinder2( coords ):
    '''
    Circle fitting algorithm from Pratt
        V. Pratt, "Direct least-squares fitting of algebraic surfaces",
          Computer Graphics, Vol. 21, pages 145-152 (1987)
    Adapted from: CircleFitByPratt.m algorithm on MatlabCentral
        Ref: http://www.mathworks.com/matlabcentral/fileexchange/22643-circle-fit-pratt-method
    '''
    
    n = coords.shape[0];        # number of data points
    
    centroid = coords.mean(axis=0)        # compute centroid location (for data centering)

    # computing moments (note: all moments will be normed, i.e. divided by n)
    Mxx=0; Myy=0; Mxy=0; Mxz=0; Myz=0; Mzz=0;
    
    for i in range(n) :
        Xi = coords[i,0] - centroid[0];  #  centering data
        Yi = coords[i,1] - centroid[1];  #  centering data
        Zi = Xi*Xi + Yi*Yi;
        Mxy = Mxy + Xi*Yi;
        Mxx = Mxx + Xi*Xi;
        Myy = Myy + Yi*Yi;
        Mxz = Mxz + Xi*Zi;
        Myz = Myz + Yi*Zi;
        Mzz = Mzz + Zi*Zi;
       
    Mxx = Mxx/n;
    Myy = Myy/n;
    Mxy = Mxy/n;
    Mxz = Mxz/n;
    Myz = Myz/n;
    Mzz = Mzz/n;

    # computing the coefficients of the characteristic polynomial
    Mz = Mxx + Myy;
    Cov_xy = Mxx*Myy - Mxy*Mxy;
    Mxz2 = Mxz*Mxz;
    Myz2 = Myz*Myz;
    
    A2 = 4*Cov_xy - 3*Mz*Mz - Mzz;
    A1 = Mzz*Mz + 4*Cov_xy*Mz - Mxz2 - Myz2 - Mz*Mz*Mz;
    A0 = Mxz2*Myy + Myz2*Mxx - Mzz*Cov_xy - 2*Mxz*Myz*Mxy + Mz*Mz*Cov_xy;
    A22 = A2 + A2;
    
    epsilon=1e-12; 
    ynew=1e+20;
    IterMax=20;
    xnew = 0;

    # Newton's method starting at x=0
    for iter in range(IterMax) :
        yold = ynew;
        ynew = A0 + xnew*(A1 + xnew*(A2 + 4.*xnew*xnew));
        if (abs(ynew)>abs(yold)) :
#            print 'Newton-Pratt goes wrong direction: |ynew| > |yold|'
            xnew = 0;
            break;
        
        Dy = A1 + xnew*(A22 + 16*xnew*xnew);
        xold = xnew;
        xnew = xold - ynew/Dy;
        if (abs((xnew-xold)/xnew) < epsilon):
            break
        if (iter >= IterMax):
#            print 'Newton-Pratt will not converge'
            xnew = 0;
        if (xnew<0.0) :
#            print 'Newton-Pratt negative root:  x=%f' % xnew;
            xnew = 0;


    # computing the circle parameters
    DET = xnew*xnew - xnew*Mz + Cov_xy;
    Center = [Mxz*(Myy-xnew)-Myz*Mxy , Myz*(Mxx-xnew)-Mxz*Mxy]/DET/2;
    
    radius = math.sqrt( numpy.dot(Center, Center) + Mz+2*xnew ) 
    
    target_loc_x = Center[0]+centroid[0];
    target_loc_y = Center[1]+centroid[1];
#    print "Target center (%f,%f) with radius %fm" % (target_loc_x, target_loc_y, radius);
    return target_loc_x, target_loc_y, radius

def odomCallback(msg):
    '''Callback function for odometry data'''
#    print msg

def laserCallback(scan):
    '''Callback function for laser data'''

    # Convert to array structure to use NumPy module
    ranges = numpy.array( scan.ranges )


    # Compute intervals of continuity
    idx_s, idx_e = computeIntervals( ranges, scan.range_max )
    
    # Iterate through each interval
    for i in range(min(len(idx_s),len(idx_e))):
        # Convert ranges to cartesian coordinates
        coords = scan2coords( scan.angle_min+idx_s[i]*scan.angle_increment, 
                     scan.angle_min+idx_e[i]*scan.angle_increment,
                     scan.angle_increment,
                     ranges[idx_s[i]:idx_e[i]] )
        
        target_loc_x, target_loc_y, radius = circleFinder2( coords )
#        target_loc_x, target_loc_y, radius = circleFinder( coords )
#        print "Target center (%f,%f) with radius %fm" % (target_loc_x, target_loc_y, radius);

        # Filter unlikely targets, either by radius or location
        #  Radius threshold is 3m
        print radius
        if (math.fabs(radius - 0.25 / 2) <= 0.1):
            targets.append([target_loc_x, target_loc_y, radius]);


if __name__=="__main__":
# this quick check means that the following code runs ONLY if this is the 
# main file -- if we "import randomwalker" in another file, this code will not execute.

    # first thing, init a node!
    rospy.init_node('circleFinder_node')

    # Subscribe to laser topic
    rospy.Subscriber("/scan", LaserScan, laserCallback) # subscribe on the <laser> topic

    # Subscribe to odom topic
    # rospy.Subscriber("/odom", Odometry, odomCallback) # subscribe on the <laser> topic

    # publish to circle_detect topic
    pub1 = rospy.Publisher('circle_detect', Point)
    
    # publish to circle_detect topic
    pub2 = rospy.Publisher('visualization_marker', Marker)

    
    # set up a loop rate to query subscribers and publish commands
    rate = rospy.Rate(10.0)             # 10 Hz

    # main loop
    while not rospy.is_shutdown():      # is_shutdown calls spinonce() to update ROS queues

        targetFound = False;

        # Check whether or not to publish any messages
        if (~targetFound):
            sendPoint(pub1);  # Decide what to do and publish commands
            sendMarker(pub2);    # Populate and publish markers

            targets = [];
         #    rospy.loginfo('Target found')
         # else:
         #     rospy.loginfo('Nothing found yet');
         #     print

        # wait if we're too fast
        rate.sleep()