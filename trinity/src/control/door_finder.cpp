#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>
#include <sstream>
#include <vector>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/ColorRGBA.h>
using std::vector;
using std::string;
using sensor_msgs::LaserScan;
using geometry_msgs::Point;

#define doorDistMin 0.35
#define doorDistMax 0.45
#define spikeDist 0.25

ros::Publisher doors_pub;
ros::Publisher points_pub;
ros::Publisher walls_pub;

vector<int> extractKeyIndices(const LaserScan& scan){
  vector<int> keyPoints;
  for(int i = 0; i < scan.ranges.size(); i++){
    if(fabs(scan.ranges[i] - scan.ranges[(i+1) % scan.ranges.size()]) > spikeDist){
      ROS_INFO("Found key point with spike of %g",fabs(scan.ranges[i] - scan.ranges[(i+1) % scan.ranges.size()]));
      if(scan.ranges[i] < scan.ranges[(i+1) % scan.ranges.size()])
        keyPoints.push_back(i);
      else
        keyPoints.push_back((i+1) % scan.ranges.size());
    }
  }
  return keyPoints;
}

Point scanToPoint(const LaserScan& scan, int i){
  Point p;
  double theta = i*2*M_PI/scan.ranges.size(); 

  // Idk why we are multiplying by -1 but it works
  p.x = cos(theta)*scan.ranges[i]*-1;
  p.y = sin(theta)*scan.ranges[i]*-1;
  return p;
}

// Pretend point constructor
Point GetPoint(double x, double y){
  Point p;
  p.x = x;
  p.y = y;
  return p;
}

Point findClosestWall(const LaserScan& scan, int ind){
  double closestDist = -1;
  Point p1 = scanToPoint(scan, ind);
  Point closest = GetPoint(-1, -1);
  double scanDist;
  bool seenSpike = false;
  int closestIndex = 0;
  for(int j = 0; j < scan.ranges.size()/2; j++){
    int ind2 = (ind+j)%scan.ranges.size();
    if(fabs(scan.ranges[ind2] - scan.ranges[(ind2+1+scan.ranges.size())%scan.ranges.size()]) > spikeDist){
      seenSpike = true;
      continue; // The point before a spike will never be the other side of the door
    }
    if(seenSpike){
      Point p2 = scanToPoint(scan, ind2);
      scanDist = sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
      if(scanDist > 0 && (scanDist < closestDist || closestDist == -1) && scanDist > doorDistMin && scanDist < doorDistMax){
        closestDist = scanDist;
        closest = p2;
        closestIndex = ind2; // Saving index for debug purposes
      }  
    }
  }
  seenSpike = false;
  for(int j = 0; j < scan.ranges.size()/2; j++){
    int ind2 = (ind-j + scan.ranges.size())%scan.ranges.size();
    if(fabs(scan.ranges[ind2] - scan.ranges[(ind2-1+scan.ranges.size())%scan.ranges.size()]) > spikeDist){
      seenSpike = true;
      continue;
    }
    if(seenSpike){
      Point p2 = scanToPoint(scan, ind2);
      scanDist = sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
      if(scanDist > 0 && (scanDist < closestDist || closestDist == -1) && scanDist > doorDistMin && scanDist < doorDistMax){
        closestDist = scanDist;
        closest = p2;
        closestIndex = ind2;
      }  
    }
  }
  return closest;
}

// Create a marker for our marker array
visualization_msgs::Marker CreateMarker(std_msgs::Header h, Point p, string text, double r, double g, double b, int id){
  visualization_msgs::Marker m;
  m.id = id;
  m.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  m.header = h;
  m.action = 0;
  m.text = text;
  m.pose.position = p;
  m.scale.x = 0.1;
  m.scale.y = 0.1;
  m.scale.z = 0.1;
  m.pose.orientation.x = 0.0;
  m.pose.orientation.y = 0.0;
  m.pose.orientation.z = 0.0;
  m.pose.orientation.w = 1.0;
  m.color.r = r;
  m.color.b = b;
  m.color.g = g;
  m.color.a = 1.0;
  m.lifetime = ros::Duration(0.5);
  return m;
}

// Find doors
void findDoors(const LaserScan& scan){
  ROS_INFO("----------------------------------------------------------------------------------");
  visualization_msgs::MarkerArray m;
  visualization_msgs::MarkerArray k;
  visualization_msgs::MarkerArray w;
  vector<Point> walls;
  
  vector<Point> doors;
  vector<int> keyPoints = extractKeyIndices(scan);
  bool foundPair = false;

  // Create key point markers labelled with their number
  for(int i = 0; i < keyPoints.size(); i++){
    Point p = scanToPoint(scan, keyPoints[i]);
    k.markers.push_back(CreateMarker(scan.header, p, std::to_string(i), 0, 0, 1, i));
    walls.push_back(GetPoint(-1, -1));
  }

  Point p1, p2;
  ROS_INFO("Found %d key points", keyPoints.size());
  for(int i = 0; i < keyPoints.size(); i++){
    ROS_INFO("Key point: #%d: %d", i, keyPoints[i]);
    foundPair = false;
    if(keyPoints[i] == -1){
      continue; // already been recorded, skip
    }
    p1 = scanToPoint(scan, keyPoints[i]);
    
    for(int j = i + 1; j < keyPoints.size(); j++){
      if(keyPoints[j] == -1)
        continue; // already been recordedm skip
      p2 = scanToPoint(scan, keyPoints[j]);
      double scanDist = sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
      if(scanDist < doorDistMax && scanDist > doorDistMin){
        // two edges of a door, mark their midpoint as a door.
        doors.push_back(GetPoint((p1.x+p2.x)/2, (p1.y+p2.y)/2));

        // Create marker for door labelled with the key points that it is between
        m.markers.push_back(CreateMarker(scan.header, doors.back(), std::to_string(i) + "/" + std::to_string(j), 1, 0, 1, i));
        ROS_INFO("Key point %d matched with %d (%g apart), marked as -1", i, j, scanDist);
        ROS_INFO("Added door. Total doors: %d", doors.size());
        keyPoints[i] = keyPoints[j] = -1;
        break;
      }
    }
    if(keyPoints[i] != -1){
      //find closest wall to the point
      p1 = scanToPoint(scan, keyPoints[i]);
      p2 = findClosestWall(scan, keyPoints[i]);
      if(p2.x != -1 && p2.y != -1){
        walls[i] = p2;
        doors.push_back(GetPoint((p1.x+p2.x)/2,(p1.y+p2.y)/2));

        // Create marker for door labelled with its key point
        m.markers.push_back(CreateMarker(scan.header, doors.back(), std::to_string(i) + "/" + std::to_string(i), 1, 0, 1, i));
        ROS_INFO("Added door. Total doors: %d", doors.size());
      } else {
        ROS_INFO("Discarded door because the closest wall wasn't door distance away");
      }
    }
  }
  
  for (int i = 0; i < walls.size(); i++){
    if(walls[i].x != -1 && walls[i].y != -1){
      // Create marker for a "closest wall" point
      w.markers.push_back(CreateMarker(scan.header, walls[i], std::to_string(i), 1, 0, 0, i));
    }
  }

  ROS_INFO("Publishing %d key point markers", k.markers.size());
  ROS_INFO("Publishing %d door markers", m.markers.size());
  doors_pub.publish(m);
  points_pub.publish(k);
  walls_pub.publish(w);
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "door_finder");
  ros::NodeHandle n;
  doors_pub = n.advertise<visualization_msgs::MarkerArray>("doors", 1000);
  points_pub = n.advertise<visualization_msgs::MarkerArray>("key_points", 1000);
  walls_pub = n.advertise<visualization_msgs::MarkerArray>("walls", 1000);
  ros::Subscriber sub = n.subscribe("/scan", 1000, findDoors);
  ros::spin();
  return 0;
}
