#include <ros/ros.h>
#include "vision_msgs/Yolov3_detector.h"
#include "vision_msgs/ObjectCoordinatesForDetection.h"
#include "vision_msgs/VisionObject.h"
#include "point_cloud_manager/GetRgbd.h"
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include "takeshi_tools/TakeshiTools.h"
#include "takeshi_tools/TakeshiVision.h"

darknet_ros_msgs::BoundingBoxes lastBounding_boxes;
ros::ServiceClient cltRgbdRobot;
ros::ServiceServer srvDetectAllYoloObjects;
bool yoloBoundingBoxesRecived=false;
bool read_objects_categories=true;
std::string object_categories_file;
std::vector<std::vector<std::string> >object_categories;



void printMessage(std::string message){std::cout << "\033[1;34m     YOLOv3 node.->" << message << "\033[0m" << std::endl;}
void printErrorMessage(std::string message){std::cout << "\033[1;31m     YOLOv3 node ->" << message << "\033[0m" << std::endl;}

void print_object_coordinates(vision_msgs::ObjectCoordinatesForDetection object_coordinates){

        printMessage("\nLooking for objects in: ");
        printMessage("x: " + std::to_string(object_coordinates.x_min) + "->" + std::to_string(object_coordinates.x_max));
        printMessage("y: " + std::to_string(object_coordinates.y_min) + "->" + std::to_string(object_coordinates.y_max));
        printMessage("z: " + std::to_string(object_coordinates.z_min) + "->" + std::to_string(object_coordinates.z_max) + "\n");
}

bool getImagesFromTakeshi(cv::Mat& imaBGR, cv::Mat& imaPCL){

    point_cloud_manager::GetRgbd srv;
    if(!cltRgbdRobot.call(srv)){
        printErrorMessage("Can not get the image from Takeshi");
        return false;
    }
    TakeshiTools::PointCloud2Msg_ToCvMat(srv.response.point_cloud, imaBGR, imaPCL);
    return true; 
}

void callback_darknet_ros_boundig_boxes(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg){

        lastBounding_boxes.header = msg->header;
        lastBounding_boxes.image_header = msg->image_header;
        lastBounding_boxes.bounding_boxes = msg->bounding_boxes;
        if(lastBounding_boxes.bounding_boxes.size() > 0)
                yoloBoundingBoxesRecived=true;
}

bool get_3d_bb_centroid(cv::Mat &imaPCL,cv::Vec3f& centroid_3d, int i){
	cv::Vec3f imagePoint;
	int point_count=0;
	centroid_3d.val[0]=centroid_3d.val[1]=centroid_3d.val[2]=0.0;
	
	for(int k=lastBounding_boxes.bounding_boxes[i].xmin; k<=lastBounding_boxes.bounding_boxes[i].xmax; k++) {
        for(int j=lastBounding_boxes.bounding_boxes[i].ymin; j<=lastBounding_boxes.bounding_boxes[i].ymax; j++) {
            imagePoint=imaPCL.at<cv::Vec3f>(j,k);
                                        
            if(imagePoint.val[0] != 0.0 && imagePoint.val[2] != 0.0) {
                centroid_3d.val[0]+=imagePoint[0];
                centroid_3d.val[1]+=imagePoint[1];
                centroid_3d.val[2]+=imagePoint[2];
                point_count++;
            }
        }
    }
    centroid_3d.val[0]/=point_count;
    centroid_3d.val[1]/=point_count;
    centroid_3d.val[2]/=point_count;
    if(point_count < 100)
    	return false;
    return true;
}

bool readObjectsCategories(std::string obj_file){
        object_categories_file+=obj_file;
        printMessage("Loading objects categories from: " + object_categories_file);
        std::ifstream file(object_categories_file.c_str());
        if (!file.is_open()) {
                printErrorMessage("Could not open file");
                file.close();
                return false;
        }
        std::vector<std::string> object_c(2);
        std::string line;
        while (std::getline(file,line)) {
                std::istringstream iss(line);
                if (!(iss >> object_c[0] >> object_c[1])) {
                        printErrorMessage("Parsing Error");
                        return false;
                }
                object_categories.push_back(object_c);
        }
        file.close();
        return true;
}

std::string getObjectCategory(std::string object){
        if(read_objects_categories) {
                readObjectsCategories("objects_abril_categories.txt");
                read_objects_categories=false;
        }
        for(int i=0; i< object_categories.size(); i++)
                if(object_categories[i][0].compare(object)==0)
                        return object_categories[i][1];
        printErrorMessage("Unknown Category to: " + object);
        return "unknown_category";
}

std::vector<vision_msgs::VisionObject> sort_nearest_object(std::vector<vision_msgs::VisionObject> recognizedYoloObjectsAux){
		float euclideanDistance, minEuclidenDistance=100;
        int index;
        for(int i=0; i<recognizedYoloObjectsAux.size(); i++) {
            euclideanDistance=sqrt(pow(0 - recognizedYoloObjectsAux[i].pose.position.x, 2) + pow(0 - recognizedYoloObjectsAux[i].pose.position.y, 2));
                if(euclideanDistance <= minEuclidenDistance) {
                    minEuclidenDistance=euclideanDistance;
                    index=i;
                }
            }
        std::vector<vision_msgs::VisionObject> recognizedYoloObjects;
        recognizedYoloObjects.clear();
        recognizedYoloObjects.push_back(recognizedYoloObjectsAux[index]);
        for(int i=0; i<recognizedYoloObjectsAux.size(); i++)
                if(i != index)
                    recognizedYoloObjects.push_back(recognizedYoloObjectsAux[i]);
        return recognizedYoloObjects;
}
