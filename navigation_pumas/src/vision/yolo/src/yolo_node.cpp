#include <std_msgs/Empty.h>
#include <iostream>
#include <sstream>
#include <string>
#include "opencv2/opencv.hpp"

#include "yolo_node.h"






vision_msgs::ObjectCoordinatesForDetection objectCoordinates;
ros::Subscriber subYoloBoundingBoxes;

ros::Publisher pubYoloImage;



bool detect_all_yolo_objects(std::vector<vision_msgs::VisionObject>& recognizedYoloObjects, int timeOut_ms,vision_msgs::ObjectCoordinatesForDetection object_coordinates){

        int attempts = timeOut_ms / 100;
        cv::Mat imaBGR,imaPCL;
        sensor_msgs::Image img_msg;
        std_msgs::Header header;
        header.seq = 1;
        header.stamp = ros::Time::now();
        cv_bridge::CvImage img_bridge;        
        vision_msgs::VisionObject yoloObject;
        std::vector<vision_msgs::VisionObject> recognizedYoloObjectsAux;
        
        yoloBoundingBoxesRecived=false;
        recognizedYoloObjects.clear();       
        ros::Rate loop(10);
        
        print_object_coordinates(object_coordinates);

        if(!getImagesFromTakeshi(imaBGR,imaPCL))
            return false;
        
        img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, imaBGR);
        img_bridge.toImageMsg(img_msg);

        for(int i=0; i<5; i++) {
            //we have a problem with the darknetros subscriber 
            pubYoloImage.publish(img_msg);
            ros::spinOnce();
            loop.sleep();
        }

        //wait for the response of darknet-ros        
        while(ros::ok() && !yoloBoundingBoxesRecived && --attempts > 0) {
                ros::spinOnce();
                loop.sleep();
        }

        if(yoloBoundingBoxesRecived) {
                printMessage("\n\nYolo Objects: ");
                printMessage("Yolo Objects detected: "+ std::to_string(lastBounding_boxes.bounding_boxes.size()));
                
                for(int i=0; i<lastBounding_boxes.bounding_boxes.size(); i++) {
                        cv::Vec3f centroid_3d;                        
                        if(get_3d_bb_centroid(imaPCL,centroid_3d,i)) {
                                yoloObject.id = lastBounding_boxes.bounding_boxes[i].Class;
                                yoloObject.category = getObjectCategory(yoloObject.id);
                                yoloObject.confidence = lastBounding_boxes.bounding_boxes[i].probability;
                                yoloObject.pose.position.x = centroid_3d.val[0];
                                yoloObject.pose.position.y = centroid_3d.val[1];
                                yoloObject.pose.position.z = centroid_3d.val[2]+0.03;
                                yoloObject.bounding_box.xmin=lastBounding_boxes.bounding_boxes[i].xmin;
                                yoloObject.bounding_box.ymin=lastBounding_boxes.bounding_boxes[i].ymin;
                                yoloObject.bounding_box.xmax=lastBounding_boxes.bounding_boxes[i].xmax;
                                yoloObject.bounding_box.ymax=lastBounding_boxes.bounding_boxes[i].ymax;
                                    
                                if(TakeshiVision::object_is_graspeable(centroid_3d, object_coordinates)){
                                    recognizedYoloObjectsAux.push_back(yoloObject);
                                    printMessage("Class: " + yoloObject.id + " - Confidence: " + std::to_string(yoloObject.confidence));
                                    printMessage("Centroid 3D  X: " + std::to_string(yoloObject.pose.position.x) + " Y: " + std::to_string(yoloObject.pose.position.y) + " Z: " + std::to_string(yoloObject.pose.position.z)+ "\n");
                                }
                                else{
                                    printMessage("Class: " + lastBounding_boxes.bounding_boxes[i].Class + " - Confidence: " + std::to_string(lastBounding_boxes.bounding_boxes[i].probability));
                                    printMessage("Centroid 3D  X: " + std::to_string(yoloObject.pose.position.x) + " Y: " + std::to_string(yoloObject.pose.position.y) + " Z: " + std::to_string(yoloObject.pose.position.z));
                                    printErrorMessage("The Object does not satisfy the objectCoordinates \n");
                                }
                        }

                        else{
                                printMessage("Class: " + lastBounding_boxes.bounding_boxes[i].Class + " - Confidence: " + std::to_string(lastBounding_boxes.bounding_boxes[i].probability));
                                printErrorMessage("Can not get the object 3D centroid");
                        }
                }
                if(recognizedYoloObjectsAux.size() > 0){
                    recognizedYoloObjects=sort_nearest_object(recognizedYoloObjectsAux);
                    yoloBoundingBoxesRecived=false;
                }
                
        }
        else{
                yoloBoundingBoxesRecived=false;  
                printErrorMessage("Can not detect yolo objects");   
                recognizedYoloObjects.clear(); 

        }

        return true;
}

bool callback_srvDetectAllYoloObjects(vision_msgs::Yolov3_detector::Request &req, vision_msgs::Yolov3_detector::Response &resp){
        return detect_all_yolo_objects(resp.recognizedYoloObjects,req.timeOut_ms.data,req.objectCoordinates);
}


int main(int argc, char** argv)
{
    printMessage("INITIALIZING YOLO NODE... by Edd-I");
    ros::init(argc, argv, "yolov3_detector");
    ros::NodeHandle n;  
    ros::Rate loop(10);

    srvDetectAllYoloObjects = n.advertiseService("/vision/yolov3_detector/detect_all_yolo_objects", callback_srvDetectAllYoloObjects);
    subYoloBoundingBoxes=n.subscribe("/darknet_ros/bounding_boxes",1,&callback_darknet_ros_boundig_boxes);
    pubYoloImage = n.advertise<sensor_msgs::Image>("/yolo_input", 1);
    cltRgbdRobot = n.serviceClient<point_cloud_manager::GetRgbd>("/hardware/point_cloud_man/get_rgbd_wrt_robot");
    TakeshiTools::setNodeHandle(&n);
    TakeshiVision::setNodeHandle(&n);
    
    n.getParam("/vision/obj_reco/object_categories_file", object_categories_file);
    printMessage("Running...");

    while(ros::ok()){        
        loop.sleep();
        ros::spinOnce();
    }

    return 0;
}