#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <std_srvs/Empty.h>

// Función para detectar esferas rojas y verdes
void detectApples(const cv::Mat& image) {
    // Convertir la imagen a espacio de color HSV
    cv::Mat hsv_image;
    cv::cvtColor(image, hsv_image, cv::COLOR_BGR2HSV);

    // Definir los rangos de colores para rojo y verde
    cv::Scalar lower_red1(0, 120, 70), upper_red1(10, 255, 255);
    cv::Scalar lower_red2(170, 120, 70), upper_red2(180, 255, 255);
    cv::Scalar lower_green(35, 100, 100), upper_green(85, 255, 255);

    // Crear máscaras para rojo y verde
    cv::Mat mask_red1, mask_red2, mask_red, mask_green;
    cv::inRange(hsv_image, lower_red1, upper_red1, mask_red1);
    cv::inRange(hsv_image, lower_red2, upper_red2, mask_red2);
    mask_red = mask_red1 | mask_red2;
    cv::inRange(hsv_image, lower_green, upper_green, mask_green);

    // Detectar contornos para ambas máscaras
    std::vector<std::vector<cv::Point>> contours_red, contours_green;
    cv::findContours(mask_red, contours_red, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    cv::findContours(mask_green, contours_green, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    // Procesar esferas rojas
    for (const auto& contour : contours_red) {
        double area = cv::contourArea(contour);
        if (area > 300) { // Filtrar áreas pequeñas
            cv::Point2f center;
            float radius;
            cv::minEnclosingCircle(contour, center, radius);
            if (radius > 10) {
                cv::circle(image, center, static_cast<int>(radius), cv::Scalar(0, 255, 0), 2);
                cv::putText(image, "red apple", center - cv::Point2f(20, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
            }
        }
    }

    // Procesar esferas verdes
    for (const auto& contour : contours_green) {
        double area = cv::contourArea(contour);
        if (area > 300) { // Filtrar áreas pequeñas
            cv::Point2f center;
            float radius;
            cv::minEnclosingCircle(contour, center, radius);
            if (radius > 10) {
                cv::circle(image, center, static_cast<int>(radius), cv::Scalar(0, 0, 255), 2);
                cv::putText(image, "damaged apple", center - cv::Point2f(20, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 2);
            }
        }
    }

    // Mostrar la imagen procesada
    cv::imshow("Apple Detection", image);
    cv::waitKey(1);
}

// Callback para el tópico de imágenes
void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        // Convertir el mensaje ROS a una imagen OpenCV
        cv::Mat image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
        detectApples(image);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "apple_detector");
    ros::NodeHandle nh;

    // Suscribirse al tópico de la cámara
    std::string camera_topic = "/rgbd_camera/rgb/image_raw";
    ros::Subscriber image_sub = nh.subscribe(camera_topic, 1, imageCallback);

    // Enviar comando a Gazebo para ponerlo en "play"
    ros::ServiceClient pauseClient = nh.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");
    std_srvs::Empty emptySrv;
    if (pauseClient.call(emptySrv)) {
        
    } else {
        ROS_WARN("No se pudo poner Gazebo en 'play'. Verifica que Gazebo esté corriendo.");
    }

    ROS_INFO("Apple Detector Node Initialized");
    ros::spin();
    return 0;
}
