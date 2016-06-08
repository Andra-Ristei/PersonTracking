#ifndef face_trackerH
#define face_trackerH

#include <opencv2\highgui.hpp>
#include <opencv2\objdetect\objdetect.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include <opencv2\video\tracking.hpp>
#include <opencv2\videoio\videoio.hpp>

#include <iostream>
#include <limits>
#include <ctype.h>
#include <stdio.h>
#include <vector>

using namespace std;


class t_object_detector {
private:
	/** Classifiers **/
	cv::String face_cascade_name = "classifiers\\lbpcascade_frontalface_visionary.xml",
		nose_cascade_name = "classifiers\\haarcascade_mcs_nose.xml",
		mouth_cascade_name = "classifiers\\haarcascade_mcs_mouth.xml",
		eyeglasses_cascade_name = "classifiers\\haarcascade_eye_tree_eyeglasses.xml",
		profile_cascade_name = "classifiers\\haarcascade_profileface.xml",
		hand_cascade_name = "classifiers\\Hand.Cascade.1.xml",
		palm_cascade_name = "classifiers\\palm.xml";
	cv::CascadeClassifier face_cascade, eyes_cascade, profile_cascade, nose_cascade, mouth_cascade, hand_cascade, palm_cascade;

	/** Coordinates of the face to be tracked **/
	cv::Point biggest_face_origin = cv::Point(0, 0);
	double biggest_face_height = 0.0, biggest_face_width = 0.0, distance_between_faces = 10.0;
	/** Coordinates of the eyes **/
	cv::Point left_eye_origin = cv::Point(0, 0), right_eye_origin = cv::Point(0, 0);
	double eyes_height = 0.0, eyes_width = 0.0;
	/** Coordinates of the nose**/
	cv::Point nose_origin = cv::Point(0, 0);
	double nose_height = 0.0, nose_width = 0.0;
	/** Coordinates of the mouth**/
	cv::Point mouth_origin = cv::Point(0, 0);
	double mouth_height = 0.0, mouth_width = 0.0;

	/** Coordinates of the right/left corners in which to detect the hand signaling a change of distance **/
	cv::Mat right_corner, left_corner;


	void detect_eyes(cv::Mat&, vector<cv::Rect_<int> >&);
	void mark_eyes(cv::Mat&, vector<cv::Rect_<int> >&, vector<cv::Rect_<int> >&);
	void detect_nose(cv::Mat&, vector<cv::Rect_<int> >&);
	void mark_nose(cv::Mat&, vector<cv::Rect_<int> >&, vector<cv::Rect_<int> >&);
	void detect_mouth(cv::Mat&, vector<cv::Rect_<int> >&);
	void mark_mouth(cv::Mat&, vector<cv::Rect_<int> >&, vector<cv::Rect_<int> >&);

public:
	cv::String window_name;

	t_object_detector();
	~t_object_detector();
	bool detect_faces_and_display(cv::Mat);
	bool right_hand_detected(cv::Mat);
	bool left_hand_detected(cv::Mat);
	double get_biggest_face_width();
	double get_biggest_face_height();
	cv::Point get_biggest_face_origin();
	void set_biggest_face_X(int);
	void set_biggest_face_Y(int);
};


#endif