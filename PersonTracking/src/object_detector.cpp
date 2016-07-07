#include "../include/object_detector.h"

using namespace cv;
//----------------------------------------------------------------
t_object_detector::t_object_detector() {
	if (!face_cascade.load(face_cascade_name)) {
		printf("Error loading face cascade!\n");
		return;
	} else {
		printf("Loaded face cascade...\n");
	}
	if (!eyes_cascade.load(eyeglasses_cascade_name)) {
		printf("Error loading eyes cascade!\n");
		return;
	} else {
		printf("Loaded eyes cascade...\n");
	}
	if (!nose_cascade.load(nose_cascade_name)) {
		printf("Error loading nose cascade!\n");
		return;
	} else {
		printf("Loaded nose cascade...\n");
	}
	if (!mouth_cascade.load(mouth_cascade_name)) {
		printf("Error loading mouth cascade!\n");
		return;
	} else {
		printf("Loaded mouth cascade...\n");
	}
	if (!profile_cascade.load(profile_cascade_name)) {
		printf("Error loading profile cascade!\n");
		return;
	} else {
		printf("Loaded profile cascade...\n");
	}
	if (!hand_cascade.load(hand_cascade_name)) {
		printf("Error loading hand cascade!\n");
		return;
	} else {
		printf("Loaded hand cascade...\n");
	}
	if (!palm_cascade.load(palm_cascade_name)) {
		printf("Error loading palm cascade!\n");
		return;
	} else {
		printf("Loaded palm cascade...\n");
	}

	window_name = "Person detection";
}
//----------------------------------------------------------------
t_object_detector::~t_object_detector() {
}
//----------------------------------------------------------------
/// <summary>
/// Detects a hand in the left upper corners in the specified frame.
/// If the hand is in the upper left corner => increase distance to the person being tracked
/// </summary>
/// <param name="frame">The frame.</param>
bool t_object_detector::left_hand_detected(Mat frame) {
	//flip the image and apply the detection method
	Mat flipped_frame;
	flip(frame, flipped_frame, 1);
	//Set the corners for hand detection and mark them
	right_corner = flipped_frame(Rect(0, 0, flipped_frame.size().width / 4, flipped_frame.size().height / 2));
	//right_corner = frame(Rect(frame.size().width / 2 + frame.size().width / 4, 0, frame.size().width / 4, frame.size().height / 2));

	//Transform the image into grayscale so that the classifier can work with less info
	cvtColor(right_corner, right_corner, COLOR_BGR2GRAY);
	equalizeHist(right_corner, right_corner);

	//Store the hands that the classifier detected
	vector<Rect> hands, palms;

	//Detect hands with the classifier
	hand_cascade.detectMultiScale(right_corner, hands, 1.05, 6, CV_HAAR_FIND_BIGGEST_OBJECT | CV_HAAR_SCALE_IMAGE, Size(50, 50), Size(200, 200));
	if (hands.size() > 0) {
		//If a hand was detected, chack if a palm can be found to confirm that it's a hand
		Mat hand_frame = frame(Rect(hands[0].x, hands[0].y, hands[0].width, hands[0].height));
		palm_cascade.detectMultiScale(hand_frame, palms, 1.05, 3, CV_HAAR_FIND_BIGGEST_OBJECT | CV_HAAR_SCALE_IMAGE, Size(30, 30), Size(200, 200));
		if (palms.size() > 0) {
			double x = frame.size().width / 4 + frame.size().width / 2 + hands[0].x;
			rectangle(frame, Point(x, hands[0].y), Point(x + hands[0].width, hands[0].y + hands[0].height), Scalar(0, 0, 255), 2);
			//Show the results
			imshow(window_name, frame);

			return true;
		}
	}
	return false;
}
//----------------------------------------------------------------
/// <summary>
/// Detects a hand in the right upper corner in the specified frame.
/// If the hand is in the upper right corner => reduce distance to the person being tracked
/// </summary>
/// <param name="frame">The frame.</param>
/// <returns></returns>
bool t_object_detector::right_hand_detected(Mat frame) {
	//Set the corners for hand detection and mark them
	left_corner = frame(Rect(0, 0, frame.size().width / 4, frame.size().height / 2));

	//Transform the image into grayscale so that the classifier can work with less info
	cvtColor(left_corner, left_corner, COLOR_BGR2GRAY);
	equalizeHist(left_corner, left_corner);

	//Store the hands that the classifier detected
	vector<Rect> hands, palms;

	//Detect hands with the classifier
	hand_cascade.detectMultiScale(left_corner, hands, 1.1, 6, CV_HAAR_FIND_BIGGEST_OBJECT | CV_HAAR_SCALE_IMAGE, Size(50, 50), Size(200, 200));
	if (hands.size() > 0) {
		//If a hand was detected, check if a palm can be found to be sure that it is a hand
		Mat hand_frame = frame(Rect(hands[0].x, hands[0].y, hands[0].width, hands[0].height));
		palm_cascade.detectMultiScale(left_corner, palms, 1.05, 3, CV_HAAR_FIND_BIGGEST_OBJECT | CV_HAAR_SCALE_IMAGE, Size(30, 30), Size(200, 200));
		if (palms.size() > 0) {
			rectangle(frame, Point(hands[0].x, hands[0].y), Point(hands[0].x + hands[0].width, hands[0].y + hands[0].height), Scalar(0, 255, 0), 2);
			//Show the results
			imshow(window_name, frame);

			return true;
		}
	}
	return false;
}
//----------------------------------------------------------------
/// <summary>
/// Detect faces (then eyes, nose, mouth) and displays them in the specified frame.
/// </summary>
/// <param name="frame">The frame.</param>
bool t_object_detector::detect_faces_and_display(Mat frame) {
	//Store the features detected by the classifiers
	vector<Rect> faces, eyes, noses, mouths, profiles;
	Mat frame_gray;
	bool faceExists = false;

	//Transform the image into grayscale so that the classifier can work with less info
	cvtColor(frame, frame_gray, COLOR_BGR2GRAY);
	equalizeHist(frame_gray, frame_gray);

	//Detect faces and features with the classifiers
	face_cascade.detectMultiScale(frame_gray, faces, 1.1, 3, CV_HAAR_FIND_BIGGEST_OBJECT | CV_HAAR_SCALE_IMAGE, Size(30, 30), Size(200, 200));
	if (faces.size() > 0) {
		//The index of the first and only face detected
		int i = 0;
		Mat ROI = frame(Rect(faces[i].x, faces[i].y, faces[i].width, faces[i].height));

		//Check if face characteristics can be found, to be sure it is a face
		detect_eyes(ROI, eyes);
		detect_mouth(ROI, mouths);
		detect_nose(ROI, noses);
		if (eyes.size() > 0 || mouths.size() > 0 || noses.size() > 0) {
			//It is definitely a face, mark all the features found
			faceExists = true;

			Point face_origin = Point(faces[i].x, faces[i].y);
			double distance = norm(biggest_face_origin - face_origin);
			//Check if the face moved more than a certain number of pixels
			if (abs(distance) > distance_between_faces) {
				biggest_face_origin = face_origin;
				biggest_face_height = faces[i].height;
				biggest_face_width = faces[i].width;

				//Mark the face in the frame
				Point upper_left = Point(faces[i].x, faces[i].y);
				Point lower_right = Point(faces[i].x + faces[i].width, faces[i].y + faces[i].height);
				rectangle(frame, upper_left, lower_right, Scalar(100, 0, 100), 4);
			} else {
				//Mark the face in the frame
				rectangle(frame, biggest_face_origin, Point(biggest_face_origin.x + biggest_face_width, biggest_face_origin.y + biggest_face_height), Scalar(100, 0, 10), 4);
			}
		}
	} else {
		//Detect profile faces
		profile_cascade.detectMultiScale(frame_gray, profiles, 1.1, 4, CV_HAAR_FIND_BIGGEST_OBJECT | CV_HAAR_SCALE_IMAGE, Size(30, 30), Size(200, 200));

		if (profiles.size() > 0) {
			//The index of the first and biggest profile face detected
			faceExists = true;
			int j = 0;

			Point face_origin = Point(profiles[j].x, profiles[j].y);
			double distance = norm(biggest_face_origin - face_origin);
			//Check if the face moved more than a certain number of pixels
			if (abs(distance) > distance_between_faces) {
				biggest_face_origin = face_origin;
				biggest_face_height = profiles[j].height;
				biggest_face_width = profiles[j].width;

				Point profile_top_left = Point(profiles[j].x, profiles[j].y);
				Point profile_lower_right = Point(profiles[j].x + profiles[j].width, profiles[j].y + profiles[j].height);
				rectangle(frame, profile_top_left, profile_lower_right, Scalar(100, 100, 0), 4);

			} else {
				//Mark the face in the frame
				Point lower_right = Point(biggest_face_origin.x + biggest_face_width, biggest_face_origin.y + biggest_face_height);
				rectangle(frame, biggest_face_origin, lower_right, Scalar(100, 10, 0), 4);
			}
		} else {
			Mat flipped_frame;
			//Flip the image to try and find a profile face
			flip(frame_gray, flipped_frame, 1);
			//Detect profile faces
			profile_cascade.detectMultiScale(flipped_frame, profiles, 1.1, 4, CV_HAAR_FIND_BIGGEST_OBJECT | CV_HAAR_SCALE_IMAGE, Size(30, 30), Size(200, 200));
			if (profiles.size() > 0) {
				faceExists = true;
				//The index of the first and biggest profile face detected
				int j = 0;
				//Get the profile face origin coordinates from the mirrored image
				double x = frame.size().width / 2 + profiles[0].x;
				double y = frame.size().height / 2 + profiles[0].y;
				Point face_origin = Point(x, y);
				double distance = norm(biggest_face_origin - face_origin);
				//Check if the face moved more than a certain number of pixels
				if (abs(distance) > distance_between_faces) {
					biggest_face_origin = face_origin;
					biggest_face_height = profiles[j].height;
					biggest_face_width = profiles[j].width;

					Point profile_top_left = Point(x, y);
					Point profile_lower_right = Point(x + profiles[j].width, y + profiles[j].height);
					rectangle(frame, profile_top_left, profile_lower_right, Scalar(100, 160, 0), 7);
				} else {
					//Mark the face in the frame
					Point lower_right = Point(biggest_face_origin.x + biggest_face_width, biggest_face_origin.y + biggest_face_height);
					rectangle(frame, biggest_face_origin, lower_right, Scalar(100, 16, 0), 7);
				}
			}
		}
	}
	//Show the results
	imshow(window_name, frame);
	return faceExists;
}
//----------------------------------------------------------------
void t_object_detector::detect_eyes(Mat& img, vector<Rect_<int> >& eyes) {
	eyes_cascade.detectMultiScale(img, eyes, 1.2, 3, 0 | CV_HAAR_SCALE_IMAGE, Size(20, 20), Size(50, 50));
	return;
}
//----------------------------------------------------------------
void t_object_detector::mark_eyes(cv::Mat& ROI, vector<cv::Rect_<int> >& faces, vector<cv::Rect_<int> >& eyes) {
	Point eye_center;
	for (size_t j = 0; j < eyes.size(); j++) {
		eye_center = Point2f(eyes[j].x + eyes[j].width / 2, eyes[j].y + eyes[j].height / 2);
		rectangle(ROI, Point(eyes[j].x, eyes[j].y), Point(eyes[j].x + eyes[j].width, eyes[j].y + eyes[j].height), Scalar(255, 0, 0), 1, 4);
	}
	if (eyes.size() == 2) {
		right_eye_origin = Point(eyes[0].x, eyes[0].y);
		left_eye_origin = Point(eyes[1].x, eyes[1].y);
		eyes_height = eyes[0].height;
		eyes_width = eyes[0].width;
	} else if (eyes.size() == 1) {
		//Check which eye was detected and determine the other eye
		if (eyes[0].x > (faces[0].x + faces[0].width / 2)) {
			left_eye_origin = Point(eyes[0].x, eyes[0].y);
			right_eye_origin = Point(faces[0].width / 2 - eyes[0].width - eyes[0].x, eyes[0].y);
		} else {
			right_eye_origin = Point(eyes[0].x, eyes[0].y);
			double distance_between_nose_and_eye = faces[0].width / 2 - (eyes[0].x - faces[0].x) - eyes[0].width;
			left_eye_origin = Point(faces[0].width / 2 + distance_between_nose_and_eye, eyes[0].y);
		}
		eyes_height = eyes[0].height;
		eyes_width = eyes[0].width;
	}
}
//----------------------------------------------------------------
void t_object_detector::detect_nose(Mat& img, vector<Rect_<int> >& nose) {
	nose_cascade.detectMultiScale(img, nose, 1.1, 4, 0 | CV_HAAR_SCALE_IMAGE, Size(20, 20), Size(50, 50));
	return;
}
//----------------------------------------------------------------
void t_object_detector::mark_nose(cv::Mat& ROI, vector<cv::Rect_<int> >& faces, vector<cv::Rect_<int> >& noses) {
	double nose_center_height = 0.0, nose_center_width = 0.0;
	Point nose_top, nose_left, nose_right;
	if (noses.size() > 0) {
		//The index of the only nose detected in the face
		int j = 0;
		//Set the most recent detected object
		nose_origin = Point(noses[0].x, noses[0].y);
		nose_height = noses[0].height;
		nose_width = noses[0].width;
		//Mark the nose in the frame
		rectangle(ROI, Point(noses[j].x, noses[j].y), Point(noses[j].x + noses[j].width, noses[j].y + noses[j].height), Scalar(255, 0, 0), 1, 4);
	}
}
//----------------------------------------------------------------
void t_object_detector::detect_mouth(Mat& img, vector<Rect_<int> >& mouth) {
	mouth_cascade.detectMultiScale(img, mouth, 1.1, 4, 0 | CV_HAAR_SCALE_IMAGE, Size(20, 20), Size(50, 50));
	return;
}
//----------------------------------------------------------------
void t_object_detector::mark_mouth(cv::Mat& ROI, vector<cv::Rect_<int> >& faces, vector<cv::Rect_<int> >& mouths) {
	double mouth_center_height = 0.0, mouth_center_width = 0.0;
	if (mouths.size() > 0) {
		//The index of the only mouth detected in the face
		int j = 0;
		//Set the most recent detected object
		mouth_origin = Point(mouths[0].x, mouths[0].y);
		mouth_height = mouths[0].height;
		mouth_width = mouths[0].width;
		//Mark the mouth in the frame
		double nose_center_height = nose_origin.y + nose_height / 2;
		if (mouth_center_height > nose_center_height) {
			rectangle(ROI, mouth_origin, Point(mouths[j].x + mouths[j].width, mouths[j].y + mouths[j].height), Scalar(255, 0, 0), 1, 4);
		} else if (mouth_center_height <= nose_center_height) {
			//Do nothing
		} else
			rectangle(ROI, mouth_origin, Point(mouths[j].x + mouths[j].width, mouths[j].y + mouths[j].height), Scalar(255, 0, 0), 1, 4);
	}
}
//----------------------------------------------------------------
double t_object_detector::get_biggest_face_height() {
	return biggest_face_height;

}
//----------------------------------------------------------------
Point t_object_detector::get_biggest_face_origin() {
	return biggest_face_origin;
}
//----------------------------------------------------------------
void t_object_detector::set_biggest_face_X(int displacement) {
	biggest_face_origin.x += displacement;
}
//----------------------------------------------------------------
void t_object_detector::set_biggest_face_Y(int displacement) {
	biggest_face_origin.y += displacement;
}
//----------------------------------------------------------------
double t_object_detector::get_biggest_face_width() {
	return biggest_face_width;
}
//----------------------------------------------------------------