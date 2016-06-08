#include "../include/object_detector.h"
#include "../include/tracks_controller.h"
#include "../include/head_controller.h"
#include "../include/jenny5_command_module.h"
#include "../include/point_tracker.h"

//----------------------------------------------------------------
using namespace cv;

#define ULTRASONIC 0
#define TOLERANCE 20
#define DISTANCE_TOLERANCE 10

#define MOVE_LEFT -1
#define MOVE_RIGHT 1
#define MOVE_UP 2
#define MOVE_DOWN 3

//----------------------------------------------------------------
void read_from_serial_port(t_head_controller *head_controller) {
	head_controller->read_from_serial_port();
}
//----------------------------------------------------------------
bool go_to_home_position(t_head_controller *head_controller) {
	return head_controller->go_to_home_position();
}
//----------------------------------------------------------------
void align_with_person(int displacement_x, t_tracks_controller *tracks_controller, int direction) {
	if (direction == MOVE_LEFT) {
		tracks_controller->turn_left(displacement_x);
	} else if (direction == MOVE_RIGHT) {
		tracks_controller->turn_right(displacement_x);
	}
}
//----------------------------------------------------------------
void align_head(int displacement_y, t_head_controller *ctrl, int direction) {
	if (direction == MOVE_UP) {
		ctrl->move_up(displacement_y);
	} else if (direction == MOVE_DOWN) {
		ctrl->move_down(displacement_y);
	}
}
//----------------------------------------------------------------
void reduce_distance_to_person(t_tracks_controller *tracks_controller) {
	double new_distance = tracks_controller->get_distance_to_person() - tracks_controller->get_modify_distance_by();
	if (new_distance > tracks_controller->get_minimum_distance_allowed() && new_distance < tracks_controller->get_maximum_distance_allowed()) {
		tracks_controller->set_distance_to_person(new_distance);
	}
}
//----------------------------------------------------------------
void increase_distance_to_person(t_tracks_controller *tracks_controller) {
	double new_distance = tracks_controller->get_distance_to_person() + tracks_controller->get_modify_distance_by();
	if (new_distance < tracks_controller->get_maximum_distance_allowed() && new_distance > tracks_controller->get_minimum_distance_allowed()) {
		tracks_controller->set_distance_to_person(new_distance);
	}
}
//----------------------------------------------------------------
void ping_sensor(int sensor, t_head_controller *head_controller) {
	head_controller->ping_sensor(sensor);
}
//----------------------------------------------------------------
int read_sensor(int sensor, t_head_controller *head_controller) {
	return head_controller->read_sensor(sensor);
}
//----------------------------------------------------------------
void move_closer(t_tracks_controller *tracks_controller, int current_distance) {
	tracks_controller->move_forward(current_distance, tracks_controller->get_distance_to_person());
}
//----------------------------------------------------------------
void move_farther(t_tracks_controller *tracks_controller, int current_distance) {
	tracks_controller->move_backwards(current_distance, tracks_controller->get_distance_to_person());
}
//----------------------------------------------------------------
void execute_commands_from_queue(t_head_controller *head_controller, t_tracks_controller *tracks_controller) {
	head_controller->execute_commands_from_queue();
	tracks_controller->execute_commands_from_queue();
}
//----------------------------------------------------------------


int main() {
	freopen("out.txt", "w", stdout);

	t_object_detector object_detector;
	t_tracks_controller tracks_controller;
	t_head_controller head_controller;
	VideoCapture capture;
	Mat frame;
	bool face_detected = false;
	Point biggest_face_detected;
	int center_x = 0, center_y = 0;
	//-----------------------------------------------
	//move the head to the home position
	go_to_home_position(&head_controller);
	//-----------------------------------------------
	//open the video stream
	capture.open(0);
	if (!capture.isOpened()) {
		printf("Error opening video capture\n");
		return -1;
	}
	//-----------------------------------------------
	while (capture.read(frame)) {
		//-----------------------------------------------
		//read the data from the serial port
		read_from_serial_port(&head_controller);
		//-----------------------------------------------
		//read a frame and display it
		if (frame.empty()) {
			printf("No captured frame!");
			return -1;
		}
		imshow(object_detector.window_name, frame);
		//-----------------------------------------------
		//draw the rectangles in which the hands are being searched for
		rectangle(frame, Point(0, 0), Point(frame.size().width / 4, frame.size().height / 2), Scalar(200, 0, 0), 8);
		rectangle(frame, Point(frame.size().width / 2 + frame.size().width / 4, 0), Point(frame.size().width, frame.size().height / 2), Scalar(200, 0, 0), 8);
		//-----------------------------------------------
		//detect the biggest face in the frame
		face_detected = object_detector.detect_faces_and_display(frame);
		//-----------------------------------------------
		if (face_detected) {
			printf("-----------------------------------------------\n");
			//check if the face detected is at the center of the image
			biggest_face_detected = object_detector.get_biggest_face_origin();
			center_x = biggest_face_detected.x + (object_detector.get_biggest_face_width() / 2);
			center_y = biggest_face_detected.y + (object_detector.get_biggest_face_height() / 2);
			//-----------------------------------------------
			//if detected face moved vertically, robot must align its head with the person
			if (center_y < frame.rows / 2 - TOLERANCE) {
				//if b.y > c.y => move down
				//compute the displacement and correct it
				tracking_data angle_offset = get_offset_angles(920, Point(center_x, center_y));
				int num_steps_y = angle_offset.degrees_from_center_y / HEAD_MOTOR_STEP_ANGLE * HEAD_MOTOR_REDUCTION;
				printf("align head down\n");
				//align_head(num_steps_y, &head_controller, MOVE_DOWN);
			} else if (center_y > frame.rows / 2 + TOLERANCE) {
				//if b.y < c.y => move up
				//compute the displacement and correct it
				tracking_data angle_offset = get_offset_angles(920, Point(center_x, center_y));
				int num_steps_y = angle_offset.degrees_from_center_y / HEAD_MOTOR_STEP_ANGLE * HEAD_MOTOR_REDUCTION;
				printf("align head up\n");
				//align_head(num_steps_y, &head_controller, MOVE_UP);
			}
			//-----------------------------------------------
			//if detected face moved horizontally, robot must align itself with the person
			if (center_x > frame.cols / 2 + TOLERANCE) {
				//if b.x > c.x => move left
				//compute the displacement and correct it
				tracking_data angle_offset = get_offset_angles(920, Point(center_x, center_y));
				int num_steps_x = (int) (angle_offset.degrees_from_center_x / TRACKS_MOTOR_STEP_ANGLE * 8) * TRACKS_MOTOR_REDUCTION;
				printf("align right\n");
				//align_with_person(num_steps_x, &tracks_controller, MOVE_RIGHT);
			} else if (center_x < frame.cols / 2 - TOLERANCE) {
				//if b.x < c.x => move right
				//compute the displacement and correct it
				tracking_data angle_offset = get_offset_angles(920, Point(center_x, center_y));
				int num_steps_x = (int) (angle_offset.degrees_from_center_x / TRACKS_MOTOR_STEP_ANGLE * 8) * TRACKS_MOTOR_REDUCTION;
				printf("align left\n");
				//align_with_person(num_steps_x, &tracks_controller, MOVE_LEFT);
			} else {
				//robot is aligned with the person, so now it has to move accordingly
				//-----------------------------------------------
				//check if a hand is detected to signal the changing of the distance to the person
				if (object_detector.right_hand_detected(frame)) {
					//if right hand was detected reduce the distance
					printf("reduce distance\n");
					reduce_distance_to_person(&tracks_controller);
				}
				if (object_detector.left_hand_detected(frame)) {
					//if left hand was detected increase the distance
					printf("increase distance\n");
					increase_distance_to_person(&tracks_controller);
				}
				//-----------------------------------------------
				//ping the distance sensor for later readings
				ping_sensor(ULTRASONIC, &head_controller);
				int current_distance = read_sensor(ULTRASONIC, &head_controller);
				//-----------------------------------------------
				//determine distance to person and move forward/backward accordingly
				if (current_distance > 0) {
					double d = tracks_controller.get_distance_to_person();
					printf("d=%f\n", d);
					if (current_distance < tracks_controller.get_maximum_distance_allowed()) {
						if (current_distance >= (d + DISTANCE_TOLERANCE)) {
							printf("move closer\n");
							//move_closer(&tracks_controller, current_distance);
						} else {
							if (current_distance < tracks_controller.get_minimum_distance_allowed()) {
								printf("move farther\n");
								//move_farther(&tracks_controller, current_distance);
							}
						}
					} else {
						printf("move closer\n");
						//move_closer(&tracks_controller, current_distance);
					}
				}
				//-----------------------------------------------
			}

			//if (current_distance > tracks_controller.get_minimum_distance_allowed()) {
			//	if (current_distance < tracks_controller.get_maximum_distance_allowed()) {
			//		if (current_distance < tracks_controller.get_distance_to_person() - TOLERANCE ||
			//			current_distance < tracks_controller.get_distance_to_person() + TOLERANCE) {
			//			move_closer(&tracks_controller, current_distance);
			//		}
			//	}
			//	else {
			//		move_closer(&tracks_controller, current_distance);
			//	}
			//}
			//else {
			//	move_farther(&tracks_controller, current_distance);
			//}

		}
		//-----------------------------------------------
		//make sure the above commands are executed and not stuck in queue
		execute_commands_from_queue(&head_controller, &tracks_controller);
		//-----------------------------------------------
		//escape
		int c = waitKey(10);
		if ((char) c == 27) {
			break;
		}
		//-----------------------------------------------
	}

	return 0;
}




/**Tests for detection accuracy**/
//int main() {
//	t_object_detector object_detector;
//
//	String hand_cascade_name = "classifiers\\Hand.Cascade.1.xml", palm_cascade_name = "classifiers\\palm.xml";
//	cv::CascadeClassifier hand_cascade, palm_cascade;
//	Scalar right(235, 100, 10), left(80, 227, 80);
//
//	VideoCapture capture;
//	Mat frame;
//
//	String filename = "frames/";
//	int count = 0, hand_count = 0, palm_count = 0;//face_count = 0;
//
//	if (!hand_cascade.load(hand_cascade_name)) {
//		printf("--(!)Error loading hand cascade!\n");
//		return -1;
//	}
//	else {
//		printf("Loaded hand cascade...\n");
//	}
//	if (!palm_cascade.load(palm_cascade_name)) {
//		printf("--(!)Error loading palm cascade!\n");
//		return -1;
//	}
//	else {
//		printf("Loaded palm cascade...\n");
//	}
//
//	Open and Read the video stream from the webcam
//		capture.open(0);
//	if (!capture.isOpened()) {
//		printf("--(!)Error opening video capture\n");
//		return -1;
//	}
//	capture.read(frame);
//
//	time_t start = time(NULL), now = time(NULL);
//	double seconds = 0;
//	while (capture.read(frame) && (difftime(now, start) <= 20)) {
//		now = time(NULL);
//		printf("loop time is : %s", ctime(&now));
//
//		//Open a window to display the resulting frames
//		imshow(object_detector.window_name, frame);
//		//imshow("Hands", frame);
//
//		//Check if a frame was read
//		if (frame.empty()) {
//			printf(" --(!) No captured frame -- Break!");
//			return -1;
//		}
//		//Mark the corners in which a hand can be detected
//		rectangle(frame, Point(0, 0), Point(frame.size().width / 4, frame.size().height / 2), Scalar(200, 0, 0), 8);
//		rectangle(frame, Point(frame.size().width / 2 + frame.size().width / 4, 0), Point(frame.size().width, frame.size().height / 2), Scalar(200, 0, 0), 8);
//		//Call method for detecting a face
//		//if (object_detector.detect_faces_and_display(frame)) {
//		//	face_count++;
//		//	cout << "Detected face: " << faceCount << endl ;
//		//}
//		//Call methods for detecting the hands in the corners
//		if (object_detector.right_hand_detected(frame)) {
//			hand_count++;
//		}
//		if (object_detector.left_hand_detected(frame)) {
//			hand_count++;
//		}
//
//		//vector<Rect> hands, palms;
//		////Detect hands with the classifier
//		//hand_cascade.detectMultiScale(frame, hands, 1.1, 3, CV_HAAR_FIND_BIGGEST_OBJECT | CV_HAAR_SCALE_IMAGE);
//		//for (int i = 0; i < hands.size(); i++) {
//		//	//Mat hand_frame = frame(Rect(hands[i].x, hands[i].y, hands[i].width, hands[i].height));
//		//	//palm_cascade.detectMultiScale(hand_frame, palms, 1.1, 3, CV_HAAR_FIND_BIGGEST_OBJECT | CV_HAAR_SCALE_IMAGE);
//		//	//if (palms.size()>0) {
//		//		if (hands[i].x < frame.cols / 2) {
//		//			rectangle(frame, Point(hands[i].x, hands[i].y), Point(hands[i].x + hands[i].width, hands[i].y + hands[i].height), right);
//		//			//rectangle(hand_frame, Point(palms[0].x, palms[0].y), Point(palms[0].x + palms[0].width, palms[0].y + palms[0].height), Scalar(255, 10, 10));
//		//			hand_count++;
//		//		}
//		//	//}
//		//	imshow("Hands", frame);
//		//}
//
//		count++;
//		stringstream file;
//		file << filename << count << ".jpg";
//		imwrite(file.str(), frame);
//
//		//Check if ESC was pressed in order to stop the program
//		int c = waitKey(10);
//		if ((char)c == 27) {
//			break;
//		}
//	}
//	printf("count: %d\nhand: %d\n", count, hand_count);
//	cin.get();
//	return 0;
//}
//
