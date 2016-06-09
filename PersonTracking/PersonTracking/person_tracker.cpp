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
				printf("-----align head down\n");
				align_head(num_steps_y, &head_controller, MOVE_DOWN);
			}
			else if (center_y > frame.rows / 2 + TOLERANCE) {
				//if b.y < c.y => move up
				//compute the displacement and correct it
				tracking_data angle_offset = get_offset_angles(920, Point(center_x, center_y));
				int num_steps_y = angle_offset.degrees_from_center_y / HEAD_MOTOR_STEP_ANGLE * HEAD_MOTOR_REDUCTION;
				printf("-----align head up\n");
				align_head(num_steps_y, &head_controller, MOVE_UP);
			}
			//-----------------------------------------------
			//if detected face moved horizontally, robot must align itself with the person
			if (center_x > frame.cols / 2 + TOLERANCE) {
				//if b.x > c.x => move left
				//compute the displacement and correct it
				tracking_data angle_offset = get_offset_angles(920, Point(center_x, center_y));
				int num_steps_x = (int)(angle_offset.degrees_from_center_x / TRACKS_MOTOR_STEP_ANGLE * 8) * TRACKS_MOTOR_REDUCTION;
				printf("-----align tracks right\n");
				align_with_person(num_steps_x, &tracks_controller, MOVE_RIGHT);
			}
			else if (center_x < frame.cols / 2 - TOLERANCE) {
				//if b.x < c.x => move right
				//compute the displacement and correct it
				tracking_data angle_offset = get_offset_angles(920, Point(center_x, center_y));
				int num_steps_x = (int)(angle_offset.degrees_from_center_x / TRACKS_MOTOR_STEP_ANGLE * 8) * TRACKS_MOTOR_REDUCTION;
				printf("-----align tracks left\n");
				align_with_person(num_steps_x, &tracks_controller, MOVE_LEFT);
			}
			else {
			//robot is aligned with the person, so now it has to move accordingly
			//-----------------------------------------------
			//check if a hand is detected to signal the changing of the distance to the person
			if (object_detector.right_hand_detected(frame)) {
				//if right hand was detected reduce the distance
				printf("-----reduce distance\n");
				reduce_distance_to_person(&tracks_controller);
			}
			if (object_detector.left_hand_detected(frame)) {
				//if left hand was detected increase the distance
				printf("-----increase distance\n");
				increase_distance_to_person(&tracks_controller);
			}
			//-----------------------------------------------
			//ping the distance sensor for later readings
			ping_sensor(ULTRASONIC, &head_controller);
			int current_distance = read_sensor(ULTRASONIC, &head_controller);
			//-----------------------------------------------
			//determine distance to person and move forward/backward accordingly
			//if the distance read is relevant, or if the distance was read correctly
			if (current_distance > 0) {
				double d = tracks_controller.get_distance_to_person();
				printf("---distance to maintain=%f\n", d);
				if (current_distance < tracks_controller.get_maximum_distance_allowed()) {
					if (current_distance >= (d + DISTANCE_TOLERANCE)) {
						printf("-----move closer\n");
						move_closer(&tracks_controller, current_distance);
					} else {
						if ((current_distance < (d - DISTANCE_TOLERANCE)) ||
							(current_distance < tracks_controller.get_minimum_distance_allowed())) {
							printf("-----move farther\n");
							move_farther(&tracks_controller, current_distance);
						} else {
							printf("-----do nothing\n");
						}
					}
				} else {
					printf("-----move closer\n");
					move_closer(&tracks_controller, current_distance);
				}
				}
				//-----------------------------------------------
			}
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
