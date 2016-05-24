#include "../include/FaceTracker.h"
#include "../include/WheelsController.h"
#include "../include/HeadController.h"
#include "../include/jenny5_command_module.h"

//----------------------------------------------------------------
using namespace cv;

#define ULTRASONIC 0
#define TOLERANCE 20
//----------------------------------------------------------------
bool go_to_home_position(t_jenny5_command_module *head_controller) {
	// must home the head
	head_controller->send_go_home_stepper_motor(MOTOR_HEAD_HORIZONTAL);
	head_controller->send_go_home_stepper_motor(MOTOR_HEAD_VERTICAL);

	printf("Head motors home started ...");
	clock_t start_time = clock();
	bool horizontal_motor_homed = false;
	bool vertical_motor_homed = false;

	while (1) {
		if (!head_controller->update_commands_from_serial())
			Sleep(5); // no new data from serial ... we make a little pause so that we don't kill the processor

		if (!horizontal_motor_homed)
			if (head_controller->query_for_event(STEPPER_MOTOR_MOVE_DONE_EVENT, MOTOR_HEAD_HORIZONTAL))  // have we received the event from Serial ?
				horizontal_motor_homed = true;

		if (!vertical_motor_homed)
			if (head_controller->query_for_event(STEPPER_MOTOR_MOVE_DONE_EVENT, MOTOR_HEAD_VERTICAL))  // have we received the event from Serial ?
				vertical_motor_homed = true;

		if (horizontal_motor_homed && vertical_motor_homed)
			break;

		// measure the passed time 
		clock_t end_time = clock();

		double wait_time = (double) (end_time - start_time) / CLOCKS_PER_SEC;
		// if more than 5 seconds and no home
		if (wait_time > 5) {
			if (!vertical_motor_homed)
				printf("Cannot home vertical motor! Game over!");
			if (!horizontal_motor_homed)
				printf("Cannot home vertical motor! Game over!");
			return false;
		}
	}

	printf("DONE\n");
	return true;
}
//----------------------------------------------------------------
void align_with_person(int displacement_x, t_wheels_controller *ctrl) {
	if (displacement_x > 0) {
		ctrl->turn_left(displacement_x, WHEELS_MOVING);
	} else if (displacement_x < 0) {
		ctrl->turn_right(displacement_x, WHEELS_MOVING);
	}
}
//----------------------------------------------------------------
void align_head(int displacement_y, t_head_controller *ctrl) {
	displacement_y = displacement_y / 1.8 * 16.0;
	if (displacement_y > 0) {
		ctrl->move_up(displacement_y);
	} else {
		if (displacement_y < 0) {
			ctrl->move_down(displacement_y);
		}
	}
}
//----------------------------------------------------------------
void reduce_distance_to_person(t_wheels_controller *ctrl) {
	double new_distance = ctrl->get_distance_to_person() - ctrl->get_modify_distance_by();
	if (new_distance > ctrl->get_minimum_distance_allowed() && new_distance < ctrl->get_maximum_distance_allowed()) {
		ctrl->set_distance_to_person(new_distance);
	}
}
//----------------------------------------------------------------
void increase_distance_to_person(t_wheels_controller *ctrl) {
	double new_distance = ctrl->get_distance_to_person() + ctrl->get_modify_distance_by();
	if (new_distance < ctrl->get_maximum_distance_allowed() && new_distance > ctrl->get_minimum_distance_allowed()) {
		ctrl->set_distance_to_person(new_distance);
	}
}
//----------------------------------------------------------------
void ping_sensor(int sensor, t_jenny5_command_module *head_controller) {
	if (sensor == ULTRASONIC) {
		if (head_controller->get_sonar_state(0) == COMMAND_DONE) {// I ping the sonar only if no ping was sent before
			head_controller->send_get_sonar_distance(0);
			head_controller->set_sonar_state(0, COMMAND_SENT);
		}
	}
}
//----------------------------------------------------------------
int read_ultrasonic_sensor(t_jenny5_command_module *head_controller) {
	int distance = 0;;
	if (head_controller->get_sonar_state(0) == COMMAND_SENT) {// if a command has been sent
		if (head_controller->query_for_event(SONAR_EVENT, 0, &distance)) { // have we received the event from Serial ?
			head_controller->set_sonar_state(0, COMMAND_DONE);
		}
	}
	return distance;
}
//----------------------------------------------------------------
void move_closer(int wait_for, t_wheels_controller *wheels_ctrl, t_jenny5_command_module *head_ctrl) {
	int current_distance;
	while ((current_distance = read_ultrasonic_sensor(head_ctrl)) > wheels_ctrl->get_distance_to_person()) {
		wheels_ctrl->move_forward(current_distance, wait_for);
		current_distance += 100;
	}
}
//----------------------------------------------------------------
void move_farther(int wait_for, t_wheels_controller *wheels_ctrl, t_jenny5_command_module *head_ctrl) {
	int current_distance;
	while ((current_distance = read_ultrasonic_sensor(head_ctrl)) < wheels_ctrl->get_distance_to_person()) {
		wheels_ctrl->move_backwards(current_distance, wait_for);
		current_distance += 100;
	}
}
//----------------------------------------------------------------

int main() {
	t_face_tracker face_tracking;
	t_wheels_controller wheels_controller;
	t_head_controller head_controller;
	t_jenny5_command_module command_module;

	//move the head to the home position
	go_to_home_position(&command_module);

	VideoCapture capture;
	Mat frame;

	//Read the video stream
	capture.open(0);
	if (!capture.isOpened()) {
		printf("--(!)Error opening video capture\n");
		return -1;
	}
	capture.read(frame);

	Point center_frame = Point(frame.size().width / 2, frame.size().height / 2);
	while (capture.read(frame)) {
		//read a frame
		imshow(face_tracking.window_name, frame);
		if (frame.empty()) {
			printf(" --(!) No captured frame -- Break!");
			return -1;
		}

		//draw the rectangles in which the hands are being searched for
		rectangle(frame, Point(0, 0), Point(frame.size().width / 4, frame.size().height / 2), Scalar(200, 0, 0), 8);
		rectangle(frame, Point(frame.size().width / 2 + frame.size().width / 4, 0), Point(frame.size().width, frame.size().height / 2), Scalar(200, 0, 0), 8);
		
		//detect the biggest face in the frame
		bool face_detected = face_tracking.detect_faces_and_display(frame);

		//-----------------------------------------------
		if (face_detected) {
			//check if the face detected is at the center of the image
			Point biggest_face_detected = face_tracking.get_biggest_face_origin();
			//if detected face moved horizontally, robot must align itself with the person
			if (!(biggest_face_detected.x <= (center_frame.x + TOLERANCE)) && !(biggest_face_detected.x >= (center_frame.x + TOLERANCE))) {
				//compute the displacement and correct it
				//reset the center of the detected face after the correction
				int displacement_x = biggest_face_detected.x - center_frame.x;
				align_with_person(displacement_x, &wheels_controller);
			}
			//if detected face moved vertically, robot must align it's head with the person
			if (!(biggest_face_detected.y <= (center_frame.y + TOLERANCE)) && !(biggest_face_detected.y >= (center_frame.y + TOLERANCE))) {
				//compute the displacement and correct it
				//reset the center of the detected face after the correction
				int displacement_y = biggest_face_detected.y - center_frame.y;
				align_head(-displacement_y, &head_controller);
			}
			//-----------------------------------------------

			//-----------------------------------------------
			//check if a hand is detected to signal the changing of the distance to the person
			if (face_tracking.right_hand_detected(frame)) {
				//if right hand was detected reduce the distance
				reduce_distance_to_person(&wheels_controller);
			}
			if (face_tracking.left_hand_detected(frame)) {
				//if left hand was detected increase the distance
				increase_distance_to_person(&wheels_controller);
			}
			//-----------------------------------------------
			ping_sensor(ULTRASONIC, &command_module);
			//-----------------------------------------------
			//determine distance to person and move forward/backwards accordingly
			int current_distance = read_ultrasonic_sensor(&command_module);
			if (current_distance > wheels_controller.get_minimum_distance_allowed()) {
				if (current_distance < wheels_controller.get_maximum_distance_allowed()) {
					if (current_distance > wheels_controller.get_distance_to_person()) {
						move_closer(WHEELS_MOVING, &wheels_controller, &command_module);
					}
					if (current_distance < wheels_controller.get_distance_to_person()) {
						move_farther(WHEELS_MOVING, &wheels_controller, &command_module);
					}
				} else move_closer(WHEELS_MOVING, &wheels_controller, &command_module);
			} else move_farther(WHEELS_MOVING, &wheels_controller, &command_module);
			//-----------------------------------------------
		}

		// now extract the executed moves from the queue ... otherwise they will just stay there
		if (command_module.get_stepper_motor_state(MOTOR_HEAD_HORIZONTAL) == COMMAND_SENT) {// if a command has been sent
			if (command_module.query_for_event(STEPPER_MOTOR_MOVE_DONE_EVENT, MOTOR_HEAD_HORIZONTAL)) { // have we received the event from Serial ?
				command_module.set_stepper_motor_state(MOTOR_HEAD_HORIZONTAL, COMMAND_DONE);
			}
		}

		// now extract the moves done from the queue
		if (command_module.get_stepper_motor_state(MOTOR_HEAD_VERTICAL) == COMMAND_SENT) {// if a command has been sent
			if (command_module.query_for_event(STEPPER_MOTOR_MOVE_DONE_EVENT, MOTOR_HEAD_VERTICAL)) { // have we received the event from Serial ?
				command_module.set_stepper_motor_state(MOTOR_HEAD_VERTICAL, COMMAND_DONE);
			}
		}

		int c = waitKey(10);
		if ((char) c == 27) {
			break;
		} //Escape
	}

	return 0;
}



//int main() {
//	t_face_tracker face_tracking;
//	VideoCapture capture;
//	Mat frame;
//
//	String filename = "frames/";
//	int count = 0, faceCount = 0;
//	
//	//Open and Read the video stream from the webcam
//	capture.open(0);
//	if (!capture.isOpened()) {
//		printf("--(!)Error opening video capture\n");
//		return -1;
//	}
//	capture.read(frame);
//
//	time_t start = time(NULL), now = time(NULL);
//	double seconds = 0;
//	while (capture.read(frame) && (difftime(now, start) <= 60)) {
//		now = time(NULL);
//		printf("loop time is : %s", ctime(&now));
//
//		//Open a window to display the resulting frames
//		imshow(face_tracking.window_name, frame);
//
//		//Check if a frame was read
//		if (frame.empty()) {
//			printf(" --(!) No captured frame -- Break!");
//			return -1;
//		}
//		//Mark the corners in which a hand can be detected
//		rectangle(frame, Point(0, 0), Point(frame.size().width / 4, frame.size().height / 2), Scalar(200, 0, 0), 8);
//		rectangle(frame, Point(frame.size().width / 2 + frame.size().width / 4, 0), Point(frame.size().width, frame.size().height / 2), Scalar(200, 0, 0), 8);
//		
//		//Call method for detecting a face
//		if (face_tracking.detect_faces_and_display(frame)) {
//			faceCount++;
//			cout << "Detected face: " << faceCount << endl ;
//		}
//		//Call methods for detecting the hands in the corners
//		face_tracking.right_hand_detected(frame);
//		face_tracking.left_hand_detected(frame);
//
//		count++;
//		stringstream file;
//		file << filename << count << ".jpg";
//		imwrite(file.str(), frame);
//
//		//Check if ESC was pressed in order to stop the program
//		int c = waitKey(10);
//		if ((char) c == 27) {
//			break;
//		}
//	}
//	cin >> count;
//	return 0;
//}
