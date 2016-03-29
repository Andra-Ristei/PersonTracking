#include "../include/upper_body_tracker.h"

t_upper_body_tracker::t_upper_body_tracker() {
	if (!upperbody_cascade.load(upperbody_cascade_name)) {
		printf("--(!)Error loading upperbody cascade\n");
		return;
	}
}

t_upper_body_tracker::~t_upper_body_tracker() {
}

void t_upper_body_tracker::track_features(Mat &ROI, Mat &gray_ROI, vector<Point2f> corners[]) {
	goodFeaturesToTrack(gray_ROI, corners[0], maxCorners, qualityLevel, minDistance, noArray(), blockSize);
	cornerSubPix(gray_ROI, corners[0], winSize, Size(-1, -1), TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03));

	vector<uchar> status;
	vector<float> err;
	calcOpticalFlowPyrLK(gray_ROI, gray_ROI, corners[0], corners[1], status, err, winSize, maxLevel, criteria);
	for (int i = 0; i < status.size(); i++) {
		if (status[i] > 0) {
			Point p0(corners[1][i].x, corners[1][i].y);
			circle(ROI, p0, 3, Scalar(100, 200, 0), 4, 8);
		}
	}
	swap(corners[1], corners[0]);
}

void t_upper_body_tracker::detect_and_display(Mat frame) {
	std::vector<Rect> upperbody;
	Mat frame_gray;
	vector<Point2f> corners[2];

	cvtColor(frame, frame_gray, COLOR_BGR2GRAY);
	equalizeHist(frame_gray, frame_gray);

	//-- Detect upperbody
	upperbody_cascade.detectMultiScale(frame_gray, upperbody, 1.1, 4, CASCADE_FIND_BIGGEST_OBJECT, Size(30, 30));

	for (size_t i = 0; i < upperbody.size(); i++) {
		Point2f upper_left = Point2f(upperbody[i].x, upperbody[i].y);
		Point2f lower_left = Point2f(upperbody[i].x, upperbody[i].y + upperbody[i].height);
		Point2f upper_right = Point2f(upperbody[i].x + upperbody[i].width, upperbody[i].y);
		Point2f lower_right = Point2f(upperbody[i].x + upperbody[i].width, upperbody[i].y + upperbody[i].height);
		Point2f center = Point2f(upperbody[i].x + upperbody[i].width / 2, upperbody[i].y + upperbody[i].height / 2);
		rectangle(frame, Point(upperbody[i].x, upperbody[i].y), Point(upperbody[i].x + upperbody[i].width, upperbody[i].y + upperbody[i].height), Scalar(200, 0, 0));
		corners[0].push_back(upper_left);
		corners[0].push_back(upper_right);
		corners[0].push_back(lower_left);
		corners[0].push_back(lower_right);
		corners[0].push_back(center);

		//set the features to track with LK
		Mat ROI = frame(Rect(upperbody[i].x, upperbody[i].y, upperbody[i].width, upperbody[i].height));
		Mat frame_ROI = frame_gray(upperbody[i]);
		track_features(ROI, frame_ROI, corners);

	}
	//-- Show what you got
	imshow(window_name, frame);
}

bool t_upper_body_tracker::track_upper_body() {
	VideoCapture capture;
	Mat frame;

	capture.open(0);
	if (!capture.isOpened()) {
		printf("--(!)Error opening video capture\n");
		return false;
	}

	while (capture.read(frame)) {
		if (frame.empty()) {
			printf(" --(!) No captured frame -- Break!");
			return false;
		}

		//-- 3. Apply the classifier to the frame
		detect_and_display(frame);

		int c = waitKey(10);
		if ((char) c == 27) {
			break;
		} // escape
	}
	return true;
}