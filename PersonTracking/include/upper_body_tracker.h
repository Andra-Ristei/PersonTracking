#ifndef upper_body_trackerH
#define upper_body_trackerH


#include "opencv2/video/tracking.hpp"
#include "opencv2/objdetect.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include <iostream>
#include <limits>
#include <ctype.h>
#include <stdio.h>

using namespace std;
using namespace cv;


class t_upper_body_tracker {
private:
	String upperbody_cascade_name = "..\\classifiers\\haarcascade_upperbody.xml";
	CascadeClassifier upperbody_cascade;
	String window_name = "Upperbody Capture";

	/** Features params **/
	int maxCorners = 3000, blockSize = 3;
	double qualityLevel = 0.5, minDistance = 3;

	/** LK params **/
	Size winSize = Size(10, 10);
	int maxLevel = 5;
	TermCriteria criteria = TermCriteria(TermCriteria::EPS | TermCriteria::COUNT, 10, 0.03);

	void detect_and_display(Mat frame);
	void track_features(Mat&, Mat&, vector<Point2f>[]);

public:
	t_upper_body_tracker();
	~t_upper_body_tracker();
	bool track_upper_body();

};


#endif