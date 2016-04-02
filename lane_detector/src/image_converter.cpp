#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

static const std::string OPENCV_WINDOW = "Image window";

float left_s=90.00,right_s=90.00;

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  
public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/image", 1, &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
	cv::Mat img=cv_ptr->image;
    // Draw an example circle on the video stream

    /*if (img.rows > 60 && img.cols > 60)
      cv::circle(img, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, img);
    cv::waitKey(3);*/
	
	resize(img,img, Size(640,360));

	//split image in three channels, run edge detector on each and merge
	Mat edges, b_edges, g_edges, r_edges, b_channel, g_channel, r_channel, temp;
	vector<Mat> spl;
	split(img,spl);
	b_channel=spl[0];
	g_channel=spl[1];
	r_channel=spl[2];

	int low_threshold=85, high_threshold=150;
  	blur( b_channel, temp, Size(3,3) );
  	Canny( temp, b_edges, low_threshold, high_threshold, 3 );

  	blur( g_channel, temp, Size(3,3) );
  	Canny( temp, g_edges, low_threshold, high_threshold, 3 );

  	blur( r_channel, temp, Size(3,3) );
  	Canny( temp, r_edges, low_threshold, high_threshold, 3 );

  	//imshow( "blue edges", b_edges  );
  	//imshow( "green edges", g_edges  );
  	//imshow( "red edges", r_edges  );

	edges=b_edges+ g_edges+ r_edges;

	//find hough lines and display
	vector<Vec2f> lines;
  	HoughLines(edges, lines, 1, CV_PI/180, 120, 0, 0 );

	Mat hlines(img.rows,img.cols,CV_8UC3,Scalar(0,0,0));
	for( size_t i = 0; i < lines.size(); i++ )
  	{
     		float rho = lines[i][0], theta = lines[i][1];
     		Point pt1, pt2;
     		double a = cos(theta), b = sin(theta);
     		double x0 = a*rho, y0 = b*rho;
     		pt1.x = cvRound(x0 + 1000*(-b));
     		pt1.y = cvRound(y0 + 1000*(a));
     		pt2.x = cvRound(x0 - 1000*(-b));
     		pt2.y = cvRound(y0 - 1000*(a));
     		line( hlines, pt1, pt2, Scalar(0,0,255), 3, CV_AA);
  	}

	//remove non-lane lines
	bool touches_edges[lines.size()];
	int i, j, k;
	for(i=0;i<lines.size();i++) touches_edges[i]=false;


	/*Mat test(img.rows,img.cols,CV_8UC3,Scalar(0,0,0));
	for(i=0;i<img.rows;i++)
		for(j=0;j<img.cols;j++)
			for(k=0;k<lines.size();k++)
				if( abs( j*cos(lines[k][1]) +i*sin(lines[k][1]) - lines[k][0])  <=5 ) test.at<Vec3b>(i,j)[1]=255;
	imshow("test", test);*/

	int margin=10;
	int line_error=4;
	for(i=0;i<img.rows;i++)
		for(j=0;j<img.cols;j++)
		{
			if(!( i>= img.rows-1-margin || j<=margin || j>=img.cols-1-margin )) continue;
			for(k=0;k<lines.size();k++)
			{
				if( abs( j*cos(lines[k][1]) +i*sin(lines[k][1]) - lines[k][0]) <=line_error && edges.at<uchar>(i,j)>=20 )
					touches_edges[k]=true;
			}
		}

	for(i=lines.size()-1; i>=0; i--)
		if(touches_edges[i]==false) lines.erase(lines.begin()+i);

	Mat lanes(img.rows,img.cols,CV_8UC3,Scalar(0,0,0));
	for( size_t i = 0; i < lines.size(); i++ )
  	{
     		float rho = lines[i][0], theta = lines[i][1];
     		Point pt1, pt2;
     		double a = cos(theta), b = sin(theta);
     		double x0 = a*rho, y0 = b*rho;
     		pt1.x = cvRound(x0 + 1000*(-b));
     		pt1.y = cvRound(y0 + 1000*(a));
     		pt2.x = cvRound(x0 - 1000*(-b));
     		pt2.y = cvRound(y0 - 1000*(a));
     		line( lanes, pt1, pt2, Scalar(255,255,0), 3, CV_AA);
  	}
	
	//divide lanes into right and left and display
	vector<Vec2f> right_lines;
	vector<Vec2f> left_lines;
	for(i=0;i<lines.size();i++)
		if( ( lines[i][0]-img.rows*sin(lines[i][1]) )/cos(lines[i][1]) >=img.cols/2  ) right_lines.push_back(lines[i]);
		else left_lines.push_back(lines[i]);

	//cout<<"lines detected:"<<lines.size()<<endl;
	//cout<<"left lines detected:"<<left_lines.size()<<endl;
	//cout<<"right lines detected:"<<right_lines.size()<<endl;
	


	Mat right_lanes(img.rows,img.cols,CV_8UC3,Scalar(0,0,0));
	for( size_t i = 0; i < right_lines.size(); i++ )
  	{
     		float rho = right_lines[i][0], theta = right_lines[i][1];
     		Point pt1, pt2;
     		double a = cos(theta), b = sin(theta);
     		double x0 = a*rho, y0 = b*rho;
     		pt1.x = cvRound(x0 + 1000*(-b));
     		pt1.y = cvRound(y0 + 1000*(a));
     		pt2.x = cvRound(x0 - 1000*(-b));
     		pt2.y = cvRound(y0 - 1000*(a));
     		line( right_lanes, pt1, pt2, Scalar(255,255,0), 3, CV_AA);
  	}

	Mat left_lanes(img.rows,img.cols,CV_8UC3,Scalar(0,0,0));
	for( size_t i = 0; i < left_lines.size(); i++ )
  	{
     		float rho = left_lines[i][0], theta = left_lines[i][1];
     		Point pt1, pt2;
     		double a = cos(theta), b = sin(theta);
     		double x0 = a*rho, y0 = b*rho;
     		pt1.x = cvRound(x0 + 1000*(-b));
     		pt1.y = cvRound(y0 + 1000*(a));
     		pt2.x = cvRound(x0 - 1000*(-b));
     		pt2.y = cvRound(y0 - 1000*(a));
     		line( left_lanes, pt1, pt2, Scalar(255,255,0), 3, CV_AA);
  	}

	//imshow("left lanes", left_lanes);
	//imshow("right lanes", right_lanes);

	//merge lanes and display
	float theta_margin=2, rho_margin=60;
	int breaker;
	int iterations=5;
	Vec2f temp_line;
	while(iterations--)
	{
		breaker=0;
		for(i=0;i<=right_lines.size() && breaker==0;i++)
			for(j=0;j<=right_lines.size() && breaker==0;j++)
			{
				if(i==j) continue;
				if( abs(right_lines[i][1]-right_lines[j][1])<=theta_margin && abs(right_lines[i][0]-right_lines[j][0])<=rho_margin )
				{
					temp_line[0]=(right_lines[i][0]+right_lines[j][0])/2;
					temp_line[1]=(right_lines[i][1]+right_lines[j][1])/2;
					right_lines.push_back(temp_line);
					right_lines.erase(right_lines.begin()+i);
					right_lines.erase(right_lines.begin()+j);
					if(i>=right_lines.size() || j>=right_lines.size()) breaker=1;
					if(right_lines.size()==1) {breaker=1; iterations=0;}
				}
			}
	}
	//cout<<"right lines after merging:"<<right_lines.size()<<endl;
	

	float hori_margin=10*CV_PI/180;
	for(i=0;i<right_lines.size();i++)
	if( abs(right_lines[i][1]-90*CV_PI/180)<=hori_margin || abs( 270*CV_PI/180-right_lines[i][1]<=hori_margin)) {right_lines.erase(right_lines.begin()+i); cout<<"deleted"<<endl;}
	//cout<<CV_PI/180<<"  "<<right_lines[0][1]<<endl;
	//cout<<right_lines.size()<<endl;
	
	Mat right_merged_lanes(img.rows,img.cols,CV_8UC3,Scalar(0,0,0));
	for( size_t i = 0; i < right_lines.size(); i++ )
  	{
     		float rho = right_lines[i][0], theta = right_lines[i][1];
     		Point pt1, pt2;
     		double a = cos(theta), b = sin(theta);
     		double x0 = a*rho, y0 = b*rho;
     		pt1.x = cvRound(x0 + 1000*(-b));
     		pt1.y = cvRound(y0 + 1000*(a));
     		pt2.x = cvRound(x0 - 1000*(-b));
     		pt2.y = cvRound(y0 - 1000*(a));
     		line( right_merged_lanes, pt1, pt2, Scalar(255,255,0), 3, CV_AA);
  	}

	iterations=5;
	while(iterations--)
	{
		breaker=0;
		for(i=0;i<=left_lines.size() && breaker==0;i++)
			for(j=0;j<=left_lines.size() && breaker==0;j++)
			{
				if(i==j) continue;
				if( abs(left_lines[i][1]-left_lines[j][1])<=theta_margin && abs(left_lines[i][0]-left_lines[j][0])<=rho_margin )
				{
					temp_line[0]=(left_lines[i][0]+left_lines[j][0])/2;
					temp_line[1]=(left_lines[i][1]+left_lines[j][1])/2;
					left_lines.push_back(temp_line);
					left_lines.erase(left_lines.begin()+i);
					left_lines.erase(left_lines.begin()+j);
					if(i>=left_lines.size() || j>=left_lines.size() ) breaker=1;
					if(left_lines.size()==1) {breaker=1; iterations=0;}
				}
			}
	}
	//cout<<"left lines after merging:"<<left_lines.size()<<endl;
	hori_margin=20*CV_PI/180;
	for(i=0;i<left_lines.size();i++)
	if( abs(left_lines[i][1]-90*CV_PI/180)<=hori_margin || abs( 270*CV_PI/180-left_lines[i][1]<=hori_margin)) {left_lines.erase(left_lines.begin()+i); cout<<"deleted"<<endl;}
	//cout<<CV_PI/180<<"  "<<left_lines[0][1]<<endl;
	//cout<<left_lines.size()<<endl;
	
	Mat left_merged_lanes(img.rows,img.cols,CV_8UC3,Scalar(0,0,0));
	for( size_t i = 0; i < left_lines.size(); i++ )
  	{
     		float rho = left_lines[i][0], theta = left_lines[i][1];
     		Point pt1, pt2;
     		double a = cos(theta), b = sin(theta);
     		double x0 = a*rho, y0 = b*rho;
     		pt1.x = cvRound(x0 + 1000*(-b));
     		pt1.y = cvRound(y0 + 1000*(a));
     		pt2.x = cvRound(x0 - 1000*(-b));
     		pt2.y = cvRound(y0 - 1000*(a));
     		line( left_merged_lanes, pt1, pt2, Scalar(255,255,0), 3, CV_AA);
  	}
	
	//inverse perspective mapping
	Mat output=right_merged_lanes+left_merged_lanes;
	
	cv::Mat quad = cv::Mat::zeros(800, 800 ,CV_8UC3);

	std::vector<cv::Point2f> quad_pts;
	quad_pts.push_back(cv::Point2f(149, 183));
quad_pts.push_back(cv::Point2f(484, 183));
quad_pts.push_back(cv::Point2f(627, 283));
quad_pts.push_back(cv::Point2f(37, 283));

  	std::vector<cv::Point2f> corners;
	/*corners.push_back(cv::Point2f(0, 0));
	corners.push_back(cv::Point2f(360, 0));
  	corners.push_back(cv::Point2f(360, 360));
	corners.push_back(cv::Point2f(0, 360));*/

	corners.push_back(cv::Point2f(200, 200));
	corners.push_back(cv::Point2f(600, 200));
  	corners.push_back(cv::Point2f(600, 600));
	corners.push_back(cv::Point2f(200, 600));
	
	cv::Mat transmtx = cv::getPerspectiveTransform(quad_pts, corners);

	vector<Point2f> lane_points;
	vector<Point2f> top_points;
	Mat top_view= cv::Mat::zeros(800, 800 ,CV_8UC3);
	
	if(left_lines.size()!=0)
	{
		float rho = left_lines[0][0], theta = left_lines[0][1];
     		Point pt1, pt2;
     		double a = cos(theta), b = sin(theta);
     		double x0 = a*rho, y0 = b*rho;
     		pt1.x = cvRound(x0 + 1000*(-b));
     		pt1.y = cvRound(y0 + 1000*(a));
     		pt2.x = cvRound(x0 - 1000*(-b));
     		pt2.y = cvRound(y0 - 1000*(a));
		lane_points.push_back(pt1);
		lane_points.push_back(pt2);
		perspectiveTransform(lane_points, top_points, transmtx);
		//cout<<top_points[0].x<<","<<top_points[0].y<<" "<<top_points[1].x<<","<<top_points[1].y<<endl;
		//cout<<"left slope="<<180/CV_PI*acos((float)(top_points[1].x-top_points[0].x)/sqrt(((top_points[1].x-top_points[0].x)*(top_points[1].x-top_points[0].x))+((top_points[1].y-top_points[0].y)*(top_points[1].y-top_points[0].y)))<<endl;
		left_s=180/CV_PI*atan(((float)top_points[1].y-top_points[0].y)/(top_points[1].x-top_points[0].x));
		if(left_s<0)left_s=left_s+180;
		//left_s=180/CV_PI*acos((float)(top_points[1].x-top_points[0].x)/sqrt(((top_points[1].x-top_points[0].x)*(top_points[1].x-top_points[0].x))+((top_points[1].y-top_points[0].y)*(top_points[1].y-top_points[0].y))));
		cout<<"left slope="<<left_s<<endl;
		line( top_view, top_points[0], top_points[1], Scalar(255,255,0), 3, CV_AA);		

		lane_points.erase(lane_points.begin()+0);
		lane_points.erase(lane_points.begin()+1);
		

		
	}
	if(right_lines.size()!=0)
	{
		float rho = right_lines[0][0], theta = right_lines[0][1];
     		Point pt1, pt2;
     		double a = cos(theta), b = sin(theta);
     		double x0 = a*rho, y0 = b*rho;
     		pt1.x = cvRound(x0 + 1000*(-b));
     		pt1.y = cvRound(y0 + 1000*(a));
     		pt2.x = cvRound(x0 - 1000*(-b));
     		pt2.y = cvRound(y0 - 1000*(a));
		lane_points.push_back(pt1);
		lane_points.push_back(pt2);
		perspectiveTransform(lane_points, top_points, transmtx);
		//cout<<top_points[0].x<<","<<top_points[0].y<<" "<<top_points[1].x<<","<<top_points[1].y<<endl;
		//cout<<"right slope="<<180/CV_PI*atan(((float)top_points[1].y-top_points[0].y)/(top_points[1].x-top_points[0].x))<<endl;
			right_s=180/CV_PI*atan(((float)top_points[1].y-top_points[0].y)/(top_points[1].x-top_points[0].x));
		if(right_s<0)right_s=right_s+180;

		//right_s=180/CV_PI*acos((float)(top_points[1].x-top_points[0].x)/sqrt(((top_points[1].x-top_points[0].x)*(top_points[1].x-top_points[0].x))+((top_points[1].y-top_points[0].y)*(top_points[1].y-top_points[0].y))));
		cout<<"right slope="<<right_s<<endl;
		line( top_view, top_points[0], top_points[1], Scalar(255,255,0), 3, CV_AA);	
		
		lane_points.erase(lane_points.begin()+0);
		lane_points.erase(lane_points.begin()+1);
	}
	cout<<"Angle:"<<(right_s)<<endl;
	cv::Mat quad1 = cv::Mat::zeros(800, 800 ,CV_8UC3);
	cv::warpPerspective(output, quad1, transmtx, quad.size());
	imshow("IPM-Lanedetector", quad1);

	imshow("Lanedetector",output);

	//imshow("wow", top_view);

	//imshow("merged right lanes", right_merged_lanes);
	//imshow("merged left lanes", left_merged_lanes);
	//imshow("lanes", lanes);
	//imshow("hough lines", hlines);
	//imwrite("output2.png",output);
	imshow("edges", edges);
    

	


	waitKey(5);
	cv_ptr->image=top_view;
    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
