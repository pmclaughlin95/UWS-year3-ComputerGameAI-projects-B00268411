#include "opencv2/core.hpp"
#include "opencv2/face.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
 #include <iostream>
#include <fstream>
#ifdef __GNUC__
#include <experimental/filesystem> // Full support in C++17
namespace fs = std::experimental::filesystem;
#else
#include <filesystem>
namespace fs = std::tr2::sys;
#endif

// https://msdn.microsoft.com/en-us/library/dn986850.aspx
// GCC 7.2.0 Ok on Linux
// g++ -std=c++1z 1_simple_facerec_eigenfaces.cpp -lopencv_face -lopencv_core -lopencv_imgcodecs -lstdc++fs


//disclaimer at the top of the faceRec_video which i used in an attempt do get face detection to work.
/*
* Copyright (c) 2011. Philipp Wagner <bytefish[at]gmx[dot]de>.
* Released to public domain under terms of the BSD Simplified license.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above copyright
*     notice, this list of conditions and the following disclaimer in the
*     documentation and/or other materials provided with the distribution.
*   * Neither the name of the organization nor the names of its contributors
*     may be used to endorse or promote products derived from this software
*     without specific prior written permission.
*
*   See <http://www.opensource.org/licenses/bsd-license>
*/


// Paul McLaughlin - B00268411

int main(int argc, char *argv[])
{
  std::vector<cv::Mat> images ;
  std::vector<int>     labels;
  cv::Mat frame;
  double fps = 60;
  const char win_name[] = "Live Video...";
  
  cv::VideoCapture vid_in(0);   // argument is the camera id

 

  if (!vid_in.isOpened()) {
	  std::cout << "error: Camera 0 could not be opened for capture.\n";
	  return -1;
  }

  cv::namedWindow(win_name);
   while (1) {
	  vid_in >> frame;
	  imshow(win_name, frame);
	  if (cv::waitKeyEx(1000 / fps) >= 0) // how long to wait for a key (milliseconds)
		  break;
  }

  vid_in.release();




  // Iterate through all subdirectories, looking for .pgm files
  fs::path p(argc > 1 ? argv[1] : "../att_faces");
  for (const auto &entry : fs::recursive_directory_iterator{ p }) {
//    if (fs::is_regular_file(entry.status())) { // Always false in VS
      if (entry.path().extension() == ".pgm") {
        std::string str = entry.path().parent_path().stem().string(); // s26 s27 etc.
        int label = atoi(str.c_str() + 1); // s1 -> 1
        images.push_back(cv::imread(entry.path().string().c_str(), 0));
        labels.push_back(label);
      }
//    }
  }


  // Randomly choose an image, and remove it from the main collection
  std::srand(std::time(0));
  int rand_image_id = std::rand() % images.size();
  cv::Mat testSample = images[rand_image_id];
  int   testLabel  = labels[rand_image_id];
  images.erase(images.begin() + rand_image_id);
  labels.erase(labels.begin() + rand_image_id);
  std::cout << "Actual class  = "<< testLabel<< '\n';
  std::cout << " training...";


  cv::Ptr<cv::face::BasicFaceRecognizer> model = cv::face::createEigenFaceRecognizer();

  int predictedLabel = model->predict(frame);
   model->train(images, labels);
   // an attempt using the facerec_video code from eigenfaces lab in week 10
 	  vid_in >> frame;
	  cv::Mat original = frame.clone();
	  cv::Mat gray;
	  cvtColor(original, gray, cv::COLOR_BGR2GRAY);
	  std::vector< cv::Rect_<int> > faces;

	  for (size_t i = 0; i < faces.size(); i++) {
		  cv::Rect face_i = faces[i];
		  // Crop the face from the image. So simple with OpenCV C++:
		  cv::Mat face = gray(face_i);
 

		  cv::Mat face_resized;
		  cv::resize(face, face_resized, cv::Size(92, 112), 1.0, 1.0, cv::INTER_CUBIC);

		 
		
		  std::string box_text = cv::format("Prediction = %d", predictedLabel);
		  // Calculate the position for annotated text (make sure we don't
		  // put illegal values in there):
		  int pos_x = std::max(face_i.tl().x - 10, 0);
		  int pos_y = std::max(face_i.tl().y - 10, 0);
		  // And now put it into the image:
		  putText(original, box_text, cv::Point(pos_x, pos_y), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0, 255, 0), 2);
	  }
	  
	  // Show the result:
	  imshow("face_recognizer", original);
  


 



  return 0;
}
