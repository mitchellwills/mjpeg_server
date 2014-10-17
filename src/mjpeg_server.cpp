/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Robert Bosch LLC.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Robert Bosch nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <vector>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>

#include "mjpeg_server/mjpeg_server.h"
#include "mjpeg_server/http_server/http_reply.hpp"


namespace mjpeg_server
{

  ImageStreamer::ImageStreamer(const http_server::HttpRequest& request,
			       http_server::HttpConnectionPtr connection,
			       image_transport::ImageTransport it)
    : connection_(connection), inactive_(false) {
  std::string topic = request.get_query_param_value_or_default("topic", "");
  width_ = request.get_query_param_value_or_default<int>("width", -1);
  height_ = request.get_query_param_value_or_default<int>("height", -1);
  image_sub_ = it.subscribe(topic, 1, &ImageStreamer::imageCallback, this);
}
void ImageStreamer::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  if(inactive_)
    return;
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat img = cv_ptr->image;

    if(width_ > 0 && height_ > 0) {
      cv::Mat img_resized;
      cv::Size new_size(width_, height_);
      cv::resize(img, img_resized, new_size);
      sendImage(img_resized);
    }
    else{
      sendImage(img);
    }
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    inactive_ = true;
    return;
  }
  catch (boost::system::system_error& e) {
    ROS_ERROR("system_error exception: %s", e.what());
    inactive_ = true;
    return;
  }
}
bool ImageStreamer::isInactive() {
  return inactive_;
}

MjpegStreamer::MjpegStreamer(const http_server::HttpRequest& request,
			       http_server::HttpConnectionPtr connection,
			       image_transport::ImageTransport it)
  : ImageStreamer(request, connection, it){
  quality_ = request.get_query_param_value_or_default<int>("quality", 95);

  http_server::HttpReply::builder(http_server::HttpReply::ok)
    .header("Connection", "close")
    .header("Server", "mjpeg_server")
    .header("Cache-Control", "no-cache, no-store, must-revalidate, pre-check=0, post-check=0, max-age=0")
    .header("Pragma", "no-cache")
    .header("Content-type", "multipart/x-mixed-replace;boundary=--boundarydonotcross ")
    .header("Access-Control-Allow-Origin", "*")
    .write(connection);
  connection->write("--boundarydonotcross \r\n");
}
bool MjpegStreamer::sendImage(const cv::Mat& img) {
  std::vector<int> encode_params;
  encode_params.push_back(CV_IMWRITE_JPEG_QUALITY);
  encode_params.push_back(quality_);

  std::vector<uchar> encoded_buffer;
  cv::imencode(".jpeg", img, encoded_buffer, encode_params);

  std::vector<http_server::HttpHeader> headers;
  headers.push_back(http_server::HttpHeader("Content-type", "image/jpeg"));
  headers.push_back(http_server::HttpHeader("Content-Length", boost::lexical_cast<std::string>(encoded_buffer.size())));
  connection_->write(http_server::HttpReply::to_buffers(headers));
  connection_->write(boost::asio::buffer(encoded_buffer));
  connection_->write("\r\n--boundarydonotcross \r\n");
}

JpegSnapshotStreamer::JpegSnapshotStreamer(const http_server::HttpRequest& request,
			       http_server::HttpConnectionPtr connection,
			       image_transport::ImageTransport it)
  : ImageStreamer(request, connection, it){
  quality_ = request.get_query_param_value_or_default<int>("quality", 95);
}
bool JpegSnapshotStreamer::sendImage(const cv::Mat& img) {
  std::vector<int> encode_params;
  encode_params.push_back(CV_IMWRITE_JPEG_QUALITY);
  encode_params.push_back(quality_);

  std::vector<uchar> encoded_buffer;
  cv::imencode(".jpeg", img, encoded_buffer, encode_params);

  http_server::HttpReply::builder(http_server::HttpReply::ok)
    .header("Connection", "close")
    .header("Server", "mjpeg_server")
    .header("Cache-Control", "no-cache, no-store, must-revalidate, pre-check=0, post-check=0, max-age=0")
    .header("Pragma", "no-cache")
    .header("Content-type", "image/jpeg")
    .header("Access-Control-Allow-Origin", "*")
    .header("Content-Length", boost::lexical_cast<std::string>(encoded_buffer.size()))
    .write(connection_);
  connection_->write(boost::asio::buffer(encoded_buffer));
  inactive_ = true;
}


MjpegServer::MjpegServer(ros::NodeHandle& nh, ros::NodeHandle& private_nh) :
  nh_(nh), image_transport_(nh),
  handler_group_(http_server::HttpReply::stock_reply(http_server::HttpReply::not_found)) {

  int port;
  private_nh.param("port", port, 8080);

  handler_group_.addHandlerForPath("/", boost::bind(&MjpegServer::handle_list_streams, this, _1, _2));
  handler_group_.addHandlerForPath("/stream", boost::bind(&MjpegServer::handle_stream, this, _1, _2));
  handler_group_.addHandlerForPath("/stream_viewer", boost::bind(&MjpegServer::handle_stream_viewer, this, _1, _2));
  handler_group_.addHandlerForPath("/snapshot", boost::bind(&MjpegServer::handle_snapshot, this, _1, _2));

  server_.reset(new http_server::HttpServer("0.0.0.0", boost::lexical_cast<std::string>(port), handler_group_, 5));
}

MjpegServer::~MjpegServer() {
}

void MjpegServer::spin() {
  server_->run();
  while(ros::ok()){
    ros::spinOnce();
    cleanup_inactive_streams();
  }
  server_->stop();
}

void MjpegServer::cleanup_inactive_streams(){
  boost::mutex::scoped_lock lock(subscriber_mutex_, boost::try_to_lock);
  if(lock) {
    image_subscribers_.erase(std::remove_if(image_subscribers_.begin(), image_subscribers_.end(), boost::bind(&ImageStreamer::isInactive, _1)),
			     image_subscribers_.end());
  }
}


void MjpegServer::handle_stream(const http_server::HttpRequest& request,
				http_server::HttpConnectionPtr connection) {
  boost::mutex::scoped_lock lock(subscriber_mutex_);
  image_subscribers_.push_back(boost::shared_ptr<ImageStreamer>(new MjpegStreamer(request, connection, image_transport_)));
}

void MjpegServer::handle_snapshot(const http_server::HttpRequest& request,
				  http_server::HttpConnectionPtr connection) {
  boost::mutex::scoped_lock lock(subscriber_mutex_);
  image_subscribers_.push_back(boost::shared_ptr<ImageStreamer>(new JpegSnapshotStreamer(request, connection, image_transport_)));
}

void MjpegServer::handle_stream_viewer(const http_server::HttpRequest& request,
				      http_server::HttpConnectionPtr connection) {
  std::string topic = request.get_query_param_value_or_default("topic", "");
  std::string width = request.get_query_param_value_or_default("width", "640");
  std::string height = request.get_query_param_value_or_default("height", "480");

  http_server::HttpReply::builder(http_server::HttpReply::ok)
    .header("Connection", "close")
    .header("Server", "mjpeg_server")
    .header("Content-type", "text/html;")
    .write(connection);

  std::stringstream ss;
  ss << "<html><head><title>" << topic << "</title></head><body>";
  ss << "<h1>" << topic << "</h1>";
  ss << "<img src=\"/stream?";
  ss << request.query;
  ss << "\" width=\"" << width << "\" height=\"" << height << "\"></img>";
  ss << "</body></html>";
  connection->write(ss.str());
}

void MjpegServer::handle_list_streams(const http_server::HttpRequest& request,
				      http_server::HttpConnectionPtr connection) {
  std::string image_message_type = ros::message_traits::datatype<sensor_msgs::Image>();
  std::string camera_info_message_type = ros::message_traits::datatype<sensor_msgs::CameraInfo>();

  ros::master::V_TopicInfo topics;
  ros::master::getTopics( topics );
  ros::master::V_TopicInfo::iterator it;
  std::vector<std::string> image_topics;
  std::vector<std::string> camera_info_topics;
  for( it = topics.begin(); it != topics.end(); ++it ) {
    const ros::master::TopicInfo& topic = *it;
    if (topic.datatype == image_message_type) {
	image_topics.push_back(topic.name);
    }
    else if (topic.datatype == camera_info_message_type) {
	camera_info_topics.push_back(topic.name);
    }
  }


  http_server::HttpReply::builder(http_server::HttpReply::ok)
    .header("Connection", "close")
    .header("Server", "mjpeg_server")
    .header("Cache-Control", "no-cache, no-store, must-revalidate, pre-check=0, post-check=0, max-age=0")
    .header("Pragma", "no-cache")
    .header("Content-type", "text/html;")
    .write(connection);

  connection->write("<html>"
		    "<head><title>ROS Image Topic List</title></head>"
		    "<body><h1>Available ROS Image Topics:</h1>");
  connection->write("<ul>");
  BOOST_FOREACH(std::string& camera_info_topic, camera_info_topics){
    if(boost::algorithm::ends_with(camera_info_topic, "/camera_info")){
	std::string base_topic = camera_info_topic.substr(0, camera_info_topic.size()-strlen("camera_info"));
	connection->write("<li>");
	connection->write(base_topic);
	connection->write("<ul>");
	std::vector<std::string>::iterator image_topic_itr = image_topics.begin();
	for ( ; image_topic_itr != image_topics.end(); ) {
	  if (boost::starts_with(*image_topic_itr, base_topic)) {
	    connection->write("<li><a href=\"/stream_viewer?topic=");
	    connection->write(*image_topic_itr);
	    connection->write("\">");
	    connection->write(image_topic_itr->substr(base_topic.size()));
	    connection->write("</a> (");
	    connection->write("<a href=\"/snapshot?topic=");
	    connection->write(*image_topic_itr);
	    connection->write("\">Snapshot</a>)");
	    connection->write("</li>");

	    image_topic_itr = image_topics.erase(image_topic_itr);
	  } else {
	    ++image_topic_itr;
	  }
	}
	connection->write("</ul>");
    }
    connection->write("</li>");
  }
  connection->write("</ul></body></html>");
}

}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "mjpeg_server");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  mjpeg_server::MjpegServer server(nh, private_nh);
  server.spin();

  return (0);
}

