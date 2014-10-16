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

#include <mjpeg_server/mjpeg_server.h>
#include <boost/network/uri.hpp>
#include <boost/network/uri/uri_io.hpp>
#include <boost/foreach.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <vector>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>


namespace mjpeg_server
{

ImageStreamer::ImageStreamer(http_server::connection_ptr connection,
			     image_transport::ImageTransport it,
			     std::string topic) : connection_(connection), active_(true) {
  image_sub_ = it.subscribe(topic, 1, &ImageStreamer::imageCallback, this);
}
void ImageStreamer::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    sendImage(cv_ptr);
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    active_ = false;
    return;
  }
}
bool ImageStreamer::isActive() {
  return active_;
}

bool ImageStreamer::sendImage(const cv_bridge::CvImagePtr& cv_ptr) {
  cv::Mat img = cv_ptr->image;

  int quality = 95;

  std::vector<int> encode_params;
  encode_params.push_back(CV_IMWRITE_JPEG_QUALITY);
  encode_params.push_back(quality);

  std::vector<uchar> encoded_buffer;
  cv::imencode(".jpeg", img, encoded_buffer, encode_params);


  std::stringstream content;
  content << ("HTTP/1.0 200 OK\r\n"
		     "Connection: close\r\nServer: mjpeg_server\r\n"
		     "Cache-Control: no-cache, no-store, must-revalidate, pre-check=0, post-check=0, max-age=0\r\n"
		     "Pragma: no-cache\r\n"
		     "Content-type: image/jpeg\r\n"
		     "\r\n");
  for(int i = 0; i < encoded_buffer.size(); ++i){
    content << (encoded_buffer[i]);
  }
  content << ("\r\n--boundarydonotcross \r\n");

  connection_->write(content.str());
}


MjpegServer::MjpegServer(ros::NodeHandle& nh, ros::NodeHandle& private_nh) :
  nh_(nh), image_transport_(nh), request_handler_(this)
{
  int port;
  private_nh.param("port", port, 8080);

  http_server::options options(request_handler_);
  options.address("0.0.0.0")
    .port(boost::to_string(port))
    .io_service(boost::make_shared<boost::asio::io_service>())
    .thread_pool(boost::make_shared<boost::network::utils::thread_pool>(10))
    .reuse_address(true);
  server_.reset(new http_server(options));
}

MjpegServer::~MjpegServer()
{
}

void MjpegServer::spin()
{
  boost::thread server_event_thread(boost::bind(&http_server::run, server_));
  ros::spin();
  server_->stop();
  server_event_thread.join();
}


static http_response_header create_response_header(std::string name, std::string value) {
  http_response_header header;
  header.name = name;
  header.value = value;
  return header;
}

bool MjpegServer::parse_url(std::string& url_string, std::string* path, std::map<std::string, std::string>* query_parameters) {
  boost::network::uri::uri url(std::string("http://0.0.0.0") + url_string);
  if(!url.is_valid())
    return false;
  query_parameters->clear();
  *path = url.path();
  std::string query_string = url.query();
  std::vector<std::string> pair_strings;
  boost::split(pair_strings, query_string, boost::is_any_of("&"));
  BOOST_FOREACH(std::string& pair_string, pair_strings) {
    std::vector<std::string> pair_data;
    const int eq_index = pair_string.find_first_of('=');
    if(eq_index == std::string::npos)
      continue;
    else
      (*query_parameters)[pair_string.substr(0, eq_index)] = pair_string.substr(eq_index + 1);
  }

  return true;
}


void MjpegServer::handle_stream(
				std::map<std::string, std::string> parameters,
				 http_server::connection_ptr connection)
{
  connection->set_status(http_server::connection::ok);
  std::vector<http_response_header> headers;
  headers.push_back(create_response_header("Content-Type", "multipart/x-mixed-replace;boundary=boundarydonotcross "));
  connection->set_headers(headers);
  connection->write("--boundarydonotcross \r\n");
  image_subscribers_.push_back(boost::shared_ptr<ImageStreamer>(new ImageStreamer(connection, image_transport_, parameters["topic"])));
}

void MjpegServer::handle_snapshot(
				std::map<std::string, std::string> parameters,
				 http_server::connection_ptr connection)
{
  connection->set_status(http_server::connection::not_supported);
  std::vector<http_response_header> headers;
  headers.push_back(create_response_header("Content-Type", "text/html;"));
  connection->set_headers(headers);
  connection->write("<h1>Snapshot not yet supported</h1>");
}

void MjpegServer::handle_list_streams(
				 http_server::connection_ptr connection)
{
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

  connection->set_status(http_server::connection::ok);
  std::vector<http_response_header> headers;
  headers.push_back(create_response_header("Content-Type", "text/html;"));
  connection->set_headers(headers);

  std::stringstream content;

  content << ("<html>"
		    "<head><title>ROS Image Topic List</title></head>"
		    "<body><h1>Available ROS Image Topics:</h1>");
  content << ("<ul>");
  BOOST_FOREACH(std::string& camera_info_topic, camera_info_topics){
    if(boost::algorithm::ends_with(camera_info_topic, "/camera_info")){
      std::string base_topic = camera_info_topic.substr(0, camera_info_topic.size()-strlen("camera_info"));
      content << ("<li>");
      content << (base_topic);
      content << ("<ul>");
      std::vector<std::string>::iterator image_topic_itr = image_topics.begin();
      for ( ; image_topic_itr != image_topics.end(); ) {
	if (boost::starts_with(*image_topic_itr, base_topic)) {
	  content << ("<li><a href=\"/stream?topic=");
	  content << (*image_topic_itr);
	  content << ("\">");
	  content << (image_topic_itr->substr(base_topic.size()));
	  content << ("</a></li>");

	  image_topic_itr = image_topics.erase(image_topic_itr);
	} else {
	  ++image_topic_itr;
	}
      }
      content << ("</ul>");
    }
    content << ("</li>");
  }
  content << ("</ul></body></html>");
  connection->write(content.str());
}

void MjpegServer::handle_request(
				 http_server::request const & request,
				 http_server::connection_ptr connection)
{
  ROS_INFO_STREAM("Got Request: " << request.source << ", " << request.method << " -> " << request.destination);

  std::string path;
  std::map<std::string, std::string> query_parameters;
  if(parse_url(request.destination, &path, &query_parameters)) {
    if(path == "/"){
      handle_list_streams(connection);
    }
    else if(path == "/stream"){
      handle_stream(query_parameters, connection);
    }
    else if(path == "/snapshot"){
      handle_snapshot(query_parameters, connection);
    }
    else {
      connection->set_status(http_server::connection::not_found);
      std::vector<http_response_header> headers;
      headers.push_back(create_response_header("Content-Type", "text/html;"));
      connection->set_headers(headers);
      connection->write("<h1>Page not found</h1>");
    }
  }
  else {
    connection->set_status(http_server::connection::bad_request);
    std::vector<http_response_header> headers;
    headers.push_back(create_response_header("Content-Type", "text/html;"));
    connection->set_headers(headers);
    connection->write("<h1>Error parsing URL</h1>");
  }

}





void MjpegServerRequestHandler::operator()(http_server::request const & request, http_server::connection_ptr connection) {
  server_->handle_request(request, connection);
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

