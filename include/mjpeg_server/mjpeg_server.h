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
#ifndef MJPEG_SERVER_H_
#define MJPEG_SERVER_H_

#include <ros/ros.h>
#include <boost/network/include/http/server.hpp>
#include <image_transport/image_transport.h>
#include <boost/network/utils/thread_pool.hpp>
#include <cv_bridge/cv_bridge.h>
#include <vector>

namespace mjpeg_server
{

class MjpegServerRequestHandler;
class MjpegServer;
typedef boost::network::http::async_server<MjpegServerRequestHandler> http_server;
typedef boost::network::http::response_header_narrow http_response_header;


class MjpegServerRequestHandler {
public:
  MjpegServerRequestHandler(MjpegServer* server) : server_(server) {};
  void operator()(http_server::request const & request, http_server::connection_ptr connection);
private:
  MjpegServer* server_;
};


class ImageStreamer {
public:
  ImageStreamer(http_server::connection_ptr connection, image_transport::ImageTransport it, std::string topic);
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);
  bool isActive();
  bool sendImage(const cv_bridge::CvImagePtr& cv_ptr);
private:
  image_transport::Subscriber image_sub_;
  http_server::connection_ptr connection_;
  bool active_;
};

/**
 * @class MjpegServer
 * @brief
 */
class MjpegServer
{
public:
  /**
   * @brief  Constructor
   * @return
   */
  MjpegServer(ros::NodeHandle& nh, ros::NodeHandle& private_nh);

  /**
   * @brief  Destructor - Cleans up
   */
  virtual ~MjpegServer();


  /**
   * @brief  Starts the server and spins
   */
  void spin();

  void handle_stream(
		     std::map<std::string, std::string> parameters,
		     http_server::connection_ptr connection);
  void handle_snapshot(
		       std::map<std::string, std::string> parameters,
		       http_server::connection_ptr connection);
  void handle_list_streams(http_server::connection_ptr connection);


  void handle_request(http_server::request const & request, http_server::connection_ptr connection);

private:
  bool parse_url(std::string& url, std::string* path, std::map<std::string, std::string>* query_parameters);



private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport image_transport_;
  MjpegServerRequestHandler request_handler_;
  boost::shared_ptr<http_server> server_;

  std::vector<boost::shared_ptr<ImageStreamer> > image_subscribers_;
};

}

#endif

