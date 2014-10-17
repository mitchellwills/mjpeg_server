/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Robert Bosch LLC, Worcester Polytechnic Institute
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

#ifndef MJPEG_SERVER_HTTP_SERVER_HTTP_CONNECTION_HPP
#define MJPEG_SERVER_HTTP_SERVER_HTTP_CONNECTION_HPP

#include <boost/asio.hpp>
#include <boost/noncopyable.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include "mjpeg_server/http_server/request_handler.hpp"
#include "mjpeg_server/http_server/http_request.hpp"
#include "mjpeg_server/http_server/http_request_parser.hpp"

namespace mjpeg_server {
namespace http_server {


// Represents a connection to a client
// To keep the connection alive keep a shared pointer to this object
class HttpConnection : public boost::enable_shared_from_this<HttpConnection>,
		       private boost::noncopyable {
public:
  explicit HttpConnection(boost::asio::io_service& io_service,
			  HttpServerRequestHandler request_handler);

  boost::asio::ip::tcp::socket& socket();

  // Start async operation to read request
  void start();
  // Perform a synchronous write operation to the client
  void write(const std::vector<boost::asio::const_buffer>&);

private:
  void handle_read(const boost::system::error_code& e,
		   std::size_t bytes_transferred);

  boost::asio::io_service::strand strand_;
  boost::asio::ip::tcp::socket socket_;
  HttpServerRequestHandler request_handler_;
  boost::array<char, 8192> buffer_;
  HttpRequest request_;
  HttpRequestParser request_parser_;
};



}
}

#endif
