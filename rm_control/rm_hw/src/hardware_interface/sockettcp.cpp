/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2021, Qiayuan Liao
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

//
// Created by qiayuan on 3/3/21.
//
#include "rm_hw/hardware_interface/sockettcp.h"
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <utility>
#include <ros/ros.h>

namespace can
{
/* ref:
 * https://github.com/JCube001/socketcan-demo
 * http://blog.mbedded.ninja/programming/operating-systems/linux/how-to-use-socketcan-with-c-in-linux
 * https://github.com/linux-can/can-utils/blob/master/candump.c
 */

SocketTcp::~SocketTcp()
{
  if (this->isOpen())
    this->close();
}

bool SocketTcp::open(const std::string& interface, boost::function<void(const can_frame& frame)> handler,
                     int thread_priority)
{


  if(interface == "can0")
  {
    memset(&remote_addr,0,sizeof(remote_addr)); //数据初始化--清零
    remote_addr.sin_family=AF_INET; //设置为IP通信
    remote_addr.sin_addr.s_addr=inet_addr("192.168.4.101"); //服务器IP地址
    remote_addr.sin_port=htons(8881); //服务器端口号
  }
  if(interface == "can1")
  {
    memset(&remote_addr,0,sizeof(remote_addr)); //数据初始化--清零
    remote_addr.sin_family=AF_INET; //设置为IP通信
    remote_addr.sin_addr.s_addr=inet_addr("192.168.4.101"); //服务器IP地址
    remote_addr.sin_port=htons(8882); //服务器端口号
  }   
            //创建客户端套接字--IPv4协议，面向连接通信，TCP协议
    if((sock_fd_ = socket(PF_INET,SOCK_STREAM,0))<0){
        std::cout<<"Error: Unable to create a TCP socket"<<std::endl;
        return false;
    }
  //将套接字绑定到服务器的网络地址上reinterpret_cast<struct sockaddr*>(&address_)
  if(connect(sock_fd_,reinterpret_cast<struct sockaddr*>(&remote_addr),sizeof(struct sockaddr_in)) < 0){
        std::cout<<"TCP connect error"<<std::endl;
        return false;
    }
  reception_handler = std::move(handler);
  return startReceiverThread(thread_priority);
}

void SocketTcp::close()
{
  terminate_receiver_thread_ = true;
  while (receiver_thread_running_)
    ;

  if (!isOpen())
    return;
  // Close the file descriptor for our socket
  ::close(sock_fd_);
  sock_fd_ = -1;
}

bool SocketTcp::isOpen() const
{
  return (sock_fd_ != -1);
}

void SocketTcp::write(can_frame* frame) const
{
  char send_buf[13];
  if (!isOpen())
  {
    ROS_ERROR_THROTTLE(5., "tcp Unable to write: Socket %s not open", interface_request_.ifr_name);
    return;
  }

  // can帧转tcp
  // send_buf[0] = 8;
  // send_buf[1] = 0;
  // send_buf[2] = 0;
  // send_buf[3] = frame->can_id >> 8;

  // for(int i=0;i<8;i++)
  // {
  //   send_buf[i+5] = frame->data[i];
  // }
  // // if(frame->can_id == 0x200)
  //   // {
  //     send_buf[4] = 0;
  //   // }
  // if(frame->can_id == 0x1FF)
  //   {
  //   send_buf[4] = 255 ;
  //   }

  // 新
  send_buf[0] = 8;
  send_buf[1] = 0;
  send_buf[2] = 0;
  send_buf[3] = 0;
  // if(frame->can_id == 0x200)
  // {
  send_buf[4] = frame->can_id;
  // }
  // if(frame->can_id == 0x1FF)
  //   {
  //   send_buf[4] = 255 ;
  //   }
  // memcpy(send_buf + 4, frame->data, 8);
  for (int i = 0; i < 8; i++)
  {
    send_buf[i + 5] = frame->data[i];
  }

  if (::send(sock_fd_, send_buf, sizeof(send_buf), 0) == -1)  
  ROS_DEBUG_THROTTLE(5., "Unable to write: The %s tx buffer may be full", interface_request_.ifr_name);  
}

static void* socketcan_receiver_thread(void* argv)
{
  /*
   * The first and only argument to this function
   * is the pointer to the object, which started the thread.
   */
  auto* sock = (SocketTcp*)argv;

  fd_set descriptors;

  int maxfd = sock->sock_fd_;

  struct timeval timeout
  {
  };
  char tcp_recive_buffer[13];
  can_frame rx_frame{};


  sock->receiver_thread_running_ = true;
  
  while (!sock->terminate_receiver_thread_)
  {
    // Clear descriptor set
    FD_ZERO(&descriptors);
    // Add socket descriptor
    FD_SET(sock->sock_fd_, &descriptors);

    timeout.tv_sec = 1.; 

    if (select(maxfd + 10, &descriptors, nullptr, nullptr, &timeout))
    {
      size_t len = recv(sock->sock_fd_, (void*)&tcp_recive_buffer, 13 , 0);
      if (len < 0)
        continue;
      if (sock->reception_handler != nullptr)
       {

        // rx_frame.can_dlc = tcp_recive_buffer[0];
        if(tcp_recive_buffer[0]>>7 == 0)         rx_frame.can_id =  (tcp_recive_buffer[4] | tcp_recive_buffer[3]<<8);
        if(tcp_recive_buffer[0]>>7 == 1)         rx_frame.can_id =  (tcp_recive_buffer[4] | tcp_recive_buffer[3]<<8 | tcp_recive_buffer[2] | tcp_recive_buffer[1]);

        memcpy(rx_frame.data, tcp_recive_buffer + 5, 8);
        sock->reception_handler(rx_frame);
       } 
    }
  }
  sock->receiver_thread_running_ = false;
  return nullptr;
}

bool SocketTcp::startReceiverThread(int thread_priority)
{
  // Frame reception is accomplished in a separate, event-driven thread.
  // See also: https://www.thegeekstuff.com/2012/04/create-threads-in-linux/
  terminate_receiver_thread_ = false;
  int rc = pthread_create(&receiver_thread_id_, nullptr, &socketcan_receiver_thread, this);
  if (rc != 0)
  {
    ROS_ERROR("Unable to start receiver thread");
    return false;
  }
  ROS_INFO("Successfully started receiver thread with ID %lu", receiver_thread_id_);
  sched_param sched{ .sched_priority = thread_priority };
  pthread_setschedparam(receiver_thread_id_, SCHED_FIFO, &sched);
  return true;
}

}  // namespace can
