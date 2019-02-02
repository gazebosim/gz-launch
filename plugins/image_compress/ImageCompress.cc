/*
 * Copyright (C) 2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include <cstddef>
#include <stdio.h>
#include <ignition/common/Console.hh>

#include "jpeglib.h"

#include "ImageCompress.hh"

using namespace ignition;

/////////////////////////////////////////////////
ImageCompress::ImageCompress()
  : ignition::launch::Plugin()
{
}

/////////////////////////////////////////////////
ImageCompress::~ImageCompress()
{
}

/////////////////////////////////////////////////
void ImageCompress::Load(std::map<std::string, std::string> _params)
{
  std::map<std::string, std::string>::const_iterator iter;
  std::string outputTopic = "/image/jpeg";
  std::string inputTopic = "/image";

  // Get the output topic
  iter = _params.find("output_topic");
  if (iter != _params.end())
    outputTopic = iter->second;

  // Get the input topic
  iter = _params.find("input_topic");
  if (iter != _params.end())
    inputTopic = iter->second;

  // Output some useful information
  ignmsg << "ImageCompress using input topic[" << inputTopic
        << "] and output topic[" << outputTopic << "]\n";

  this->pub = this->node.Advertise<ignition::msgs::Image>(outputTopic);
  this->node.Subscribe(inputTopic, &ImageCompress::OnImage, this);
}

/////////////////////////////////////////////////
void ImageCompress::Shutdown()
{
}

/////////////////////////////////////////////////
void ImageCompress::OnImage(const ignition::msgs::Image &_msg)
{
  // This will write a raw PBM image
  /*{
   std::ofstream f("/tmp/test.pbm", std::ios_base::out | std::ios_base::binary
      | std::ios_base::trunc);
  f << "P6\n" << _msg->image().width() << " " << _msg->image().height() << "\n255\n" << _msg->image().data();
  f.close();
  }*/

  unsigned char *mem = NULL;
  unsigned long memSize = 0;
  int bytesPerPixel = 3;

  struct jpeg_compress_struct cinfo;
  struct jpeg_error_mgr jerr;
  cinfo.err = jpeg_std_error(&jerr);
  jpeg_create_compress(&cinfo);
  jpeg_mem_dest(&cinfo, &mem, &memSize);

  // todo: fix this
  // if (_msg->image().pixel_format() == robot_msgs::Image::RGB_INT8)
  {
    cinfo.in_color_space = JCS_RGB;
    bytesPerPixel = 3;
  }
  /*else
  {
    ignerr << "Unable to jpeg compress pixel format["
      << _msg->image().format() << "]\n";
    return;
  }*/

  //Setting the parameters of the output file here
  cinfo.image_width = _msg.width();
  cinfo.image_height = _msg.height();
  cinfo.input_components = bytesPerPixel;

  // default compression parameters, we shouldn't be
  // worried about these
  jpeg_set_defaults(&cinfo);

  // Now do the compression
  jpeg_start_compress(&cinfo, TRUE);

  // like reading a file, this time write one
  // row at a time
  char *rawImage = const_cast<char*>(&(_msg.data()[0]));

  JSAMPROW row_pointer[1];
  while (cinfo.next_scanline < cinfo.image_height)
  {
    row_pointer[0] = (unsigned char*)(&rawImage[
        cinfo.next_scanline * cinfo.image_width * cinfo.input_components]);
    jpeg_write_scanlines(&cinfo, row_pointer, 1);
  }

  // similar to read file, clean up after
  // we're done compressing
  jpeg_finish_compress(&cinfo);
  jpeg_destroy_compress(&cinfo);

  ignition::msgs::Image image;
  //image->set_frame(_msg->frame());
  //image->set_sequence(_msg->sequence());

  image.mutable_header()->mutable_stamp()->set_sec(
      _msg.header().stamp().sec());
  image.mutable_header()->mutable_stamp()->set_nsec(
      _msg.header().stamp().nsec());
  image.set_width(_msg.width());
  image.set_height(_msg.height());
  image.set_step(_msg.step());

  // todo: fix this
  // image->mutable_image()->set_format(ignition::msgs::Image::JPEG);

  image.set_data(mem, memSize);

  this->pub.Publish(image);

  // This will write a jpeg image
  /*{
  std::ofstream f("/tmp/test.jpg", std::ios_base::out | std::ios_base::binary
      | std::ios_base::trunc);
   f.write((char*)(mem), memSize);
   f.close();
  }*/

  delete [] mem;
}
