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

#include <cstring>
#include <string>
#include <cstdio>
#include <stdexcept>
#include <dirent.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>

#include <ignition/common/Console.hh>
#include "UVCCamera.hh"

using std::string;
using namespace UVCCamera;

/////////////////////////////////////////////////
Camera::Camera(const char *_device, mode_t _mode, int _width, int _height,
    int _fps)
: mode(_mode), device(_device),
  motionThresholdLuminance(100), motionThresholdCount(-1),
  width(_width), height(_height), fps(_fps), rgbFrame(NULL)
{
  if ((this->fd = open(this->device.c_str(), O_RDWR)) == -1)
    throw std::runtime_error("couldn't open " + this->device);

  memset(&this->fmt, 0, sizeof(v4l2_format));
  memset(&this->cap, 0, sizeof(v4l2_capability));

  if (ioctl(this->fd, VIDIOC_QUERYCAP, &this->cap) < 0)
    throw std::runtime_error("couldn't query " + this->device);

  if (!(this->cap.capabilities & V4L2_CAP_VIDEO_CAPTURE))
    throw std::runtime_error(this->device + " does not support capture");

  if (!(this->cap.capabilities & V4L2_CAP_STREAMING))
    throw std::runtime_error(this->device + " does not support streaming");

  // enumerate formats
  v4l2_fmtdesc f;
  memset(&f, 0, sizeof(f));
  f.index = 0;
  f.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  int ret;

  while ((ret = ioctl(this->fd, VIDIOC_ENUM_FMT, &f)) == 0)
  {
    // printf("pixfmt %d = '%4s' desc = '%s'\n",
    //      f.index, (char *)&f.pixelformat, f.description);

    f.index++;

    // enumerate frame sizes
    v4l2_frmsizeenum fsize;
    fsize.index = 0;
    fsize.pixel_format = f.pixelformat;

    while ((ret = ioctl(this->fd, VIDIOC_ENUM_FRAMESIZES, &fsize)) == 0)
    {
      fsize.index++;
      if (fsize.type == V4L2_FRMSIZE_TYPE_DISCRETE)
      {
        // printf("  discrete: %ux%u:   ",
        //       fsize.discrete.width, fsize.discrete.height);

        // enumerate frame rates
        v4l2_frmivalenum fival;
        fival.index = 0;
        fival.pixel_format = f.pixelformat;
        fival.width = fsize.discrete.width;
        fival.height = fsize.discrete.height;

        while ((ret = ioctl(this->fd, VIDIOC_ENUM_FRAMEINTERVALS, &fival)) == 0)
        {
          ++fival.index;
          // if (fival.type == V4L2_FRMIVAL_TYPE_DISCRETE)
          // {
          //   printf("%u/%u ",
          //       fival.discrete.numerator, fival.discrete.denominator);
          // }
          // else
          //   printf("I only handle discrete frame intervals...\n");
        }
        // printf("\n");
      }
      // else if (fsize.type == V4L2_FRMSIZE_TYPE_CONTINUOUS)
      // {
      //   printf("  continuous: %ux%u to %ux%u\n",
      //          fsize.stepwise.min_width, fsize.stepwise.min_height,
      //          fsize.stepwise.max_width, fsize.stepwise.max_height);
      // }
      // else if (fsize.type == V4L2_FRMSIZE_TYPE_STEPWISE)
      // {
      //   printf("  stepwise: %ux%u to %ux%u step %ux%u\n",
      //          fsize.stepwise.min_width,  fsize.stepwise.min_height,
      //          fsize.stepwise.max_width,  fsize.stepwise.max_height,
      //          fsize.stepwise.step_width, fsize.stepwise.step_height);
      // }
      // else
      // {
      //   printf("  fsize.type not supported: %d\n", fsize.type);
      // }
    }
  }

  if (errno != EINVAL)
    throw std::runtime_error("error enumerating frame formats");

  this->fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  this->fmt.fmt.pix.width = this->width;
  this->fmt.fmt.pix.height = this->height;

  // we'll convert later
  if (mode == MODE_RGB || mode == MODE_YUYV)
  {
    this->fmt.fmt.pix.pixelformat =
      'Y' | ('U' << 8) | ('Y' << 16) | ('V' << 24);
  }
  else
  {
    this->fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
  }

  this->fmt.fmt.pix.field = V4L2_FIELD_ANY;
  if ((ret = ioctl(this->fd, VIDIOC_S_FMT, &this->fmt)) < 0)
    throw std::runtime_error("couldn't set format");

  if (this->fmt.fmt.pix.width != this->width ||
      this->fmt.fmt.pix.height != this->height)
    throw std::runtime_error("pixel format unavailable");

  this->streamParm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  this->streamParm.parm.capture.timeperframe.numerator = 1;
  this->streamParm.parm.capture.timeperframe.denominator = this->fps;

  if ((ret = ioctl(this->fd, VIDIOC_S_PARM, &this->streamParm)) < 0)
    throw std::runtime_error("unable to set framerate");

  v4l2_queryctrl queryctrl;
  memset(&queryctrl, 0, sizeof(queryctrl));
  uint32_t i = V4L2_CID_BASE;

  while (i != V4L2_CID_LAST_EXTCTR)
  {
    queryctrl.id = i;
    if ((ret = ioctl(this->fd, VIDIOC_QUERYCTRL, &queryctrl)) == 0 &&
        !(queryctrl.flags & V4L2_CTRL_FLAG_DISABLED))
    {
      // const char *ctrlType = NULL;
      // if (queryctrl.type == V4L2_CTRL_TYPE_INTEGER)
      //   ctrlType = "int";
      // else if (queryctrl.type == V4L2_CTRL_TYPE_BOOLEAN)
      //   ctrlType = "bool";
      // else if (queryctrl.type == V4L2_CTRL_TYPE_BUTTON)
      //   ctrlType = "button";
      // else if (queryctrl.type == V4L2_CTRL_TYPE_MENU)
      //   ctrlType = "menu";
      // printf("  %s (%s, %d, id = %x): %d to %d (%d)\n",
      //        ctrlType,
      //        queryctrl.name, queryctrl.flags, queryctrl.id,
      //        queryctrl.minimum, queryctrl.maximum, queryctrl.step);
      if (queryctrl.type == V4L2_CTRL_TYPE_MENU)
      {
        v4l2_querymenu querymenu;
        memset(&querymenu, 0, sizeof(querymenu));
        querymenu.id = queryctrl.id;
        querymenu.index = 0;
        while (ioctl(this->fd, VIDIOC_QUERYMENU, &querymenu) == 0)
        {
          // printf("    %d: %s\n", querymenu.index, querymenu.name);
          ++querymenu.index;
        }
      }
    }
    else if (errno != EINVAL)
      throw std::runtime_error("couldn't query control");

    i++;

    if (i == V4L2_CID_LAST_NEW)
      i = V4L2_CID_CAMERA_CLASS_BASE_NEW;
    else if (i == V4L2_CID_CAMERA_CLASS_LAST)
      i = V4L2_CID_PRIVATE_BASE_OLD;
    else if (i == V4L2_CID_PRIVATE_LAST)
      i = V4L2_CID_BASE_EXTCTR;
  }

  if (!this->SetControl(10094851, 1))
    ignlog << "UVCCamera: Unable to set exposure, auto priority." << std::endl;
  if (!this->SetControl(10094849, 1))
    ignlog << "UVCCamera: Unable to set exposure, auto." << std::endl;
  if (!this->SetControl(9963776, 128))
    ignlog << "UVCCamera: Unable to set brightness." << std::endl;
  if (this->SetControl(9963777, 32))
    ignlog << "UVCCamera: Unable to set Contrast." << std::endl;
  if (this->SetControl(9963788, 1))
    ignlog << "UVCCamera: Unable to set White Balance Temperature, Auto.\n";
  if (this->SetControl(9963802, 5984))
    ignlog << "UVCCamera: Unable to set White Balance Temperature.\n";
  if (this->SetControl(9963800, 2))
    ignlog << "UVCCamera: Unable to set power line frequency to 60 hz.\n";
  if (this->SetControl(9963795, 200))
    ignlog << "UVCCamera: Unable to set Gain.\n";
  if (this->SetControl(9963803, 224))
    ignlog << "UVCCamera: Unable to set sharpness.\n";
  if (this->SetControl(9963804, 1))
    ignlog << "UVCCamera: Unable to set Backlight Compensation.\n";
  if (this->SetControl(10094850, 250))
    ignlog << "UVCCamera: Unable to set Exposure (Absolute).\n";
  if (this->SetControl(168062212, 16))
    ignlog << "UVCCamera: Unable to set Focus (absolute).\n";
  if (this->SetControl(168062213, 3))
    ignlog << "UVCCamera: Unable to set LED1 Mode.\n";
  if (this->SetControl(168062214, 0))
    ignlog << "UVCCamera: Unable to set LED1 Frequency.\n";
  if (this->SetControl(9963778, 32))
    ignlog << "UVCCamera: Unable to set Saturation.\n";

  // the commented labels correspond to the
  // controls in guvcview and uvcdynctrl
  // this->SetControl(V4L2_CID_EXPOSURE_AUTO_NEW, 2);
  // this->SetControl(168062321, 0); //Disable video processing
  // this->SetControl(0x9a9010, 100);
  // this->SetControl(V4L2_CID_EXPOSURE_ABSOLUTE_NEW, 300);
  // this->SetControl(V4L2_CID_BRIGHTNESS, 140);
  // this->SetControl(V4L2_CID_CONTRAST, 40);
  // this->SetControl(V4L2_CID_WHITE_BALANCE_TEMP_AUTO_OLD, 0);


  // v4l2_jpegcompression v4l2_jpeg;
  // if (ioctl(this->fd, VIDIOC_G_JPEGCOMP, &v4l2_jpeg) < 0)
  //   throw std::runtime_error("no jpeg compression iface exposed");
  // printf("jpeg quality: %d\n", v4l2_jpeg.quality);

  memset(&this->requestBuffer, 0, sizeof(this->requestBuffer));
  this->requestBuffer.count = NUM_BUFFER;
  this->requestBuffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  this->requestBuffer.memory = V4L2_MEMORY_MMAP;

  if (ioctl(this->fd, VIDIOC_REQBUFS, &this->requestBuffer) < 0)
    throw std::runtime_error("unable to allocate buffers");

  for (unsigned int j = 0; j < NUM_BUFFER; ++j)
  {
    memset(&this->buf, 0, sizeof(this->buf));
    this->buf.index = j;
    this->buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    this->buf.flags = V4L2_BUF_FLAG_TIMECODE;
    this->buf.timecode = this->timeCode;
    this->buf.timestamp.tv_sec = 0;
    this->buf.timestamp.tv_usec = 0;
    this->buf.memory = V4L2_MEMORY_MMAP;

    if (ioctl(this->fd, VIDIOC_QUERYBUF, &this->buf) < 0)
      throw std::runtime_error("unable to query this->buffer");

    if (this->buf.length <= 0)
      throw std::runtime_error("this->buffer length is bogus");

    mem[j] = mmap(0, this->buf.length, PROT_READ, MAP_SHARED,
        this->fd, this->buf.m.offset);

    // printf("this->buf length = %d at %x\n", this->buf.length, mem[j]);

    if (mem[j] == MAP_FAILED)
      throw std::runtime_error("couldn't map this->buffer");
  }

  this->bufLength = this->buf.length;

  for (unsigned int j = 0; j < NUM_BUFFER; ++j)
  {
    memset(&this->buf, 0, sizeof(this->buf));
    this->buf.index = j;
    this->buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    this->buf.flags = V4L2_BUF_FLAG_TIMECODE;
    this->buf.timecode = this->timeCode;
    this->buf.timestamp.tv_sec = 0;
    this->buf.timestamp.tv_usec = 0;
    this->buf.memory = V4L2_MEMORY_MMAP;
    if (ioctl(this->fd, VIDIOC_QBUF, &this->buf) < 0)
      throw std::runtime_error("unable to queue this->buffer");
  }

  int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (ioctl(this->fd, VIDIOC_STREAMON, &type) < 0)
    throw std::runtime_error("unable to start capture");

  this->rgbFrame = new unsigned char[this->width * this->height * 3];
  this->lastYUVFrame = new unsigned char[this->width * this->height * 2];
}

/////////////////////////////////////////////////
Camera::~Camera()
{
  // stop stream
  int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  int ret;

  if ((ret = ioctl(this->fd, VIDIOC_STREAMOFF, &type)) < 0)
    perror("VIDIOC_STREAMOFF");

  for (unsigned i = 0; i < NUM_BUFFER; ++i)
  {
    if (munmap(mem[i], this->bufLength) < 0)
      perror("failed to unmap buffer");
  }

  close(this->fd);

  if (this->rgbFrame)
  {
    delete[] this->rgbFrame;
    delete[] this->lastYUVFrame;
  }

  this->lastYUVFrame = this->rgbFrame = NULL;
}

/////////////////////////////////////////////////
void Camera::Enumerate()
{
  string v4lPath = "/sys/class/video4linux";
  DIR *d = opendir(v4lPath.c_str());
  if (!d)
    throw std::runtime_error("couldn't open " + v4lPath);

  struct dirent *ent, *ent2, *ent3;
  int fdLocal, ret;
  struct v4l2_capability v4l2_cap;

  while ((ent = readdir(d)) != NULL)
  {
    // ignore anything not starting with "video"
    if (strncmp(ent->d_name, "video", 5))
      continue;

    string dev_name = string("/dev/") + string(ent->d_name);
    // printf("enumerating %s ...\n", dev_name.c_str());

    if ((fdLocal = open(dev_name.c_str(), O_RDWR)) == -1)
      throw std::runtime_error("couldn't open " + dev_name + "  perhaps the " +
                               "permissions are not set correctly?");

    if ((ret = ioctl(fdLocal, VIDIOC_QUERYCAP, &v4l2_cap)) < 0)
      throw std::runtime_error("couldn't query " + dev_name);

    // printf("name = [%s]\n", v4l2_cap.card);
    // printf("driver = [%s]\n", v4l2_cap.driver);
    // printf("location = [%s]\n", v4l2_cap.bus_info);
    close(fdLocal);
    string v4l_dev_path = v4lPath + string("/") + string(ent->d_name) +
                          string("/device");

    // my kernel is using /sys/class/video4linux/videoN/device/inputX/id
    DIR *d2 = opendir(v4l_dev_path.c_str());

    if (!d2)
      throw std::runtime_error("couldn't open " + v4l_dev_path);

    string input_dir;
    while ((ent2 = readdir(d2)) != NULL)
    {
      if (strncmp(ent2->d_name, "input", 5))
        continue; // ignore anything not beginning with "input"

      DIR *input = opendir((v4l_dev_path + string("/") +
            string(ent2->d_name)).c_str());
      bool output_set = false;

      while ((ent3 = readdir(input)) != NULL)
      {
        if (!strncmp(ent3->d_name, "input", 5))
        {
          input_dir = (string("input/") + string(ent3->d_name )).c_str();
          output_set = true;
          break;
        }
      }

      if (!output_set)
        input_dir = ent2->d_name;
      break;
    }

    closedir(d2);

    if (!input_dir.length())
      throw std::runtime_error("couldn't find input dir in " + v4l_dev_path);

    string vidFname = v4l_dev_path + string("/") + input_dir +
                       string("/id/vendor");
    string pid_fname = v4l_dev_path + string("/") + input_dir +
                       string("/id/product");
    string ver_fname = v4l_dev_path + string("/") + input_dir +
                       string("/id/version");

    char vid[5], pid[5], ver[5];
    FILE *vidFp = fopen(vidFname.c_str(), "r");

    if (!vidFp)
      throw std::runtime_error("couldn't open " + vidFname);

    if (!fgets(vid, sizeof(vid), vidFp))
      throw std::runtime_error("couldn't read VID from " + vidFname);

    fclose(vidFp);
    vid[4] = 0;
    // printf("vid = [%s]\n", vid);

    FILE *pid_fp = fopen(pid_fname.c_str(), "r");
    if (!pid_fp)
      throw std::runtime_error("couldn't open " + pid_fname);

    if (!fgets(pid, sizeof(pid), pid_fp))
      throw std::runtime_error("couldn't read PID from " + pid_fname);

    fclose(pid_fp);
    // printf("pid = [%s]\n", pid);
    FILE *ver_fp = fopen(ver_fname.c_str(), "r");

    if (!ver_fp)
      throw std::runtime_error("couldn't open " + ver_fname);

    if (!fgets(ver, sizeof(ver), ver_fp))
      throw std::runtime_error("couldn't read version from " + ver_fname);

    fclose(ver_fp);
    // printf("ver = [%s]\n", ver);
  }

  closedir(d);
}

/////////////////////////////////////////////////
// saturate input into [0, 255]
inline unsigned char saturate(float _f)
{
  return (unsigned char)(_f >= 255 ? 255 : (_f < 0 ? 0 : _f));
}

/////////////////////////////////////////////////
int Camera::Grab(unsigned char **_frame, uint32_t &_bytesUsed)
{
  *_frame = NULL;
  int ret = 0;
  fd_set rdset;
  timeval timeout;
  FD_ZERO(&rdset);
  FD_SET(this->fd, &rdset);
  timeout.tv_sec = 1;
  timeout.tv_usec = 0;
  _bytesUsed = 0;
  ret = select(this->fd + 1, &rdset, NULL, NULL, &timeout);

  if (ret == 0)
  {
    // printf("select timeout in grab\n");
    return -1;
  }
  else if (ret < 0)
  {
    perror("couldn't grab image");
    return -1;
  }

  if (!FD_ISSET(this->fd, &rdset))
    return -1;

  memset(&this->buf, 0, sizeof(this->buf));
  this->buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  this->buf.memory = V4L2_MEMORY_MMAP;

  if (ioctl(this->fd, VIDIOC_DQBUF, &this->buf) < 0)
    throw std::runtime_error("couldn't dequeue this->buffer");

  _bytesUsed = this->buf.bytesused;
  if (mode == MODE_RGB)
  {
    // just look at the Y channel
    int numPixelsDifferent = 0;
    unsigned char *pyuv = (unsigned char *)mem[this->buf.index];

    // yuyv is 2 bytes per pixel. step through every pixel pair.
    unsigned char *prgb = this->rgbFrame;
    unsigned char *pyuv_last = this->lastYUVFrame;

    for (unsigned i = 0; i < this->width * this->height * 2; i += 4)
    {
      *prgb++ = saturate(pyuv[i]+1.402f  *(pyuv[i+3]-128));
      *prgb++ = saturate(pyuv[i]-0.34414f*(pyuv[i+1]-128) -
          0.71414f*(pyuv[i+3]-128));
      *prgb++ = saturate(pyuv[i]+1.772f  *(pyuv[i+1]-128));
      *prgb++ = saturate(pyuv[i+2]+1.402f*(pyuv[i+3]-128));
      *prgb++ = saturate(pyuv[i+2]-0.34414f*(pyuv[i+1]-128) -
          0.71414f*(pyuv[i+3]-128));
      *prgb++ = saturate(pyuv[i+2]+1.772f*(pyuv[i+1]-128));

      if ((int)pyuv[i] - (int)pyuv_last[i] > this->motionThresholdLuminance ||
          (int)pyuv_last[i] - (int)pyuv[i] > this->motionThresholdLuminance)
        numPixelsDifferent++;

      if ((int)pyuv[i+2] - (int)pyuv_last[i+2] > this->motionThresholdLuminance
          ||
          (int)pyuv_last[i+2] - (int)pyuv[i+2] > this->motionThresholdLuminance)
      {
        numPixelsDifferent++;
      }

      // this gives bgr images...
      // *prgb++ = saturate(pyuv[i]+1.772f  *(pyuv[i+1]-128));
      // *prgb++ = saturate(pyuv[i]-0.34414f*
      // (pyuv[i+1]-128)-0.71414f*(pyuv[i+3]-128));
      // *prgb++ = saturate(pyuv[i]+1.402f  *(pyuv[i+3]-128));
      // *prgb++ = saturate(pyuv[i+2]+1.772f*(pyuv[i+1]-128));
      // *prgb++ = saturate(pyuv[i+2]-0.34414f*
      // (pyuv[i+1]-128)-0.71414f*(pyuv[i+3]-128));
      // *prgb++ = saturate(pyuv[i+2]+1.402f*(pyuv[i+3]-128));
    }
    memcpy(this->lastYUVFrame, pyuv, this->width * this->height * 2);

    // default: always true
    if (numPixelsDifferent > this->motionThresholdCount)
      *_frame = this->rgbFrame;
    else
    {
      // not enough luminance change
      *_frame = NULL;

      // let go of this image
      this->Release(this->buf.index);
    }
  }
  else if (mode == MODE_YUYV)
  {
    printf("YUVV\n");
    *_frame = (uint8_t *)mem[this->buf.index];
  }
  // mode == MODE_JPEG
  else
  {
    printf("JPEG\n");
    //if (_bytesUsed > 100)
    *_frame = (unsigned char *)mem[this->buf.index];
  }
  return this->buf.index;
}

/////////////////////////////////////////////////
void Camera::Release(unsigned _bufIdx)
{
  if (_bufIdx < NUM_BUFFER)
    if (ioctl(this->fd, VIDIOC_QBUF, &this->buf) < 0)
      throw std::runtime_error("couldn't requeue buffer");
}

/////////////////////////////////////////////////
bool Camera::SetControl(uint32_t _id, int _val)
{
  bool result = true;
  v4l2_control c;
  c.id = _id;

  if (ioctl(this->fd, VIDIOC_G_CTRL, &c) == 0)
  {
    //printf("current value of %d is %d\n", _id, c.value);

    // perror("unable to get control");
    // throw std::runtime_error("unable to get control");
  }
  c.value = _val;

  if (ioctl(this->fd, VIDIOC_S_CTRL, &c) < 0)
    result = false;

  return result;
}

/////////////////////////////////////////////////
void Camera::SetMotionThresholds(int _lum, int _count)
{
  this->motionThresholdLuminance = _lum;
  this->motionThresholdCount = _count;
}
