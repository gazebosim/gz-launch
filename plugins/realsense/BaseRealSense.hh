/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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
#ifndef IGNITION_LAUNCH_REALSENSE_HH_
#define IGNITION_LAUNCH_REALSENSE_HH_

#include <ignition/math/Temperature.hh>
#include <ignition/msgs/imu.pb.h>
#include <ignition/transport/Node.hh>

#include <condition_variable>
#include <queue>
#include <mutex>
#include <atomic>
#include <thread>

namespace ignition
{
  namespace launch
  {

    /*class TemperatureDiagnostics
    {
      public: TemperatureDiagnostics(const std::string &_name,
                  const std::string &_serialNumber);

      public: void Update(ignition::math::Temperature _temperaure)
      {
        this->temperature = _temperaure;
      }

      private: ignition::math::Temperature _temperature;
    };*/

    /*
    class PipelineSyncer : public rs2::asynchronous_syncer
    {
      public: void operator()(rs2::frame _frame) const
      {
        invoke(std::move(_frame));
      }
    };
    */

    class BaseRealSenseCamera
    {
      public: BaseRealSenseCamera(rs2::device _dev,
                                  const std::string &serialNumber);

      public: virtual ~BaseRealSenseCamera();

      private: bool isRunning{true};

      private: std::string baseFrameId;

      private: std::string jsonFilePath;

      private: std::string serialNumber;

      private: std::map<rs2_stream, int> imageFormat;

      private: std::map<rs2_stream, std::string> encoding;

      /// \bref ignition::msgs::Image row step size
      private: std::map<rs2_stream, int> unitStepSize;

      private: std::shared_ptr<std::thread> tfThread;

      private: mutable std::condition_variable cv;

      private: std::shared_ptr<std::thread> monitoringThread;

      private: std::vector<rs2_option> monitorOptions;

      private: std::map<rs2_stream, std::string> streamName;

              /*
      public: void ToggleSensors(bool enabled);
      public: virtual void PublishTopics() override;

      public: enum ImuSyncMethod
              {
                NONE = 0,
                COPY = 1,
                LINEAR_INTERPOLATION = 2
              };
              */

      /*protected: virtual void CalcAndPublishStaticTransform(
                     const stream_index_pair &_stream,
                     const rs2::stream_profile &_baseProfile);
                     */

      /*protected: rs2::stream_profile Profile(
                     const stream_index_pair &_stream);
                     */

      /*protected: void PublishStaticTf(//const ros::Time &t,
                                 const float3& trans,
                                 const ignition::math::Vector3d &_trans,
                                 const ignition::math::Quaterniond &_rot,
                                 const std::string &_from,
                                 const std::string &_to);
                                 */


                 /*
      private: std::atomic_bool isInitializedTimeBase{false};
      protected: std::string odomFrameId;

      protected: std::map<stream_index_pair, std::string> frameId;

      protected: std::map<stream_index_pair, std::string> opticalFrameId;

      protected: std::map<stream_index_pair, std::string> depthAlignedFrameId;

      protected: ignition::transport::Node node;

      protected: bool alignDepth = false;

                 */
/*
      private: class CimuData
      {
        public: CimuData() = default;

        public: CimuData(const stream_index_pair _type,
                    ignition::math::Vector3d &_data, double _time):
                      type(_type),
                      data(_data),
                      time(_time)
        {
        }

        public: bool isSet()
        {
          return this->time > 0;
        }

        public: stream_index_pair type;
        public: ignition::math::Vector3d data;
        public: double time = -1;
      };
      */
/*
      private: void Parameters();

      private: void SetupDevice();

      private: void SetupErrorCallback();

      private: void SetupPublishers();

      private: void EnableDevices();

      private: void SetupFilters();

      private: void setupStreams();

      private: void setBaseTime(double frame_time, bool warn_no_metadata);

      private: cv::Mat& fix_depth_scale(const cv::Mat& from_image,
                   cv::Mat& to_image);

      private: void clip_depth(rs2::depth_frame depth_frame,
                   float clipping_dist);

               */
      // private: void UpdateStreamCalibData(
      //              const rs2::video_stream_profile &_videoProfile);

      // private: void SetBaseStream();

      //private: void PublishStaticTransforms();

      // private: void PublishDynamicTransforms();

      // private: void publishIntrinsics();

      // private: void RunFirstFrameInitialization(rs2_stream stream_type);

      /*private: void PublishPointCloud(rs2::points _pc,
                   const ros::Time &_time, const rs2::frameset &_frameset);
                   */

      /*private: Extrinsics RsExtrinsicsToMsg(
                   const rs2_extrinsics &_extrinsics,
                   const std::string &_frameId) const;
                   */

      // private: IMUInfo ImuInfo(const stream_index_pair &_streamIndex);

      /*private: void PublishFrame(
        rs2::frame _frame, const std::chrono::time_point &_time,
        const stream_index_pair &_stream,
        std::map<stream_index_pair, cv::Mat> &_images,
        const std::map<stream_index_pair, ros::Publisher> &_infoPublishers,
        const std::map<stream_index_pair,
        ImagePublisherWithFrequencyDiagnostics> &_imagePublishers,
        std::map<stream_index_pair, int> &_seq,
        std::map<stream_index_pair, sensor_msgs::CameraInfo> &_cameraInfo,
        const std::map<stream_index_pair, std::string>& _opticalFrameId,
        const std::map<rs2_stream, std::string>& _encoding,
        bool _copyDataFromFrame = true);
        */

      // Not used?
      // private: bool EnabledProfile(const stream_index_pair& stream_index, rs2::stream_profile& profile);

               /*
      private: void PublishAlignedDepthToOthers(
                   rs2::frameset _frames,
                   const std::chrono::time_point &_time);

      private: sensor_msgs::Imu CreateUnitedMessage(const CimuData accel_data, const CimuData gyro_data);

      private: void FillImuData_Copy(const CimuData imu_data, std::deque<sensor_msgs::Imu>& imu_msgs);

      private: void ImuMessage_AddDefaultValues(sensor_msgs::Imu& imu_msg);

      private: void FillImuData_LinearInterpolation(const CimuData imu_data, std::deque<sensor_msgs::Imu>& imu_msgs);

      private: void imu_callback(rs2::frame frame);

      private: void imu_callback_sync(rs2::frame frame, imu_sync_method sync_method=imu_sync_method::COPY);

      private: void pose_callback(rs2::frame frame);

      private: void multiple_message_callback(rs2::frame frame, imu_sync_method sync_method);

      private: void frame_callback(rs2::frame frame);

      private: void registerDynamicOption(ros::NodeHandle& nh, rs2::options sensor, std::string& module_name);

      private: void readAndSetDynamicParam(ros::NodeHandle& nh1, std::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure> ddynrec, const std::string option_name, const int min_val, const int max_val, rs2::sensor sensor, int* option_value);

      private: void registerAutoExposureROIOptions(ros::NodeHandle& nh);

      private: void set_auto_exposure_roi(const std::string option_name, rs2::sensor sensor, int new_value);

      private: void set_sensor_auto_exposure_roi(rs2::sensor sensor);

      private: rs2_stream Rs2StringToStream(const std::string &_str);

               */
      // private: void StartMonitoring();

      // private: void PublishTemperature();

               /*
      private: rs2::device rs2Dev;

      private: std::map<stream_index_pair, rs2::sensor> sensors;

      private: std::map<std::string, std::function<void(rs2::frame)>> _sensors_callback;

      private: std::vector<std::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure>> _ddynrec;



      private: float _depth_scale_meters;

      private: float _clipping_distance;

      private: bool _allow_no_texture_points;

      private: double _linear_accel_cov;

      private: double _angular_velocity_cov;

      private: bool  _hold_back_imu_for_frames;

      private: std::map<stream_index_pair, rs2_intrinsics> _stream_intrinsics;

      private: std::map<stream_index_pair, int> _width;

      private: std::map<stream_index_pair, int> _height;

      private: std::map<stream_index_pair, int> _fps;

      private: std::map<rs2_stream, int>        _format;

      private: std::map<stream_index_pair, bool> _enable;


      private: bool publishTf = true;

      private: double _tf_publish_rate;

      private: tf2_ros::StaticTransformBroadcaster _static_tf_broadcaster;

      private: tf2_ros::TransformBroadcaster _dynamic_tf_broadcaster;

      private: std::vector<geometry_msgs::TransformStamped> _static_tf_msgs;


      private: std::map<stream_index_pair,
               ImagePublisherWithFrequencyDiagnostics> _image_publishers;

      private: std::map<stream_index_pair, ros::Publisher> _imu_publishers;



      private: std::map<stream_index_pair, ros::Publisher> infoPublisher;

      private: std::map<stream_index_pair, cv::Mat> _image;


      private: std::map<stream_index_pair, int> _seq;


      private: std::map<stream_index_pair,
               sensor_msgs::CameraInfo> _camera_info;


      private: double _camera_time_base;

      private: std::map<stream_index_pair,
               std::vector<rs2::stream_profile>> enabledProfiles;

      private: ros::Publisher _pointcloud_publisher;

      private: std::chrono::steady_clock::time_point timeBase;

      private: bool syncFrames = false;

      private: bool pointcloud = false;

      private: bool _publish_odom_tf;

      private: imu_sync_method _imu_sync_method;

      private: std::string filtersStr = "";

      private: stream_index_pair pointCloudTexture;

      private: PipelineSyncer _syncer;

      private: std::vector<NamedFilter> _filters;

      private: std::vector<rs2::sensor> rs2Sensors;

      private: std::map<rs2_stream, std::shared_ptr<rs2::align>> _align;

      private: std::map<stream_index_pair, cv::Mat> _depth_aligned_image;

      private: std::map<stream_index_pair, cv::Mat> _depth_scaled_image;

      private: std::map<rs2_stream, std::string> _depth_aligned_encoding;

      private: std::map<stream_index_pair, sensor_msgs::CameraInfo>
               _depth_aligned_camera_info;

      private: std::map<stream_index_pair, int> _depth_aligned_seq;

      private: std::map<stream_index_pair, ros::Publisher>
               _depth_aligned_info_publisher;

      private: std::map<stream_index_pair,
               ImagePublisherWithFrequencyDiagnostics>
                 _depth_aligned_image_publishers;

      private: std::map<stream_index_pair, ros::Publisher>
               _depth_to_other_extrinsics_publishers;

      private: std::map<stream_index_pair, rs2_extrinsics>
               _depth_to_other_extrinsics;

      private: std::map<std::string, rs2::region_of_interest>
               _auto_exposure_roi;

      private: std::map<rs2_stream, bool> _is_first_frame;

      private: std::map<rs2_stream,
               std::vector<std::function<void()>>> _video_functions_stack;

      private: typedef std::pair<rs2_option,
               std::shared_ptr<TemperatureDiagnostics>> OptionTemperatureDiag;

      private: std::vector< OptionTemperatureDiag > _temperature_nodes;



      private: stream_index_pair _base_stream;

      private: sensor_msgs::PointCloud2 _msg_pointcloud;

      private: std::vector<unsigned int> _valid_pc_indices;

      private: std::shared_ptr<SyncedImuPublisher> _synced_imu_publisher;
               */
    };

/*
    class FrequencyDiagnostics
    {
      public: FrequencyDiagnostics(double _expectedFrequency,
                  const std::string &_name, const std::string &_hardwareId) :
                  expectedFrequency(_expectedFrequency),
                  hardwareId(_hardwareId)
      {
        igninfo << "Expected frequency for " << _name << " = " <<
          this->expectedFrequency;
        // TODO: Implement a diagnostic system
      }

      private: double expectedFrequency;
      private: std::string hardwareId;
    };
    */
/*
    class SyncedImuPublisher
    {
      public: SyncedImuPublisher()
      {
        this->is_enabled=false;
      }

      public: SyncedImuPublisher(
                  ignition::Transport::Node::Publisher *_imuPublisher,
                  std::size_t _waitingListSize = 1000 );
      public: ~SyncedImuPublisher();

      /// Pause sending messages. All messages from now on are saved in queue.
      public: void Pause();

      /// Send all pending messages and allow sending future messages.
      public: void Resume();

      /// either send or hold message.
      /// \return True if the message was published, or queued when pased.
      public: bool Publish(const ignition::msgs::IMU &_msg);

      public: uint32_t SubscriberCount()
      {
        // \todo Implement this.
        // return this->publisher->SubscriberCount();
      }

      public: void Enable(bool _enabled)
      {
        this->enabled = _enabled;
      }

      private: void PublishPendingMessages();

      private: std::mutex mutex;
      private: ignition::transport::Node::Publisher *publisher;
      private: bool pauseMode = false;
      private: std::queue<ignition::msgs::IMU> pendingMessages;
      private: std::size_t waitingListSize;
      private: bool enabled = false;
    };
    */

  }
}

// Register the plugin
IGNITION_ADD_PLUGIN(ignition::launch::WebsocketServer, ignition::launch::Plugin)

#endif
