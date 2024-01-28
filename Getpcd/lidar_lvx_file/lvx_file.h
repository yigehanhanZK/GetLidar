#include <condition_variable>
#include <memory>
#include <fstream>
#include <list>
#include <vector>
#include <mutex>
#include <thread>
#include <queue>
#include <iostream>
#include <string>
#include <ctime>
#include "livox_sdk.h"

#define kMaxPointSize 1500
#define kDefaultFrameDurationTime 50

typedef enum {
  kDeviceStateDisconnect = 0,
  kDeviceStateConnect = 1,
  kDeviceStateSampling = 2,
} DeviceState;

typedef struct {
  uint8_t handle;
  DeviceState device_state;
  DeviceInfo info;
} DeviceItem;

#pragma pack(1)

typedef struct {
  uint8_t signature[16];
  uint8_t version[4];
  uint32_t magic_code;
} LvxFilePublicHeader;

typedef struct {
  uint32_t frame_duration;
  uint8_t device_count;
} LvxFilePrivateHeader;

typedef struct {
  uint8_t lidar_broadcast_code[16];
  uint8_t hub_broadcast_code[16];
  uint8_t device_index;
  uint8_t device_type;
  uint8_t extrinsic_enable;
  float roll;
  float pitch;
  float yaw;
  float x;
  float y;
  float z;
} LvxDeviceInfo;

typedef struct {
  uint8_t device_index;
  uint8_t version;
  uint8_t port_id;
  uint8_t lidar_index;
  uint8_t rsvd;
  uint32_t error_code;
  uint8_t timestamp_type;
  uint8_t data_type;
  uint8_t timestamp[8];
  uint8_t raw_point[kMaxPointSize];
  uint32_t pack_size;
} LvxBasePackDetail;

typedef struct {
  uint64_t current_offset;
  uint64_t next_offset;
  uint64_t frame_index;
} FrameHeader;

#pragma pack()

typedef struct {
  float x;
  float y;
  float z;
}XYZData;

class LvxFileHandle {
public:
  LvxFileHandle();

  bool InitLvxFile();
  bool InitPcdFile();
  void InitLvxFileHeader();
  void SaveFrameToLvxFile(std::list<LvxBasePackDetail> &point_packet_list_temp);
  void CloseLvxFile();
  void ClosePcdFile();

  void PrintXYZQueue();
  std::queue<XYZData> xyzQueue;

  void AddDeviceInfo(LvxDeviceInfo &info) { device_info_list_.push_back(info); };
  int GetDeviceInfoListSize() { return device_info_list_.size(); }

  void BasePointsHandle(LivoxEthPacket *data, LvxBasePackDetail &packet);

private:
  std::ofstream lvx_file_;
  std::ofstream pcd_file_;

  std::vector<LvxDeviceInfo> device_info_list_;
  uint32_t cur_frame_index_;
  uint64_t cur_offset_;
  uint32_t frame_duration_;
};

void ParseExtrinsicXml(DeviceItem &item, LvxDeviceInfo &info);