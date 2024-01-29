#include <iostream>
#include <time.h>
#include <cmath>
#include <cstring>
#include <thread>
#include <opencv2/opencv.hpp>

#include "lvx_file.h"
#include "third_party/rapidxml/rapidxml.hpp"
#include "third_party/rapidxml/rapidxml_utils.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>


#define WRITE_BUFFER_LEN 1024 * 1024
#define MAGIC_CODE (0xac0ea767)
#define RAW_POINT_NUM 100
#define SINGLE_POINT_NUM 96
#define DUAL_POINT_NUM 48
#define TRIPLE_POINT_NUM 30
#define IMU_POINT_NUM 1
#define M_PI 3.14159265358979323846

LvxFileHandle::LvxFileHandle() : cur_frame_index_(0), cur_offset_(0), frame_duration_(kDefaultFrameDurationTime)
{
}

// 用当前时间命名创建lvx文件
bool LvxFileHandle::InitLvxFile()
{
  time_t curtime = time(nullptr);
  char filename[30] = {0};

  tm *local_time = localtime(&curtime);
  strftime(filename, sizeof(filename), "%Y-%m-%d_%H-%M-%S.lvx", local_time);
  lvx_file_.open(filename, std::ios::out | std::ios::binary);

  if (!lvx_file_.is_open())
  {
    return false;
  }
  return true;
}

// 用当前时间命名创建pcd文件
bool LvxFileHandle::InitPcdFile()
{
  time_t curtime = time(nullptr);
  char filename[30] = {0};

  tm *local_time = localtime(&curtime);
  strftime(filename, sizeof(filename), "%Y-%m-%d_%H-%M-%S.pcd", local_time);
  pcd_file_.open(filename, std::ios::out | std::ios::binary);

  pcd_file_ << "# .PCD v0.7 - Point Cloud Data file format\n";
  pcd_file_ << "VERSION 0.7\n";
  pcd_file_ << "FIELDS x y z intensity\n";
  pcd_file_ << "SIZE 4 4 4 4\n";
  pcd_file_ << "TYPE F F F U\n";
  pcd_file_ << "COUNT 1 1 1 1\n";

  pcd_file_ << "WIDTH " << std::endl;
  pcd_file_ << "HEIGHT 1\n";
  pcd_file_ << "POINTS " << std::endl;
  pcd_file_ << "DATA ascii\n";

  if (!pcd_file_.is_open())
  {
    return false;
  }
  return true;
}

Eigen::Quaternionf eulerToQuaternion(const Eigen::Vector3f &theta)
{
  using namespace Eigen;

  return Quaternionf(AngleAxisf(theta(0), Vector3f::UnitZ()) * AngleAxisf(theta(1), Vector3f::UnitY()) * AngleAxisf(theta(2), Vector3f::UnitX()));
}

// lvx文件帧头？
void LvxFileHandle::InitLvxFileHeader()
{
  extern Eigen::Vector3f t;
  extern Eigen::Quaternionf q;

  LvxFilePublicHeader lvx_file_public_header = {0};
  std::unique_ptr<char[]> write_buffer(new char[WRITE_BUFFER_LEN]);
  std::string signature = "livox_tech";
  memcpy(lvx_file_public_header.signature, signature.c_str(), signature.size());

  lvx_file_public_header.version[0] = 1;
  lvx_file_public_header.version[1] = 1;
  lvx_file_public_header.version[2] = 0;
  lvx_file_public_header.version[3] = 0;

  lvx_file_public_header.magic_code = MAGIC_CODE;

  memcpy(write_buffer.get() + cur_offset_, (void *)&lvx_file_public_header, sizeof(LvxFilePublicHeader));
  cur_offset_ += sizeof(LvxFilePublicHeader);

  uint8_t device_count = static_cast<uint8_t>(device_info_list_.size());
  LvxFilePrivateHeader lvx_file_private_header = {0};
  lvx_file_private_header.frame_duration = frame_duration_;
  lvx_file_private_header.device_count = device_count;

  memcpy(write_buffer.get() + cur_offset_, (void *)&lvx_file_private_header, sizeof(LvxFilePrivateHeader));
  cur_offset_ += sizeof(LvxFilePrivateHeader);

  for (int i = 0; i < device_count; i++)
  {
    memcpy(write_buffer.get() + cur_offset_, (void *)&device_info_list_[i], sizeof(LvxDeviceInfo));
    cur_offset_ += sizeof(LvxDeviceInfo);
  }

  lvx_file_.write((char *)write_buffer.get(), cur_offset_);
}

void LvxFileHandle::PrintXYZQueue()
{
  std::queue<XYZData> tempQueue = xyzQueue;

  while (!tempQueue.empty())
  {
    XYZData data = tempQueue.front();
    tempQueue.pop();
    std::cout << "x: " << data.x << ", y: " << data.y << ", z: " << data.z << std::endl;
  }
}

void LvxFileHandle::clear(std::queue<XYZData> q)
{
  std::queue<XYZData> empty;
  std::swap(q, empty);
}

void LvxFileHandle::SaveFrameToLvxFile(std::list<LvxBasePackDetail> &point_packet_list_temp)
{

  uint64_t cur_pos = 0;

  uint64_t cur_pos_pcd = 0;
  FrameHeader frame_header = {0};

  std::unique_ptr<char[]> write_buffer(new char[WRITE_BUFFER_LEN]);

  // 再创建一个智能指针 write_buffer_pcd，用于写入pcd文件
  std::unique_ptr<char[]> write_buffer_pcd(new char[WRITE_BUFFER_LEN]);

  frame_header.current_offset = cur_offset_;
  frame_header.next_offset = cur_offset_ + sizeof(FrameHeader);

  auto iterator = point_packet_list_temp.begin();
  for (; iterator != point_packet_list_temp.end(); iterator++)
  {
    frame_header.next_offset += iterator->pack_size;
  }

  frame_header.frame_index = cur_frame_index_;

  memcpy(write_buffer.get() + cur_pos, (void *)&frame_header, sizeof(FrameHeader));
  cur_pos += sizeof(FrameHeader);

  // auto iter = point_packet_list_temp.begin();
  // for (; iter != point_packet_list_temp.end(); iter++)
  int flag = 1;
  for (auto iter : point_packet_list_temp)
  {
    if (cur_pos + iter.pack_size >= WRITE_BUFFER_LEN)
    {
      lvx_file_.write((char *)write_buffer.get(), cur_pos);
      auto p = write_buffer_pcd.get();
      for (int i = 0; i < cur_pos_pcd / 14; i++)
      { 
        pcd_file_ << (float)(((LivoxExtendRawPoint *)p)[i].x) / 1000000.f << " "
                  << (float)(((LivoxExtendRawPoint *)p)[i].y) / 1000000.f << " "
                  << (float)(((LivoxExtendRawPoint *)p)[i].z) / 1000000.f << " "
                  << (int)(((LivoxExtendRawPoint *)p)[i].reflectivity) << " "
                  << std::endl;
      }
      cur_pos_pcd = 0;
      cur_pos_pcd += 96 * 14;
      cur_pos = 0;
      cur_pos += iter.pack_size;
    }
    else
    {
      memcpy(write_buffer.get() + cur_pos, (void *)&iter, iter.pack_size);

      // 重新复制 uint8_t raw_point[kMaxPointSize]; 数据部分
      // memcpy(write_buffer1.get() + cur_pos1, write_buffer.get() + cur_pos + 18, iter.pack_size - 18);
      memcpy(write_buffer_pcd.get() + cur_pos_pcd, &(iter.raw_point), 96 * 14);

      cur_pos += iter.pack_size;
      cur_pos_pcd += 96 * 14;
    }
  }

  lvx_file_.write((char *)write_buffer.get(), cur_pos);

  auto p = write_buffer_pcd.get();
  int i = 0;
  for (i = 0; i < cur_pos_pcd / 14; i++)
  {
    pcd_file_ << (float)(((LivoxExtendRawPoint *)p)[i].x) / 1000000.f << " "
              << (float)(((LivoxExtendRawPoint *)p)[i].y) / 1000000.f << " "
              << (float)(((LivoxExtendRawPoint *)p)[i].z) / 1000000.f << " "
              << (int)(((LivoxExtendRawPoint *)p)[i].reflectivity) << " "
              << std::endl;

    float x = (float)(((LivoxExtendRawPoint *)p)[i].x) / 1000000.f;
    float y = (float)(((LivoxExtendRawPoint *)p)[i].y) / 1000000.f;
    float z = (float)(((LivoxExtendRawPoint *)p)[i].z) / 1000000.f;

    xyzQueue.push({x, y, z});          
  }
  // pcd_file_.write((char*)write_buffer1.get(), kMaxPointSize);
  cur_offset_ = frame_header.next_offset;
  cur_frame_index_++;
}

void LvxFileHandle::CloseLvxFile()
{
  if (lvx_file_.is_open())
    lvx_file_.close();
}

void LvxFileHandle::ClosePcdFile()
{
  if (pcd_file_.is_open())
    pcd_file_.close();
}

void LvxFileHandle::BasePointsHandle(LivoxEthPacket *data, LvxBasePackDetail &packet)
{
  packet.version = data->version;
  packet.port_id = data->slot;
  packet.lidar_index = data->id;
  packet.rsvd = data->rsvd;
  packet.error_code = data->err_code;
  packet.timestamp_type = data->timestamp_type;
  packet.data_type = data->data_type;
  memcpy(packet.timestamp, data->timestamp, 8 * sizeof(uint8_t));
  switch (packet.data_type)
  {
  case PointDataType::kCartesian:
    packet.pack_size = sizeof(LvxBasePackDetail) - sizeof(packet.raw_point) -
                       sizeof(packet.pack_size) + RAW_POINT_NUM * sizeof(LivoxRawPoint);
    memcpy(packet.raw_point, (void *)data->data, RAW_POINT_NUM * sizeof(LivoxRawPoint));
    break;
  case PointDataType::kSpherical:
    packet.pack_size = sizeof(LvxBasePackDetail) - sizeof(packet.raw_point) - sizeof(packet.pack_size) + RAW_POINT_NUM * sizeof(LivoxSpherPoint);
    memcpy(packet.raw_point, (void *)data->data, RAW_POINT_NUM * sizeof(LivoxSpherPoint));
    break;
  case PointDataType::kExtendCartesian:
    packet.pack_size = sizeof(LvxBasePackDetail) - sizeof(packet.raw_point) - sizeof(packet.pack_size) + SINGLE_POINT_NUM * sizeof(LivoxExtendRawPoint);
    memcpy(packet.raw_point, (void *)data->data, SINGLE_POINT_NUM * sizeof(LivoxExtendRawPoint));
    break;
  case PointDataType::kExtendSpherical:
    packet.pack_size = sizeof(LvxBasePackDetail) - sizeof(packet.raw_point) - sizeof(packet.pack_size) + SINGLE_POINT_NUM * sizeof(LivoxExtendSpherPoint);
    memcpy(packet.raw_point, (void *)data->data, SINGLE_POINT_NUM * sizeof(LivoxExtendSpherPoint));
    break;
  case PointDataType::kDualExtendCartesian:
    packet.pack_size = sizeof(LvxBasePackDetail) - sizeof(packet.raw_point) - sizeof(packet.pack_size) + DUAL_POINT_NUM * sizeof(LivoxDualExtendRawPoint);
    memcpy(packet.raw_point, (void *)data->data, DUAL_POINT_NUM * sizeof(LivoxDualExtendRawPoint));
    break;
  case PointDataType::kDualExtendSpherical:
    packet.pack_size = sizeof(LvxBasePackDetail) - sizeof(packet.raw_point) - sizeof(packet.pack_size) + DUAL_POINT_NUM * sizeof(LivoxDualExtendSpherPoint);
    memcpy(packet.raw_point, (void *)data->data, DUAL_POINT_NUM * sizeof(LivoxDualExtendSpherPoint));
    break;
  case PointDataType::kImu:
    packet.pack_size = sizeof(LvxBasePackDetail) - sizeof(packet.raw_point) - sizeof(packet.pack_size) + IMU_POINT_NUM * sizeof(LivoxImuPoint);
    memcpy(packet.raw_point, (void *)data->data, IMU_POINT_NUM * sizeof(LivoxImuPoint));
    break;
  case PointDataType::kTripleExtendCartesian:
    packet.pack_size = sizeof(LvxBasePackDetail) - sizeof(packet.raw_point) - sizeof(packet.pack_size) + TRIPLE_POINT_NUM * sizeof(LivoxTripleExtendRawPoint);
    memcpy(packet.raw_point, (void *)data->data, TRIPLE_POINT_NUM * sizeof(LivoxTripleExtendRawPoint));
    break;
  case PointDataType::kTripleExtendSpherical:
    packet.pack_size = sizeof(LvxBasePackDetail) - sizeof(packet.raw_point) - sizeof(packet.pack_size) + TRIPLE_POINT_NUM * sizeof(LivoxTripleExtendSpherPoint);
    memcpy(packet.raw_point, (void *)data->data, TRIPLE_POINT_NUM * sizeof(LivoxTripleExtendSpherPoint));
    break;
  default:
    break;
  }
}

void ParseExtrinsicXml(DeviceItem &item, LvxDeviceInfo &info)
{
  rapidxml::file<> extrinsic_param("extrinsic.xml");
  rapidxml::xml_document<> doc;
  doc.parse<0>(extrinsic_param.data());
  rapidxml::xml_node<> *root = doc.first_node();
  if ("Livox" == (std::string)root->name())
  {
    for (rapidxml::xml_node<> *device = root->first_node(); device; device = device->next_sibling())
    {
      if ("Device" == (std::string)device->name() && (strncmp(item.info.broadcast_code, device->value(), kBroadcastCodeSize) == 0))
      {
        memcpy(info.lidar_broadcast_code, device->value(), kBroadcastCodeSize);
        memset(info.hub_broadcast_code, 0, kBroadcastCodeSize);
        info.device_type = item.info.type;
        info.device_index = item.handle;
        for (rapidxml::xml_attribute<> *param = device->first_attribute(); param; param = param->next_attribute())
        {
          if ("roll" == (std::string)param->name())
            info.roll = static_cast<float>(atof(param->value()));
          if ("pitch" == (std::string)param->name())
            info.pitch = static_cast<float>(atof(param->value()));
          if ("yaw" == (std::string)param->name())
            info.yaw = static_cast<float>(atof(param->value()));
          if ("x" == (std::string)param->name())
            info.x = static_cast<float>(atof(param->value()));
          if ("y" == (std::string)param->name())
            info.y = static_cast<float>(atof(param->value()));
          if ("z" == (std::string)param->name())
            info.z = static_cast<float>(atof(param->value()));
        }
      }
    }
  }
}
