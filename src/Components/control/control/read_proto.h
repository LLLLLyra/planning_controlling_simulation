#ifndef _READ_PROTO_H_
#define _READ_PROTO_H_

#include <fcntl.h>
#include <glob.h>
#include <glog/logging.h>

#include <fstream>

#include "google/protobuf/io/zero_copy_stream_impl.h"
#include "google/protobuf/text_format.h"

namespace Controlling {
bool GetProtoFromASCIIFile(const std::string &file_name,
                           google::protobuf::Message *message) {
  using google::protobuf::TextFormat;
  using google::protobuf::io::FileInputStream;
  using google::protobuf::io::ZeroCopyInputStream;
  int file_descriptor = open(file_name.c_str(), O_RDONLY);
  if (file_descriptor < 0) {
    LOG(ERROR) << "Failed to open file " << file_name << " in text mode.";
    // Failed to open;
    return false;
  }

  ZeroCopyInputStream *input = new FileInputStream(file_descriptor);
  bool success = TextFormat::Parse(input, message);
  if (!success) {
    LOG(ERROR) << "Failed to parse file " << file_name << " as text proto.";
  }
  delete input;
  close(file_descriptor);
  return success;
}
}  // namespace Controlling

#endif