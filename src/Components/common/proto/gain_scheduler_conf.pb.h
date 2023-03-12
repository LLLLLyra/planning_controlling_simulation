// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: gain_scheduler_conf.proto

#ifndef PROTOBUF_INCLUDED_gain_5fscheduler_5fconf_2eproto
#define PROTOBUF_INCLUDED_gain_5fscheduler_5fconf_2eproto

#include <limits>
#include <string>

#include <google/protobuf/port_def.inc>
#if PROTOBUF_VERSION < 3007000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers. Please update
#error your headers.
#endif
#if 3007001 < PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers. Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/port_undef.inc>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_table_driven.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/inlined_string_field.h>
#include <google/protobuf/metadata.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>  // IWYU pragma: export
#include <google/protobuf/extension_set.h>  // IWYU pragma: export
#include <google/protobuf/unknown_field_set.h>
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_gain_5fscheduler_5fconf_2eproto

// Internal implementation detail -- do not use these members.
struct TableStruct_gain_5fscheduler_5fconf_2eproto {
  static const ::google::protobuf::internal::ParseTableField entries[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::google::protobuf::internal::AuxillaryParseTableField aux[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::google::protobuf::internal::ParseTable schema[2]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::google::protobuf::internal::FieldMetadata field_metadata[];
  static const ::google::protobuf::internal::SerializationTable serialization_table[];
  static const ::google::protobuf::uint32 offsets[];
};
void AddDescriptors_gain_5fscheduler_5fconf_2eproto();
namespace controller {
class GainScheduler;
class GainSchedulerDefaultTypeInternal;
extern GainSchedulerDefaultTypeInternal _GainScheduler_default_instance_;
class GainSchedulerInfo;
class GainSchedulerInfoDefaultTypeInternal;
extern GainSchedulerInfoDefaultTypeInternal _GainSchedulerInfo_default_instance_;
}  // namespace controller
namespace google {
namespace protobuf {
template<> ::controller::GainScheduler* Arena::CreateMaybeMessage<::controller::GainScheduler>(Arena*);
template<> ::controller::GainSchedulerInfo* Arena::CreateMaybeMessage<::controller::GainSchedulerInfo>(Arena*);
}  // namespace protobuf
}  // namespace google
namespace controller {

// ===================================================================

class GainScheduler :
    public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:controller.GainScheduler) */ {
 public:
  GainScheduler();
  virtual ~GainScheduler();

  GainScheduler(const GainScheduler& from);

  inline GainScheduler& operator=(const GainScheduler& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  GainScheduler(GainScheduler&& from) noexcept
    : GainScheduler() {
    *this = ::std::move(from);
  }

  inline GainScheduler& operator=(GainScheduler&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  inline const ::google::protobuf::UnknownFieldSet& unknown_fields() const {
    return _internal_metadata_.unknown_fields();
  }
  inline ::google::protobuf::UnknownFieldSet* mutable_unknown_fields() {
    return _internal_metadata_.mutable_unknown_fields();
  }

  static const ::google::protobuf::Descriptor* descriptor() {
    return default_instance().GetDescriptor();
  }
  static const GainScheduler& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const GainScheduler* internal_default_instance() {
    return reinterpret_cast<const GainScheduler*>(
               &_GainScheduler_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  void Swap(GainScheduler* other);
  friend void swap(GainScheduler& a, GainScheduler& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline GainScheduler* New() const final {
    return CreateMaybeMessage<GainScheduler>(nullptr);
  }

  GainScheduler* New(::google::protobuf::Arena* arena) const final {
    return CreateMaybeMessage<GainScheduler>(arena);
  }
  void CopyFrom(const ::google::protobuf::Message& from) final;
  void MergeFrom(const ::google::protobuf::Message& from) final;
  void CopyFrom(const GainScheduler& from);
  void MergeFrom(const GainScheduler& from);
  PROTOBUF_ATTRIBUTE_REINITIALIZES void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  #if GOOGLE_PROTOBUF_ENABLE_EXPERIMENTAL_PARSER
  static const char* _InternalParse(const char* begin, const char* end, void* object, ::google::protobuf::internal::ParseContext* ctx);
  ::google::protobuf::internal::ParseFunc _ParseFunc() const final { return _InternalParse; }
  #else
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input) final;
  #endif  // GOOGLE_PROTOBUF_ENABLE_EXPERIMENTAL_PARSER
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const final;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      ::google::protobuf::uint8* target) const final;
  int GetCachedSize() const final { return _cached_size_.Get(); }

  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(GainScheduler* other);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return nullptr;
  }
  inline void* MaybeArenaPtr() const {
    return nullptr;
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const final;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // repeated .controller.GainSchedulerInfo scheduler = 1;
  int scheduler_size() const;
  void clear_scheduler();
  static const int kSchedulerFieldNumber = 1;
  ::controller::GainSchedulerInfo* mutable_scheduler(int index);
  ::google::protobuf::RepeatedPtrField< ::controller::GainSchedulerInfo >*
      mutable_scheduler();
  const ::controller::GainSchedulerInfo& scheduler(int index) const;
  ::controller::GainSchedulerInfo* add_scheduler();
  const ::google::protobuf::RepeatedPtrField< ::controller::GainSchedulerInfo >&
      scheduler() const;

  // @@protoc_insertion_point(class_scope:controller.GainScheduler)
 private:
  class HasBitSetters;

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::internal::HasBits<1> _has_bits_;
  mutable ::google::protobuf::internal::CachedSize _cached_size_;
  ::google::protobuf::RepeatedPtrField< ::controller::GainSchedulerInfo > scheduler_;
  friend struct ::TableStruct_gain_5fscheduler_5fconf_2eproto;
};
// -------------------------------------------------------------------

class GainSchedulerInfo :
    public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:controller.GainSchedulerInfo) */ {
 public:
  GainSchedulerInfo();
  virtual ~GainSchedulerInfo();

  GainSchedulerInfo(const GainSchedulerInfo& from);

  inline GainSchedulerInfo& operator=(const GainSchedulerInfo& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  GainSchedulerInfo(GainSchedulerInfo&& from) noexcept
    : GainSchedulerInfo() {
    *this = ::std::move(from);
  }

  inline GainSchedulerInfo& operator=(GainSchedulerInfo&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  inline const ::google::protobuf::UnknownFieldSet& unknown_fields() const {
    return _internal_metadata_.unknown_fields();
  }
  inline ::google::protobuf::UnknownFieldSet* mutable_unknown_fields() {
    return _internal_metadata_.mutable_unknown_fields();
  }

  static const ::google::protobuf::Descriptor* descriptor() {
    return default_instance().GetDescriptor();
  }
  static const GainSchedulerInfo& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const GainSchedulerInfo* internal_default_instance() {
    return reinterpret_cast<const GainSchedulerInfo*>(
               &_GainSchedulerInfo_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  void Swap(GainSchedulerInfo* other);
  friend void swap(GainSchedulerInfo& a, GainSchedulerInfo& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline GainSchedulerInfo* New() const final {
    return CreateMaybeMessage<GainSchedulerInfo>(nullptr);
  }

  GainSchedulerInfo* New(::google::protobuf::Arena* arena) const final {
    return CreateMaybeMessage<GainSchedulerInfo>(arena);
  }
  void CopyFrom(const ::google::protobuf::Message& from) final;
  void MergeFrom(const ::google::protobuf::Message& from) final;
  void CopyFrom(const GainSchedulerInfo& from);
  void MergeFrom(const GainSchedulerInfo& from);
  PROTOBUF_ATTRIBUTE_REINITIALIZES void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  #if GOOGLE_PROTOBUF_ENABLE_EXPERIMENTAL_PARSER
  static const char* _InternalParse(const char* begin, const char* end, void* object, ::google::protobuf::internal::ParseContext* ctx);
  ::google::protobuf::internal::ParseFunc _ParseFunc() const final { return _InternalParse; }
  #else
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input) final;
  #endif  // GOOGLE_PROTOBUF_ENABLE_EXPERIMENTAL_PARSER
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const final;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      ::google::protobuf::uint8* target) const final;
  int GetCachedSize() const final { return _cached_size_.Get(); }

  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(GainSchedulerInfo* other);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return nullptr;
  }
  inline void* MaybeArenaPtr() const {
    return nullptr;
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const final;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // optional double speed = 1;
  bool has_speed() const;
  void clear_speed();
  static const int kSpeedFieldNumber = 1;
  double speed() const;
  void set_speed(double value);

  // optional double ratio = 2;
  bool has_ratio() const;
  void clear_ratio();
  static const int kRatioFieldNumber = 2;
  double ratio() const;
  void set_ratio(double value);

  // @@protoc_insertion_point(class_scope:controller.GainSchedulerInfo)
 private:
  class HasBitSetters;

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::internal::HasBits<1> _has_bits_;
  mutable ::google::protobuf::internal::CachedSize _cached_size_;
  double speed_;
  double ratio_;
  friend struct ::TableStruct_gain_5fscheduler_5fconf_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// GainScheduler

// repeated .controller.GainSchedulerInfo scheduler = 1;
inline int GainScheduler::scheduler_size() const {
  return scheduler_.size();
}
inline void GainScheduler::clear_scheduler() {
  scheduler_.Clear();
}
inline ::controller::GainSchedulerInfo* GainScheduler::mutable_scheduler(int index) {
  // @@protoc_insertion_point(field_mutable:controller.GainScheduler.scheduler)
  return scheduler_.Mutable(index);
}
inline ::google::protobuf::RepeatedPtrField< ::controller::GainSchedulerInfo >*
GainScheduler::mutable_scheduler() {
  // @@protoc_insertion_point(field_mutable_list:controller.GainScheduler.scheduler)
  return &scheduler_;
}
inline const ::controller::GainSchedulerInfo& GainScheduler::scheduler(int index) const {
  // @@protoc_insertion_point(field_get:controller.GainScheduler.scheduler)
  return scheduler_.Get(index);
}
inline ::controller::GainSchedulerInfo* GainScheduler::add_scheduler() {
  // @@protoc_insertion_point(field_add:controller.GainScheduler.scheduler)
  return scheduler_.Add();
}
inline const ::google::protobuf::RepeatedPtrField< ::controller::GainSchedulerInfo >&
GainScheduler::scheduler() const {
  // @@protoc_insertion_point(field_list:controller.GainScheduler.scheduler)
  return scheduler_;
}

// -------------------------------------------------------------------

// GainSchedulerInfo

// optional double speed = 1;
inline bool GainSchedulerInfo::has_speed() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void GainSchedulerInfo::clear_speed() {
  speed_ = 0;
  _has_bits_[0] &= ~0x00000001u;
}
inline double GainSchedulerInfo::speed() const {
  // @@protoc_insertion_point(field_get:controller.GainSchedulerInfo.speed)
  return speed_;
}
inline void GainSchedulerInfo::set_speed(double value) {
  _has_bits_[0] |= 0x00000001u;
  speed_ = value;
  // @@protoc_insertion_point(field_set:controller.GainSchedulerInfo.speed)
}

// optional double ratio = 2;
inline bool GainSchedulerInfo::has_ratio() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void GainSchedulerInfo::clear_ratio() {
  ratio_ = 0;
  _has_bits_[0] &= ~0x00000002u;
}
inline double GainSchedulerInfo::ratio() const {
  // @@protoc_insertion_point(field_get:controller.GainSchedulerInfo.ratio)
  return ratio_;
}
inline void GainSchedulerInfo::set_ratio(double value) {
  _has_bits_[0] |= 0x00000002u;
  ratio_ = value;
  // @@protoc_insertion_point(field_set:controller.GainSchedulerInfo.ratio)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace controller

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // PROTOBUF_INCLUDED_gain_5fscheduler_5fconf_2eproto
