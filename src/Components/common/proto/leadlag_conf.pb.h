// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: leadlag_conf.proto

#ifndef PROTOBUF_INCLUDED_leadlag_5fconf_2eproto
#define PROTOBUF_INCLUDED_leadlag_5fconf_2eproto

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
#define PROTOBUF_INTERNAL_EXPORT_leadlag_5fconf_2eproto

// Internal implementation detail -- do not use these members.
struct TableStruct_leadlag_5fconf_2eproto {
  static const ::google::protobuf::internal::ParseTableField entries[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::google::protobuf::internal::AuxillaryParseTableField aux[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::google::protobuf::internal::ParseTable schema[1]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::google::protobuf::internal::FieldMetadata field_metadata[];
  static const ::google::protobuf::internal::SerializationTable serialization_table[];
  static const ::google::protobuf::uint32 offsets[];
};
void AddDescriptors_leadlag_5fconf_2eproto();
namespace controller {
class LeadlagConf;
class LeadlagConfDefaultTypeInternal;
extern LeadlagConfDefaultTypeInternal _LeadlagConf_default_instance_;
}  // namespace controller
namespace google {
namespace protobuf {
template<> ::controller::LeadlagConf* Arena::CreateMaybeMessage<::controller::LeadlagConf>(Arena*);
}  // namespace protobuf
}  // namespace google
namespace controller {

// ===================================================================

class LeadlagConf :
    public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:controller.LeadlagConf) */ {
 public:
  LeadlagConf();
  virtual ~LeadlagConf();

  LeadlagConf(const LeadlagConf& from);

  inline LeadlagConf& operator=(const LeadlagConf& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  LeadlagConf(LeadlagConf&& from) noexcept
    : LeadlagConf() {
    *this = ::std::move(from);
  }

  inline LeadlagConf& operator=(LeadlagConf&& from) noexcept {
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
  static const LeadlagConf& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const LeadlagConf* internal_default_instance() {
    return reinterpret_cast<const LeadlagConf*>(
               &_LeadlagConf_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  void Swap(LeadlagConf* other);
  friend void swap(LeadlagConf& a, LeadlagConf& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline LeadlagConf* New() const final {
    return CreateMaybeMessage<LeadlagConf>(nullptr);
  }

  LeadlagConf* New(::google::protobuf::Arena* arena) const final {
    return CreateMaybeMessage<LeadlagConf>(arena);
  }
  void CopyFrom(const ::google::protobuf::Message& from) final;
  void MergeFrom(const ::google::protobuf::Message& from) final;
  void CopyFrom(const LeadlagConf& from);
  void MergeFrom(const LeadlagConf& from);
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
  void InternalSwap(LeadlagConf* other);
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

  // optional double tau = 4 [default = 0];
  bool has_tau() const;
  void clear_tau();
  static const int kTauFieldNumber = 4;
  double tau() const;
  void set_tau(double value);

  // optional double innerstate_saturation_level = 1 [default = 300];
  bool has_innerstate_saturation_level() const;
  void clear_innerstate_saturation_level();
  static const int kInnerstateSaturationLevelFieldNumber = 1;
  double innerstate_saturation_level() const;
  void set_innerstate_saturation_level(double value);

  // optional double alpha = 2 [default = 0.1];
  bool has_alpha() const;
  void clear_alpha();
  static const int kAlphaFieldNumber = 2;
  double alpha() const;
  void set_alpha(double value);

  // optional double beta = 3 [default = 1];
  bool has_beta() const;
  void clear_beta();
  static const int kBetaFieldNumber = 3;
  double beta() const;
  void set_beta(double value);

  // @@protoc_insertion_point(class_scope:controller.LeadlagConf)
 private:
  class HasBitSetters;

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::internal::HasBits<1> _has_bits_;
  mutable ::google::protobuf::internal::CachedSize _cached_size_;
  double tau_;
  double innerstate_saturation_level_;
  double alpha_;
  double beta_;
  friend struct ::TableStruct_leadlag_5fconf_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// LeadlagConf

// optional double innerstate_saturation_level = 1 [default = 300];
inline bool LeadlagConf::has_innerstate_saturation_level() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void LeadlagConf::clear_innerstate_saturation_level() {
  innerstate_saturation_level_ = 300;
  _has_bits_[0] &= ~0x00000002u;
}
inline double LeadlagConf::innerstate_saturation_level() const {
  // @@protoc_insertion_point(field_get:controller.LeadlagConf.innerstate_saturation_level)
  return innerstate_saturation_level_;
}
inline void LeadlagConf::set_innerstate_saturation_level(double value) {
  _has_bits_[0] |= 0x00000002u;
  innerstate_saturation_level_ = value;
  // @@protoc_insertion_point(field_set:controller.LeadlagConf.innerstate_saturation_level)
}

// optional double alpha = 2 [default = 0.1];
inline bool LeadlagConf::has_alpha() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void LeadlagConf::clear_alpha() {
  alpha_ = 0.1;
  _has_bits_[0] &= ~0x00000004u;
}
inline double LeadlagConf::alpha() const {
  // @@protoc_insertion_point(field_get:controller.LeadlagConf.alpha)
  return alpha_;
}
inline void LeadlagConf::set_alpha(double value) {
  _has_bits_[0] |= 0x00000004u;
  alpha_ = value;
  // @@protoc_insertion_point(field_set:controller.LeadlagConf.alpha)
}

// optional double beta = 3 [default = 1];
inline bool LeadlagConf::has_beta() const {
  return (_has_bits_[0] & 0x00000008u) != 0;
}
inline void LeadlagConf::clear_beta() {
  beta_ = 1;
  _has_bits_[0] &= ~0x00000008u;
}
inline double LeadlagConf::beta() const {
  // @@protoc_insertion_point(field_get:controller.LeadlagConf.beta)
  return beta_;
}
inline void LeadlagConf::set_beta(double value) {
  _has_bits_[0] |= 0x00000008u;
  beta_ = value;
  // @@protoc_insertion_point(field_set:controller.LeadlagConf.beta)
}

// optional double tau = 4 [default = 0];
inline bool LeadlagConf::has_tau() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void LeadlagConf::clear_tau() {
  tau_ = 0;
  _has_bits_[0] &= ~0x00000001u;
}
inline double LeadlagConf::tau() const {
  // @@protoc_insertion_point(field_get:controller.LeadlagConf.tau)
  return tau_;
}
inline void LeadlagConf::set_tau(double value) {
  _has_bits_[0] |= 0x00000001u;
  tau_ = value;
  // @@protoc_insertion_point(field_set:controller.LeadlagConf.tau)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace controller

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // PROTOBUF_INCLUDED_leadlag_5fconf_2eproto
