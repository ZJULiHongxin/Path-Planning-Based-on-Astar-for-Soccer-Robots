// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: example_robo.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_example_5frobo_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_example_5frobo_2eproto

#include <limits>
#include <string>

#include <google/protobuf/port_def.inc>
#if PROTOBUF_VERSION < 3009000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers. Please update
#error your headers.
#endif
#if 3009001 < PROTOBUF_MIN_PROTOC_VERSION
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
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>  // IWYU pragma: export
#include <google/protobuf/extension_set.h>  // IWYU pragma: export
#include <google/protobuf/unknown_field_set.h>
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_example_5frobo_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_example_5frobo_2eproto {
  static const ::PROTOBUF_NAMESPACE_ID::internal::ParseTableField entries[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::AuxillaryParseTableField aux[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::ParseTable schema[1]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::FieldMetadata field_metadata[];
  static const ::PROTOBUF_NAMESPACE_ID::internal::SerializationTable serialization_table[];
  static const ::PROTOBUF_NAMESPACE_ID::uint32 offsets[];
};
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_example_5frobo_2eproto;
namespace tutorial {
class Robot;
class RobotDefaultTypeInternal;
extern RobotDefaultTypeInternal _Robot_default_instance_;
}  // namespace tutorial
PROTOBUF_NAMESPACE_OPEN
template<> ::tutorial::Robot* Arena::CreateMaybeMessage<::tutorial::Robot>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace tutorial {

// ===================================================================

class Robot :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:tutorial.Robot) */ {
 public:
  Robot();
  virtual ~Robot();

  Robot(const Robot& from);
  Robot(Robot&& from) noexcept
    : Robot() {
    *this = ::std::move(from);
  }

  inline Robot& operator=(const Robot& from) {
    CopyFrom(from);
    return *this;
  }
  inline Robot& operator=(Robot&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }

  inline const ::PROTOBUF_NAMESPACE_ID::UnknownFieldSet& unknown_fields() const {
    return _internal_metadata_.unknown_fields();
  }
  inline ::PROTOBUF_NAMESPACE_ID::UnknownFieldSet* mutable_unknown_fields() {
    return _internal_metadata_.mutable_unknown_fields();
  }

  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* descriptor() {
    return GetDescriptor();
  }
  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* GetDescriptor() {
    return GetMetadataStatic().descriptor;
  }
  static const ::PROTOBUF_NAMESPACE_ID::Reflection* GetReflection() {
    return GetMetadataStatic().reflection;
  }
  static const Robot& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const Robot* internal_default_instance() {
    return reinterpret_cast<const Robot*>(
               &_Robot_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(Robot& a, Robot& b) {
    a.Swap(&b);
  }
  inline void Swap(Robot* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline Robot* New() const final {
    return CreateMaybeMessage<Robot>(nullptr);
  }

  Robot* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<Robot>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const Robot& from);
  void MergeFrom(const Robot& from);
  PROTOBUF_ATTRIBUTE_REINITIALIZES void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  #if GOOGLE_PROTOBUF_ENABLE_EXPERIMENTAL_PARSER
  const char* _InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) final;
  #else
  bool MergePartialFromCodedStream(
      ::PROTOBUF_NAMESPACE_ID::io::CodedInputStream* input) final;
  #endif  // GOOGLE_PROTOBUF_ENABLE_EXPERIMENTAL_PARSER
  void SerializeWithCachedSizes(
      ::PROTOBUF_NAMESPACE_ID::io::CodedOutputStream* output) const final;
  ::PROTOBUF_NAMESPACE_ID::uint8* InternalSerializeWithCachedSizesToArray(
      ::PROTOBUF_NAMESPACE_ID::uint8* target) const final;
  int GetCachedSize() const final { return _cached_size_.Get(); }

  private:
  inline void SharedCtor();
  inline void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(Robot* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "tutorial.Robot";
  }
  private:
  inline ::PROTOBUF_NAMESPACE_ID::Arena* GetArenaNoVirtual() const {
    return nullptr;
  }
  inline void* MaybeArenaPtr() const {
    return nullptr;
  }
  public:

  ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadata() const final;
  private:
  static ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadataStatic() {
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_example_5frobo_2eproto);
    return ::descriptor_table_example_5frobo_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kRobotIdFieldNumber = 1,
    kXFieldNumber = 2,
    kYFieldNumber = 3,
    kOrientationFieldNumber = 4,
  };
  // optional uint32 robot_id = 1;
  bool has_robot_id() const;
  void clear_robot_id();
  ::PROTOBUF_NAMESPACE_ID::uint32 robot_id() const;
  void set_robot_id(::PROTOBUF_NAMESPACE_ID::uint32 value);

  // required float x = 2;
  bool has_x() const;
  void clear_x();
  float x() const;
  void set_x(float value);

  // required float y = 3;
  bool has_y() const;
  void clear_y();
  float y() const;
  void set_y(float value);

  // optional float orientation = 4;
  bool has_orientation() const;
  void clear_orientation();
  float orientation() const;
  void set_orientation(float value);

  // @@protoc_insertion_point(class_scope:tutorial.Robot)
 private:
  class _Internal;

  // helper for ByteSizeLong()
  size_t RequiredFieldsByteSizeFallback() const;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::uint32 robot_id_;
  float x_;
  float y_;
  float orientation_;
  friend struct ::TableStruct_example_5frobo_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// Robot

// optional uint32 robot_id = 1;
inline bool Robot::has_robot_id() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void Robot::clear_robot_id() {
  robot_id_ = 0u;
  _has_bits_[0] &= ~0x00000001u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 Robot::robot_id() const {
  // @@protoc_insertion_point(field_get:tutorial.Robot.robot_id)
  return robot_id_;
}
inline void Robot::set_robot_id(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000001u;
  robot_id_ = value;
  // @@protoc_insertion_point(field_set:tutorial.Robot.robot_id)
}

// required float x = 2;
inline bool Robot::has_x() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void Robot::clear_x() {
  x_ = 0;
  _has_bits_[0] &= ~0x00000002u;
}
inline float Robot::x() const {
  // @@protoc_insertion_point(field_get:tutorial.Robot.x)
  return x_;
}
inline void Robot::set_x(float value) {
  _has_bits_[0] |= 0x00000002u;
  x_ = value;
  // @@protoc_insertion_point(field_set:tutorial.Robot.x)
}

// required float y = 3;
inline bool Robot::has_y() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void Robot::clear_y() {
  y_ = 0;
  _has_bits_[0] &= ~0x00000004u;
}
inline float Robot::y() const {
  // @@protoc_insertion_point(field_get:tutorial.Robot.y)
  return y_;
}
inline void Robot::set_y(float value) {
  _has_bits_[0] |= 0x00000004u;
  y_ = value;
  // @@protoc_insertion_point(field_set:tutorial.Robot.y)
}

// optional float orientation = 4;
inline bool Robot::has_orientation() const {
  return (_has_bits_[0] & 0x00000008u) != 0;
}
inline void Robot::clear_orientation() {
  orientation_ = 0;
  _has_bits_[0] &= ~0x00000008u;
}
inline float Robot::orientation() const {
  // @@protoc_insertion_point(field_get:tutorial.Robot.orientation)
  return orientation_;
}
inline void Robot::set_orientation(float value) {
  _has_bits_[0] |= 0x00000008u;
  orientation_ = value;
  // @@protoc_insertion_point(field_set:tutorial.Robot.orientation)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace tutorial

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_example_5frobo_2eproto