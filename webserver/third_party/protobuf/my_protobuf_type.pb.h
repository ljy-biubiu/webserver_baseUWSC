// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: my_protobuf_type.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_my_5fprotobuf_5ftype_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_my_5fprotobuf_5ftype_2eproto

#include <limits>
#include <string>

#include <google/protobuf/port_def.inc>
#if PROTOBUF_VERSION < 3017000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers. Please update
#error your headers.
#endif
#if 3017003 < PROTOBUF_MIN_PROTOC_VERSION
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
#include <google/protobuf/metadata_lite.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>  // IWYU pragma: export
#include <google/protobuf/extension_set.h>  // IWYU pragma: export
#include <google/protobuf/unknown_field_set.h>
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_my_5fprotobuf_5ftype_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_my_5fprotobuf_5ftype_2eproto {
  static const ::PROTOBUF_NAMESPACE_ID::internal::ParseTableField entries[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::AuxiliaryParseTableField aux[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::ParseTable schema[4]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::FieldMetadata field_metadata[];
  static const ::PROTOBUF_NAMESPACE_ID::internal::SerializationTable serialization_table[];
  static const ::PROTOBUF_NAMESPACE_ID::uint32 offsets[];
};
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_my_5fprotobuf_5ftype_2eproto;
namespace pt {
class Point;
struct PointDefaultTypeInternal;
extern PointDefaultTypeInternal _Point_default_instance_;
class Point2;
struct Point2DefaultTypeInternal;
extern Point2DefaultTypeInternal _Point2_default_instance_;
class Points;
struct PointsDefaultTypeInternal;
extern PointsDefaultTypeInternal _Points_default_instance_;
class Points2;
struct Points2DefaultTypeInternal;
extern Points2DefaultTypeInternal _Points2_default_instance_;
}  // namespace pt
PROTOBUF_NAMESPACE_OPEN
template<> ::pt::Point* Arena::CreateMaybeMessage<::pt::Point>(Arena*);
template<> ::pt::Point2* Arena::CreateMaybeMessage<::pt::Point2>(Arena*);
template<> ::pt::Points* Arena::CreateMaybeMessage<::pt::Points>(Arena*);
template<> ::pt::Points2* Arena::CreateMaybeMessage<::pt::Points2>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace pt {

// ===================================================================

class Point final :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pt.Point) */ {
 public:
  inline Point() : Point(nullptr) {}
  ~Point() override;
  explicit constexpr Point(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized);

  Point(const Point& from);
  Point(Point&& from) noexcept
    : Point() {
    *this = ::std::move(from);
  }

  inline Point& operator=(const Point& from) {
    CopyFrom(from);
    return *this;
  }
  inline Point& operator=(Point&& from) noexcept {
    if (this == &from) return *this;
    if (GetOwningArena() == from.GetOwningArena()) {
      InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }

  inline const ::PROTOBUF_NAMESPACE_ID::UnknownFieldSet& unknown_fields() const {
    return _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance);
  }
  inline ::PROTOBUF_NAMESPACE_ID::UnknownFieldSet* mutable_unknown_fields() {
    return _internal_metadata_.mutable_unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
  }

  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* descriptor() {
    return GetDescriptor();
  }
  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* GetDescriptor() {
    return default_instance().GetMetadata().descriptor;
  }
  static const ::PROTOBUF_NAMESPACE_ID::Reflection* GetReflection() {
    return default_instance().GetMetadata().reflection;
  }
  static const Point& default_instance() {
    return *internal_default_instance();
  }
  static inline const Point* internal_default_instance() {
    return reinterpret_cast<const Point*>(
               &_Point_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(Point& a, Point& b) {
    a.Swap(&b);
  }
  inline void Swap(Point* other) {
    if (other == this) return;
    if (GetOwningArena() == other->GetOwningArena()) {
      InternalSwap(other);
    } else {
      ::PROTOBUF_NAMESPACE_ID::internal::GenericSwap(this, other);
    }
  }
  void UnsafeArenaSwap(Point* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetOwningArena() == other->GetOwningArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline Point* New() const final {
    return new Point();
  }

  Point* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<Point>(arena);
  }
  using ::PROTOBUF_NAMESPACE_ID::Message::CopyFrom;
  void CopyFrom(const Point& from);
  using ::PROTOBUF_NAMESPACE_ID::Message::MergeFrom;
  void MergeFrom(const Point& from);
  private:
  static void MergeImpl(::PROTOBUF_NAMESPACE_ID::Message*to, const ::PROTOBUF_NAMESPACE_ID::Message&from);
  public:
  PROTOBUF_ATTRIBUTE_REINITIALIZES void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  const char* _InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) final;
  ::PROTOBUF_NAMESPACE_ID::uint8* _InternalSerialize(
      ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const final;
  int GetCachedSize() const final { return _cached_size_.Get(); }

  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(Point* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pt.Point";
  }
  protected:
  explicit Point(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                       bool is_message_owned = false);
  private:
  static void ArenaDtor(void* object);
  inline void RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena* arena);
  public:

  static const ClassData _class_data_;
  const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*GetClassData() const final;

  ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadata() const final;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kXFieldNumber = 1,
    kYFieldNumber = 2,
    kZFieldNumber = 3,
  };
  // required double x = 1;
  bool has_x() const;
  private:
  bool _internal_has_x() const;
  public:
  void clear_x();
  double x() const;
  void set_x(double value);
  private:
  double _internal_x() const;
  void _internal_set_x(double value);
  public:

  // required double y = 2;
  bool has_y() const;
  private:
  bool _internal_has_y() const;
  public:
  void clear_y();
  double y() const;
  void set_y(double value);
  private:
  double _internal_y() const;
  void _internal_set_y(double value);
  public:

  // required double z = 3;
  bool has_z() const;
  private:
  bool _internal_has_z() const;
  public:
  void clear_z();
  double z() const;
  void set_z(double value);
  private:
  double _internal_z() const;
  void _internal_set_z(double value);
  public:

  // @@protoc_insertion_point(class_scope:pt.Point)
 private:
  class _Internal;

  // helper for ByteSizeLong()
  size_t RequiredFieldsByteSizeFallback() const;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  double x_;
  double y_;
  double z_;
  friend struct ::TableStruct_my_5fprotobuf_5ftype_2eproto;
};
// -------------------------------------------------------------------

class Points final :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pt.Points) */ {
 public:
  inline Points() : Points(nullptr) {}
  ~Points() override;
  explicit constexpr Points(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized);

  Points(const Points& from);
  Points(Points&& from) noexcept
    : Points() {
    *this = ::std::move(from);
  }

  inline Points& operator=(const Points& from) {
    CopyFrom(from);
    return *this;
  }
  inline Points& operator=(Points&& from) noexcept {
    if (this == &from) return *this;
    if (GetOwningArena() == from.GetOwningArena()) {
      InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }

  inline const ::PROTOBUF_NAMESPACE_ID::UnknownFieldSet& unknown_fields() const {
    return _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance);
  }
  inline ::PROTOBUF_NAMESPACE_ID::UnknownFieldSet* mutable_unknown_fields() {
    return _internal_metadata_.mutable_unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
  }

  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* descriptor() {
    return GetDescriptor();
  }
  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* GetDescriptor() {
    return default_instance().GetMetadata().descriptor;
  }
  static const ::PROTOBUF_NAMESPACE_ID::Reflection* GetReflection() {
    return default_instance().GetMetadata().reflection;
  }
  static const Points& default_instance() {
    return *internal_default_instance();
  }
  static inline const Points* internal_default_instance() {
    return reinterpret_cast<const Points*>(
               &_Points_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(Points& a, Points& b) {
    a.Swap(&b);
  }
  inline void Swap(Points* other) {
    if (other == this) return;
    if (GetOwningArena() == other->GetOwningArena()) {
      InternalSwap(other);
    } else {
      ::PROTOBUF_NAMESPACE_ID::internal::GenericSwap(this, other);
    }
  }
  void UnsafeArenaSwap(Points* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetOwningArena() == other->GetOwningArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline Points* New() const final {
    return new Points();
  }

  Points* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<Points>(arena);
  }
  using ::PROTOBUF_NAMESPACE_ID::Message::CopyFrom;
  void CopyFrom(const Points& from);
  using ::PROTOBUF_NAMESPACE_ID::Message::MergeFrom;
  void MergeFrom(const Points& from);
  private:
  static void MergeImpl(::PROTOBUF_NAMESPACE_ID::Message*to, const ::PROTOBUF_NAMESPACE_ID::Message&from);
  public:
  PROTOBUF_ATTRIBUTE_REINITIALIZES void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  const char* _InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) final;
  ::PROTOBUF_NAMESPACE_ID::uint8* _InternalSerialize(
      ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const final;
  int GetCachedSize() const final { return _cached_size_.Get(); }

  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(Points* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pt.Points";
  }
  protected:
  explicit Points(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                       bool is_message_owned = false);
  private:
  static void ArenaDtor(void* object);
  inline void RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena* arena);
  public:

  static const ClassData _class_data_;
  const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*GetClassData() const final;

  ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadata() const final;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDataFieldNumber = 1,
  };
  // repeated .pt.Point data = 1;
  int data_size() const;
  private:
  int _internal_data_size() const;
  public:
  void clear_data();
  ::pt::Point* mutable_data(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pt::Point >*
      mutable_data();
  private:
  const ::pt::Point& _internal_data(int index) const;
  ::pt::Point* _internal_add_data();
  public:
  const ::pt::Point& data(int index) const;
  ::pt::Point* add_data();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pt::Point >&
      data() const;

  // @@protoc_insertion_point(class_scope:pt.Points)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pt::Point > data_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  friend struct ::TableStruct_my_5fprotobuf_5ftype_2eproto;
};
// -------------------------------------------------------------------

class Point2 final :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pt.Point2) */ {
 public:
  inline Point2() : Point2(nullptr) {}
  ~Point2() override;
  explicit constexpr Point2(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized);

  Point2(const Point2& from);
  Point2(Point2&& from) noexcept
    : Point2() {
    *this = ::std::move(from);
  }

  inline Point2& operator=(const Point2& from) {
    CopyFrom(from);
    return *this;
  }
  inline Point2& operator=(Point2&& from) noexcept {
    if (this == &from) return *this;
    if (GetOwningArena() == from.GetOwningArena()) {
      InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }

  inline const ::PROTOBUF_NAMESPACE_ID::UnknownFieldSet& unknown_fields() const {
    return _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance);
  }
  inline ::PROTOBUF_NAMESPACE_ID::UnknownFieldSet* mutable_unknown_fields() {
    return _internal_metadata_.mutable_unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
  }

  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* descriptor() {
    return GetDescriptor();
  }
  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* GetDescriptor() {
    return default_instance().GetMetadata().descriptor;
  }
  static const ::PROTOBUF_NAMESPACE_ID::Reflection* GetReflection() {
    return default_instance().GetMetadata().reflection;
  }
  static const Point2& default_instance() {
    return *internal_default_instance();
  }
  static inline const Point2* internal_default_instance() {
    return reinterpret_cast<const Point2*>(
               &_Point2_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    2;

  friend void swap(Point2& a, Point2& b) {
    a.Swap(&b);
  }
  inline void Swap(Point2* other) {
    if (other == this) return;
    if (GetOwningArena() == other->GetOwningArena()) {
      InternalSwap(other);
    } else {
      ::PROTOBUF_NAMESPACE_ID::internal::GenericSwap(this, other);
    }
  }
  void UnsafeArenaSwap(Point2* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetOwningArena() == other->GetOwningArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline Point2* New() const final {
    return new Point2();
  }

  Point2* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<Point2>(arena);
  }
  using ::PROTOBUF_NAMESPACE_ID::Message::CopyFrom;
  void CopyFrom(const Point2& from);
  using ::PROTOBUF_NAMESPACE_ID::Message::MergeFrom;
  void MergeFrom(const Point2& from);
  private:
  static void MergeImpl(::PROTOBUF_NAMESPACE_ID::Message*to, const ::PROTOBUF_NAMESPACE_ID::Message&from);
  public:
  PROTOBUF_ATTRIBUTE_REINITIALIZES void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  const char* _InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) final;
  ::PROTOBUF_NAMESPACE_ID::uint8* _InternalSerialize(
      ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const final;
  int GetCachedSize() const final { return _cached_size_.Get(); }

  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(Point2* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pt.Point2";
  }
  protected:
  explicit Point2(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                       bool is_message_owned = false);
  private:
  static void ArenaDtor(void* object);
  inline void RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena* arena);
  public:

  static const ClassData _class_data_;
  const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*GetClassData() const final;

  ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadata() const final;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kXFieldNumber = 1,
  };
  // repeated string x = 1;
  int x_size() const;
  private:
  int _internal_x_size() const;
  public:
  void clear_x();
  const std::string& x(int index) const;
  std::string* mutable_x(int index);
  void set_x(int index, const std::string& value);
  void set_x(int index, std::string&& value);
  void set_x(int index, const char* value);
  void set_x(int index, const char* value, size_t size);
  std::string* add_x();
  void add_x(const std::string& value);
  void add_x(std::string&& value);
  void add_x(const char* value);
  void add_x(const char* value, size_t size);
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField<std::string>& x() const;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField<std::string>* mutable_x();
  private:
  const std::string& _internal_x(int index) const;
  std::string* _internal_add_x();
  public:

  // @@protoc_insertion_point(class_scope:pt.Point2)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField<std::string> x_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  friend struct ::TableStruct_my_5fprotobuf_5ftype_2eproto;
};
// -------------------------------------------------------------------

class Points2 final :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pt.Points2) */ {
 public:
  inline Points2() : Points2(nullptr) {}
  ~Points2() override;
  explicit constexpr Points2(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized);

  Points2(const Points2& from);
  Points2(Points2&& from) noexcept
    : Points2() {
    *this = ::std::move(from);
  }

  inline Points2& operator=(const Points2& from) {
    CopyFrom(from);
    return *this;
  }
  inline Points2& operator=(Points2&& from) noexcept {
    if (this == &from) return *this;
    if (GetOwningArena() == from.GetOwningArena()) {
      InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }

  inline const ::PROTOBUF_NAMESPACE_ID::UnknownFieldSet& unknown_fields() const {
    return _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance);
  }
  inline ::PROTOBUF_NAMESPACE_ID::UnknownFieldSet* mutable_unknown_fields() {
    return _internal_metadata_.mutable_unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
  }

  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* descriptor() {
    return GetDescriptor();
  }
  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* GetDescriptor() {
    return default_instance().GetMetadata().descriptor;
  }
  static const ::PROTOBUF_NAMESPACE_ID::Reflection* GetReflection() {
    return default_instance().GetMetadata().reflection;
  }
  static const Points2& default_instance() {
    return *internal_default_instance();
  }
  static inline const Points2* internal_default_instance() {
    return reinterpret_cast<const Points2*>(
               &_Points2_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    3;

  friend void swap(Points2& a, Points2& b) {
    a.Swap(&b);
  }
  inline void Swap(Points2* other) {
    if (other == this) return;
    if (GetOwningArena() == other->GetOwningArena()) {
      InternalSwap(other);
    } else {
      ::PROTOBUF_NAMESPACE_ID::internal::GenericSwap(this, other);
    }
  }
  void UnsafeArenaSwap(Points2* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetOwningArena() == other->GetOwningArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline Points2* New() const final {
    return new Points2();
  }

  Points2* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<Points2>(arena);
  }
  using ::PROTOBUF_NAMESPACE_ID::Message::CopyFrom;
  void CopyFrom(const Points2& from);
  using ::PROTOBUF_NAMESPACE_ID::Message::MergeFrom;
  void MergeFrom(const Points2& from);
  private:
  static void MergeImpl(::PROTOBUF_NAMESPACE_ID::Message*to, const ::PROTOBUF_NAMESPACE_ID::Message&from);
  public:
  PROTOBUF_ATTRIBUTE_REINITIALIZES void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  const char* _InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) final;
  ::PROTOBUF_NAMESPACE_ID::uint8* _InternalSerialize(
      ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const final;
  int GetCachedSize() const final { return _cached_size_.Get(); }

  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(Points2* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pt.Points2";
  }
  protected:
  explicit Points2(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                       bool is_message_owned = false);
  private:
  static void ArenaDtor(void* object);
  inline void RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena* arena);
  public:

  static const ClassData _class_data_;
  const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*GetClassData() const final;

  ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadata() const final;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDataFieldNumber = 1,
  };
  // repeated .pt.Point2 data = 1;
  int data_size() const;
  private:
  int _internal_data_size() const;
  public:
  void clear_data();
  ::pt::Point2* mutable_data(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pt::Point2 >*
      mutable_data();
  private:
  const ::pt::Point2& _internal_data(int index) const;
  ::pt::Point2* _internal_add_data();
  public:
  const ::pt::Point2& data(int index) const;
  ::pt::Point2* add_data();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pt::Point2 >&
      data() const;

  // @@protoc_insertion_point(class_scope:pt.Points2)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pt::Point2 > data_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  friend struct ::TableStruct_my_5fprotobuf_5ftype_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// Point

// required double x = 1;
inline bool Point::_internal_has_x() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  return value;
}
inline bool Point::has_x() const {
  return _internal_has_x();
}
inline void Point::clear_x() {
  x_ = 0;
  _has_bits_[0] &= ~0x00000001u;
}
inline double Point::_internal_x() const {
  return x_;
}
inline double Point::x() const {
  // @@protoc_insertion_point(field_get:pt.Point.x)
  return _internal_x();
}
inline void Point::_internal_set_x(double value) {
  _has_bits_[0] |= 0x00000001u;
  x_ = value;
}
inline void Point::set_x(double value) {
  _internal_set_x(value);
  // @@protoc_insertion_point(field_set:pt.Point.x)
}

// required double y = 2;
inline bool Point::_internal_has_y() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  return value;
}
inline bool Point::has_y() const {
  return _internal_has_y();
}
inline void Point::clear_y() {
  y_ = 0;
  _has_bits_[0] &= ~0x00000002u;
}
inline double Point::_internal_y() const {
  return y_;
}
inline double Point::y() const {
  // @@protoc_insertion_point(field_get:pt.Point.y)
  return _internal_y();
}
inline void Point::_internal_set_y(double value) {
  _has_bits_[0] |= 0x00000002u;
  y_ = value;
}
inline void Point::set_y(double value) {
  _internal_set_y(value);
  // @@protoc_insertion_point(field_set:pt.Point.y)
}

// required double z = 3;
inline bool Point::_internal_has_z() const {
  bool value = (_has_bits_[0] & 0x00000004u) != 0;
  return value;
}
inline bool Point::has_z() const {
  return _internal_has_z();
}
inline void Point::clear_z() {
  z_ = 0;
  _has_bits_[0] &= ~0x00000004u;
}
inline double Point::_internal_z() const {
  return z_;
}
inline double Point::z() const {
  // @@protoc_insertion_point(field_get:pt.Point.z)
  return _internal_z();
}
inline void Point::_internal_set_z(double value) {
  _has_bits_[0] |= 0x00000004u;
  z_ = value;
}
inline void Point::set_z(double value) {
  _internal_set_z(value);
  // @@protoc_insertion_point(field_set:pt.Point.z)
}

// -------------------------------------------------------------------

// Points

// repeated .pt.Point data = 1;
inline int Points::_internal_data_size() const {
  return data_.size();
}
inline int Points::data_size() const {
  return _internal_data_size();
}
inline void Points::clear_data() {
  data_.Clear();
}
inline ::pt::Point* Points::mutable_data(int index) {
  // @@protoc_insertion_point(field_mutable:pt.Points.data)
  return data_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pt::Point >*
Points::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:pt.Points.data)
  return &data_;
}
inline const ::pt::Point& Points::_internal_data(int index) const {
  return data_.Get(index);
}
inline const ::pt::Point& Points::data(int index) const {
  // @@protoc_insertion_point(field_get:pt.Points.data)
  return _internal_data(index);
}
inline ::pt::Point* Points::_internal_add_data() {
  return data_.Add();
}
inline ::pt::Point* Points::add_data() {
  ::pt::Point* _add = _internal_add_data();
  // @@protoc_insertion_point(field_add:pt.Points.data)
  return _add;
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pt::Point >&
Points::data() const {
  // @@protoc_insertion_point(field_list:pt.Points.data)
  return data_;
}

// -------------------------------------------------------------------

// Point2

// repeated string x = 1;
inline int Point2::_internal_x_size() const {
  return x_.size();
}
inline int Point2::x_size() const {
  return _internal_x_size();
}
inline void Point2::clear_x() {
  x_.Clear();
}
inline std::string* Point2::add_x() {
  std::string* _s = _internal_add_x();
  // @@protoc_insertion_point(field_add_mutable:pt.Point2.x)
  return _s;
}
inline const std::string& Point2::_internal_x(int index) const {
  return x_.Get(index);
}
inline const std::string& Point2::x(int index) const {
  // @@protoc_insertion_point(field_get:pt.Point2.x)
  return _internal_x(index);
}
inline std::string* Point2::mutable_x(int index) {
  // @@protoc_insertion_point(field_mutable:pt.Point2.x)
  return x_.Mutable(index);
}
inline void Point2::set_x(int index, const std::string& value) {
  x_.Mutable(index)->assign(value);
  // @@protoc_insertion_point(field_set:pt.Point2.x)
}
inline void Point2::set_x(int index, std::string&& value) {
  x_.Mutable(index)->assign(std::move(value));
  // @@protoc_insertion_point(field_set:pt.Point2.x)
}
inline void Point2::set_x(int index, const char* value) {
  GOOGLE_DCHECK(value != nullptr);
  x_.Mutable(index)->assign(value);
  // @@protoc_insertion_point(field_set_char:pt.Point2.x)
}
inline void Point2::set_x(int index, const char* value, size_t size) {
  x_.Mutable(index)->assign(
    reinterpret_cast<const char*>(value), size);
  // @@protoc_insertion_point(field_set_pointer:pt.Point2.x)
}
inline std::string* Point2::_internal_add_x() {
  return x_.Add();
}
inline void Point2::add_x(const std::string& value) {
  x_.Add()->assign(value);
  // @@protoc_insertion_point(field_add:pt.Point2.x)
}
inline void Point2::add_x(std::string&& value) {
  x_.Add(std::move(value));
  // @@protoc_insertion_point(field_add:pt.Point2.x)
}
inline void Point2::add_x(const char* value) {
  GOOGLE_DCHECK(value != nullptr);
  x_.Add()->assign(value);
  // @@protoc_insertion_point(field_add_char:pt.Point2.x)
}
inline void Point2::add_x(const char* value, size_t size) {
  x_.Add()->assign(reinterpret_cast<const char*>(value), size);
  // @@protoc_insertion_point(field_add_pointer:pt.Point2.x)
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField<std::string>&
Point2::x() const {
  // @@protoc_insertion_point(field_list:pt.Point2.x)
  return x_;
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField<std::string>*
Point2::mutable_x() {
  // @@protoc_insertion_point(field_mutable_list:pt.Point2.x)
  return &x_;
}

// -------------------------------------------------------------------

// Points2

// repeated .pt.Point2 data = 1;
inline int Points2::_internal_data_size() const {
  return data_.size();
}
inline int Points2::data_size() const {
  return _internal_data_size();
}
inline void Points2::clear_data() {
  data_.Clear();
}
inline ::pt::Point2* Points2::mutable_data(int index) {
  // @@protoc_insertion_point(field_mutable:pt.Points2.data)
  return data_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pt::Point2 >*
Points2::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:pt.Points2.data)
  return &data_;
}
inline const ::pt::Point2& Points2::_internal_data(int index) const {
  return data_.Get(index);
}
inline const ::pt::Point2& Points2::data(int index) const {
  // @@protoc_insertion_point(field_get:pt.Points2.data)
  return _internal_data(index);
}
inline ::pt::Point2* Points2::_internal_add_data() {
  return data_.Add();
}
inline ::pt::Point2* Points2::add_data() {
  ::pt::Point2* _add = _internal_add_data();
  // @@protoc_insertion_point(field_add:pt.Points2.data)
  return _add;
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pt::Point2 >&
Points2::data() const {
  // @@protoc_insertion_point(field_list:pt.Points2.data)
  return data_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------

// -------------------------------------------------------------------

// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace pt

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_my_5fprotobuf_5ftype_2eproto