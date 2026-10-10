#include "still_asset.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <limits>
#include <sstream>
#include <stdexcept>

#include <opencv2/imgcodecs.hpp>
#include <yaml-cpp/yaml.h>

namespace xgc2_camera_driver {
namespace {

constexpr const char* kIntrinsicSchema = "xgc2.camera.intrinsic.v1";
constexpr const char* kExtrinsicSchema = "xgc2.camera.extrinsic.v1";
constexpr const char* kFrameConvention = "parent_T_camera_optical";
constexpr const char* kOpticalSuffix = "_optical_frame";
constexpr const char* kLinkSuffix = "_link";
constexpr std::size_t kMaximumImageBytes = 256U << 20U;

[[noreturn]] void fail(const std::string& message)
{
  throw std::invalid_argument(message);
}

std::string baseName(const std::string& path)
{
  const std::size_t separator = path.find_last_of('/');
  return separator == std::string::npos ? path : path.substr(separator + 1U);
}

YAML::Node loadDocument(const std::string& path)
{
  YAML::Node document;
  try {
    document = YAML::LoadFile(path);
  } catch (const YAML::BadFile&) {
    fail("cannot read " + path);
  } catch (const YAML::Exception& error) {
    fail("invalid YAML in " + path + ": " + error.what());
  }
  if (!document.IsMap()) {
    fail(path + " must be a YAML mapping");
  }
  return document;
}

double requireNumber(const YAML::Node& node, const std::string& label)
{
  if (!node || !node.IsScalar()) {
    fail(label + " must be a number");
  }
  const std::string text = node.Scalar();
  char* end = nullptr;
  const double value = std::strtod(text.c_str(), &end);
  if (text.empty() || end == nullptr || *end != '\0' || text == "true" ||
      text == "false") {
    fail(label + " must be a number");
  }
  if (!std::isfinite(value)) {
    fail(label + " must be finite");
  }
  return value;
}

std::uint32_t requirePositiveInteger(const YAML::Node& node, const std::string& label)
{
  if (!node || !node.IsScalar()) {
    fail(label + " must be a positive integer");
  }
  const std::string text = node.Scalar();
  if (text.empty() || text.size() > 9U ||
      !std::all_of(text.begin(), text.end(), [](unsigned char c) {
        return std::isdigit(c) != 0;
      })) {
    fail(label + " must be a positive integer");
  }
  const unsigned long value = std::strtoul(text.c_str(), nullptr, 10);
  if (value == 0UL) {
    fail(label + " must be a positive integer");
  }
  return static_cast<std::uint32_t>(value);
}

std::string requireText(const YAML::Node& node, const std::string& label)
{
  if (!node || !node.IsScalar()) {
    fail(label + " must be a non-empty string");
  }
  std::string text = node.Scalar();
  const auto first = text.find_first_not_of(" \t\r\n");
  const auto last = text.find_last_not_of(" \t\r\n");
  if (first == std::string::npos) {
    fail(label + " must be a non-empty string");
  }
  return text.substr(first, last - first + 1U);
}

std::vector<double> requireMatrix(
    const YAML::Node& document,
    const char* key,
    const std::string& label,
    const std::size_t expected_rows,
    const std::size_t expected_cols)
{
  const YAML::Node matrix = document[key];
  if (!matrix || !matrix.IsMap()) {
    fail(label + " " + key + " must be a mapping");
  }
  const YAML::Node rows = matrix["rows"];
  const YAML::Node cols = matrix["cols"];
  const YAML::Node data = matrix["data"];
  if (!rows || !rows.IsScalar() || rows.Scalar() != std::to_string(expected_rows)) {
    fail(label + " " + key + ".rows must be " + std::to_string(expected_rows));
  }
  if (!data || !data.IsSequence() || data.size() == 0U) {
    fail(label + " " + key + ".data must be a non-empty list");
  }
  if (expected_cols != 0U) {
    if (!cols || !cols.IsScalar() || cols.Scalar() != std::to_string(expected_cols)) {
      fail(label + " " + key + ".cols must be " + std::to_string(expected_cols));
    }
    if (data.size() != expected_rows * expected_cols) {
      fail(
          label + " " + key + ".data must hold " +
          std::to_string(expected_rows * expected_cols) + " values");
    }
  } else if (!cols || !cols.IsScalar() || cols.Scalar() != std::to_string(data.size())) {
    fail(label + " " + key + ".cols must match data length");
  }
  std::vector<double> values;
  values.reserve(data.size());
  for (const auto& item : data) {
    values.push_back(requireNumber(item, label + " " + key + ".data"));
  }
  return values;
}

bool endsWith(const std::string& text, const std::string& suffix)
{
  return text.size() >= suffix.size() &&
         text.compare(text.size() - suffix.size(), suffix.size(), suffix) == 0;
}

std::string lowerExtension(const std::string& path)
{
  const std::string name = baseName(path);
  const std::size_t dot = name.find_last_of('.');
  if (dot == std::string::npos) {
    return "";
  }
  std::string extension = name.substr(dot);
  std::transform(extension.begin(), extension.end(), extension.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return extension;
}

std::vector<std::uint8_t> readFile(const std::string& path)
{
  std::ifstream stream(path, std::ios::binary);
  if (!stream) {
    fail("cannot read " + path);
  }
  std::vector<std::uint8_t> bytes(
      (std::istreambuf_iterator<char>(stream)), std::istreambuf_iterator<char>());
  if (bytes.empty() || bytes.size() > kMaximumImageBytes) {
    fail(path + " must hold between 1 byte and 256 MiB");
  }
  return bytes;
}

}  // namespace

const std::array<double, 4> kStandardLinkToOpticalQuaternion{
    {-0.5, 0.5, -0.5, 0.5}};

Quaternion quaternionMultiply(const Quaternion& a, const Quaternion& b)
{
  return Quaternion{{
      a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1],
      a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
      a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3],
      a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2],
  }};
}

Quaternion quaternionNormalize(const Quaternion& value)
{
  const double norm = std::sqrt(
      value[0] * value[0] + value[1] * value[1] + value[2] * value[2] +
      value[3] * value[3]);
  if (!(norm > 0.0) || !std::isfinite(norm)) {
    fail("quaternion has no usable norm");
  }
  return Quaternion{{value[0] / norm, value[1] / norm, value[2] / norm, value[3] / norm}};
}

Quaternion quaternionInverse(const Quaternion& value)
{
  const Quaternion unit = quaternionNormalize(value);
  return Quaternion{{-unit[0], -unit[1], -unit[2], unit[3]}};
}

std::string deriveLinkFrame(const std::string& optical_frame)
{
  const std::string suffix(kOpticalSuffix);
  if (!endsWith(optical_frame, suffix) || optical_frame.size() == suffix.size()) {
    fail(
        "extrinsic child_frame must end with " + suffix + ": '" + optical_frame +
        "'");
  }
  return optical_frame.substr(0, optical_frame.size() - suffix.size()) + kLinkSuffix;
}

CameraStaticTransforms composeStaticCameraTransforms(
    const Extrinsic& extrinsic, const Quaternion& link_to_optical)
{
  // The extrinsic stores the composed parent->optical transform while the
  // physical contract publishes parent->link and link->optical separately, with
  // a rotation-only link->optical. T_parent_link = T_parent_optical *
  // inverse(T_link_optical), so the translation passes through unchanged.
  // Publishing both children under the parent with a zero optical translation
  // would leave the CameraInfo frame at the origin and AR overlays could not
  // land on the historical pixels.
  const std::string link_frame = deriveLinkFrame(extrinsic.child_frame);
  const Quaternion optical = quaternionNormalize(link_to_optical);
  CameraStaticTransforms result;
  result.parent_to_link.parent_frame = extrinsic.parent_frame;
  result.parent_to_link.child_frame = link_frame;
  result.parent_to_link.translation = extrinsic.translation;
  result.parent_to_link.quaternion = quaternionNormalize(quaternionMultiply(
      quaternionNormalize(extrinsic.quaternion), quaternionInverse(optical)));
  result.link_to_optical.parent_frame = link_frame;
  result.link_to_optical.child_frame = extrinsic.child_frame;
  result.link_to_optical.translation = {{0.0, 0.0, 0.0}};
  result.link_to_optical.quaternion = optical;
  return result;
}

Intrinsics loadIntrinsics(const std::string& path)
{
  const YAML::Node document = loadDocument(path);
  const std::string label = baseName(path);
  const YAML::Node schema = document["schema"];
  if (!schema || !schema.IsScalar() || schema.Scalar() != kIntrinsicSchema) {
    fail(label + " schema must be " + kIntrinsicSchema);
  }
  Intrinsics result;
  result.width = requirePositiveInteger(document["image_width"], label + " image_width");
  result.height = requirePositiveInteger(document["image_height"], label + " image_height");
  const std::vector<double> matrix = requireMatrix(document, "camera_matrix", label, 3U, 3U);
  std::copy(matrix.begin(), matrix.end(), result.camera_matrix.begin());
  const YAML::Node model = document["distortion_model"];
  if (!model || !model.IsScalar() || model.Scalar().empty()) {
    fail(label + " distortion_model must be a non-empty string");
  }
  result.distortion_model = model.Scalar();
  result.distortion = requireMatrix(document, "distortion_coefficients", label, 1U, 0U);
  return result;
}

Extrinsic loadExtrinsic(const std::string& path)
{
  const YAML::Node document = loadDocument(path);
  const std::string label = baseName(path);
  const YAML::Node schema = document["schema"];
  if (!schema || !schema.IsScalar() || schema.Scalar() != kExtrinsicSchema) {
    fail(label + " schema must be " + kExtrinsicSchema);
  }
  const YAML::Node convention = document["frame_convention"];
  if (!convention || !convention.IsScalar() || convention.Scalar() != kFrameConvention) {
    fail(label + " frame_convention must be " + kFrameConvention);
  }
  Extrinsic result;
  result.parent_frame = requireText(document["parent_frame"], label + " parent_frame");
  result.child_frame = requireText(document["child_frame"], label + " child_frame");
  const YAML::Node translation = document["translation"];
  if (!translation || !translation.IsMap()) {
    fail(label + " translation must be a mapping");
  }
  const char* const axes[] = {"x", "y", "z"};
  for (std::size_t index = 0; index < 3U; ++index) {
    result.translation[index] =
        requireNumber(translation[axes[index]], label + " translation." + axes[index]);
  }
  const YAML::Node rotation = document["quaternion_xyzw"];
  if (!rotation || !rotation.IsMap()) {
    fail(label + " quaternion_xyzw must be a mapping");
  }
  const char* const components[] = {"x", "y", "z", "w"};
  Quaternion raw{};
  for (std::size_t index = 0; index < 4U; ++index) {
    raw[index] = requireNumber(
        rotation[components[index]], label + " quaternion_xyzw." + components[index]);
  }
  try {
    result.quaternion = quaternionNormalize(raw);
  } catch (const std::invalid_argument&) {
    fail(label + " quaternion_xyzw has no usable norm");
  }
  return result;
}

StillImage loadStillImage(
    const std::string& path, const Intrinsics& intrinsics, const int jpeg_quality)
{
  if (jpeg_quality < 1 || jpeg_quality > 100) {
    fail("still_jpeg_quality must be in [1, 100]");
  }
  const std::string extension = lowerExtension(path);
  const bool is_jpeg = extension == ".jpg" || extension == ".jpeg";
  if (!is_jpeg && extension != ".png") {
    fail("unsupported still image extension '" + extension + "' (expected .png, .jpg or .jpeg): " + path);
  }
  std::vector<std::uint8_t> bytes = readFile(path);
  const cv::Mat decoded = cv::imdecode(bytes, cv::IMREAD_COLOR);
  if (decoded.empty()) {
    fail("cannot decode still image: " + path);
  }
  StillImage result;
  result.width = static_cast<std::uint32_t>(decoded.cols);
  result.height = static_cast<std::uint32_t>(decoded.rows);
  if (result.width != intrinsics.width || result.height != intrinsics.height) {
    std::ostringstream message;
    message << "still image " << result.width << "x" << result.height
            << " does not match the intrinsics " << intrinsics.width << "x"
            << intrinsics.height << "; use the intrinsics recorded with this image";
    fail(message.str());
  }
  if (is_jpeg) {
    if (bytes.size() < 4U || bytes[0] != 0xffU || bytes[1] != 0xd8U) {
      fail("still image is not a JPEG bitstream: " + path);
    }
    result.jpeg = std::move(bytes);
  } else if (!cv::imencode(
                 ".jpg", decoded, result.jpeg, {cv::IMWRITE_JPEG_QUALITY, jpeg_quality})) {
    fail("JPEG encoding failed for still image: " + path);
  }
  return result;
}

}  // namespace xgc2_camera_driver
