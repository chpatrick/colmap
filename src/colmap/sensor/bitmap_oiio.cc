// Copyright (c), ETH Zurich and UNC Chapel Hill.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//
//     * Neither the name of ETH Zurich and UNC Chapel Hill nor the names of
//       its contributors may be used to endorse or promote products derived
//       from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "colmap/math/math.h"
#include "colmap/sensor/bitmap.h"
#include "colmap/sensor/database.h"
#include "colmap/util/file.h"
#include "colmap/util/logging.h"
#include "colmap/util/misc.h"

#include "thirdparty/VLFeat/imopv.h"

#include <array>
#include <cstdint>
#include <cstring>
#include <memory>
#include <regex>
#include <stdexcept>
#include <unordered_map>
#include <vector>

#include <OpenImageIO/imagebuf.h>
#include <OpenImageIO/imageio.h>
#include <OpenImageIO/imagebufalgo.h>
#include <OpenImageIO/typedesc.h>
#include <colmap/sensor/bitmap_oiio.h>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>
#endif

namespace colmap {
namespace {

// bool ReadExifTag(FIBITMAP* ptr,
//                  const FREE_IMAGE_MDMODEL model,
//                  const std::string& tag_name,
//                  std::string* result) {
//   FITAG* tag = nullptr;
//   FreeImage_GetMetadata(model, ptr, tag_name.c_str(), &tag);
//   if (tag == nullptr) {
//     *result = "";
//     return false;
//   } else {
//     if (tag_name == "FocalPlaneXResolution") {
//       // This tag seems to be in the wrong category.
//       *result = std::string(FreeImage_TagToString(FIMD_EXIF_INTEROP, tag));
//     } else {
//       *result = FreeImage_TagToString(model, tag);
//     }
//     return true;
//   }
// }

// bool IsPtrGrey(FIBITMAP* ptr) {
//   return FreeImage_GetColorType(ptr) == FIC_MINISBLACK &&
//          FreeImage_GetBPP(ptr) == 8;
// }

// bool IsPtrRGB(FIBITMAP* ptr) {
//   return FreeImage_GetColorType(ptr) == FIC_RGB && FreeImage_GetBPP(ptr) == 24;
// }

// bool IsPtrSupported(FIBITMAP* ptr) { return IsPtrGrey(ptr) || IsPtrRGB(ptr); }

}  // namespace

BitmapOIIO::BitmapOIIO() {}

// BitmapOIIO::BitmapOIIO(const BitmapOIIO& other) : BitmapOIIO() {
//   if (other.handle_.ptr != nullptr) {
//     SetPtr(FreeImage_Clone(other.handle_.ptr));
//   }
// }


BitmapOIIO::BitmapOIIO(const OIIO::ImageBuf& buffer) : BitmapOIIO() {
  if (buffer.spec().format != OIIO::TypeUInt8) {
    throw std::runtime_error("Only uint8 bitmaps are supported.");
  }
  if (buffer.nchannels() != 1 && buffer.nchannels() != 3) {
    throw std::runtime_error("Only 1 and 3-channel images are supported.");
  }
  buf_ = buffer;
}

// BitmapOIIO& BitmapOIIO::operator=(const BitmapOIIO& other) {
//   if (other.handle_.ptr != nullptr) {
//     SetPtr(FreeImage_Clone(other.handle_.ptr));
//   }
//   return *this;
// }


bool BitmapOIIO::Allocate(const int width,
                          const int height,
                          const bool as_rgb) {
  OIIO::ImageSpec spec(width, height, as_rgb ? 3 : 1, OIIO::TypeUInt8);
  buf_.reset(spec, OIIO::InitializePixels::Yes);

  return true;
}

void BitmapOIIO::Deallocate() {
  buf_.reset();
}

size_t BitmapOIIO::NumBytes() const {
  return buf_.spec().height * buf_.scanline_stride();
}

unsigned int BitmapOIIO::BitsPerPixel() const {
  return buf_.spec().pixel_bytes() * 8;
}

unsigned int BitmapOIIO::Pitch() const {
  return buf_.scanline_stride();
}

std::vector<uint8_t> BitmapOIIO::ConvertToRowMajorArray() const {
  std::vector<uint8_t> pixels(buf_.spec().image_pixels() * buf_.spec().nchannels);
  buf_.get_pixels(OIIO::ROI::All(), OIIO::span<uint8_t>(pixels));
  return pixels;
}

std::vector<uint8_t> BitmapOIIO::ConvertToColMajorArray() const {
  std::vector<uint8_t> pixels(buf_.spec().image_pixels() * buf_.nchannels());
  for (OIIO::ImageBuf::ConstIterator<uint8_t, uint8_t> it(buf_); !it.done(); it++) {
    for (int channel = 0; channel < buf_.nchannels(); channel++) {
      pixels[channel * Width() * Height() + it.x() * Height() + it.y()] = it[channel];
    }
  }
  return pixels;
}

std::vector<uint8_t> BitmapOIIO::ConvertToRawBits() const {
  std::vector<uint8_t> raw_bits(NumBytes());
  buf_.get_pixels(
    OIIO::ROI::All(),
    buf_.spec().format,
    OIIO::span<std::byte>(reinterpret_cast<std::byte*>(raw_bits.data()), raw_bits.size()),
    nullptr,
    OIIO::AutoStride,
    buf_.scanline_stride()
  );
  return raw_bits;
}

BitmapOIIO BitmapOIIO::ConvertFromRawBits(const uint8_t* data, int pitch, int width, int height, bool rgb) {
  OIIO::ImageBuf buf(OIIO::ImageSpec(width, height, rgb ? 3 : 1, OIIO::TypeUInt8));
  buf.set_pixels(
    OIIO::ROI::All(),
    OIIO::TypeUInt8,
    OIIO::span<const std::byte>(reinterpret_cast<const std::byte*>(data), pitch * height),
    nullptr,
    OIIO::AutoStride,
    pitch
  );
  return BitmapOIIO(buf);
}

bool BitmapOIIO::GetPixel(const int x,
                          const int y,
                          BitmapColor<uint8_t>* color) const {
  if (x < 0 || x >= Width() || y < 0 || y >= Height()) {
    return false;
  }

  OIIO::ImageBuf::ConstIterator<uint8_t, uint8_t> iter(buf_, x, y);

  if (IsGrey()) {
    color->r = iter[0];
    return true;
  } else if (IsRGB()) {
    color->b = iter[0];
    color->g = iter[1];
    color->r = iter[2];
    return true;
  }

  return false;
}

bool BitmapOIIO::SetPixel(const int x,
                          const int y,
                          const BitmapColor<uint8_t>& color) {
  if (x < 0 || x >= Width() || y < 0 || y >= Height()) {
    return false;
  }

  OIIO::ImageBuf::Iterator<uint8_t, uint8_t> iter(buf_, x, y);

  if (IsGrey()) {
    iter[0] = color.r;
    return true;
  } else if (IsRGB()) {
    iter[0] = color.b;
    iter[1] = color.g;
    iter[2] = color.r;
    return true;
  }

  return false;
}

const uint8_t* BitmapOIIO::GetScanline(const int y) const {
  return reinterpret_cast<const uint8_t*>(buf_.pixeladdr(0, y));
}

void BitmapOIIO::Fill(const BitmapColor<uint8_t>& color) {
  if (IsGrey()) {
    for (OIIO::ImageBuf::Iterator<uint8_t, uint8_t> it(buf_); !it.done(); it++) {
      it[0] = color.r;
    }
  } else if (IsRGB()) {
    for (OIIO::ImageBuf::Iterator<uint8_t, uint8_t> it(buf_); !it.done(); it++) {
      it[0] = color.b;
      it[1] = color.g;
      it[2] = color.r;
    }
  } else {
    throw std::runtime_error("Unexpected depth.");
  }
}

bool BitmapOIIO::InterpolateNearestNeighbor(const double x,
                                            const double y,
                                            BitmapColor<uint8_t>* color) const {
  const int xx = static_cast<int>(std::round(x));
  const int yy = static_cast<int>(std::round(y));
  return GetPixel(xx, yy, color);
}

bool BitmapOIIO::InterpolateBilinear(const double x,
                                     const double y,
                                     BitmapColor<float>* color) const {
  const int x0 = static_cast<int>(std::floor(x));
  const int x1 = x0 + 1;
  const int y0 = static_cast<int>(std::floor(y));
  const int y1 = y0 + 1;

  if (x0 < 0 || x1 >= Width() || y0 < 0 || y1 >= Height()) {
    return false;
  }

  if (IsGrey()) {
    std::array<float, 1> pixel;
    buf_.interppixel(x + 0.5, y + 0.5, pixel, OIIO::ImageBuf::WrapClamp);
    color->r = pixel[0] * 255;
  } else if (IsRGB()) {
    std::array<float, 3> pixel;
    buf_.interppixel(x + 0.5, y + 0.5, pixel, OIIO::ImageBuf::WrapClamp);
    color->b = pixel[0] * 255;
    color->g = pixel[1] * 255;
    color->r = pixel[2] * 255;
  } else {
    throw std::runtime_error("Unexpected depth.");
  }

  return true;
}

bool BitmapOIIO::ExifCameraModel(std::string* camera_model) const {
  throw std::runtime_error("TODO");
  // // Read camera make and model
  // std::string make_str;
  // std::string model_str;
  // std::string focal_length;
  // *camera_model = "";
  // if (ReadExifTag(handle_.ptr, FIMD_EXIF_MAIN, "Make", &make_str)) {
  //   *camera_model += (make_str + "-");
  // } else {
  //   *camera_model = "";
  //   return false;
  // }
  // if (ReadExifTag(handle_.ptr, FIMD_EXIF_MAIN, "Model", &model_str)) {
  //   *camera_model += (model_str + "-");
  // } else {
  //   *camera_model = "";
  //   return false;
  // }
  // if (ReadExifTag(handle_.ptr,
  //                 FIMD_EXIF_EXIF,
  //                 "FocalLengthIn35mmFilm",
  //                 &focal_length) ||
  //     ReadExifTag(handle_.ptr, FIMD_EXIF_EXIF, "FocalLength", &focal_length)) {
  //   *camera_model += (focal_length + "-");
  // } else {
  //   *camera_model = "";
  //   return false;
  // }
  // *camera_model += (std::to_string(width_) + "x" + std::to_string(height_));
  // return true;
}

bool BitmapOIIO::ExifFocalLength(double* focal_length) const {
  throw std::runtime_error("TODO");
  // const double max_size = std::max(width_, height_);

  // //////////////////////////////////////////////////////////////////////////////
  // // Focal length in 35mm equivalent
  // //////////////////////////////////////////////////////////////////////////////

  // std::string focal_length_35mm_str;
  // if (ReadExifTag(handle_.ptr,
  //                 FIMD_EXIF_EXIF,
  //                 "FocalLengthIn35mmFilm",
  //                 &focal_length_35mm_str)) {
  //   static const std::regex regex(".*?([0-9.]+).*?mm.*?");
  //   std::cmatch result;
  //   if (std::regex_search(focal_length_35mm_str.c_str(), result, regex)) {
  //     const double focal_length_35 = std::stold(result[1]);
  //     if (focal_length_35 > 0) {
  //       *focal_length = focal_length_35 / 35.0 * max_size;
  //       return true;
  //     }
  //   }
  // }

  // //////////////////////////////////////////////////////////////////////////////
  // // Focal length in mm
  // //////////////////////////////////////////////////////////////////////////////

  // std::string focal_length_str;
  // if (ReadExifTag(
  //         handle_.ptr, FIMD_EXIF_EXIF, "FocalLength", &focal_length_str)) {
  //   std::regex regex(".*?([0-9.]+).*?mm");
  //   std::cmatch result;
  //   if (std::regex_search(focal_length_str.c_str(), result, regex)) {
  //     const double focal_length_mm = std::stold(result[1]);

  //     // Lookup sensor width in database.
  //     std::string make_str;
  //     std::string model_str;
  //     if (ReadExifTag(handle_.ptr, FIMD_EXIF_MAIN, "Make", &make_str) &&
  //         ReadExifTag(handle_.ptr, FIMD_EXIF_MAIN, "Model", &model_str)) {
  //       CameraDatabase database;
  //       double sensor_width;
  //       if (database.QuerySensorWidth(make_str, model_str, &sensor_width)) {
  //         *focal_length = focal_length_mm / sensor_width * max_size;
  //         return true;
  //       }
  //     }

  //     // Extract sensor width from EXIF.
  //     std::string pixel_x_dim_str;
  //     std::string x_res_str;
  //     std::string res_unit_str;
  //     if (ReadExifTag(handle_.ptr,
  //                     FIMD_EXIF_EXIF,
  //                     "PixelXDimension",
  //                     &pixel_x_dim_str) &&
  //         ReadExifTag(handle_.ptr,
  //                     FIMD_EXIF_EXIF,
  //                     "FocalPlaneXResolution",
  //                     &x_res_str) &&
  //         ReadExifTag(handle_.ptr,
  //                     FIMD_EXIF_EXIF,
  //                     "FocalPlaneResolutionUnit",
  //                     &res_unit_str)) {
  //       regex = std::regex(".*?([0-9.]+).*?");
  //       if (std::regex_search(pixel_x_dim_str.c_str(), result, regex)) {
  //         const double pixel_x_dim = std::stold(result[1]);
  //         regex = std::regex(".*?([0-9.]+).*?/.*?([0-9.]+).*?");
  //         if (std::regex_search(x_res_str.c_str(), result, regex)) {
  //           const double x_res = std::stold(result[2]) / std::stold(result[1]);
  //           // Use PixelXDimension instead of actual width of image, since
  //           // the image might have been resized, but the EXIF data preserved.
  //           const double ccd_width = x_res * pixel_x_dim;
  //           if (ccd_width > 0 && focal_length_mm > 0) {
  //             if (res_unit_str == "cm") {
  //               *focal_length = focal_length_mm / (ccd_width * 10.0) * max_size;
  //               return true;
  //             } else if (res_unit_str == "inches") {
  //               *focal_length = focal_length_mm / (ccd_width * 25.4) * max_size;
  //               return true;
  //             }
  //           }
  //         }
  //       }
  //     }
  //   }
  // }

  // return false;
}

bool BitmapOIIO::ExifLatitude(double* latitude) const {
  throw std::runtime_error("TODO");

//   std::string str;
//   double sign = 1.0;
//   if (ReadExifTag(handle_.ptr, FIMD_EXIF_GPS, "GPSLatitudeRef", &str)) {
//     StringTrim(&str);
//     StringToLower(&str);
//     if (!str.empty() && str[0] == 's') {
//       sign = -1.0;
//     }
//   }
//   if (ReadExifTag(handle_.ptr, FIMD_EXIF_GPS, "GPSLatitude", &str)) {
//     static const std::regex regex(".*?([0-9.]+):([0-9.]+):([0-9.]+).*?");
//     std::cmatch result;
//     if (std::regex_search(str.c_str(), result, regex)) {
//       const double hours = std::stold(result[1]);
//       const double minutes = std::stold(result[2]);
//       const double seconds = std::stold(result[3]);
//       double value = hours + minutes / 60.0 + seconds / 3600.0;
//       if (value > 0 && sign < 0) {
//         value *= sign;
//       }
//       *latitude = value;
//       return true;
//     }
//   }
//   return false;
// }

// bool BitmapOIIO::ExifLongitude(double* longitude) const {
//   std::string str;
//   double sign = 1.0;
//   if (ReadExifTag(handle_.ptr, FIMD_EXIF_GPS, "GPSLongitudeRef", &str)) {
//     StringTrim(&str);
//     StringToLower(&str);
//     if (!str.empty() && str[0] == 'w') {
//       sign = -1.0;
//     }
//   }
//   if (ReadExifTag(handle_.ptr, FIMD_EXIF_GPS, "GPSLongitude", &str)) {
//     static const std::regex regex(".*?([0-9.]+):([0-9.]+):([0-9.]+).*?");
//     std::cmatch result;
//     if (std::regex_search(str.c_str(), result, regex)) {
//       const double hours = std::stold(result[1]);
//       const double minutes = std::stold(result[2]);
//       const double seconds = std::stold(result[3]);
//       double value = hours + minutes / 60.0 + seconds / 3600.0;
//       if (value > 0 && sign < 0) {
//         value *= sign;
//       }
//       *longitude = value;
//       return true;
//     }
//   }
//   return false;
}

bool BitmapOIIO::ExifAltitude(double* altitude) const {
  throw std::runtime_error("TODO");

  // std::string str;
  // if (ReadExifTag(handle_.ptr, FIMD_EXIF_GPS, "GPSAltitude", &str)) {
  //   static const std::regex regex(".*?([0-9.]+).*?/.*?([0-9.]+).*?");
  //   std::cmatch result;
  //   if (std::regex_search(str.c_str(), result, regex)) {
  //     *altitude = std::stold(result[1]) / std::stold(result[2]);
  //     return true;
  //   }
  // }
  // return false;
}

bool BitmapOIIO::Read(const std::string& path, const bool as_rgb) {
  OIIO::ImageBuf inputBuf;

  if (!inputBuf.init_spec(path, 0, 0)) {
    return false;
  }
  if (!inputBuf.read()) {
    return false;
  }

  if (as_rgb) {
    if (inputBuf.nchannels() == 1) {
      buf_ = OIIO::ImageBufAlgo::channels(inputBuf, 3, { 0, 0, 0 });
      return true;
    } else if (inputBuf.nchannels() == 3 || inputBuf.nchannels() == 4) {
      // RGB -> BGR
      buf_ = OIIO::ImageBufAlgo::channels(inputBuf, 3, { 2, 1, 0 });
      return true;
    } else {
      return false;
    }
  } else {
    if (inputBuf.nchannels() == 1) {
      buf_ = inputBuf;
      return true;
    } else if (inputBuf.nchannels() == 3 || inputBuf.nchannels() == 4) {
      // Desaturate (this uses the same weights as FreeImage)
      OIIO::ImageBufAlgo::saturate(inputBuf, 0);
      buf_ = OIIO::ImageBufAlgo::channels(inputBuf, 1, { 0 });
      return true;
    } else {
      return false;
    }
  }
}

bool BitmapOIIO::Write(const std::string& path, const int flags) const {
  if (flags != 0) {
    throw std::runtime_error("Unsupported Write flag.");
  }

  std::unique_ptr<OIIO::ImageOutput> output = OIIO::ImageOutput::create(path);
  
  if (!output) {
    OIIO::geterror();
    // If we can't figure out the output implementation, use png.
    output = OIIO::ImageOutput::create("png");
  }

  if (!output) {
    throw std::runtime_error("Could not create ImageOutput.");
  }

  OIIO::ImageSpec outputSpec(Width(), Height(), buf_.nchannels());

  // Set 100 quality if we're writing JPEG.
  if (std::string(output->format_name()) == "jpeg") {
    outputSpec.attribute("Compression", "jpeg:100");
  }

  if (!output->open(path, outputSpec)) {
    return false;
  }

  OIIO::ImageBuf outBuf;
  if (IsGrey()) {
    outBuf = buf_;
  } else if (IsRGB()) {
    // BGR -> RGB
    outBuf = OIIO::ImageBufAlgo::channels(buf_, 3, { 2, 1, 0 });
  } else {
    throw std::runtime_error("Unknown depth");
  }

  return outBuf.write(output.get());
}

void BitmapOIIO::Smooth(const float sigma_x, const float sigma_y) {
  std::vector<float> array(buf_.spec().image_pixels());
  std::vector<float> array_smoothed(buf_.spec().image_pixels());
  for (int d = 0; d < buf_.nchannels(); ++d) {
    OIIO::ROI channel_roi(0, Width(), 0, Height(), 0, 1, d, d + 1);
    buf_.get_pixels(channel_roi, OIIO::span<float>(array));

    vl_imsmooth_f(array_smoothed.data(),
                  Width(),
                  array.data(),
                  Width(),
                  Height(),
                  Width(),
                  sigma_x,
                  sigma_y);

    buf_.set_pixels(channel_roi, OIIO::span<float>(array_smoothed));
  }
}

void BitmapOIIO::Rescale(const int new_width,
                         const int new_height,
                         RescaleFilter filter) {
  buf_ = OIIO::ImageBufAlgo::resample(
    buf_,
    true,
    OIIO::ROI(0, new_width, 0, new_height, 0, 1, 0, buf_.nchannels())
  );
}

BitmapOIIO BitmapOIIO::Clone() const {
  return BitmapOIIO(buf_.copy(OIIO::TypeUnknown));
}

BitmapOIIO BitmapOIIO::CloneAsGrey() const {
  if (IsGrey()) {
    return Clone();
  } else if (IsRGB()) {
    OIIO::ImageBuf grey;
    // First flip from BGR to RGB
    OIIO::ImageBufAlgo::channels(grey, buf_, 3, { 2, 1, 0});
    // Now desaturate (this uses the same weights as FreeImage)
    OIIO::ImageBufAlgo::saturate(grey, 0);
    return BitmapOIIO(OIIO::ImageBufAlgo::channels(grey, 1, { 0 }));
  } else {
    throw std::runtime_error("Unknown depth.");
  }
}

BitmapOIIO BitmapOIIO::CloneAsRGB() const {
  if (IsRGB()) {
    return Clone();
  } else if (IsGrey()) {
    return BitmapOIIO(OIIO::ImageBufAlgo::channels(buf_, 3, { 0, 0, 0 }));
  } else {
    throw std::runtime_error("Unknown depth.");
  }
}

void BitmapOIIO::CloneMetadata(BitmapOIIO* target) const {
  throw std::runtime_error("TODO");

  // THROW_CHECK_NOTNULL(target);
  // THROW_CHECK_NOTNULL(target->Data());
  // FreeImage_CloneMetadata(handle_.ptr, target->Data());
}

// void BitmapOIIO::SetPtr(FIBITMAP* ptr) {
//   THROW_CHECK_NOTNULL(ptr);

//   if (!IsPtrSupported(ptr)) {
//     FreeImageHandle temp_handle(ptr);
//     ptr = FreeImage_ConvertTo24Bits(temp_handle.ptr);
//     THROW_CHECK(IsPtrSupported(ptr));
//   }

//   handle_ = FreeImageHandle(ptr);
//   width_ = FreeImage_GetWidth(handle_.ptr);
//   height_ = FreeImage_GetHeight(handle_.ptr);
//   channels_ = IsPtrRGB(handle_.ptr) ? 3 : 1;
// }

std::ostream& operator<<(std::ostream& stream, const BitmapOIIO& bitmap) {
  stream << "Bitmap(width=" << bitmap.Width() << ", height=" << bitmap.Height()
         << ", channels=" << bitmap.Channels() << ")";
  return stream;
}

}  // namespace colmap
