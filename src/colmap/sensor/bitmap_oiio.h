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

#pragma once

#include "colmap/util/string.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <ios>
#include <limits>
#include <string>
#include <vector>

#include <colmap/sensor/bitmap.h>
#include <OpenImageIO/imagebuf.h>

namespace colmap {

// Wrapper class around OpenImageIO bitmaps.
class BitmapOIIO {
 public:
  BitmapOIIO();

  // Copy constructor.
  BitmapOIIO(const BitmapOIIO& other) = default;
  // Move constructor.
  BitmapOIIO(BitmapOIIO&& other) noexcept = default;

  // Create bitmap object from existing OIIO buffer object.
  explicit BitmapOIIO(const OIIO::ImageBuf& buffer);

  // Copy assignment.
  BitmapOIIO& operator=(const BitmapOIIO& other) = default;
  // Move assignment.
  BitmapOIIO& operator=(BitmapOIIO&& other) noexcept = default;

  // Allocate bitmap by overwriting the existing data.
  bool Allocate(int width, int height, bool as_rgb);

  // Deallocate the bitmap by releasing the existing data.
  void Deallocate();

  // Get pointer to underlying FreeImage object.
  inline const OIIO::ImageBuf& Data() const;
  inline OIIO::ImageBuf& Data();

  // Dimensions of bitmap.
  inline int Width() const;
  inline int Height() const;
  inline int Channels() const;

  // Number of bits per pixel. This is 8 for grey and 24 for RGB image.
  unsigned int BitsPerPixel() const;

  // Scan width of bitmap which differs from the actual image width to achieve
  // 32 bit aligned memory. Also known as stride.
  unsigned int Pitch() const;

  // Check whether image is grey- or colorscale.
  inline bool IsRGB() const;
  inline bool IsGrey() const;

  // Number of bytes required to store image.
  size_t NumBytes() const;

  // Copy raw image data to array.
  std::vector<uint8_t> ConvertToRowMajorArray() const;
  std::vector<uint8_t> ConvertToColMajorArray() const;

  // Convert to/from raw bits.
  std::vector<uint8_t> ConvertToRawBits() const;
  static BitmapOIIO ConvertFromRawBits(
      const uint8_t* data, int pitch, int width, int height, bool rgb = true);

  // Manipulate individual pixels. For grayscale images, only the red element
  // of the RGB color is used.
  bool GetPixel(int x, int y, BitmapColor<uint8_t>* color) const;
  bool SetPixel(int x, int y, const BitmapColor<uint8_t>& color);

  // Get pointer to y-th scanline, where the 0-th scanline is at the top.
  const uint8_t* GetScanline(int y) const;

  // Fill entire bitmap with uniform color. For grayscale images, the first
  // element of the vector is used.
  void Fill(const BitmapColor<uint8_t>& color);

  // Interpolate color at given floating point position.
  bool InterpolateNearestNeighbor(double x,
                                  double y,
                                  BitmapColor<uint8_t>* color) const;
  bool InterpolateBilinear(double x, double y, BitmapColor<float>* color) const;

  // Extract EXIF information from bitmap. Returns false if no EXIF information
  // is embedded in the bitmap.
  bool ExifCameraModel(std::string* camera_model) const;
  bool ExifFocalLength(double* focal_length) const;
  bool ExifLatitude(double* latitude) const;
  bool ExifLongitude(double* longitude) const;
  bool ExifAltitude(double* altitude) const;

  // Read bitmap at given path and convert to grey- or colorscale.
  bool Read(const std::string& path, bool as_rgb = true);

  // Write image to file. Flags can be used to set e.g. the JPEG quality.
  // Consult the FreeImage documentation for all available flags.
  bool Write(const std::string& path, int flags = 0) const;

  // Smooth the image using a Gaussian kernel.
  void Smooth(float sigma_x, float sigma_y);

  // Rescale image to the new dimensions.
  enum class RescaleFilter {
    kBilinear,
    kBox,
  };
  void Rescale(int new_width,
               int new_height,
               RescaleFilter filter = RescaleFilter::kBilinear);

  // Clone the image to a new bitmap object.
  BitmapOIIO Clone() const;
  BitmapOIIO CloneAsGrey() const;
  BitmapOIIO CloneAsRGB() const;

  // Clone metadata from this bitmap object to another target bitmap object.
  void CloneMetadata(BitmapOIIO* target) const;

 private:
  OIIO::ImageBuf buf_;
};

std::ostream& operator<<(std::ostream& stream, const BitmapOIIO& bitmap);

OIIO::ImageBuf& BitmapOIIO::Data() { return buf_; }
const OIIO::ImageBuf& BitmapOIIO::Data() const { return buf_; }

int BitmapOIIO::Width() const { return buf_.spec().width; }
int BitmapOIIO::Height() const { return buf_.spec().height; }
int BitmapOIIO::Channels() const { return buf_.spec().nchannels; }

bool BitmapOIIO::IsRGB() const { return Channels() == 3; }

bool BitmapOIIO::IsGrey() const { return Channels() == 1; }

}  // namespace colmap
