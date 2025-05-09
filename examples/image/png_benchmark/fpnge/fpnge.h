// Copyright 2021 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#ifndef FPNGE_H
#define FPNGE_H
#include <stdlib.h>

// -mavx2 -mpclmul
#if defined(__AVX2__) && defined(__PCLMUL__) && defined(__clang__)
#define CAN_COMPILE_FPNGE

// bytes_per_channel = 1/2 for 8-bit and 16-bit. num_channels: 1/2/3/4
// (G/GA/RGB/RGBA)
size_t FPNGEEncode(size_t bytes_per_channel, size_t num_channels,
                   const unsigned char *data, size_t width, size_t row_stride,
                   size_t height, unsigned char **output);

#endif // AVX2 / PCLMUL

#endif
