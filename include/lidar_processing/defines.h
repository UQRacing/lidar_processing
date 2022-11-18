// LiDAR processing definitions header
// Copyright (c) 2022 Matt Young (UQ Racing Formula SAE Team)
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

namespace uqr {

/**
 * Version history:
 * v0.0.1: initial release
 * v1.0.0: first proper release
 * v1.0.1: fixed the pipeline saying camera info unavailable when it was available
 * v1.0.2: fixed image topics not setting the header timestamp
 * v1.0.3: refactored topic names
 * v1.1.0: added uqr_msgs/DepthImage publisher
 */
#define LIDAR_PROCESSING_VERSION "1.1.0"

}