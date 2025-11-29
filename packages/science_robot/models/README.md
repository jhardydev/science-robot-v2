# MediaPipe Models

This directory contains MediaPipe model files used by the science_robot package.

## Models Included

### 1. Hand Landmarker Model

**Model**: Hand Landmarker  
**Source**: Google MediaPipe  
**Version**: float16/latest  
**License**: Apache 2.0  
**Download URL**: https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/latest/hand_landmarker.task

**Usage**: Hand detection and landmark tracking (21 landmarks per hand). Uses the newer MediaPipe Tasks API (Hand Landmarker) instead of the older Solutions API for improved accuracy, especially for small hands.

**File**: `hand_landmarker.task`  
**Size**: ~10-15 MB  
**Format**: MediaPipe Task model format

### 2. Gesture Recognizer Model

**Model**: Gesture Recognizer  
**Source**: Google MediaPipe  
**Version**: float16/latest  
**License**: Apache 2.0  
**Download URL**: https://storage.googleapis.com/mediapipe-models/gesture_recognizer/gesture_recognizer/float16/latest/gesture_recognizer.task

**Usage**: Gesture recognition, specifically detecting:
- Thumb Up gesture
- Open Palm gesture (used for stop command)

**File**: `gesture_recognizer.task`  
**Size**: ~8 MB  
**Format**: MediaPipe Task model format

## License

Both models are provided by Google MediaPipe under the Apache License 2.0.

Copyright 2023 The MediaPipe Authors.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

## Attribution

- **Hand Landmarker**: [MediaPipe Hand Landmarker](https://ai.google.dev/edge/mediapipe/solutions/vision/hand_landmarker)
- **Gesture Recognizer**: [MediaPipe Gesture Recognizer](https://ai.google.dev/edge/mediapipe/solutions/vision/gesture_recognizer)
- **Model Repository**: https://github.com/google/mediapipe
- **Documentation**: 
  - [Hand Landmarker Python API](https://ai.google.dev/edge/mediapipe/solutions/vision/hand_landmarker/python)
  - [Gesture Recognizer Python API](https://ai.google.dev/edge/mediapipe/solutions/vision/gesture_recognizer/python)
