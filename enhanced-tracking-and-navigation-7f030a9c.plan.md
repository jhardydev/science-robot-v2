<!-- 7f030a9c-6c7e-40bc-bd30-60ed0d614f19 a0d2a621-2f6c-4af5-93d0-24c8b8a6df2f -->
# MediaPipe Gesture Recognizer Migration Plan

## Overview

Migrate gesture detection from custom MediaPipe Hands classification to MediaPipe Gesture Recognizer (Tasks API) for improved accuracy, particularly for thumbs-up detection. Implementation will use a hybrid approach, keeping existing code as fallback.

## Phase 1: Branch Setup and Model Preparation

### 1.1 Create feature branch

- Create branch: `feature/mediapipe-gesture-recognizer`
- Create backup tag: `backup-before-gesture-recognizer`

### 1.2 Add configuration

**File**: `packages/science_robot/src/science_robot/config.py`

- Add `GESTURE_RECOGNIZER_MODEL_PATH` pointing to `{PACKAGE_PATH}/models/gesture_recognizer.task`
- Add `GESTURE_RECOGNIZER_ENABLED` env var (default: `False`)
- Add `GESTURE_RECOGNIZER_MIN_DETECTION_CONFIDENCE` (default: `0.5`)
- Add `GESTURE_RECOGNIZER_MIN_GESTURE_CONFIDENCE` (default: `0.5`)

### 1.3 Create model download script

**File**: `packages/science_robot/scripts/download_gesture_model.py`

- Download gesture recognizer model from MediaPipe releases
- Verify model file exists before download
- Store in config-specified path
- Handle errors gracefully with logging

## Phase 2: Hybrid Implementation

### 2.1 Add Gesture Recognizer imports

**File**: `packages/science_robot/src/science_robot/gesture_detector.py`

- Import `mediapipe.tasks.python` and `mediapipe.tasks.python.vision`
- Add conditional import with `GESTURE_RECOGNIZER_AVAILABLE` flag
- Handle ImportError gracefully (log warning if unavailable)

### 2.2 Initialize Gesture Recognizer

**File**: `packages/science_robot/src/science_robot/gesture_detector.py`

- Add `_initialize_gesture_recognizer()` method to `GestureDetector.__init__`
- Initialize only if `GESTURE_RECOGNIZER_ENABLED=True` and model file exists
- Use VIDEO running mode for real-time processing
- Store recognizer instance as `self.gesture_recognizer`
- Set `self.gesture_recognizer_enabled` flag

### 2.3 Add gesture classification method

**File**: `packages/science_robot/src/science_robot/gesture_detector.py`

- Add `classify_gesture_with_recognizer(frame)` method
- Convert BGR frame to MediaPipe Image format
- Call `recognize_for_video()` with timestamp
- Map MediaPipe gesture names to internal format:
- `Thumb_Up` → `'thumbs_up'`
- Return gesture string or None
- Handle errors and log debug messages

### 2.4 Update classify_gesture() for hybrid mode

**File**: `packages/science_robot/src/science_robot/gesture_detector.py`

- Modify `classify_gesture(hands, frame=None)` signature to accept optional frame
- Try Gesture Recognizer first if enabled and frame provided
- Fallback to existing custom gesture logic if recognizer unavailable/fails
- Maintain backward compatibility (works without frame parameter)

## Phase 3: Update Integration Points

### 3.1 Update classify_gesture() calls

**File**: `packages/science_robot/src/science_robot_node.py`

- Update all `classify_gesture()` calls to pass `frame` parameter:
- Line ~515: `classify_gesture(hands_data, frame=frame)`
- Line ~617: `classify_gesture(hands_data, frame=frame)`
- Line ~760: `classify_gesture(hands_data, frame=frame)`

**File**: `packages/science_robot/src/science_robot/gesture_detector.py`

- Update `draw_landmarks()` method (~line 622) to pass frame:
- `classify_gesture([landmarks], frame=frame)`

### 3.2 Add timestamp tracking

**File**: `packages/science_robot/src/science_robot/gesture_detector.py`

- Add `self.last_frame_timestamp` to track video timestamps
- Initialize in `__init__` to 0
- Update in `classify_gesture_with_recognizer()` using `time.time() * 1000`
- Pass timestamp to `recognize_for_video()` (required for VIDEO mode)

### 3.3 Update close() method

**File**: `packages/science_robot/src/science_robot/gesture_detector.py`

- Add cleanup for `self.gesture_recognizer` if exists
- Call `recognizer.close()` in close() method

## Phase 4: Testing and Validation

### 4.1 Functional testing

- Test thumbs-up detection with recognizer enabled
- Verify fallback to custom detection when disabled
- Test all existing gestures still work (treat, etc.)
- Verify face detection unchanged
- Test wave detection still works

### 4.2 Performance validation

- Measure frame processing time (should be similar)
- Check for any frame drops or lag
- Verify on robot hardware (ARM64 compatibility)

### 4.3 Edge cases

- Multiple hands in frame
- Partial occlusion
- Low light conditions
- Fast gesture transitions

## Phase 5: Documentation and Cleanup

### 5.1 Update documentation

- Add model download instructions to README
- Document environment variables
- Add troubleshooting section

### 5.2 Optional: Enable by default

- After successful testing, consider setting `GESTURE_RECOGNIZER_ENABLED=True` by default
- Or keep as opt-in feature for flexibility

## Implementation Notes

- **Model URL**: Use MediaPipe official model repository URL for `gesture_recognizer.task`
- **Backward Compatibility**: All changes maintain backward compatibility - existing code works if recognizer disabled
- **Error Handling**: Graceful degradation - if recognizer fails, falls back to custom detection
- **Performance**: Gesture Recognizer may be slightly slower; monitor and adjust if needed
- **ARM64**: Verify MediaPipe Gesture Recognizer works on robot's ARM64 architecture

## Rollback Strategy

- Feature branch allows easy reversion: `git checkout main`
- Configuration flag allows runtime disable: `GESTURE_RECOGNIZER_ENABLED=False`
- Backup tag provides restore point: `git checkout backup-before-gesture-recognizer`

## Success Criteria

- Thumbs-up detected more reliably with recognizer
- No regressions in existing functionality
- Performance acceptable (no frame drops)
- Easy to enable/disable via configuration
- Works on robot hardware (ARM64)

### To-dos

- [ ] Add thumbs up gesture detection method to gesture_detector.py
- [ ] Update classify_gesture() to include thumbs up
- [ ] Update config.py to change STOP_GESTURE to THUMBS_UP parameters
- [ ] Update wave_detector.py __init__ to use thumbs_up variables
- [ ] Rename _detect_stop_gesture to _detect_thumbs_up_gesture in wave_detector.py