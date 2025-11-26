# Display Test Pattern Instructions

## Quick Start

Run the test pattern script directly with Python:

```bash
# From the science-robot-v2 directory
cd packages/science_robot/scripts
python3 test_display_patterns.py
```

Or use the helper script from the project root:

```bash
# From science-robot-v2 directory
./test_display.sh
```

## Prerequisites

1. **ROS must be running** - Make sure roscore is running or ROS master is available
2. **Display driver node must be running** - The `/robot1/display_driver_node` should be active
3. **Python path** - The script will automatically find the package modules

## Running Tests

### All Test Patterns (Recommended)

This will run 8 different test patterns to understand display behavior:

```bash
python3 packages/science_robot/scripts/test_display_patterns.py
```

### Single Custom Test

Run a single test with specific parameters:

```bash
python3 packages/science_robot/scripts/test_display_patterns.py --single <pattern_type> <region> <x_offset> <y_offset> <width> <height>
```

**Example:**
```bash
# Test a small grid at position (14, 0) with size 100x6
python3 packages/science_robot/scripts/test_display_patterns.py --single grid 0 14 0 100 6
```

**Parameters:**
- `pattern_type`: `grid`, `corners`, `numbers`, `lines`, or `position`
- `region`: `0` (FULL), `1` (HEADER), `2` (BODY), `3` (FOOTER)
- `x_offset`: X position in pixels (0-127)
- `y_offset`: Y position in pixels (0-31 or 0-63)
- `width`: Image width in pixels
- `height`: Image height in pixels

## What to Look For

After running the tests, take a photo of the display. The patterns will help us determine:

1. **Display Resolution**: Which test pattern fits (32px or 64px height)?
2. **Region Behavior**: How do REGION_HEADER, BODY, FOOTER position content?
3. **Offset Behavior**: Do x_offset and y_offset work as expected?
4. **Best Position**: Where should network info be placed (between yellow icons)?

## Troubleshooting

### "No module named 'science_robot'"

Make sure you're running from the correct directory or the Python path is set:

```bash
# Add the src directory to PYTHONPATH
export PYTHONPATH=$PYTHONPATH:$(pwd)/packages/science_robot/src
python3 packages/science_robot/scripts/test_display_patterns.py
```

### "Display driver node not found"

Make sure the display driver is running:

```bash
# Check if display driver node is running
rosnode list | grep display_driver_node
```

### "No connection to ROS master"

Make sure ROS is running:

```bash
# Start ROS master if needed
roscore
```

## Next Steps

Once you have a photo of the test patterns, we can:
1. Determine the correct display resolution
2. Understand how regions and offsets work
3. Update the display controller with the correct positioning
4. Place network info in the optimal location

