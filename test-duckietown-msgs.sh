#!/bin/bash
# Test script to verify duckietown_msgs is accessible

echo "Testing duckietown_msgs import..."

docker run --rm --network host \
  -e ROS_MASTER_URI=http://localhost:11311 \
  -e VEHICLE_NAME=robot1 \
  -v /usr/lib/python3/dist-packages/vpi:/host/usr/lib/python3/dist-packages/vpi:ro \
  -v /usr/lib/aarch64-linux-gnu:/host/usr/lib/aarch64-linux-gnu:ro \
  science-robot-v2:latest \
  bash -c "
    source /opt/ros/noetic/setup.bash
    source /code/devel/setup.bash 2>/dev/null || true
    source /code/packages/devel/setup.bash 2>/dev/null || true
    
    echo 'PYTHONPATH:'
    echo \$PYTHONPATH
    echo ''
    
    echo 'Testing duckietown_msgs import...'
    python3 -c \"
import sys
print('Python path:')
for p in sys.path:
    print(f'  {p}')

print('')
print('Testing duckietown_msgs import...')
try:
    from duckietown_msgs.msg import WheelsCmdStamped
    print('✓ SUCCESS: duckietown_msgs imported successfully!')
    print(f'  Location: {WheelsCmdStamped.__file__ if hasattr(WheelsCmdStamped, \"__file__\") else \"N/A\"}')
except ImportError as e:
    print(f'✗ FAILED: {e}')
    import traceback
    traceback.print_exc()
    sys.exit(1)
\"
"

