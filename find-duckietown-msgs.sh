#!/bin/bash
# Diagnostic script to find where duckietown_msgs is located

echo "Searching for duckietown_msgs in container..."

docker run --rm science-robot-v2:latest bash -c "
echo 'Checking for duckietown_msgs in common locations...'
echo ''

# Check common paths
PATHS=(
    '/code/devel/lib/python3/dist-packages'
    '/code/install/lib/python3/dist-packages'
    '/code/catkin_ws/devel/lib/python3/dist-packages'
    '/opt/ros/noetic/lib/python3/dist-packages'
    '/usr/lib/python3/dist-packages'
)

for PATH in \"\${PATHS[@]}\"; do
    if [ -d \"\$PATH\" ]; then
        echo \"Checking: \$PATH\"
        if [ -d \"\$PATH/duckietown_msgs\" ]; then
            echo \"  ✓ Found duckietown_msgs directory\"
            ls -la \"\$PATH/duckietown_msgs\" | head -5
        elif find \"\$PATH\" -maxdepth 1 -name 'duckietown_msgs*' 2>/dev/null | grep -q .; then
            echo \"  ✓ Found duckietown_msgs files\"
            find \"\$PATH\" -maxdepth 1 -name 'duckietown_msgs*' 2>/dev/null | head -3
        else
            echo \"  ✗ Not found\"
        fi
        echo ''
    fi
done

echo 'Testing import with different paths...'
for PATH in \"\${PATHS[@]}\"; do
    if [ -d \"\$PATH\" ]; then
        if python3 -c \"import sys; sys.path.insert(0, '\$PATH'); import duckietown_msgs; print('SUCCESS from', '\$PATH')\" 2>/dev/null; then
            echo \"✓ duckietown_msgs can be imported from: \$PATH\"
        fi
    fi
done
"

