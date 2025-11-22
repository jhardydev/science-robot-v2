# Building Science Robot v2.0 Docker Image

## Building Locally (Recommended)

Since you're on an ARM64 system (Apple Silicon Mac), you can build directly for the Jetson Nano without cross-compilation.

### Quick Build

```bash
cd science-robot-v2
docker build -t science-robot-v2:latest .
```

### Build with Progress Output

```bash
docker build --progress=plain -t science-robot-v2:latest .
```

### Build and Save Image

```bash
# Build the image
docker build -t science-robot-v2:latest .

# Save to tar file for transfer
docker save science-robot-v2:latest | gzip > science-robot-v2.tar.gz
```

## Transferring to Robot

### Option 1: Docker Registry (Recommended)

```bash
# Tag for your registry
docker tag science-robot-v2:latest your-registry/science-robot-v2:latest

# Push to registry
docker push your-registry/science-robot-v2:latest

# On robot, pull the image
docker pull your-registry/science-robot-v2:latest
```

### Option 2: Save and Transfer via SCP

```bash
# On local machine - build and save
docker build -t science-robot-v2:latest .
docker save science-robot-v2:latest | gzip > science-robot-v2.tar.gz

# Transfer to robot
scp science-robot-v2.tar.gz user@robot-ip:/tmp/

# On robot - load the image
ssh user@robot-ip
docker load < /tmp/science-robot-v2.tar.gz
```

### Option 3: Build on Robot (Current Method)

If you prefer to build directly on the robot:

```bash
# On robot
cd ~/science-robot-v2
docker build -t science-robot-v2:latest .
```

## Cross-Platform Build (For x86_64 Systems)

If you need to build from an x86_64 system, use Docker buildx:

```bash
# Create a builder instance (one-time setup)
docker buildx create --name arm64-builder --use
docker buildx inspect --bootstrap

# Build for ARM64
docker buildx build \
  --platform linux/arm64 \
  -t science-robot-v2:latest \
  --load \
  .

# Or build and push directly
docker buildx build \
  --platform linux/arm64 \
  -t your-registry/science-robot-v2:latest \
  --push \
  .
```

## Build Optimization Tips

### Use Build Cache

```bash
# Build with cache from previous build
docker build --cache-from science-robot-v2:latest -t science-robot-v2:latest .
```

### Build Specific Stage (if using multi-stage)

The current Dockerfile is single-stage, but if you add stages later:

```bash
docker build --target build-stage -t science-robot-v2:build .
```

### Reduce Build Time

The Dockerfile is already optimized with:
- Dependencies installed before copying code
- Separate RUN commands for better caching
- Requirements.txt copied early

## Troubleshooting

### Build Fails with "package not found"
- Check if base image `duckietown/dt-ros-commons:ente-arm64v8` is correct
- Verify Ubuntu 22.04 package names

### Build is Slow
- First build will be slow (downloading base image, installing packages)
- Subsequent builds use cache and are much faster
- Consider using a local registry for faster transfers

### Image Too Large
- Current image includes all dependencies
- Consider multi-stage build if size becomes an issue
- Remove build tools in final stage if needed

## Verification After Build

```bash
# Check image was created
docker images | grep science-robot-v2

# Test image (without running full robot)
docker run --rm science-robot-v2:latest python3 -c \
  "import sys; sys.path.insert(0, '/code/packages/src'); \
   from science_robot import config; print('OK')"

# Check ROS package discovery
docker run --rm science-robot-v2:latest bash -c \
  "source /opt/ros/noetic/setup.bash && \
   source /code/packages/devel/setup.bash && \
   rospack find science_robot"
```

## Recommended Workflow

1. **Develop locally** - Make code changes
2. **Build locally** - Test Docker build works
3. **Transfer to robot** - Use registry or SCP
4. **Test on robot** - Run with actual hardware

This saves time and allows you to catch build issues before deploying to the robot.

