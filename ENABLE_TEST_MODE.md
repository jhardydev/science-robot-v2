# How to Enable Display Test Mode

## Quick Enable

**Before starting the robot app, set the environment variable:**

```bash
export DISPLAY_TEST_MODE=true
```

Then start your robot app normally.

## If Using Docker

### Option 1: Environment Variable
```bash
export DISPLAY_TEST_MODE=true
docker-compose up
# OR
docker-compose restart
```

### Option 2: Add to docker-compose.yml
Add this to the `environment:` section:
```yaml
environment:
  - DISPLAY_TEST_MODE=true
```

### Option 3: Docker run command
```bash
docker run -e DISPLAY_TEST_MODE=true ...
```

## Verify Test Mode is Active

Check the robot logs - you should see:
```
[INFO] Display controller initialized in TEST MODE - will show test patterns
```

If you see:
```
[INFO] Display controller initialized
```
Then test mode is NOT enabled.

## What to Expect

When test mode is enabled:
- Display will cycle through 8 different test patterns
- Each pattern shows for 3 seconds
- Patterns include grids, lines, and position markers
- This helps identify correct positioning for network info

## Disable Test Mode

```bash
unset DISPLAY_TEST_MODE
# OR
export DISPLAY_TEST_MODE=false
```

Then restart the robot app.


