#!/usr/bin/env bash
set -euo pipefail

# Simple helper to build a Docker image from a Dockerfile in `tests/` and run
# the integration test crate with a given feature.
# Usage: ./scripts/run-ros-tests-docker.sh <feature> [dockerfile] [mode]
#
# Mode: 'copy' (default) creates a tarball of the workspace and copies it into a temporary container (suitable for CI).
#       'bind' mounts the workspace into the container and runs tests on the real workspace (useful for iterative development).
#
# Example: ./integration/scripts/run-ros-tests-docker.sh r2r docker/Dockerfile_r2r_jazzy

FEATURE=${1:-}
DOCKERFILE=${2:-}
ROOT=$(cd "$(dirname "$0")/../.." && pwd)
INTEGRATION_DIR=$(cd "$(dirname "$0")/.." && pwd)

if [[ -z "$FEATURE" ]]; then
  echo "Usage: $0 <feature> [dockerfile]"
  exit 2
fi

# If a Dockerfile wasn't provided, attempt to pick a reasonable default
if [[ -z "$DOCKERFILE" ]]; then
  case "$FEATURE" in
    ros2-interfaces-jazzy-serde|ros2-interfaces-jazzy-rkyv|r2r)
      # Use Dockerfiles maintained in integration/docker by default
      DOCKERFILE="docker/Dockerfile_r2r_jazzy"
      ;;
    rosrust)
      DOCKERFILE="docker/Dockerfile_ros1_noetic"
      ;;
    safe_drive)
      DOCKERFILE="docker/Dockerfile_safe_drive_jazzy"
      ;;
    *)
      echo "No default Dockerfile for feature '$FEATURE'. Provide one as second argument." >&2
      exit 2
      ;;
  esac
fi

if [[ ! -f "$INTEGRATION_DIR/$DOCKERFILE" ]]; then
  echo "Dockerfile not found: $INTEGRATION_DIR/$DOCKERFILE" >&2
  exit 2
fi

TAG="ros_pointcloud2_test:${FEATURE}"

echo "Building Docker image $TAG from $DOCKERFILE..."
cd "$INTEGRATION_DIR"
docker build -f "$DOCKERFILE" -t "$TAG" .

echo "Running tests in container..."

# Container path to place the project (choose a project-specific path to avoid collisions)
CONTAINER_DIR="/ros_pointcloud2"
# Mode: 'bind' (default) mounts the workspace into the container. 'copy' copies the workspace into a temporary container.
MODE=${3:-bind}

if [[ "$MODE" == "bind" ]]; then
  echo "Mounting project into container at $CONTAINER_DIR..."
  docker run --rm -v "$ROOT":"$CONTAINER_DIR" -w "$CONTAINER_DIR" -e CARGO_TERM_COLOR=always "$TAG" \
    bash -lc 'for s in /opt/ros/*/setup.sh; do [ -f "$s" ] && . "$s"; done && cargo test -p ros_integration_tests --features '${FEATURE}' -- --nocapture'
elif [[ "$MODE" == "copy" ]]; then
  echo "Copying project into container image (no host mount)..."
  # Create a stopped container with ENTRYPOINT overridden so we can copy files into it
  # (some images define an ENTRYPOINT that would run immediately on start and fail
  # because the workspace hasn't been extracted yet)
  container_id=$(docker create --entrypoint "" "$TAG" tail -f /dev/null)

  # Create a temp dir for the tarball
  tmpdir=$(mktemp -d)

  # Ensure the container and temp files are removed on script exit
  cleanup() {
    docker rm -f "$container_id" > /dev/null 2>&1 || true
    rm -rf "$tmpdir" > /dev/null 2>&1 || true
  }
  trap cleanup EXIT

  echo "Creating tarball excluding 'target', '.git', 'node_modules'..."
  tar -C "$ROOT" --exclude='target' --exclude='.git' --exclude='node_modules' -cf "$tmpdir"/workspace.tar .

  echo "Copying tarball to container $container_id:/tmp/workspace.tar..."
  docker cp "$tmpdir"/workspace.tar "$container_id":/tmp/workspace.tar

  echo "Starting container $container_id..."
  docker start "$container_id" > /dev/null

  echo "Extracting workspace inside container..."
  docker exec -e CARGO_TERM_COLOR=always "$container_id" bash -lc "mkdir -p '$CONTAINER_DIR' && tar -xf /tmp/workspace.tar -C '$CONTAINER_DIR' && rm /tmp/workspace.tar"

  echo "Running tests inside container..."
  docker exec -e CARGO_TERM_COLOR=always "$container_id" bash -lc "for s in /opt/ros/*/setup.sh; do [ -f \"\$s\" ] && . \"\$s\"; done && cd $CONTAINER_DIR && cargo test -p ros_integration_tests --features ${FEATURE} -- --nocapture"

else
  echo "Unknown mode: $MODE. Use 'bind' (default) or 'copy'." >&2
  exit 2
fi

echo "Done."
