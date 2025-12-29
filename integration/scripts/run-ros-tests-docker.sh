#!/usr/bin/env bash
set -euo pipefail

# Simple helper to build a Docker image from a Dockerfile (path is repo-root relative) and run
# the integration test crate inside the container. The Dockerfile path MUST be relative to the
# repository root (e.g. `integration/docker/Dockerfile_rclrs_humble`).
# Usage: ./scripts/run-ros-tests-docker.sh <target> [dockerfile-relative-path] [mode]
#
# Mode: 'copy' (default) creates a tarball of the workspace and copies it into a temporary container (suitable for CI).
#       'bind' mounts the repository root into the container and runs tests on the real workspace (useful for iterative development).
#
# Example: ./integration/scripts/run-ros-tests-docker.sh r2r integration/docker/Dockerfile_r2r_humble

TARGET=${1:-}
DOCKERFILE=${2:-}
ROOT=$(cd "$(dirname "$0")/../.." && pwd)
INTEGRATION_DIR=$(cd "$(dirname "$0")/.." && pwd)

if [[ -z "$TARGET" && -z "$DOCKERFILE" ]]; then
  echo "Usage: $0 <target> [dockerfile-relative-path] [mode]"
  exit 2
fi

# If a Dockerfile wasn't provided, attempt to pick a reasonable default (repo-root relative)
if [[ -z "$DOCKERFILE" ]]; then
  case "$TARGET" in
    ros2-interfaces-jazzy-serde|ros2-interfaces-jazzy-rkyv|r2r)
      # Use Dockerfiles maintained in integration/docker by default
      DOCKERFILE="integration/docker/Dockerfile_r2r_humble"
      ;;
    rosrust)
      DOCKERFILE="integration/docker/Dockerfile_ros1_noetic"
      ;;
    rclrs)
      DOCKERFILE="integration/docker/Dockerfile_rclrs_humble"
      ;;
    safe_drive)
      DOCKERFILE="integration/docker/Dockerfile_safe_drive_jazzy"
      ;;
    *)
      echo "No default Dockerfile for target '$TARGET'. Provide one as second argument." >&2
      exit 2
      ;;
  esac
fi

if [[ ! -f "$ROOT/$DOCKERFILE" ]]; then
  echo "Dockerfile not found: $ROOT/$DOCKERFILE" >&2
  exit 2
fi

# Sanitize TARGET to make a valid Docker tag (commas/spaces -> underscores)
SANITIZED_TARGET="${TARGET//,/__}"
SANITIZED_TARGET="${SANITIZED_TARGET// /_}"
TAG="ros_pointcloud2_test:${SANITIZED_TARGET}"

echo "Building Docker image $TAG from $DOCKERFILE..."
cd "$ROOT"
docker build -f "$DOCKERFILE" -t "$TAG" .

echo "Running tests in container..."

# Container path to place the project (choose a project-specific path to avoid collisions)
CONTAINER_DIR="/ros_pointcloud2"
# Mode: 'bind' (default) mounts the repository root into the container. 'copy' copies the repository into a temporary container.
MODE=${3:-bind}

# Special-case behavior for rclrs: it expects the workspace at /ros2_rust_ws and the image
# ENTRYPOINT runs a script that builds and runs tests. Detect target or dockerfile name.
IS_RCLRS=0
if [[ "$TARGET" == "rclrs" || "$DOCKERFILE" =~ rclrs ]]; then
  IS_RCLRS=1
  CONTAINER_DIR="/ros2_rust_ws"
fi

if [[ "$MODE" == "bind" ]]; then
  if [[ "$IS_RCLRS" -eq 1 ]]; then
    echo "Mounting package into container at /ros2_rust_ws/src/ros_pointcloud2 (rclrs mode)..."
    # Mount only the package into a standard colcon workspace location so the container ENTRYPOINT can build it.
    docker run --rm -v "$ROOT/ros_pointcloud2":"/ros2_rust_ws/src/ros_pointcloud2" -w "/ros2_rust_ws" -e CARGO_TERM_COLOR=always "$TAG"
  else
    echo "Mounting repository root into container at $CONTAINER_DIR..."
    # Map targets to features
    FEATURES=""
    case "$TARGET" in
      r2r) FEATURES="r2r" ;;
      rosrust) FEATURES="rosrust" ;;
      ros2-interfaces-jazzy-serde|ros2_jazzy_serde) FEATURES="ros2-interfaces-jazzy-serde" ;;
      ros2-interfaces-jazzy-rkyv|ros2_jazzy_rkyv) FEATURES="ros2-interfaces-jazzy-rkyv" ;;
      nalgebra) FEATURES="nalgebra" ;;
      *) FEATURES="" ;;
    esac

    if [[ -n "$FEATURES" ]]; then
      echo "Running tests in container with features: $FEATURES"
      docker run --rm -v "$ROOT":"$CONTAINER_DIR" -w "$CONTAINER_DIR" -e CARGO_TERM_COLOR=always "$TAG" \
        bash -lc 'for s in /opt/ros/*/setup.sh; do [ -f "$s" ] && . "$s"; done && cargo test -p ros_integration_tests --features "'$FEATURES'" -- --nocapture'
    else
      docker run --rm -v "$ROOT":"$CONTAINER_DIR" -w "$CONTAINER_DIR" -e CARGO_TERM_COLOR=always "$TAG" \
        bash -lc 'for s in /opt/ros/*/setup.sh; do [ -f "$s" ] && . "$s"; done && cargo test -p ros_integration_tests -- --nocapture'
    fi
  fi

elif [[ "$MODE" == "copy" ]]; then
  echo "Copying repository into container image (no host mount)..."

  if [[ "$IS_RCLRS" -eq 1 ]]; then
    # Create a container (stopped) using image default entrypoint; copy the whole repository into /ros2_rust_ws
    container_id=$(docker create "$TAG")

    # Ensure the container is removed on cleanup
    cleanup() {
      docker rm -f "$container_id" > /dev/null 2>&1 || true
    }
    trap cleanup EXIT

    echo "Copying package to container $container_id:/ros2_rust_ws/src/ros_pointcloud2..."
    docker cp "$ROOT/ros_pointcloud2" "$container_id":/ros2_rust_ws/src/ros_pointcloud2

    echo "Starting container $container_id..."
    docker start "$container_id"

    echo "Streaming container logs (Ctrl-C to detach)..."
    docker logs -f "$container_id"

    # Wait for the container to finish and propagate exit code
    exit_code=$(docker wait "$container_id")
    echo "Container exited with code $exit_code"
    # Remove container (cleanup trap will also attempt)
    docker rm -f "$container_id" > /dev/null 2>&1 || true
    if [[ "$exit_code" -ne 0 ]]; then
      exit "$exit_code"
    fi

  else
    # Non-rclrs flow: tar the repo and extract inside a temporary container (keeps entrypoint overridden so we can run commands)
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
    # Map targets to features for the copy-mode run as well
    FEATURES=""
    case "$TARGET" in
      r2r) FEATURES="r2r" ;;
      rosrust) FEATURES="rosrust" ;;
      ros2-interfaces-jazzy-serde|ros2_jazzy_serde) FEATURES="ros2-interfaces-jazzy-serde" ;;
      ros2-interfaces-jazzy-rkyv|ros2_jazzy_rkyv) FEATURES="ros2-interfaces-jazzy-rkyv" ;;
      nalgebra) FEATURES="nalgebra" ;;
      *) FEATURES="" ;;
    esac

    if [[ -n "$FEATURES" ]]; then
      docker exec -e CARGO_TERM_COLOR=always "$container_id" bash -lc "for s in /opt/ros/*/setup.sh; do [ -f \"\$s\" ] && . \"\$s\"; done && cd $CONTAINER_DIR && cargo test -p ros_integration_tests --features \"$FEATURES\" -- --nocapture"
    else
      docker exec -e CARGO_TERM_COLOR=always "$container_id" bash -lc "for s in /opt/ros/*/setup.sh; do [ -f \"\$s\" ] && . \"\$s\"; done && cd $CONTAINER_DIR && cargo test -p ros_integration_tests -- --nocapture"
    fi
  fi

else
  echo "Unknown mode: $MODE. Use 'bind' (default) or 'copy'." >&2
  exit 2
fi

echo "Done."
