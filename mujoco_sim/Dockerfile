# Standalone MuJoCo simulator for Star Nav (no ROS).
#
#   docker build -t 1nrobotics/star-nav-mujoco mujoco_sim
#   docker run --rm -p 8080:8080 1nrobotics/star-nav-mujoco --bind 0.0.0.0
#
# Rendering uses OSMesa (software) by default so it runs on any host; pass
# MUJOCO_GL=egl with a GPU runtime (e.g. --gpus all) for hardware rendering.
FROM python:3.12-slim-bookworm

RUN apt-get update && apt-get install -y --no-install-recommends \
        libosmesa6 \
        libegl1 \
        libgl1 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/mujoco_sim
COPY pyproject.toml ./
COPY star_nav_mujoco ./star_nav_mujoco
COPY tests ./tests
RUN pip install --no-cache-dir ".[test]"

ENV MUJOCO_GL=osmesa \
    PYOPENGL_PLATFORM=osmesa \
    PYTHONUNBUFFERED=1

EXPOSE 8080 5555 5556
ENTRYPOINT ["python", "-m", "star_nav_mujoco"]
