ARG BASE_IMAGE=d018-ros:jazzy-autoware
FROM ${BASE_IMAGE}

USER root
ARG ADAS_UID=1000
ARG ADAS_GID=1000
RUN groupmod --non-unique --gid ${ADAS_GID} aw \
    && usermod --non-unique --uid ${ADAS_UID} --gid ${ADAS_GID} aw \
    && chown ${ADAS_UID}:${ADAS_GID} /home/aw
COPY --from=carla_api carla-0.10.0-cp312-cp312-linux_x86_64.whl /tmp/carla-0.10.0-cp312-cp312-linux_x86_64.whl
RUN python3 -m venv --system-site-packages /opt/adas-venv \
    && /opt/adas-venv/bin/pip install --no-index --no-deps /tmp/carla-0.10.0-cp312-cp312-linux_x86_64.whl \
    && rm /tmp/carla-0.10.0-cp312-cp312-linux_x86_64.whl \
    && /opt/adas-venv/bin/python -c "import carla, matplotlib, numpy, PIL" \
    && command -v ffmpeg
COPY --chmod=755 docker/adas-entrypoint.sh /usr/local/bin/adas-entrypoint
ENV PATH="/opt/adas-venv/bin:${PATH}" PYTHONUNBUFFERED=1
WORKDIR /workspace/adas
USER aw
ENTRYPOINT ["/usr/local/bin/adas-entrypoint"]
CMD ["sleep", "infinity"]
