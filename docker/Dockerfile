FROM build-env:multipanda_ros2-amd64
USER root
COPY src/openpi/packages/openpi-client /opt/openpi-client
RUN python3 -m pip install --no-cache-dir "numpy<2" h5py \
    "websockets==15.0.1" "typing-extensions==4.13.2" /opt/openpi-client
USER developer
