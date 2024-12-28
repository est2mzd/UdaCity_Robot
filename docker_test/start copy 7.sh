docker run -it --rm \
    --gpus all \
    --runtime=nvidia \
    --security-opt apparmor=unconfined \
    --privileged \
    --env="DISPLAY=${DISPLAY}" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="XAUTHORITY=${XAUTH}" \
    --env="LIBGL_ALWAYS_INDIRECT=1" \
    --env="__GLX_VENDOR_LIBRARY_NAME=nvidia" \
    --env="LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:/usr/lib/x86_64-linux-gnu/mesa:$LD_LIBRARY_PATH" \
    --volume="${XSOCK}:${XSOCK}:rw" \
    --volume="${XAUTH}:${XAUTH}:rw" \
    --volume="/sys:/sys:ro" \
    --volume="/proc:/proc:ro" \
    --volume="/dev:/dev" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --device=/dev/dri \
    --device=/dev/nvidia0 \
    --device=/dev/nvidiactl \
    --device=/dev/nvidia-modeset \
    --device=/dev/nvidia-uvm \
    --name=ubuntu16-opengl \
    ubuntu16-opengl-container