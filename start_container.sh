set +x
export XSOCK=/tmp/.X11-unix
export XAUTH=/tmp/.docker.xauth

xhost +local:root


docker run --gpus all \
    -v $XSOCK:$XSOCK \
    -v $XAUTH:$XAUTH \
    -e XAUTHORITY=$XAUTH \
    -e DISPLAY=:1 \
    --rm \
    -it \
    --name test1 \
    --label etiket=valu3s \
    valu3s:robosimit \
    bash
