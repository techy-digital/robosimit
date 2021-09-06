import subprocess
import logging
import uuid

ID = str(uuid.uuid4())

log_file = "./output_log.txt"

with open('sc.sh','w') as f:
    f.write("set +x \n")
    f.write("export XSOCK=/tmp/.X11-unix \n")
    f.write("export XAUTH=/tmp/.docker.xauth \n")
    f.write("xhost +local:root \n")
    f.write("docker run --gpus all \\\n")
    f.write("   -v $XSOCK:$XSOCK \\\n")
    f.write("   -v $XAUTH:$XAUTH \\\n")
    f.write("   -e XAUTHORITY=$XAUTH \\\n")
    f.write("   -e DISPLAY=:1 \\\n")
    f.write("   --rm \\\n")
    f.write("   -it \\\n")
    f.write("   --name " + ID + " \\\n")
    f.write("   valu3s:robosimit \\\n")
    f.write("   bash \n")


myprocess = subprocess.Popen(["/bin/sh","sc.sh"], stdout = subprocess.PIPE, stderr = subprocess.STDOUT, shell=False)


logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO,filename=log_file,format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')

def check_io():
    while True:
        output = myprocess.stdout.readline().decode()
        if output:
            logger.log(logging.INFO, output)
        else:
            break

while myprocess.poll() is None:
    check_io()