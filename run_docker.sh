sudo docker run --rm --env="DISPLAY" -it --privileged -v /home/julan/PerceptIn/BasePlate:/build -v /dev/bus/usb:/dev/bus/usb -v /tmp/.X11-unix:/tmp/.X11-unix:rw qingyuchen/perceptin-test-docker
