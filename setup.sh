source /opt/ros/foxy/setup.bash

apt update && apt -y install openssh-server
echo root:"agx" | chpasswd
service  ssh start
echo -e "Port 10022\nPermitRootLogin yes\nPermitEmptyPasswords yes" >> /etc/ssh/sshd_config.d/dev.conf
service ssh restart
service ssh enable

while sleep 1000; do :; done