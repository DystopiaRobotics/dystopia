#!/bin/bash
# TODO: This is a simple script that can build an image to setup a bastion with Duo MFA

echo "Script begins here"
echo $USER
cd /home/ubuntu
pwd
sudo apt-get update
sudo apt-get remove docker docker-engine docker-ce docker.io
sudo apt-get install -y apt-transport-https ca-certificates curl software-properties-common
curl -fsSL https://download.docker.com/linux/ubuntu/gpg  | sudo apt-key add -
sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu  $(lsb_release -cs) stable"
sudo apt-get -y install docker-ce
sudo systemctl start docker
sudo apt-get install -y docker-compose
sudo systemctl enable docker

# setup keys and download bastion from s3
export AWS_DEFAULT_REGION=us-east-1
export AWS_ACCESS_KEY_ID=<your aws id>
export AWS_SECRET_ACCESS_KEY=<your aws key>
apt-get install awscli -y

cd bastion/examples/compose
sudo docker-compose up --build
#cat /var/log/cloud-init-output.log
