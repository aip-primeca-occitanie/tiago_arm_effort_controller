#! /usr/bin/env bash
SC="\e["
EC="\e[0m"

TB="1;" #bold
TF="2;" #faint
TI="3;" #italic
TU="4;" #underlined

O="38;5;214m"
R="91m"
G="92m"
Y="93m"
B="94m"
M="95m"
C="96m"
W="97m"

echo -e "${SC}${O}############################################################################${EC}"
echo -e "${SC}${O}#### Welcome to the easy script managin the installation of the TIAGo ! ####${EC}"
echo -e "${SC}${O}####     by Guillaume GRUNENBERGER (⌐■_■) for the TIAGo-155c <(✪_✪)>    ####${EC}"
echo -e "${SC}${O}############################################################################${EC}"

echo -e "${SC}${TB}${B}>>> Starting by transfering the required files to the TIAGo !${EC}"

echo -e "${SC}${B}transfering assimp-utils_4.1.0~dfsg-3 to pal@tiago-155c:/home/pal${EC}"
expect -c "set timeout -1
spawn scp ./assimp-utils_4.1.0~dfsg-3_amd64.deb root@tiago-155c:/home/pal
expect {
    password: { send palroot\r ; exp_continue }
}"

echo -e "${SC}${B}transfering robotpkg-hpp-fcl+doc_1.8.1 to pal@tiago-155c:/home/pal${EC}"
expect -c "set timeout -1
spawn scp ./robotpkg-hpp-fcl+doc_1.8.1_amd64.deb root@tiago-155c:/home/pal
expect {
    password: { send palroot\r ; exp_continue }
}"

echo -e "${SC}${B}transfering robotpkg-pinocchio_2.6.8 to pal@tiago-155c:/home/pal${EC}"
expect -c "set timeout -1
spawn scp ./robotpkg-pinocchio_2.6.8_amd64.deb root@tiago-155c:/home/pal
expect {
    password: { send palroot\r ; exp_continue }
}"

echo -e "${SC}${B}transfering TIAGo's urdf${EC}"
expect -c "set timeout -1
spawn scp ../urdf/tiagoSteel.urdf root@tiago-155c:/opt/pal/ferrum/share/tiago_description/robots/
expect {
    password: { send palroot\r ; exp_continue }
}"


echo -e "${SC}${TB}${M}>>> Switching to the TIAGo environment !${EC}"
export SSHPASS="palroot"
set = SSHPASS
echo -e "${SC}${M}installing assimp-utils_4.1.0~dfsg-3${EC}"
sshpass -e ssh -t -t -o StrictHostKeyChecking=no root@tiago-155c "sudo -S dpkg -i /home/pal/assimp-utils_4.1.0~dfsg-3_amd64.deb"
echo -e "${SC}${M}installing robotpkg-hpp-fcl+doc_1.8.1${EC}"
sshpass -e ssh -t -t -o StrictHostKeyChecking=no root@tiago-155c "sudo -S dpkg -i /home/pal/robotpkg-hpp-fcl+doc_1.8.1_amd64.deb"
echo -e "${SC}${M}installing robotpkg-pinocchio_2.6.8${EC}"
sshpass -e ssh -t -t -o StrictHostKeyChecking=no root@tiago-155c "sudo -S dpkg -i /home/pal/robotpkg-pinocchio_2.6.8_amd64.deb"
echo -e "${SC}${M}deleting obsolete files${EC}"
sshpass -e ssh -t -t -o StrictHostKeyChecking=no root@tiago-155c "sudo rm /home/pal/robotpkg-hpp-fcl+doc_1.8.1_amd64.deb /home/pal/robotpkg-pinocchio_2.6.8_amd64.deb"

echo -e "${SC}${R}...now do 'pal_restart_deployer' on the TIAGo terminal (ssh pal@tiago-155c) or restart the TIAGo (づ￣3￣)づ╭❤️～${EC}"