cd /home/pi
mkdir git  # may be already created
cd git 
# clone git repo (do nothing if already created) 
git clone https://gitlab.ensta-bretagne.fr/zerrbe/drivers-ddboat-v2.git 
# update git repo
cd drivers-ddboat-v2
git pull
cd /home/pi
cd ddboat
# may be already created
git clone https://gitlab.ensta-bretagne.fr/zerrbe/arduino-ddboat.git arduino
# update
cd arduino
git config user.email "benblop@gmail.com"
git config user.name "Ben Blop"
git stash
git pull
cd mega/cmd_motors_cpu_rc_1ch_ppm_ensv
make && make upload


