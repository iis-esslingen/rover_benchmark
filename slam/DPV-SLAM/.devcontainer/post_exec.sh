#!/bin/bash
set -eux

echo ">>> Start of post install script <<<"

if [[ ! -d "/workspace/DPV-SLAM" ]]; then
  echo "Cloning DPV-SLAM repository..."
  
  cd /workspace

  git clone --recursive https://github.com/iis-esslingen/DPV-SLAM.git

  cd /workspace/DPV-SLAM

  wget https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.zip
  unzip eigen-3.4.0.zip -d thirdparty

  pip install .

  wget https://www.dropbox.com/s/nap0u8zslspdwm4/models.zip && unzip models.zip

  # apt-get install --fix-missing

  # ./Pangolin/scripts/install_prerequisites.sh recommended
  # mkdir Pangolin/build && cd Pangolin/build
  # cmake ..
  # make -j8
  # make install
  # cd ../..

  # pip install ./DPViewer

  # ldconfig
fi

echo ">>> End of post install script <<<" 