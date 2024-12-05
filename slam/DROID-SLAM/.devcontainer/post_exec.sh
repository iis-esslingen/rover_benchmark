#!/bin/bash
set -eux

echo ">>> Start of post install script <<<"

if [[ ! -d "/workspace/DROID-SLAM" ]]; then
  echo "Cloning DROID-SLAM repository..."
  
  cd /workspace

  git clone --recursive https://github.com/iis-esslingen/DROID-SLAM.git

  cd /workspace/DROID-SLAM

  pip install evo --upgrade --no-binary evo
  pip install gdown

  python setup.py install
fi

echo ">>> End of post install script <<<" 