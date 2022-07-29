wget https://repo.anaconda.com/archive/Anaconda3-2020.02-Linux-x86_64.sh
bash Anaconda3-2020.02-Linux-x86_64.sh
conda create -n myenv python==3.7
conda activate myenv
git clone https://github.com/bigscience-workshop/t-zero.git
cd t-zero
pip install -e .
pip install protobuf==3.20.0
python inference/model_parallelism.py