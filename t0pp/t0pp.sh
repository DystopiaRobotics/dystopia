git clone https://github.com/DystopiaRobotics/dystopia.git
wget https://repo.anaconda.com/archive/Anaconda3-2020.02-Linux-x86_64.sh
sh Anaconda3-2020.02-Linux-x86_64.sh
$SHELL
conda update --prefix /home/ubuntu/anaconda3 anaconda
conda create -n 3.7env python=3.7 -c conda-forge
conda deactivate
conda activate 3.7env
cd dystopia/t0pp
pip install protobuf==3.20
pip install -e .
mkdir ~/t0pp_finetune
python ~/dystopia/t0pp/single_task_fine_tune.py --dataset_name super_glue \
	--dataset_config_name record \
	--template_name "GPT-3 style summary only (continuation choices)" \
	--model_name_or_path bigscience/T0pp \
	--output_dir ./debug \
	--parallelize