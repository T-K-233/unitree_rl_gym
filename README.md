详细使用说明，请参考 https://support.unitree.com/home/zh/developer/rl_example


# Setup instructions

```bash
conda create -yn unitree python=3.8
```

```bash
conda create -yp ./conda-env/ python=3.8
```


```bash
pip install -r requirements.txt

pip install -e ~/Documents/isaacgym/python/

# install rsl_rl
git clone https://github.com/leggedrobotics/rsl_rl
cd rsl_rl/
# git checkout v1.0.2
pip install -e .
cd ..

# install legged_gym
pip install -e .

```
