#ENV_LIST='SparseMountainCarContinuous-v0 MountainCarContinuous-v0 HalfCheetah-v2 SparseHalfCheetah-v2 Reacher-v2 Hopper-v2 Humanoid-v2 Ant-v2 SparseInvertedPendulumEnv-v2 Walker2d-v2 Swimmer-v2'

rm exp_list.txt
touch exp_list.txt

SEED_LIST='2 3 5 7'

setting_num=0
setting_num=$[setting_num+1]; SETTING[$setting_num]='';                                EXP_NAME[$setting_num]='no_exploration'

setting_num=$[setting_num+1]; SETTING[$setting_num]='--action-noise';                  EXP_NAME[$setting_num]='action_noise'
setting_num=$[setting_num+1]; SETTING[$setting_num]='--parameter-noise';               EXP_NAME[$setting_num]='parameter_noise'
setting_num=$[setting_num+1]; SETTING[$setting_num]='--SGLD-mode 2';                   EXP_NAME[$setting_num]='SGLD_s'
#setting_num=$[setting_num+1]; SETTING[$setting_num]='--SGLD-mode 2 --train-mode 1';    EXP_NAME[$setting_num]='SGLD_h'


ENV_LIST='MAC_1_10 MAC_1_3 MAC_1_1 MAC_1_0'
for env_name in $ENV_LIST
do
    for i in `seq 1 $setting_num`
    do
      for seed in $SEED_LIST
      do
        echo "python DDPG/trainer.py  --env $env_name  --exp-name ${EXP_NAME[$i]}  --rand-seed $seed ${SETTING[$i]} " >> exp_list.txt
      done
    done 
done

cp exp_list.txt results/
