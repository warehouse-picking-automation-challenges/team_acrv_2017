# Things to do before competition run

## Perception

1. Load new item information
  * string identifier into `acrv_apc_2017_perception/src/eval_semseg/items.py` TODO: script or manual?
  * place jsons into ? for weight / dimensions TODO

2. F1 Shots
  * Take shots with `autoseg2` on
    - 2x tote
    - curtain?
    
3. Finetune RefineNet
  * call ... script to train


## Manipulation

1. Take weight of all new items, update jsons?
2. Decide on suction/gripper
3. Add heuristics?