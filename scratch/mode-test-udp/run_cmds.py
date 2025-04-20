from multiprocessing import Pool
import numpy as np
import os
import time

def runcmd(cmd):
    time.sleep(1)
    print(cmd)
    os.system(cmd)

if __name__ == "__main__":
    cmds = []
    for i, id in enumerate(range(3)):
        cmd = f"./ns3 run mode-test-udp -- --scene_id={1000+id+1} --seed={1000+id+1} --simt={16 * 10 + 1}"
        cmds.append(cmd)
    with Pool(4) as pool:
        results = [pool.apply_async(runcmd, args=(cmd, )) for cmd in cmds]
        # 等待所有任务完成
        for result in results:
            result.get() 


 