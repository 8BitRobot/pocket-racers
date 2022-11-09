import os
import psutil
from multiprocessing import Pipe
import time
import sys
import numpy as np

def save_data(pipe, address="data"):
    time.sleep(3)
    while True:
        if pipe.poll():
            data = pipe.recv()
            video = data[0]
            motor = data[1]
            servo = data[2]
            if(len(sys.argv) == 1):
                name = address+"/driving_data_1"
                while os.path.exists(name):
                    name = name[:-1]+str(int(name[-1])+1)
            else:
                name = f"{address}/{sys.argv[1]}"
                while os.path.exists(name):
                    try:
                        name = name[:-1]+str(int(name[-1])+1)
                    except:
                        name = name + "_1"
            os.mkdir(name)
            # np.savez_compressed(name+"/zipped_data", video=video, motor=motor.get_data(), servo=servo.get_data())
            np.save(name+"/video", video)
            np.save(name+"/motor", motor)
            np.save(name+"/servo", servo)
            pipe.send([f"datapoint taken : saved to {name}.npy"])
            # print(f"datapoint taken : saved to {name}.npy")
            del video