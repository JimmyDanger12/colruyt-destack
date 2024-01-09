import time
start_time = time.time()
from vision.vision_client import VisionClient
import_time = time.time()
print("Import Dur:",import_time-start_time)

if __name__ == "__main__":
    init_time = time.time()
    vision = VisionClient()
    vision.connect()
    connect_time = time.time()
    print("Connect Dur",connect_time-init_time)
    coords, height = vision.get_valid_pickup_loc()
    print("Coords",coords,"Height",height)
    complete_time = time.time()
    print("Total vision pipe time",complete_time-connect_time)
    coords, height = vision.get_valid_pickup_loc()
    print("Coords",coords,"Height",height)
    complete_time2 = time.time()
    print("Total vision pipe time 2",complete_time2-complete_time)