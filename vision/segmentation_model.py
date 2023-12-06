from ultralytics import YOLO
import cv2
from PIL import Image
import numpy as np

IMG_SIZE = "img_size"
BATCH_SIZE = "batch_size"
OPTIMIZER = "optimizer"
EPOCHS = "epochs"
LOSS = "loss"
METRICS = "metrics"
LEARNING_RATE = "learning_rate"
PATIENCE = "patience"
PATH = "path"

class SegmentationModel():
    def __init__(self):
        self.model = None

    def load_model(self):
        self.model = YOLO("vision/runs/segment/train5/weights/best.pt", task="segment")
    
    def train(self, params):
        path = params[PATH]
        image_size = params[IMG_SIZE]
        epochs = params[EPOCHS]
        batch_size = params[BATCH_SIZE]
        optimizer = params[OPTIMIZER]
        lr = params[LEARNING_RATE]
        patience = params[PATIENCE]

        if not self.model:
            self.reset_model()
        
        self.model.train(data=path+"/data.yaml",
                         imgsz=image_size,
                         epochs=epochs,
                         batch=batch_size,
                         optimizer=optimizer,
                         lr0=lr,
                         patience=patience,
                         retina_masks=True)
    
    def test(self):
        metrics = self.model.val(split="test")
        return metrics

    def predict(self, path, show=False, save_crops=False):
        results = self.model.predict(source=path,
                            save=True,
                            conf=0.4,
                            save_crop=save_crops,
                            project="vision/crops",
                            name="monkey")
        
        if show:
            for r in results:
                im_array = r.plot()
                im = Image.fromarray(im_array[..., ::-1])
                im.show()
        
        return results
    
    def reset_model(self):
        self.model = YOLO("vision/yolov8s-seg.pt", task="segment")


params = {
    PATH: "vision/data/no-bars_no-classes",
    IMG_SIZE: 640,
    OPTIMIZER: "AdamW",
    LEARNING_RATE: 0.0015,
    EPOCHS: 80,
    BATCH_SIZE: 8,
    PATIENCE: 10
}

if __name__ == "__main__":
    b = SegmentationModel()

    b.load_model()

    #results = b.train(params)

    #metrics = b.test()

    results = b.predict("vision/test_frame.jpg", True, True)