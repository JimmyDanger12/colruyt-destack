from ultralytics import YOLO
from PIL import Image

IMG_SIZE = "img_size"
BATCH_SIZE = "batch_size"
OPTIMIZER = "optimizer"
EPOCHS = "epochs"
LOSS = "loss"
METRICS = "metrics"
LEARNING_RATE = "learning_rate"
PATIENCE = "patience"
PATH = "path"

BEST_MODEL_PATH = "vision/runs/segment/train8/weights/best.pt"

class SegmentationModel():
    def __init__(self):
        self.model = None

    def load_model(self):
        self.model = YOLO(BEST_MODEL_PATH, task="segment")
    
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
                         single_cls=True,
                         imgsz=image_size,
                         epochs=epochs,
                         batch=batch_size,
                         optimizer=optimizer,
                         lr0=lr,
                         patience=patience,
                         retina_masks=True,
                         close_mosaic=10,
                         project="vision/runs/segment")
    
    def test(self):
        metrics = self.model.val(split="test")
        return metrics

    def predict(self, path, show=False, save=True, save_crops=False):
        results = self.model.predict(source=path,
                            save=save,
                            conf=0.4,
                            iou=0.45,
                            save_crop=save_crops,
                            project="vision/crops/monkey",
                            exist_ok=True)
        
        if show:
            for r in results:
                im_array = r.plot()
                im = Image.fromarray(im_array[..., ::-1])
                im.show()
        return results
    
    def reset_model(self):
        self.model = YOLO("vision/yolov8s-seg.pt", task="segment")


params = {
    PATH: "vision/data/only_lab_total",
    IMG_SIZE: 640,
    OPTIMIZER: "AdamW",
    LEARNING_RATE: 0.0012,
    EPOCHS: 80,
    BATCH_SIZE: 16,
    PATIENCE: 10
}

if __name__ == "__main__":
    b = SegmentationModel()

    b.load_model()
    print("model loaded")

    #results = b.train(params)
    print("model trained")

    #metrics = b.test()
    print("model evaluated")

    test_image = "vision/data/only_lab_total/test/images"
    results = b.predict(test_image, False, False, True)
    print("model tested")