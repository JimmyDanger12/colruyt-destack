import matplotlib.pyplot as plt
import numpy as np
import PIL
import tensorflow as tf
import keras

from keras import layers
from keras.models import Sequential
import os
from sklearn import metrics
import numpy as np

IMG_SIZE = "img_size"
BATCH_SIZE = "batch_size"
OPTIMIZER = "optimizer"
EPOCHS = "epochs"
LOSS = "loss"
METRICS = "metrics"
CALLBACKS = "callbacks"

class ClassificationModel():
    def __init__(self):
        self.model = None
        self.class_names = ["NoCrate","NoPickupCrate","PickupCrate"]

    def load_model(self):
        self.model = keras.models.load_model("vision/class_model_new.keras",safe_mode=False)

    def train(self, params, show=False, save=True):
        
        img_height,img_width = params[IMG_SIZE]
        optimizer = params[OPTIMIZER]
        epochs = params[EPOCHS]
        batch_size = params[BATCH_SIZE]
        loss = params[LOSS]
        metrics = params[METRICS]
        callbacks = params[CALLBACKS]

        class_dir = "./vision/data/crops_detections"

        train_ds = keras.utils.image_dataset_from_directory(class_dir+"/train",
                                                            image_size=(img_height,img_width),
                                                            batch_size=batch_size)
        val_ds = keras.utils.image_dataset_from_directory(class_dir+"/valid",
                                                            image_size=(img_height,img_width),
                                                            batch_size=batch_size)
        self.class_names = train_ds.class_names
        
        AUTOTUNE = tf.data.AUTOTUNE
        train_ds = train_ds.cache().shuffle(1000).prefetch(buffer_size=AUTOTUNE)
        val_ds = val_ds.cache().prefetch(buffer_size=AUTOTUNE)
        num_classes = len(self.class_names)

        if not self.model:
            self.model = self.reset_model(img_height, img_width, num_classes)

            self.model.compile(optimizer=optimizer, 
                               loss=loss,
                               metrics=metrics)
        
        epochs=epochs
        history = self.model.fit(
            train_ds,
            validation_data=val_ds,
            epochs=epochs,
            callbacks=callbacks
        )

        if show:
            acc = history.history['accuracy']
            val_acc = history.history['val_accuracy']

            loss = history.history['loss']
            val_loss = history.history['val_loss']

            epochs_range = range(len(loss))
            plt.figure(figsize=(8, 8))
            plt.subplot(1, 2, 1)
            plt.plot(epochs_range, acc, label='Training Accuracy')
            plt.plot(epochs_range, val_acc, label='Validation Accuracy')
            plt.legend(loc='lower right')
            plt.title('Training and Validation Accuracy')

            plt.subplot(1, 2, 2)
            plt.plot(epochs_range, loss, label='Training Loss')
            plt.plot(epochs_range, val_loss, label='Validation Loss')
            plt.legend(loc='upper right')
            plt.title('Training and Validation Loss')
            plt.show()
        
        if save:
            self.model.save("vision/class_model_new.keras")

    def reset_model(self, img_height, img_width, num_classes):
        data_augmentation = keras.Sequential([
            #layers.Lambda(lambda x: x[...,::-1],input_shape=(img_height,img_width,3)),
            layers.RandomRotation(0.3,
                                        fill_mode="reflect",
                                        interpolation="bilinear"),
            layers.RandomZoom(0.1)])

        model = keras.Sequential([
            data_augmentation,
            layers.Rescaling(1./255, input_shape=(img_height, img_width, 3)),
            layers.Conv2D(16, 3, padding="same", activation="relu"),
            layers.MaxPooling2D(),
            layers.Conv2D(32, 3, padding="same", activation="relu"),
            layers.MaxPooling2D(),
            layers.Conv2D(64, 3, padding="same", activation="relu"),
            layers.MaxPooling2D(),
            layers.Dropout(0.1),
            layers.Flatten(),
            layers.Dense(128, activation="relu"),
            layers.Dense(64, activation="relu"),
            layers.Dense(num_classes, activation="softmax")
        ])
        return model
    
    def predict(self, path):
        labels = []
        conf_list = []
        for file in os.listdir(path):
            file = os.path.join(path,file)
            img = keras.utils.load_img(file)
            img_array = np.array(keras.utils.img_to_array(img),np.int32)
            img_array = img_array[:,:,::-1]
            img_array = tf.image.resize(img_array,(256,256),method="bicubic")
            img_array = np.array(img_array,dtype=np.int32)
            plt.imshow(img_array)
            plt.show()
            img_array = tf.expand_dims(img_array, 0)

            predictions = self.model.predict(img_array)
            predicted_class_idx = np.argmax(predictions[0])
            class_name = self.class_names[predicted_class_idx]
            conf = np.max(predictions[0])
            print(file,class_name,conf)
            labels.append(class_name)
            conf_list.append(conf)

        return labels, conf_list

    def test(self):
        data_dir = "vision/data/crops_detections/test"
        all_predictions = []
        all_labels = []

        for folder_idx, folder in enumerate(self.class_names):
            folder_path = os.path.join(data_dir, folder)
            for file_name in os.listdir(folder_path):
                img_path = os.path.join(folder_path, file_name)
                img = keras.utils.load_img(img_path)
                img_array = keras.utils.img_to_array(img)
                img_array = tf.expand_dims(img_array, 0)
                    
                predictions = self.model.predict(img_array)
                predicted_class_idx = np.argmax(predictions[0])

                all_predictions.append(predicted_class_idx)
                all_labels.append(folder_idx)

                print(
                    f"Predicted class: {self.class_names[predicted_class_idx]}, "
                    f"{100 * np.max(predictions[0]):.2f} confidence.\n"
                    f"Real class: {folder}"
                )

        confusion_mat = metrics.confusion_matrix(all_labels, all_predictions)
        cm_display = metrics.ConfusionMatrixDisplay(confusion_matrix = confusion_mat, display_labels = self.class_names)
        cm_display.plot()
        plt.title("Confusion Matrix")
        plt.show() 

learning_rate = 0.001
weight_decay = 0.004
callback = keras.callbacks.EarlyStopping(monitor="val_loss", patience=5)
params = {
    IMG_SIZE: (256,256),
    BATCH_SIZE: 16,
    OPTIMIZER: keras.optimizers.AdamW(learning_rate,weight_decay),
    LOSS: keras.losses.SparseCategoricalCrossentropy(from_logits=False),
    METRICS: ["accuracy"],
    EPOCHS: 20,
    CALLBACKS: [callback]
}

if __name__ == "__main__":
    a = ClassificationModel()
    a.load_model()
    #a.train(params, show=True, save=True)
    #a.test()
    path="vision/crops/monkey/predict/crops/Crate"
    #path="vision/data/detected_crops"
    a.predict(path=path)