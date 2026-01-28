# train_trajectory_model.py
import numpy as np
import tensorflow as tf
from tensorflow.keras import layers, models
from sklearn.model_selection import train_test_split

TIME_STEPS = 256
FEATURES = 2
NUM_CLASSES = 6  # adjust for your spells

# Load dataset
X = np.load("X_trajectories.npy")   # shape [N, 256, 2]
y = np.load("y_labels.npy")         # shape [N]

X_train, X_val, y_train, y_val = train_test_split(
    X, y, test_size=0.2, stratify=y, random_state=42
)

def build_model():
    inputs = layers.Input(shape=(TIME_STEPS, FEATURES))

    x = layers.Conv1D(32, 5, activation='relu', padding='same')(inputs)
    x = layers.Conv1D(32, 5, activation='relu', padding='same')(x)
    x = layers.MaxPooling1D(2)(x)

    x = layers.LSTM(32)(x)

    x = layers.Dense(32, activation='relu')(x)
    outputs = layers.Dense(NUM_CLASSES, activation='softmax')(x)

    model = models.Model(inputs, outputs)
    model.compile(
        optimizer='adam',
        loss='sparse_categorical_crossentropy',
        metrics=['accuracy']
    )
    return model

model = build_model()
model.summary()

history = model.fit(
    X_train, y_train,
    validation_data=(X_val, y_val),
    epochs=40,
    batch_size=32,
    shuffle=True
)

model.save("wand_trajectory_model.h5")

# Export to TFLite
converter = tf.lite.TFLiteConverter.from_keras_model(model)
converter.optimizations = [tf.lite.Optimize.DEFAULT]
tflite_model = converter.convert()

with open("wand_trajectory_model.tflite", "wb") as f:
    f.write(tflite_model)

# Convert to C array
import subprocess
subprocess.run(["xxd", "-i", "wand_trajectory_model.tflite", "model.cc"])
