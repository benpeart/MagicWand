import tensorflow as tf
from tensorflow.keras import layers, models
import numpy as np

TIME_STEPS = 256
FEATURES = 10
NUM_CLASSES = 6   # adjust for your gestures

def build_model():
    inputs = layers.Input(shape=(TIME_STEPS, FEATURES))

    # 1D CNN stack
    x = layers.Conv1D(32, 5, activation='relu', padding='same')(inputs)
    x = layers.Conv1D(32, 5, activation='relu', padding='same')(x)
    x = layers.MaxPooling1D(2)(x)

    # Depthwise conv
    x = layers.DepthwiseConv1D(3, activation='relu', padding='same')(x)

    # LSTM for temporal structure
    x = layers.LSTM(32, return_sequences=False)(x)

    # Dense classifier
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

# Load your dataset
X = np.load("X.npy")   # shape: [samples, 256, 10]
y = np.load("y.npy")   # shape: [samples]

model.fit(
    X, y,
    batch_size=32,
    epochs=30,
    validation_split=0.2,
    shuffle=True
)

model.save("gesture_model.h5")
