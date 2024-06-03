import os
os.environ['TF_ENABLE_ONEDNN_OPTS'] = '0'

import pandas as pd
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
import tensorflow as tf
import numpy as np

# Load data
data = pd.read_csv('data.txt', names=['x', 'y', 'z', 'microphone'])

# Add labels to the data (1 for accident, 0 for no accident)
data['label'] = np.random.randint(0, 2, size=len(data))

# Preprocess data
X = data[['x', 'y', 'z', 'microphone']]
y = data['label']

scaler = StandardScaler()
X_scaled = scaler.fit_transform(X)

X_train, X_test, y_train, y_test = train_test_split(X_scaled, y, test_size=0.2, random_state=42)

# Define a simple neural network model using the Sequential API
model = tf.keras.Sequential([
    tf.keras.layers.InputLayer(input_shape=(4,), name="input_layer"),
    tf.keras.layers.Dense(64, activation='relu', name="dense_1"),
    tf.keras.layers.Dense(64, activation='relu', name="dense_2"),
    tf.keras.layers.Dense(1, activation='sigmoid', name="output_layer")
])

model.compile(optimizer='adam', loss='binary_crossentropy', metrics=['accuracy'])
model.fit(X_train, y_train, epochs=10, validation_data=(X_test, y_test))

# Check model summary to debug
model.summary()

# Convert model to TensorFlow Lite
# Define the concrete function
@tf.function
def model_to_convert(x):
    return model(x)

# Get a concrete function
concrete_func = model_to_convert.get_concrete_function(tf.TensorSpec(shape=[None, 4], dtype=tf.float32))

# Convert the model to TensorFlow Lite
converter = tf.lite.TFLiteConverter.from_concrete_functions([concrete_func])

# Set optimization settings
converter.optimizations = [tf.lite.Optimize.DEFAULT]

# Define the input signature
def representative_dataset_gen():
    for i in range(len(X_test)):
        yield [X_test[i:i+1].astype(np.float32)]

converter.representative_dataset = representative_dataset_gen
converter.target_spec.supported_ops = [
    tf.lite.OpsSet.TFLITE_BUILTINS,  # or tf.lite.OpsSet.SELECT_TF_OPS
]

# Convert the model
try:
    tflite_model = converter.convert()
    # Save the TensorFlow Lite model to a file
    with open('model.tflite', 'wb') as f:
        f.write(tflite_model)
    print("Model successfully converted to TensorFlow Lite format and saved as 'model.tflite'.")
except Exception as e:
    print("An error occurred during model conversion:", str(e))
