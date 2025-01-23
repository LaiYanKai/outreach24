from ultralytics import YOLO

# Load a model
model = YOLO("yolo11n.pt")

# Train the model
train_results = model.train(
    data="traffic_train.yaml",  # path to dataset YAML
    epochs=100,  # number of training epochs
    imgsz=100,  # training image size
    device="cpu",  # device to run on, i.e. device=0 or device=0,1,2,3 or device=cpu
)

# Evaluate model performance on the validation set
metrics = model.val()

# Export the model to ONNX format
path = model.export(format="onnx")  # return path to exported model

# Perform object detection on an image
#results = model("../datasets/traffic/images/left0000.jpg")
#results[0].show()

