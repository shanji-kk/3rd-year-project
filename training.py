from ultralytics import YOLO
import torch

# clean the GPU useage and set the device to GPU
torch.backends.cudnn.benchmark = True
torch.cuda.device(0)  # Set to your desired GPU number
torch.cuda.empty_cache()


if __name__ == '__main__':
    # Import YOLOV8 model
    model = YOLO("yolov8n.pt")
    model.train(data="config2.yaml",epochs=100,batch=16)  


