import cv2
from ultralytics import YOLO

def main():
    model = YOLO('/home/danish/Amarnath/arm/yolov8n.pt')
    image_path = '/home/danish/Amarnath/arm/camera_shot.png'
    img = cv2.imread(image_path)
    if img is None:
        print(f"Failed to read image at {image_path}")
        return

    print("Running YOLO inference with conf=0.1...")
    results = model.predict(img, conf=0.1, verbose=True)
    if not results or len(results) == 0:
        print("No results returned.")
        return

    result = results[0]
    print(f"Found {len(result.boxes)} boxes:")
    for i, box in enumerate(result.boxes):
        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().tolist()
        conf = float(box.conf[0].cpu().numpy())
        cls_id = int(box.cls[0].cpu().numpy())
        cls_name = model.names[cls_id]
        print(f"Box {i}: class={cls_name} ({cls_id}), conf={conf:.3f}, coords=[{x1:.1f}, {y1:.1f}, {x2:.1f}, {y2:.1f}]")

if __name__ == '__main__':
    main()
