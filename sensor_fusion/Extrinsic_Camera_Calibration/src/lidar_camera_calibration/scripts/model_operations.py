from ultralytics import YOLO
import os

class ModelOperations:

    def __init__(self, root_dir):
        self.model_path = '/home/daniel/segment_bounding_box_cones.pt'
        #print(self.model_path)
        # self.model_path = "/home/daniel/testing_model_new.pt"
        self.model = YOLO(self.model_path)
        self.model.fuse()
    
    def predict(self, img_msg, CV_BRIDGE):
        img = CV_BRIDGE.imgmsg_to_cv2(img_msg, 'bgr8')
        results = self.model.predict(source=img, save=True, save_txt=True)
        # print(results)
        classes = results[0].boxes.cls.tolist()
        conf = results[0].boxes.conf.tolist()
        segmentation_outputs = []
        scale_x, scale_y = img.shape[:2]
        print("scales")
        print(scale_x)
        print(scale_y)
        scale_x = 1
        scale_y = 1
        # scale_x /= 416
        # scale_y /= 640
        for result in results:
            masks = result.masks
            # print("mask shape")
            # print(masks.shape)
            if masks is not None:
                masks = masks.xy
                for mask in masks:
                    scaled_mask = [(int(x * scale_x), int(y * scale_y)) for x, y in mask]
                    segmentation_outputs.append(scaled_mask)
        return segmentation_outputs, classes, conf
