from .camera_geometry import CameraGeometry
import numpy as np
import albumentations as albu
import cv2
import torch
import segmentation_models_pytorch as smp


class LaneDetector():
    def __init__(self, cam_geom=CameraGeometry(), model_path='./lane_segmentation_model.pth', 
    encoder = 'efficientnet-b0', encoder_weights = 'imagenet'):
        self.cg = cam_geom
        self.cut_v, self.grid = self.cg.precompute_grid()
        if torch.cuda.is_available():
            self.device = "cuda"
            self.model = torch.load(model_path).to(self.device)
        else:
            self.model = torch.load(model_path, map_location=torch.device("cpu"))
            self.device = "cpu"
        self.encoder = encoder
        self.encoder_weights = encoder_weights
        preprocessing_fn = smp.encoders.get_preprocessing_fn(self.encoder, self.encoder_weights)
        self.to_tensor_func = self._get_preprocessing(preprocessing_fn)

    def _get_preprocessing(self, preprocessing_fn):
        def to_tensor(x, **kwargs):
            return x.transpose(2, 0, 1).astype('float32')
        transform = [
            albu.Lambda(image=preprocessing_fn),
            albu.Lambda(image=to_tensor),
        ]
        return albu.Compose(transform)

    
    def read_imagefile_to_array(self, filename):
        image = cv2.imread(filename)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        return image
    

    def detect_from_file(self, filename):
        img_array = self.read_imagefile_to_array(filename)
        return self.detect(img_array)


    def detect(self, img_array):
        image_tensor = self.to_tensor_func(image=img_array)["image"]
        x_tensor = torch.from_numpy(image_tensor).to(self.device).unsqueeze(0)
        model_output = self.model.predict(x_tensor).cpu().numpy()
        background, left, right = model_output[0,0,:,:], model_output[0,1,:,:], model_output[0,2,:,:] 
        return background, left, right
    
    def detect_and_fit(self, img_array):
        _, left, right = self.detect(img_array)
        left_poly = self.fit_poly(left)
        right_poly = self.fit_poly(right)
        return left_poly, right_poly

    def fit_poly(self, probs):
        probs_flat = np.ravel(probs[self.cut_v:, :])
        mask = probs_flat > 0.3
        coeffs = np.polyfit(self.grid[:,0][mask], self.grid[:,1][mask], deg=3, w=probs_flat[mask])
        return np.poly1d(coeffs)

    def __call__(self, img):
        if isinstance(img, str):
            img = self.read_imagefile_to_array(img)
        return self.detect_and_fit(img)
    
