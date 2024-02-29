"""
Gets the colmap parameters of a given image.
"""

import fire
import colmap_utils
from colmap_utils import read_colmap_output, orderedImageKeys, orderedOpenCvModelCameraKeys
from pprint import pprint

def get_colmap_parameters(colmapOutputDir, imageName):

    images, cameras, _ = read_colmap_output(colmapOutputDir)
    
    image = images[imageName]
    camId = image["camId"]
    camera = cameras[camId]

    print(f"\nExtrinsic camera parameters ({imageName} from images.txt):")
    print(", ".join([str(image[key]) for key in orderedImageKeys]))
    print("\nAs JSON:")
    pprint(image, indent=4)
    print(f"\nIntrinsic camera parameters (camera {camId} from cameras.txt):")
    print(", ".join([str(camera[key]) for key in orderedOpenCvModelCameraKeys]))
    print("\nAs JSON:")
    pprint(camera, indent=4)


if __name__ == "__main__":
    fire.Fire(get_colmap_parameters)