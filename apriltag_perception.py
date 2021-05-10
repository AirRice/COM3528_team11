import apriltag
import cv2
import math
import numpy as np
import miro_constants as con
from tag import Tag

class AprilTagPerception:
    def __init__(self, size, family='tag36h11'):
        options = apriltag.DetectorOptions(
            families=family,
            border=1,
            nthreads=4,
            quad_decimate=1.0,
            quad_blur=0.0,
            refine_edges=True,
            refine_decode=True,
            refine_pose=True,
            debug=False,
            quad_contours=True
        )

        self.detector = apriltag.Detector(options)
        self.size = size
        self.perceived = None


    def detect_tags(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

        result = self.detector.detect(gray)

        if result:
            tag = []
            for t, r in enumerate(result):
                tag.append(Tag())

                tag[t].apparent_size = np.mean([
                    math.hypot(r.corners[1][0] - r.corners[0][0], r.corners[1][1] - r.corners[0][1]),
                    math.hypot(r.corners[2][0] - r.corners[1][0], r.corners[2][1] - r.corners[1][1]),
                    math.hypot(r.corners[3][0] - r.corners[2][0], r.corners[3][1] - r.corners[2][1]),
                    math.hypot(r.corners[0][0] - r.corners[3][0], r.corners[0][1] - r.corners[3][1]),
                ])

                tag[t].centre = r.center.astype(int)
                tag[t].corners = r.corners.astype(int)
                tag[t].distance = (self.size * con.FOCAL_LENGTH) / tag[t].apparent_size
                tag[t].family = r.tag_family
                tag[t].id = r.tag_id

        else:
            tag = None

        return tag