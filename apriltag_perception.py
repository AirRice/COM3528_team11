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
        
    def draw_box(self, image, tag, colour=(0, 0, 0), width=200):
		# Verify colour name or format
		colour = self.check_colour(colour)

		# Dynamic border width
		width = int(width / tag.distance)

		# Draw the AprilTag bounding box
		for i in range(4):
			try:
				cv2.line(image, tuple(tag.corners[i]), tuple(tag.corners[i + 1]), colour, width)
			except IndexError:
				cv2.line(image, tuple(tag.corners[i]), tuple(tag.corners[0]), colour, width)

	def draw_center(self, image, tag, colour=(0, 0, 0), width=250):
		# Verify colour name or format
		colour = self.check_colour(colour)

		# Dynamic dot size
		width = int(width / tag.distance)

		# Draw the center (x, y)-coordinates of the AprilTag
		cv2.circle(image, tuple(tag.centre), width, colour, -1)

	def draw_distance(self, image, tag, colour=(0, 0, 0), size=20):
		# Verify colour name or format
		colour = self.check_colour(colour)

		# Dynamic number size
		scale = np.max([0.5, size / tag.distance])
		thickness = int(np.max([1, (size * 3) / tag.distance]))

		# Draw the tag distance on the image
		cv2.putText(
			image,
			'{0:.2f}cm'.format(tag.distance),
			tuple(tag.corners[0]),
			cv2.FONT_HERSHEY_SIMPLEX,
			scale,
			colour,
			thickness
		)

	def draw_id(self, image, tag, colour=(0, 0, 0), size=70):
		# Verify colour name or format
		colour = self.check_colour(colour)

		# Dynamic number size
		scale = np.max([0.25, size / tag.distance])
		thickness = int(np.max([2, (size * 3) / tag.distance]))

		# Draw the tag ID on the image
		cv2.putText(
			image,
			str(tag.id),
			tuple(tag.corners[3]),
			cv2.FONT_HERSHEY_SIMPLEX,
			scale,
			colour,
			thickness
		)