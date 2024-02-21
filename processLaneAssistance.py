
if __name__ == "__main__":
    import sys

    sys.path.insert(0, "../../..")

from multiprocessing import Pipe
from src.templates.workerprocess import WorkerProcess
from src.data.TrafficCommunication.useful.sharedMem import sharedMem

class processLaneAssistance(WorkerProcess):
    """This process receive the location of the car and send it to the processGateway.\n
    Args:
            queueList (dictionary of multiprocessing.queues.Queue): Dictionary of queues where the ID is the type of messages.
            logging (logging object): Made for debugging.
    """

    # ====================================== INIT ==========================================
    def __init__(self, queueList, logging):
        self.queuesList = queueList
        self.logging = logging
        self.shared_memory = sharedMem()
        super(processLaneAssistance, self).__init__(self.queuesList)

    # ===================================== STOP ==========================================
    def stop(self):
        """Function for stopping threads and the process."""
        for thread in self.threads:
            thread.stop()
            thread.join()
        super(processLaneAssistance, self).stop()

    # ===================================== RUN ==========================================
    def run(self):
        """Apply the initializing methods and start the threads."""
        super(processLaneAssistance, self).run()




if __name__ == "__main__":
    from multiprocessing import Queue, Event
    from skimage import io
    from skimage.color import rgb2gray
    import time
    import logging
    import cv2
    import base64
    import numpy as np
    import skimage
    import matplotlib.pyplot as plt

    class PIDController:
        def __init__(self, Kp, Ki, Kd, setpoint=0):
            self.Kp = Kp
            self.Ki = Ki
            self.Kd = Kd
            self.r = r
            self.prev_error = 0
            self.integral = 0

        def compute(self, current_error):
            # error = self.setpoint - current_value
            error = current_error - self.r  # PROVERI

            # Proportional term
            P = self.Kp * error

            # Integral term
            self.integral += error
            I = self.Ki * self.integral

            # Derivative term
            D = self.Kd * (error - self.prev_error)
            self.prev_error = error

            # Total control output
            control_output = P + I + D

            return control_output

    def line_detection(image):
        # Canny algorithm for detecting edges
        image_gray = rgb2gray(image)
        image_canny = skimage.feature.canny(image_gray, sigma=1, low_threshold=0.4, high_threshold=0.8)

        # Region of interest
        M, N = np.shape(image_canny)
        image_canny[:int(M / 3), :] = 0

        polygons = np.array([(M, 0), (M, N), (150, N), (0, N // 2), (150, 0)])
        mask = skimage.draw.polygon2mask((M, N), polygons)

        image_canny_interest = image_canny * mask
        image_canny_interest = np.uint8(image_canny_interest)

        # Hafova transfor and finding lines

        lines = cv2.HoughLinesP(image_canny_interest, 2, np.pi / 180, 100, minLineLength=10, maxLineGap=4)
        lines = np.array(lines)

        left_fit = []
        right_fit = []

        for line in lines:
            x1, y1, x2, y2 = line.reshape(4)

            k = (y2 - y1) / (x2 - x1)
            n = y1 - k * x1

            if k < 0:
                left_fit.append([k, n])
            elif k >= 0:
                right_fit.append([k, n])

        left_avg = np.average(left_fit, axis=0)
        right_avg = np.average(right_fit, axis=0)

        k_left = left_avg[0]
        n_left = left_avg[1]
        k_right = right_avg[0]
        n_right = right_avg[1]

        y1 = np.shape(image)[0]
        y2 = int(0.4 * np.shape(image)[0])

        x1 = int((y1 - n_left) / k_left)
        x2 = int((y2 - n_left) / k_left)
        x3 = int((y1 - n_right) / k_right)
        x4 = int((y2 - n_right) / k_right)

        average_lines = np.array([[x1, y1, x2, y2], [x3, y1, x4, y2]])

        line_image = np.zeros((M, N, 3), dtype='uint8')
        #io.imshow(line_image)

        if lines is not None:
            for line in average_lines:
                x1_temp, y1_temp, x2_temp, y2_temp = np.array(line).reshape(4)
                x1_temp = int(x1_temp)
                y1_temp = int(y1_temp)
                x2_temp = int(x2_temp)
                y2_temp = int(y2_temp)
                cv2.line(line_image, (x1_temp, y1_temp), (x2_temp, y2_temp), (0, 255, 0), 5)

        x1_center, y1_center = int((x1 + x3) / 2), int(y1)
        x2_center, y2_center = int((x2 + x4) / 2), int(y2)
        cv2.line(line_image, (x1_center, y1_center), (x2_center, y2_center), (255, 0, 0), 2)

        line_image = cv2.circle(line_image, (N // 2, M - 1), 1, (0, 0, 255), 5)

        image_with_lines = cv2.addWeighted(frame, 0.5, line_image, 1, 1)

        #io.imshow(image_with_lines)
        #plt.title("Road line")
        #plt.show()

        e = int(N / 2) - (x1 + x3) / 2  # Error signal
        return e

    def generate_steering(image, pid_controller):
        e = line_detection(image)  # current error for tracking reference
        steering_value = pid_controller.compute(e)
        steering_value = max(-20, min(steering_value, 20))
        #print(f"Error: {e}")
        #print(f"Steering value: {steering_value}")
        logger.warning(f"Error: {e}")
        logger.warning(f"Steering value: {steering_value}")
        return

    allProcesses = list()

    debugg = True

    queueList = {
        "Critical": Queue(),
        "Warning": Queue(),
        "General": Queue(),
        "Config": Queue(),
    }

    logger = logging.getLogger()

    if debugg:
        logger.warning("getting")
    img = {"msgValue": 1}
    while type(img["msgValue"]) != type(":text"):
        img = queueList["General"].get()
    image_data = base64.b64decode(img["msgValue"])
    img = np.frombuffer(image_data, dtype=np.uint8)
    image = cv2.imdecode(img, cv2.IMREAD_COLOR)
    if debugg:
        logger.warning("got")

    # One frame from camera
    frame = image

    # Error signal
    e = line_detection(frame)
    # print(f"Error: {e}")

    r = 0  # Reference of x coordinates difference

    # Instance of a controler
    pid_controller = PIDController(Kp=0.1, Ki=0.01, Kd=0.1, setpoint=r)

    tic = time.time()
    steering_value = generate_steering(frame, pid_controller)
    toc = time.time()

    time = toc - tic
    logger.warning(f"Execution time: {time} s")

    queueList["General"].put(
        {
            "Owner": "processLaneAssistance",
            "msgID": 3,
            "msgType": "dictionary",
            "msgValue": {"action": "steer", "value" : steering_value},
        })