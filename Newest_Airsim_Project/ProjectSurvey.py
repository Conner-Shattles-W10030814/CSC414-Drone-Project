import setup_path
import airsim
from airsim.types import DrivetrainType, Vector3r, YawMode
import time
import numpy as np
import os
import tempfile
import pprint
import cv2
import math

class ProjectSurvey:
    def __init__(self, length, width, height):
        #temporary ish
        self.x = 509
        self.y = 5
        self.z = 458
        self.fov = math.pi / 2
        self.survey_altitude = height
        self.sum_of_offsets = -0.0292
        self.camera_height = self.survey_altitude + self.sum_of_offsets
        self.velocity = 2.5
        self.survey_length = length
        self.survey_width = width
        # connect to the AirSim simulator
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)

    def print_state(self):
        state = self.client.getMultirotorState()
        s = pprint.pformat(state)
        print("state: %s" % s)

        imu_data = self.client.getImuData()
        s = pprint.pformat(imu_data)
        print("imu_data: %s" % s)

        barometer_data = self.client.getBarometerData()
        s = pprint.pformat(barometer_data)
        print("barometer_data: %s" % s)

        magnetometer_data = self.client.getMagnetometerData()
        s = pprint.pformat(magnetometer_data)
        print("magnetometer_data: %s" % s)

        gps_data = self.client.getGpsData()
        s = pprint.pformat(gps_data)
        print("gps_data: %s" % s)

    def start(self):
        airsim.wait_key('Press any key to takeoff')
        self.takeoff()

    def takeoff(self):
        self.client.armDisarm(True)
        self.client.takeoffAsync().join()
        #enter hover state
        self.hover()
    
    def hover(self):
        self.client.hoverAsync().join()
        airsim.wait_key('Press any key to begin survey')
        self.survey()

    def lift_to_survey_altitude(self):
        self.goto(self.x, self.survey_altitude, self.z, self.velocity)
    
    def goto(self, x, y, z, speed):
        self.client.moveToPositionAsync(z, x, -y, speed).join()
        self.x = x
        self.y = y
        self.z = z

    def get_camera_width(self):
        return 2 * self.camera_height * math.tan(self.fov / 2)
    def get_camera_length(self):
        return self.get_camera_width() / math.cos(self.fov / 2)
    

    def survey(self):
        print("Area of image is " + str(self.get_camera_length() * self.get_camera_width()))
        print("Area in pixels: " + str(256 / 144))
        print("cm / px: " + str((self.get_camera_length() * self.get_camera_width()) * 100 / (256 * 144)))
        #Go to desired altitude:
        self.lift_to_survey_altitude()
        time.sleep(1)
        path = []
        distance = 0
        boxlength = self.x + self.survey_length #118
        boxwidth = self.z + self.survey_width #53
        camera_width = self.get_camera_width()
        camera_width = 0.30 * camera_width
        camera_length = self.get_camera_length()
        camera_length = 0.40 * camera_length
        #strip length is not dependent on length of the box or camera
        striplength = boxwidth - self.z
        stripewidth = camera_length
        bottomz = self.z
        topz = self.z + striplength
        leftx = self.x
        rightx = boxlength
        #append first vertex
        path.append(Vector3r(self.z, self.x, -self.y))
        #TEMPORARY
        while 1:
            #z positive
            while self.z < topz: #temporary:::
                if self.z + camera_width < topz or 1:
                    distance += camera_width
                    self.z += camera_width
                    path.append(Vector3r(self.z, self.x, -self.y))
                else:
                    distance += topz - self.z
                    self.z = topz
                    path.append(Vector3r(self.z, self.x, -self.y))
            #CHECK X
            if self.x >= boxlength:
                break
            else:
                distance += camera_length
                self.x += camera_length
                path.append(Vector3r(self.z, self.x, -self.y))
            #z negative
            while self.z > bottomz:
                if self.z - camera_width > bottomz or 1:
                    distance += camera_width
                    self.z -= camera_width
                    path.append(Vector3r(self.z, self.x, -self.y))
                else:
                    distance += self.z - bottomz
                    self.z = bottomz
                    path.append(Vector3r(self.z, self.x, -self.y))
            #CHECK X
            if self.x >= boxlength:
                break
            else:
                distance += camera_length
                self.x += camera_length
                path.append(Vector3r(self.z, self.x, -self.y))

        pic_index = 0
        for vertex in path:
            self.client.moveToPositionAsync(vertex.x_val, vertex.y_val, vertex.z_val, self.velocity).join()
            time.sleep(2)
            self.get_single_pic(str(pic_index))
            pic_index += 1
        self.await_termination()





    """
    def survey(self):
        #Go to desired altitude:
        self.lift_to_survey_altitude()
        path = []
        distance = 0
        boxlength = self.x + 55
        boxwidth = self.z + 35
        camera_width = self.get_camera_width()
        camera_length = self.get_camera_length()
        #strip length is not dependent on length of the box or camera
        striplength = boxwidth - self.z
        stripewidth = camera_length
        print(f"REPORT:\nCamera width: {camera_width}\nCamera length: {camera_length}\nstrip_length: {striplength}\nstrip_width: {stripewidth}\n")
        while self.x < boxlength:
            distance += striplength
            self.z += striplength
            path.append(Vector3r(self.z, self.x, -self.y))
            self.x += stripewidth
            distance += stripewidth
            path.append(Vector3r(self.z, self.x, -self.y))
            distance += striplength
            self.z -= striplength
            path.append(Vector3r(self.z, self.x, -self.y))
            self.x += stripewidth
            distance += stripewidth
            path.append(Vector3r(self.z, self.x, -self.y))
        trip_time = distance / self.velocity
        lookahead = self.velocity + (self.velocity / 2)
        result = self.client.moveOnPathAsync(path, self.velocity, trip_time, DrivetrainType.ForwardOnly, YawMode(False,0), lookahead, 1)
        self.image_loop(trip_time, camera_width / self.velocity)
        self.await_termination()
    

    def image_loop(self, trip_time, time_interval):
        endtime = time.time() + trip_time
        targetPic = time.time() + time_interval
        picture_index = 0
        starting_pic = True
        while(time.time() < endtime):
            if time.time() >targetPic or starting_pic:
                if starting_pic:
                    starting_pic = False
                targetPic = time.time() + time_interval
                responses = self.client.simGetImages([
                airsim.ImageRequest("bottom", airsim.ImageType.Scene, False, True)])

                print('Retrieved images: %d' % len(responses))    

                tmp_dir = os.path.join(tempfile.gettempdir(), "airsim_drone")
                print ("Saving images to %s" % tmp_dir)
                try:
                    os.makedirs(tmp_dir)
                except OSError:
                    if not os.path.isdir(tmp_dir):
                        raise
                ammendment = str("#" + str(picture_index))
                for idx, response in enumerate(responses):

                    filename = os.path.join(tmp_dir, str(idx) + "_" + ammendment)

                    if response.pixels_as_float:
                        print("Type %d, size %d" % (response.image_type, len(response.image_data_float)))
                        airsim.write_pfm(os.path.normpath(filename + '.pfm'), airsim.get_pfm_array(response))
                    elif response.compress: #png format
                        print("Type %d, size %d" % (response.image_type, len(response.image_data_uint8)))
                        airsim.write_file(os.path.normpath(filename + '.png'), response.image_data_uint8)
                    else: #uncompressed array
                        print("Type %d, size %d" % (response.image_type, len(response.image_data_uint8)))
                        img1d = np.fromstring(response.image_data_uint8, dtype=np.uint8) # get numpy array
                        img_rgb = img1d.reshape(response.height, response.width, 3) # reshape array to 4 channel image array H X W X 3
                        cv2.imwrite(os.path.normpath(filename + '.png'), img_rgb) # write to png
                    picture_index += 1
                    ammendment = str("#" + str(picture_index))
    """
    def get_single_pic(self, pic_name):
        responses = self.client.simGetImages([
        airsim.ImageRequest("bottom", airsim.ImageType.Scene, False, True)])

        print('Retrieved images: %d' % len(responses))    

        tmp_dir = os.path.join(tempfile.gettempdir(), "airsim_drone")
        print ("Saving images to %s" % tmp_dir)
        try:
            os.makedirs(tmp_dir)
        except OSError:
            if not os.path.isdir(tmp_dir):
                raise
        for idx, response in enumerate(responses):

            filename = os.path.join(tmp_dir, str(idx) + "_" + pic_name)

            if response.pixels_as_float:
                print("Type %d, size %d" % (response.image_type, len(response.image_data_float)))
                airsim.write_pfm(os.path.normpath(filename + '.pfm'), airsim.get_pfm_array(response))
            elif response.compress: #png format
                print("Type %d, size %d" % (response.image_type, len(response.image_data_uint8)))
                airsim.write_file(os.path.normpath(filename + '.png'), response.image_data_uint8)
            else: #uncompressed array
                print("Type %d, size %d" % (response.image_type, len(response.image_data_uint8)))
                img1d = np.fromstring(response.image_data_uint8, dtype=np.uint8) # get numpy array
                img_rgb = img1d.reshape(response.height, response.width, 3) # reshape array to 4 channel image array H X W X 3
                cv2.imwrite(os.path.normpath(filename + '.png'), img_rgb) # write to png



    def await_termination(self):
        airsim.wait_key('Press any key to reset to original state')
        self.terminate()
    def terminate(self):
        self.client.reset()
        self.client.armDisarm(False)
        self.client.enableApiControl(False)


if __name__ == "__main__":
    survey = ProjectSurvey(118, 53, 18)
    survey.start()