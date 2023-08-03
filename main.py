from threading import Thread, Event
from time import sleep

from pid import PID
from video import Video
from bluerov_interface import BlueROV
from pymavlink import mavutil
from dt_apriltags import Detector
import numpy as np
import cv2

# TODO: import your processing functions
from apriltags import getTag, getTagCenter, sizeF, getTagDistance, getTagAngles
from lane_detection import detect_lanes, detect_lines
from lane_following import recommend_angle, get_lane_center

cameraMatrix = np.array([1060.71, 0, 960, 0, 1060.71, 540, 0, 0, 1]).reshape((3, 3))
camera_params = (cameraMatrix[0, 0], cameraMatrix[1, 1], cameraMatrix[0, 2], cameraMatrix[1, 2])
at_detector = Detector(families='tag36h11',
                        nthreads=1,
                        quad_decimate=1.0,
                        quad_sigma=0.0,
                        refine_edges=1,
                        decode_sharpening=0.25,
                        debug=0)


# Create the video object
video = Video()
# Create the PID object
pid_vertical = PID(K_p=1, K_i=0.0, K_d=-0.7, integral_limit=1)
pid_horizontal = PID(K_p=0.7, K_i=0.0, K_d=-0.4, integral_limit=1)
pid_forward = PID(K_p=-40, K_i=0.0, K_d=-30, integral_limit=1)
pid_turn = PID(K_p=-0.85, K_i=0.0, K_d=-0.2, integral_limit=1)
# Create the mavlink connection
mav_comn = mavutil.mavlink_connection("udpin:0.0.0.0:14550")
# Create the BlueROV object
bluerov = BlueROV(mav_connection=mav_comn)

frame = None
frame_available = Event()
frame_available.set()

vertical_power = 0
lateral_power = 0
longitudinal_power = 0
yaw_power = 0


def _get_frame():
    global vertical_power, lateral_power, longitudinal_power, yaw_power
    global frame
    while not video.frame_available():
        print("Waiting for frame...")
        sleep(0.01)

    try:
        while True:
            if video.frame_available():
                frame = video.frame()
                # TODO: Add frame processing here
                
                # cv2.imwrite('feed.jpg', frame)

                middle_x, middle_y, width, height = sizeF(frame)
                tags = getTag(frame)
                detected_tags = getTagCenter(tags)

                if len(detected_tags) != 0:
                    tagX, tagY = detected_tags[0]
                    print(detected_tags)

                    distance = getTagDistance(tags)
                    print(distance)

                    tagAngles = getTagAngles(tags)
                    print(tagAngles)

                    # Calculate percent error from the desired middle coordinates
                    error_y = round((middle_y - tagY)/height * 100, 6)
                    error_x = round((middle_x - tagX)/width * 100, 6)
                    # error_z = distance[2][0]
                    error_z = round(0 - distance[2][0], 6)
                    error_A = round(0 - tagAngles[0], 6)

                    print(f"Error X: {error_x}%")
                    print(f"Error Y: {error_y}%")
                    print(f"Error Z: {error_z}")
                    print(f"Error A: {error_A}")

                    # TODO: set vertical_power and lateral_power here
                    # Update the PID controllers and get the output
                    vertical_power = int(pid_vertical.update(error_y)) * -1
                    lateral_power = int(pid_horizontal.update(error_x))
                    longitudinal_power = int(pid_forward.update(error_z))
                    yaw_power = int(pid_turn.update(error_A))

                    print("Output X:", lateral_power)
                    print("Output Y:", vertical_power)
                    print("Output Z:", longitudinal_power)
                    print("Output A", yaw_power)



                else:
                    detectedLanes = detect_lanes(frame, detect_lines(frame, 50, 70, 3, 200, 10))
                    if len(detectedLanes) != 0:
                        print("LANE DETECTED!!!")
                        slope = get_lane_center(frame, detectedLanes)[0]
                        intercept = get_lane_center(frame, detectedLanes)[1]
                        angle = recommend_angle(slope)

                        if not intercept >= (middle_x - 20) and not intercept <= (middle_x + 20):
                            error_LX = round(intercept - middle_x, 6)
                            print(f"Error LX: {error_LX}")
                            longitudinal_power = int(pid_forward.update(error_LX))
                            print(f"Output LX", longitudinal_power)

                        # LA = Lane Angle
                        if not angle == 90:
                            error_LA = np.rad2deg(round(angle, 6))
                            # error_LA = np.rad2deg(round(90 - angle, 6))
                            print(f"Error LA: {error_LA} deg")
                            yaw_power = int(pid_turn.update(error_LA))
                            print("Output LA", yaw_power)

                        if intercept >= (middle_x - 30) and intercept <= (middle_x + 30) and abs(angle) >= 85 and abs(angle) <= 95:
                            longitudinal_power = 80
                            



                    else:
                        vertical_power = 0
                        lateral_power = 0
                        longitudinal_power = 0
                        yaw_power = 0
                        print("Error Y: No tag or line detected.")
                        print("Error X: No tag or line detected.")
                        print("Error Z: No tag or line detected.")
                        print("Error A: No tag or line detected.")


                
                # else statement for only apriltags
                # else:
                #     vertical_power = 0
                #     lateral_power = 0
                #     longitudinal_power = 0
                #     yaw_power = 0
                #     print("Error Y: No tag detected.")
                #     print("Error X: No tag detected.")
                #     print("Error Z: No tag detected.")
                #     print("Error A: No tag detected.")


                print(frame.shape)
    except KeyboardInterrupt:
        return


def _send_rc():
    global vertical_power, lateral_power, longitudinal_power, yaw_power
    while True:
        # mav_comn.wait_heartbeat()
        bluerov.arm()
        bluerov.set_vertical_power(int(vertical_power))
        bluerov.set_lateral_power(-int(lateral_power))
        bluerov.set_longitudinal_power(int(longitudinal_power))
        bluerov.set_yaw_rate_power(int(yaw_power))


# Start the video thread
video_thread = Thread(target=_get_frame)
video_thread.start()

# Start the RC thread
rc_thread = Thread(target=_send_rc)
rc_thread.start()

# Main loop
try:
    while True:
        mav_comn.wait_heartbeat()
except KeyboardInterrupt:
    video_thread.join()
    rc_thread.join()
    bluerov.disarm()
    print("Exiting...")
