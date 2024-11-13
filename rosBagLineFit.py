import argparse
import cv2
import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import os

def extract_video(bag_path="./chunk_0001.bag", topic_name="/zed2/zed_node/rgb/image_rect_color", video_path="./extracted_video.mp4", first_image_path="output_img.jpg", output_dir="./images", width=1280, height=720):
    print('Opening bag')
    bag = rosbag.Bag(bag_path, "r")

    bridge = CvBridge()

    # Video writer setup
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    video_writer = cv2.VideoWriter(video_path, fourcc, 30.0, (width, height), isColor=True)  # Adjust the resolution if needed

    print('Reading video frames and saving to video file')
    frame_count = 0

    first_image_saved = False

    for topic, msg, stamp in bag.read_messages(topics=[topic_name]):
        if msg._type == 'sensor_msgs/Image':
            frame_count += 1
            # Convert ROS Image message to OpenCV image
            cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            if (cv_image.shape[1], cv_image.shape[0]) != (width, height):
                cv_image = cv2.resize(cv_image, (width, height))

            # Save the first image separately
            if not first_image_saved:
                cv2.imwrite(first_image_path, cv_image)
                first_image_saved = True

            # Write the frame to the video file
            video_writer.write(cv_image)


    bag.close()
    video_writer.release()
    print('Done. %d video frames written to %s' % (frame_count, video_path))

def extract_video_frames_to_images(bag_path="./chunk_0001.bag", topic_name="/zed2/zed_node/rgb/image_rect_color", output_dir="./images", width=1280, height=720):
    print('Opening bag')
    bag = rosbag.Bag(bag_path, "r")
    bridge = CvBridge()

   
    print('Reading video frames and saving to images folder')
    frame_count = 0

    first_image_saved = False

    for topic, msg, stamp in bag.read_messages(topics=[topic_name]):
        if msg._type == 'sensor_msgs/Image':

            cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

            cv2.imwrite(os.path.join(output_dir, "frame%06i.png" % frame_count), cv_img)
            print ("Wrote image %i" % frame_count)

            frame_count += 1

    bag.close()


def images_to_video(image_folder, output_video_path, frame_rate=30, width=1280, height=720):
    # Get a sorted list of image filenames
    image_files = sorted([f for f in os.listdir(image_folder) if f.endswith('.png')])

    if not image_files:
        print("No PNG images found in the folder.")
        return

    # Define the codec and create a VideoWriter object
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    video_writer = cv2.VideoWriter(output_video_path, fourcc, frame_rate, (width, height))

    for i, image_file in enumerate(image_files):
        image_path = os.path.join(image_folder, image_file)
        img = cv2.imread(image_path)

        if img is None:
            print(f"Warning: {image_path} could not be read.")
            continue

        # Resize image if necessary
        img_resized = cv2.resize(img, (width, height))

        # Write the frame to the video
        video_writer.write(img_resized)
        print(f"Frame {i + 1}/{len(image_files)} written: {image_file}")

    video_writer.release()
    print(f"Video successfully created: {output_video_path}")

# Set parameters
image_folder = './images'  # Path to the folder with images
output_video_path = './output_video_from_images.mp4'  # Path to save the video

# Create the video from images in two steps
# extract_video_frames_to_images()
# images_to_video(image_folder, output_video_path)

## create video directly
extract_video()
