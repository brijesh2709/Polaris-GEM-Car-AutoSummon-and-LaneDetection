#!/usr/bin/env python3
from flask import Flask, request, jsonify
import rospy
from std_msgs.msg import Bool
from gem_pure_pursuit_sim.msg import GpsCoordinates
import json
from flask_cors import CORS

app = Flask(__name__)
car_reached = False

CORS(app)

def auto_summon_pos_reached_callback(msg):
    global car_reached 
    car_reached = msg.data


# ROS Publisher setup
rospy.init_node('gps_publisher_node', anonymous=True)
publisher = rospy.Publisher('/auto_summon_gps_coords', GpsCoordinates, queue_size=1)
subscriber = rospy.Subscriber('/auto_summon_pos_reached', Bool, auto_summon_pos_reached_callback)


@app.route('/api/publish', methods=['POST'])
def publish_coordinates():
    try:
        print("hit api")
        data = request.get_json()
        latitude = data.get('latitude')
        longitude = data.get('longitude')

        if latitude is None or longitude is None:
            return jsonify({'error': 'Invalid coordinates'}), 400
        print(data)
        gps_msg = GpsCoordinates()
        gps_msg.latitude = latitude
        gps_msg.longitude = longitude
        publisher.publish(gps_msg)

        return jsonify({'message': 'Coordinates published successfully!'}), 200
    except Exception as e:
        rospy.logerr(f"Failed to publish: {str(e)}")
        return jsonify({'error': str(e)}), 500


@app.route('/api/reached', methods=['GET'])
def poll_car_reached():
    try:
        return jsonify(car_reached), 200
    except Exception as e:
        rospy.logerr(f"Failed to get car reached value: {str(e)}")
        return jsonify({'error': str(e)}), 500

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=True)
