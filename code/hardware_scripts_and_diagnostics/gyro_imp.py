# import library
from mpu6050 import mpu6050
import math

# init sensor
sensor = mpu6050(0x68)

# def functions

# function to get acceleration data
def get_sensor_accel():
    try:
        # get data
        print("Getting acceleration data...")
        accel = sensor.get_accel_data() # get the raw data
        
        print("Data is : " + str(accel))
        
        return accel
    except Exception as e:
        print("Error getting acceleration data : ", e)
        return None


# function to get gyro data
def get_sensor_gyro():
    try:
        # get data
        print("Getting gyroscope data...")
        gyro = sensor.get_gyro_data() # get the raw data
        
        print("Data is : " + str(gyro))
        
        return gyro
    except Exception as e:
        print("Error getting gyro data : ", e)
        return None


# function to get pitch and roll
def get_pitch_roll():
    
    # get sensor data
    accel = get_sensor_accel()
    
    # devide sensor values
    ax = accel['x']
    ay = accel['y']
    az = accel['z']

    # calculate pitch
    pitch = math.atan2(ax, math.sqrt(ay * ay + az * az)) * 180 / math.pi

    # calculate roll
    roll = math.atan2(ay, math.sqrt(ax * ax + az * az)) * 180 / math.pi

    # return values
    return {
        "pitch": pitch,
        "roll": roll
    }