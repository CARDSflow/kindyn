#!/usr/bin/python

from scipy import interpolate
import matplotlib.pyplot as plt
import json
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from scipy.misc import derivative

RECORDED_TRAJECTORY_FILENAME = "capture_trajectory/captured_trajectory.json"

PRINT_DEBUG = False

numTrajectoryPoints = 0
trajectoryStartingPoint = 0

pedalTrajectoryRight = [ ]
pedalAngleTrajectoryRight = []
pedalTrajectoryLeft = [ ]
hipTrajectoryRight = [ ]
kneeTrajectoryRight = [ ]
ankleTrajectoryRight = [ ]
hipTrajectoryLeft = [ ]
kneeTrajectoryLeft = [ ]
ankleTrajectoryLeft = [ ]




def importJointTrajectoryRecord():
    global number_imported_trajectory_points
    global pedalTrajectoryLeft
    global pedalTrajectoryRight
    global hipTrajectoryRight
    global kneeTrajectoryRight
    global ankleTrajectoryRight
    global hipTrajectoryLeft
    global kneeTrajectoryLeft
    global ankleTrajectoryLeft
    global PRINT_DEBUG

    with open(RECORDED_TRAJECTORY_FILENAME, "r") as read_file:
        loaded_data = json.load(read_file)

    if loaded_data[ "num_points" ] is None:
        return 0
    else:
        number_imported_trajectory_points = loaded_data[ "num_points" ]

    # Deleting previous trajectory before loading new
    del pedalTrajectoryLeft[ : ]
    del pedalTrajectoryRight[ : ]
    del pedalAngleTrajectoryRight[ : ]
    del hipTrajectoryRight[ : ]
    del kneeTrajectoryRight[ : ]
    del ankleTrajectoryRight[ : ]
    del hipTrajectoryLeft[ : ]
    del kneeTrajectoryLeft[ : ]
    del ankleTrajectoryLeft[ : ]
    for pointIterator in range(number_imported_trajectory_points):
        if "point_" + str(pointIterator) in loaded_data:
            pedalTrajectoryLeft.append(loaded_data[ "point_" + str(pointIterator) ][ "Left" ][ "Pedal" ])
            pedalTrajectoryRight.append(loaded_data[ "point_" + str(pointIterator) ][ "Right" ][ "Pedal" ])
            pedalAngleTrajectoryRight.append(loaded_data["point_"+str(pointIterator)]["Right"]["Pedal_angle"])
            hipTrajectoryRight.append(loaded_data[ "point_" + str(pointIterator) ][ "Right" ][ "Hip" ])
            kneeTrajectoryRight.append(loaded_data[ "point_" + str(pointIterator) ][ "Right" ][ "Knee" ])
            ankleTrajectoryRight.append(loaded_data[ "point_" + str(pointIterator) ][ "Right" ][ "Ankle" ])
            hipTrajectoryLeft.append(loaded_data[ "point_" + str(pointIterator) ][ "Left" ][ "Hip" ])
            kneeTrajectoryLeft.append(loaded_data[ "point_" + str(pointIterator) ][ "Left" ][ "Knee" ])
            ankleTrajectoryLeft.append(loaded_data[ "point_" + str(pointIterator) ][ "Left" ][ "Ankle" ])
        else:
            print("WARNING: No point_%s in trajectory" % pointIterator)
            number_imported_trajectory_points -= 1

    if PRINT_DEBUG:
        print("--------- Num trajectory points:")
        print(number_imported_trajectory_points)


if __name__ == '__main__':
    

    
    importJointTrajectoryRecord()
    
    pedalTrajectory = pedalTrajectoryRight
    hipTrajectory = hipTrajectoryRight	

    x = []
    y = []
    z = []
    x_new = []
    y_new = []
    z_new = []
    a = range(0, 341, 10)

    for i in range(0, len(pedalTrajectory)-1, 2):
        x.append(pedalTrajectory[i][0])
        y.append(pedalTrajectory[i][1])
        z.append(hipTrajectory[i])
        x_new.append(pedalTrajectory[i+1][0])
        y_new.append(pedalTrajectory[i+1][1])
        z_new.append(hipTrajectory[i+1])

    x.append(x[0])
    y.append(y[0])
    z.append(z[0])
    x_new.append(x_new[0])
    y_new.append(y_new[0])
    z_new.append(z_new[0])

    print len(a), len(hipTrajectory)

    f = interpolate.interp1d(a, hipTrajectory, kind = "cubic")
    f1 = interpolate.interp2d(x, y, z, kind='cubic')
    f2 = interpolate.interp2d(x, y, z, kind='linear')
    #f3 = interpolate.interp2d(x, y, z, kind='quintic')

    print("d = ",derivative(f, 50))

    interp1 = []
    interp2 = []
    interp3 = []
    for i in range(len(x_new)):
        interp1.append(f1(x_new[i], y_new[i]))
        interp2.append(f2(x_new[ i ], y_new[ i ]))
        #interp3.append(f3(x_new[ i ], y_new[ i ]))


    a_new = np.arange(0,340, 0.05)
    some_value = f(a_new)

    temp1 = []
    temp2 = []
    temp3 = []
    for e in interp1:
        temp1.append(e[0 ])
    for e in interp2:
        temp2.append(e[0 ])
    for e in interp3:
        temp3.append(e[0 ])

    mean_error1 = 0
    max_error1 = 0
    mean_error2 = 0
    max_error2 = 0
    mean_error3 = 0
    max_error3 = 0

    for i in range(len(x_new)):
        current_error = np.abs(temp1[i ] - z_new[i ])
        mean_error1+=current_error
        if current_error>max_error1:
            max_error1 = current_error

    for i in range(len(x_new)):
        current_error = np.abs(temp2[i ] - z_new[i ])
        mean_error2+=current_error
        if current_error>max_error2:
            max_error2 = current_error

    #for i in range(len(x_new)):
    #    current_error = np.abs(temp3[i ] - z_new[i ])
    #    mean_error3+=current_error
    #    if current_error>max_error3:
    #        max_error3 = current_error

    print("Mean error cubic = ", mean_error1 / len(x_new))
    print("Max error cubic = ", max_error1)
    print("Mean error linear = ", mean_error2 / len(x_new))
    print("Max error linear = ", max_error2)
    print("Mean error quintic = ", mean_error3 / len(x_new))
    print("Max error quintic = ", max_error3)

    #ax.plot(x_new, y_new, temp1, label = "cubic interpolation")
    #ax.plot(x_new, y_new, temp2, label = "linear interpolation")
    #ax.plot(x_new, y_new, temp3, label = "quintic")
    #ax.plot(x_new, y_new, z_new, label="actual values")
    plt.plot(a, hipTrajectory, "o")
    plt.plot(a_new, some_value, "-")
    #ax.legend()
    plt.show()
