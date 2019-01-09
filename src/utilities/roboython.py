import math
import matplotlib.pyplot as plt

corner_angle = math.pi/2

thigh_along = 309.75
thigh_across = 7.46

leg_along = 347.50
leg_across = 7.0

foot_along = 110.0
foot_across = 47.0

pedal_offset = 11.88
pedal_radius = 169.24

def compute_leg_length(hip_angle, knee_angle, ankle_angle):
    x_comp = math.sin(hip_angle)*thigh_along + math.sin(hip_angle-corner_angle)*thigh_across #Thigh computed
    x_comp += math.sin(hip_angle-knee_angle)*leg_along + math.sin(hip_angle-knee_angle-corner_angle)*leg_across #leg computed
    x_comp += math.sin(hip_angle - knee_angle + corner_angle + ankle_angle) * foot_along + math.sin(
        hip_angle - knee_angle + corner_angle + ankle_angle - corner_angle) * (foot_across + pedal_offset)  # foot computed

    y_comp = math.cos(hip_angle) * thigh_along + math.cos(hip_angle - corner_angle) * thigh_across  # Thigh computed
    y_comp += math.cos(hip_angle - knee_angle) * leg_along + math.cos(
        hip_angle - knee_angle - corner_angle) * leg_across  # leg computed
    y_comp += math.cos(hip_angle - knee_angle + corner_angle + ankle_angle) * foot_along + math.cos(
        hip_angle - knee_angle + corner_angle + ankle_angle - corner_angle) * (
                          foot_across + pedal_offset)  # foot computed

    length = math.sqrt(x_comp*x_comp + y_comp*y_comp)

    return length

maximum_length = compute_leg_length(0, 0, -(math.pi/4))
print("Maximum leg length:", maximum_length)
minimum_length = compute_leg_length(math.pi/2, 2*math.pi/3, (math.pi/4))
print("Minimum leg length:", minimum_length)
length_difference = maximum_length - minimum_length
print("Leg length difference:", length_difference)

def get_pedal_position_x(pedal_angle):
    return pedal_radius*math.cos(pedal_angle)

def get_pedal_position_y(pedal_angle):
    return pedal_radius*math.sin(pedal_angle)

def compute_distance(x1, y1, x2, y2):
    delta_x = x2-x1
    delta_y = y2-y1
    return math.sqrt(delta_x*delta_x + delta_y*delta_y)

candidate_x_list = []
candidate_y_list = []

for point_iter in range(200,1000,1):
    candidate_x_list.append(point_iter)
    candidate_y_list.append(point_iter)

confirmed_hip_positions = [[]]
confirmed_x_positions = []
confirmed_y_positions = []

N_ANGLES_CHECKED = 36

for candidate_x in candidate_x_list:
    for candidate_y in candidate_y_list:
        discarded = 0
        for pedal_angle_iter in range(N_ANGLES_CHECKED):
            pedal_x = get_pedal_position_x(pedal_angle_iter*math.pi*2/N_ANGLES_CHECKED)
            pedal_y = get_pedal_position_y(pedal_angle_iter * math.pi * 2 / N_ANGLES_CHECKED)
            this_distance = compute_distance(pedal_x, pedal_y, candidate_x, candidate_y)
            if this_distance > maximum_length or this_distance < minimum_length:
                discarded = 1
                break

        if not discarded:
            confirmed_hip_positions.append([candidate_x, candidate_y])
            confirmed_x_positions.append(candidate_x)
            confirmed_y_positions.append(candidate_y)

plt.plot(confirmed_x_positions, confirmed_y_positions, 'bs')
plt.show()

#print("Confirmed hip positions:", confirmed_hip_positions)
print("Computation complete.")
