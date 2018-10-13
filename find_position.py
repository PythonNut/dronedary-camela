import numpy as np
import math
import itertools
import sys
import socket
import selectors
import types
import time
import threading
import queue
import PID
from control import DroneController


def calculate_centroid(p, q, r):
    return np.mean(np.array([p,q,r]), axis=2)

def find_a_and_b(p, q, r, cent):
    B = np.amax(np.array([p,q,r]),axis=0)[0] - cent[0]
    A = cent[0] - np.amin(np.array([p,q,r]),axis=0)[0]
    return (A,B)

def find_a(A,B):
    return A/(A+B)

def find_inner_d(a,s):
    return s * math.sqrt(a*a - a + 1/3)


def find_angle_c(a):
    return math.asin(1/(math.sqrt(3)*2*math.sqrt(a*a - a + 1/3)))

def find_alpha(HFOV, HPIX, A):
    return HFOV * A / HPIX

def find_w(angle_c, s):
    return s * math.cos(math.pi/2 - angle_c)

def find_outer_d(w,alpha,a):
    return w*a/math.tan(alpha)

def is_point_front(p,q,r, cent):
    if p[0] > q[0] and p[0] > r[0]:
        if q[0] > r[0]:
            return q[1] > cent[1]
        return r[1] > cent[1]
    if q[0] > p[0] and q[0] > r[0]:
        if p[0] > r[0]:
            return p[1] > cent[1]
        return r[1] > cent[1]
    else:
        if p[0] > q[0]:
            return p[1] > cent[1]
        return q[1] > cent[1]

def find_d(d_in,d_out,pointy_front):
    if pointy_front:
        d = d_out - d_in
    else:
        d = d_out + d_in
    return d


def find_theta(angle_c, A, B, cam_const):
    out = math.pi/2 - angle_c
    if B > A:
        out *= -1
    out += cam_const
    return out

def find_k(drone, cent):
    return drone[0] - cent[0]

def find_angle_k(k, HFOV, HPIX):
    return k * HFOV / HPIX

def find_phi(theta, angle_k):
    if angle_k > 0:
        return theta + math.pi/2 - angle_k
    else:
        return theta - math.pi/2 - angle_k

def find_r(d, angle_k):
    return d * math.sin(angle_k)

def find_d_prime(d, theta, drone_pos):
    return math.sqrt((d*math.sin(theta)-drone_pos[1])**2 + (d*math.cos(theta)-drone_pos[0])**2)

def find_P_Q_M_N(p,q,r):
    thing = np.array([p,q,r])
    thing_0 = thing[thing[:,0].argsort()]
    thing_1 = thing[thing[:,1].argsort()]
    P = thing_0[1][0] - thing_0[0][0]
    Q = thing_0[2][0] - thing_0[1][0]
    M = thing_1[1][1] - thing_1[0][1]
    N = thing_1[2][1] - thing_1[1][1]
    return (P,Q,M,N)

def find_h(d,P,Q,M,N):
    return d * math.sqrt((M*M + N*N + (M+N)**2)/(A*A + B*B + (A+B)**2 -(M*M + N*N + (M+N)**2)))

def find_angle_4(h,d):
    return math.atan(h/d)

def find_Y(drone, cent):
    return cent[1] - drone[1]

def find_angle_5(Y, VFOV, VPIX):
    return Y * VFOV / VPIX

def find_h_prime(d_prime, angle_6):
    return d_prime * math.tan(angle_6)

def get_x_y_z(drone, p, q, r):
    """
    all parameters are np arrays of x,y points
    eg: [[x1,y1],[x2,y2]]
    """
    num_cameras = 2
    camera_constants = [0,math.pi/2]
    rads = np.zeros(num_cameras)
    phis = np.zeros(num_cameras)
    d = np.zeros(num_cameras)
    theta = np.zeros(num_cameras)
    Hs = np.zeros(num_cameras)
    s = 12
    HFOV = math.pi/4
    VFOV = 5*math.pi/36
    HPIX = 1280
    VPIX = 720
    #loop one, where we increment over camera number, and
    # get new information

    cent = calculate_centroid(p,q,r)
    for camera_num in range(num_cameras):

        A,B = find_a_and_b(p[camera_num],q[camera_num],r[camera_num],cent[camera_num])
        a = find_a(A,B)
        d_in = find_inner_d(a, s)
        angle_c = find_angle_c(a)
        alpha = find_alpha(HFOV, HPIX, A)
        w = find_w(angle_c, s)
        d_out = find_outer_d(w,alpha,a)
        pointy_front = is_point_front(r[camera_num],q[camera_num],p[camera_num],cent[camera_num])
        d[camera_num] = find_d(d_in,d_out,pointy_front)
        theta[camera_num] = find_theta(angle_c,A,B,camera_constants[camera_num])
        k = find_k(drone[camera_num], cent[camera_num])
        angle_k = find_angle_k(k, HFOV, HPIX)
        phi = find_phi(theta[camera_num], angle_k)
        rad = find_r(d[camera_num], angle_k)
        phis[camera_num] = phi
        rads[camera_num] = rad

    # end of first loop

    cosphis = np.cos(phis)
    sinphis = np.sin(phis)
    big_matrix = np.column_stack((cosphis,sinphis))
    points = np.zeros((int(num_cameras*(num_cameras-1)/2),2))
    i = 0
    for pair in itertools.combinations(range(num_cameras), 2):
        matrix_a = np.vstack((big_matrix[pair[0]],big_matrix[pair[1]]))
        vec_b = np.hstack((rads[pair[0]],rads[pair[1]]))
        point = np.linalg.solve(matrix_a, vec_b)
        points[i] = point
        i += 1
    drone_pos = np.mean(points,axis=0)

    # start of third loop
    for camera_num in range(num_cameras):
        d_prime = find_d_prime(d[camera_num], theta[camera_num], drone_pos)
        P,Q,M,N = find_P_Q_M_N(p[camera_num],q[camera_num],r[camera_num])
        h = find_h(d[camera_num],P,Q,M,N)
        angle_4 = find_angle_4(h,d[camera_num])
        Y = find_Y(drone[camera_num], cent[camera_num])
        angle_5 = find_angle_5(Y, VFOV, VPIX)
        angle_6 = angle_5 - angle_4
        h_prime = find_h_prime(d_prime, angle_6)
        Hs[camera_num] = h + h_prime
    drone_h = np.mean(H)
    return np.append(drone_pos,drone_h)

def set_up_server_socket(host, port):
    #Please close sel
    sel = selectors.DefaultSelector()
    lsock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    lsock.bind((host, port))
    lsock.listen()
    print("listening on", (host, port))
    lsock.setblocking(False)
    sel.register(lsock, selectors.EVENT_READ, data=None)
    return sel

def accept_wrapper(sock,sel):
    conn, addr = sock.accept()  # Should be ready to read
    print("accepted connection from", addr)
    conn.setblocking(False)
    data = types.SimpleNamespace(addr=addr, inb=b"", outb=b"")
    events = selectors.EVENT_READ | selectors.EVENT_WRITE
    sel.register(conn, events, data=data)


def service_connection(key, mask, sel):
    sock = key.fileobj
    data = key.data
    if mask & selectors.EVENT_READ:
        recv_data = sock.recv(1024)  # Should be ready to read
        if recv_data:
            data.outb += recv_data
        else:
            #print("closing connection to", data.addr)
            print("ERROR")
            print("ERROR")
            print("SOMETHING probably WENT WRONG")
    if mask & selectors.EVENT_WRITE:
        if data.outb:
            #print(type(data.outb))
            #print(repr(data.outb))
            return(repr(data.outb))
            #print("echoing", repr(data.outb), "to", data.addr)
            #return repr(data.outb)
            #following could send back to client...
            #sent = sock.send(data.outb)  # Should be ready to write
            #data.outb = data.outb[sent:]

def accept_connection(sel):
    events = sel.select(timeout=0)
    for key, mask in events:
        if key.data is None:
            accept_wrapper(key.fileobj,sel)

def get_data(sel):
    events = sel.select(timeout=0)

    for key, mask in events:
        if key.data is None:
            #should never run
            accept_wrapper(key.fileobj,sel)
        else:
            return service_connection(key, mask,sel)

def close_connection(sock, sel):
    sel.unregister(sock)
    sock.close()

camera0 = queue.Queue()
camera1 = queue.Queue()
camera2 = queue.Queue()

cameras_list = [camera0, camera1, camera2]



def cameras(host, port):
    sel = set_up_server_socket(host, port)
    while True:
        accept_connection(sel)
        new_data = get_data(sel)
        if new_data:
            # parse data, and append to queue
            new_data = new_data.replace("'","")
            new_data = new_data.replace("b","")
            #print(new_data)
            while True:
                int_list = new_data.split(',',3)
                if int_list[0] == '':
                    break
                camera = cameras_list[int(int_list[0])]

                drone_x = int(int_list[1])
                drone_y = int(int_list[2])
                #p_x = int.from_bytes(new_data[1:5],'little',signed=False)
                #p_y = int.from_bytes(new_data[5:9],'little',signed=False)
                #q_x = int.from_bytes(new_data[9:13],'little',signed=False)
                #q_y = int.from_bytes(new_data[13:17],'little',signed=False)
                #r_x = int.from_bytes(new_data[17:21],'little',signed=False)
                #r_y = int.from_bytes(new_data[21:25],'little',signed=False)
                if len(int_list) > 3:
                    new_data = int_list[3] ## remember to change THIS!!
                else:
                    break
                camera.put(np.array([drone_x,drone_y]))
                #camera.put(np.array([[drone_x,drone_y],[p_x,p_y],[q_x,q_y],[r_x,r_y]]))

def handle_everything():
    xpid = PID.PID(P=1, I=1, D=.01)
    ypid = PID.PID(P=1, I=1, D=.01)
    zpid = PID.PID(P=1, I=1, D=.01)
    xpid.set_point = 0
    ypid.set_point = 0
    zpid.set_point = 300
    controller = DroneController()
    while True:
        if not camera0.empty() and not camera1.empty():
            while not camera0.empty():
                data0 = camera0.get()
            while not camera1.empty():
                data1 = camera1.get()
            #while not camera2.empty():
            #    data2 = camera2.get()

            #drone = np.vstack((data0[0],data1[0]))
            #p = np.vstack((data0[1],data1[1]))
            #q = np.vstack((data0[2],data1[2]))
            #r = np.vstack((data0[3],data1[3]))



            #x,y,z = get_x_y_z(drone, p, q, r)

            x_x = xpid.update(data0[0] - 640)/100
            y_y = ypid.update(data1[0] - 640)/100
            t = zpid.update(720-data0[1])/100


            controller.fly(t,x_x,y_y)
            print(t,x_x,y_y)
threading.Thread(target = cameras, args=("", 8082)).start()
threading.Thread(target = handle_everything).start()
