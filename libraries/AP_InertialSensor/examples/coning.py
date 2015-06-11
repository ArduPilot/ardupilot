#!/usr/bin/python

from math import *
from pymavlink.rotmat import Vector3, Matrix3
from numpy import linspace
from visual import *

class Quat:
    def __init__(self,w=1.0,x=0.0,y=0.0,z=0.0):
        self.w = w
        self.x = x
        self.y = y
        self.z = z

    def to_euler(self):
        roll = (atan2(2.0*(self.w*self.x + self.y*self.z), 1 - 2.0*(self.x*self.x + self.y*self.y)))
        pitch = asin(2.0*(self.w*self.y - self.z*self.x))
        yaw = atan2(2.0*(self.w*self.z + self.x*self.y), 1 - 2.0*(self.y*self.y + self.z*self.z))
        return Vector3(roll,pitch,yaw)

    def from_euler(self,euler):
        #(roll,pitch,yaw)
        cr2 = cos(euler[0]*0.5)
        cp2 = cos(euler[1]*0.5)
        cy2 = cos(euler[2]*0.5)
        sr2 = sin(euler[0]*0.5)
        sp2 = sin(euler[1]*0.5)
        sy2 = sin(euler[2]*0.5)

        self.w = cr2*cp2*cy2 + sr2*sp2*sy2
        self.x = sr2*cp2*cy2 - cr2*sp2*sy2
        self.y = cr2*sp2*cy2 + sr2*cp2*sy2
        self.z = cr2*cp2*sy2 - sr2*sp2*cy2
        return self

    def from_axis_angle(self, vec):
        theta = vec.length()
        if theta == 0:
            self.w = 1.0
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0
            return

        vec_normalized = vec.normalized()

        st2 = sin(theta/2.0)
        self.w = cos(theta/2.0)
        self.x = vec_normalized.x * st2
        self.y = vec_normalized.y * st2
        self.z = vec_normalized.z * st2

    def rotate(self, vec):
        r = Quat()
        r.from_axis_angle(vec)
        q = self * r
        self.w = q.w
        self.x = q.x
        self.y = q.y
        self.z = q.z

    def to_axis_angle(self):
        l = sqrt(self.x**2+self.y**2+self.z**2)

        (x,y,z) = (self.x,self.y,self.z)
        if l != 0:
            temp = 2.0*atan2(l,self.w)
            if temp > pi:
                temp -= 2*pi
            elif temp < -pi:
                temp += 2*pi
            (x,y,z) = (temp*x/l,temp*y/l,temp*z/l)
        return Vector3(x,y,z)

    def to_rotation_matrix(self):
        m = Matrix3()
        yy = self.y**2
        yz = self.y * self.z
        xx = self.x**2
        xy = self.x * self.y
        xz = self.x * self.z
        wx = self.w * self.x
        wy = self.w * self.y
        wz = self.w * self.z
        zz = self.z**2

        m.a.x = 1.0-2.0*(yy + zz)
        m.a.y =   2.0*(xy - wz)
        m.a.z =   2.0*(xz + wy)
        m.b.x =   2.0*(xy + wz)
        m.b.y = 1.0-2.0*(xx + zz)
        m.b.z =   2.0*(yz - wx)
        m.c.x =   2.0*(xz - wy)
        m.c.y =   2.0*(yz + wx)
        m.c.z = 1.0-2.0*(xx + yy)
        return m

    def inverse(self):
        return Quat(self.w,-self.x,-self.y,-self.z)

    def __mul__(self,operand):
        ret = Quat()
        w1=self.w
        x1=self.x
        y1=self.y
        z1=self.z
        w2=operand.w
        x2=operand.x
        y2=operand.y
        z2=operand.z

        ret.w = w1*w2 - x1*x2 - y1*y2 - z1*z2
        ret.x = w1*x2 + x1*w2 + y1*z2 - z1*y2
        ret.y = w1*y2 - x1*z2 + y1*w2 + z1*x2
        ret.z = w1*z2 + x1*y2 - y1*x2 + z1*w2
        return ret

    def __str__(self):
        return "Quat(%f, %f, %f, %f)" % (self.w,self.x,self.y,self.z)

def vpy_vec(vec):
    return vector(vec.y, -vec.z, -vec.x)

def update_arrows(q,x,y,z):
    m = q.to_rotation_matrix().transposed()
    x.axis = vpy_vec(m*Vector3(1,0,0))
    x.up = vpy_vec(m*Vector3(0,1,0))
    y.axis = vpy_vec(m*Vector3(0,1,0))
    y.up = vpy_vec(m*Vector3(1,0,0))
    z.axis = vpy_vec(m*Vector3(0,0,1))
    z.up = vpy_vec(m*Vector3(1,0,0))

class Attitude:
    def __init__(self,reference=False):
        self.labels = []
        self.xarrows = []
        self.yarrows = []
        self.zarrows = []
        self.q = Quat()
        self.reference = reference
        self.update_arrows()

    def add_arrows(self, arrowpos = Vector3(0,0,0), labeltext=None):
        if labeltext is not None:
            self.labels.append(label(pos = vpy_vec(arrowpos), text=labeltext))

        sw = .005 if self.reference else .05

        self.xarrows.append(arrow(pos=vpy_vec(arrowpos),color=color.red,opacity=1,shaftwidth=sw))
        self.yarrows.append(arrow(pos=vpy_vec(arrowpos),color=color.green,opacity=1,shaftwidth=sw))
        self.zarrows.append(arrow(pos=vpy_vec(arrowpos),color=color.blue,opacity=1,shaftwidth=sw))
        self.update_arrows()

    def rotate(self, vec):
        self.q.rotate(vec)

    def update_arrows(self):
        m = self.q.to_rotation_matrix().transposed()
        sl = 1.1 if self.reference else 1.0
        for i in self.xarrows:
            i.axis = vpy_vec(m*Vector3(sl,0,0))
            i.up = vpy_vec(m*Vector3(0,1,0))
        for i in self.yarrows:
            i.axis = vpy_vec(m*Vector3(0,sl,0))
            i.up = vpy_vec(m*Vector3(1,0,0))
        for i in self.zarrows:
            i.axis = vpy_vec(m*Vector3(0,0,sl))
            i.up = vpy_vec(m*Vector3(1,0,0))
        for i in self.labels:
            i.xoffset = scene.width*0.07
            i.yoffset = scene.width*0.1

class Tian_integrator:
    def __init__(self, integrate_separately=True):
        self.alpha = Vector3(0,0,0)
        self.beta = Vector3(0,0,0)
        self.last_alpha = Vector3(0,0,0)
        self.last_delta_alpha = Vector3(0,0,0)
        self.last_sample = Vector3(0,0,0)
        self.integrate_separately = integrate_separately

    def add_sample(self, sample, dt):
        delta_alpha = (self.last_sample+sample)*0.5*dt
        self.alpha += delta_alpha
        delta_beta = 0.5 * (self.last_alpha + (1.0/6.0)*self.last_delta_alpha)%delta_alpha

        if self.integrate_separately:
            self.beta += delta_beta
        else:
            self.alpha += delta_beta

        self.last_alpha = self.alpha
        self.last_delta_alpha = delta_alpha
        self.last_sample = sample

    def pop_delta_angles(self):
        ret = self.alpha + self.beta
        self.alpha.zero()
        self.beta.zero()
        return ret

filter2p_1khz_30hz_data = {}
def filter2p_1khz_30hz(sample, key):
    global filter2p_1khz_30hz_data
    if not key in filter2p_1khz_30hz_data:
        filter2p_1khz_30hz_data[key] = (0.0,0.0)
    (delay_element_1, delay_element_2) = filter2p_1khz_30hz_data[key]
    sample_freq = 1000
    cutoff_freq = 30
    fr = sample_freq/cutoff_freq
    ohm = tan(pi/fr)
    c = 1.0+2.0*cos(pi/4.0)*ohm + ohm**2
    b0 = ohm**2/c
    b1 = 2.0*b0
    b2 = b0
    a1 = 2.0*(ohm**2-1.0)/c
    a2 = (1.0-2.0*cos(pi/4.0)*ohm+ohm**2)/c

    delay_element_0 = sample - delay_element_1 * a1 - delay_element_2 * a2
    output = delay_element_0 * b0 + delay_element_1 * b1 + delay_element_2 * b2

    filter2p_1khz_30hz_data[key] = (delay_element_0, delay_element_1)

    return output

def filter2p_1khz_30hz_vector3(sample, key):
    ret = Vector3()
    ret.x = filter2p_1khz_30hz(sample.x, "vec3f"+key+"x")
    ret.y = filter2p_1khz_30hz(sample.y, "vec3f"+key+"y")
    ret.z = filter2p_1khz_30hz(sample.z, "vec3f"+key+"z")
    return ret


reference_attitude = Attitude(True)
uncorrected_attitude_low = Attitude()
uncorrected_attitude_high = Attitude()
corrected_attitude = Attitude()
corrected_attitude_combined = Attitude()

corrected_attitude_integrator = Tian_integrator()
corrected_attitude_integrator_combined = Tian_integrator(integrate_separately = False)

reference_attitude.add_arrows(Vector3(0,-3,0))
uncorrected_attitude_low.add_arrows(Vector3(0,-3,0), "no correction\nlow rate integration\n30hz software LPF @ 1khz\n(ardupilot 2015-02-18)")

reference_attitude.add_arrows(Vector3(0,-1,0))
uncorrected_attitude_high.add_arrows(Vector3(0,-1,0), "no correction\nhigh rate integration")

reference_attitude.add_arrows(Vector3(0,1,0))
corrected_attitude.add_arrows(Vector3(0,1,0), "Tian et al\nseparate integration")

reference_attitude.add_arrows(Vector3(0,3,0))
corrected_attitude_combined.add_arrows(Vector3(0,3,0), "Tian et al\ncombined_integration\n(proposed patch)")

#scene.scale = (0.3,0.3,0.3)
scene.fov = 0.001
scene.forward = (-0.5, -0.5, -1)

coning_frequency_hz = 50
coning_magnitude_rad_s = 2
label_text = (
    "coning motion frequency %f hz\n"
    "coning motion peak amplitude %f deg/s\n"
    "thin arrows are reference attitude"
    ) % (coning_frequency_hz, degrees(coning_magnitude_rad_s))
label(pos = vpy_vec(Vector3(0,0,2)), text=label_text)

t = 0.0
dt_10000 = 0.0001
dt_1000 = 0.001
dt_333 = 0.003

accumulated_delta_angle = Vector3(0,0,0)
last_gyro_10000 = Vector3(0,0,0)
last_gyro_1000 = Vector3(0,0,0)
last_filtered_gyro_333 = Vector3(0,0,0)
filtered_gyro = Vector3(0,0,0)

while True:
    rate(66)
    for i in range(5):
        for j in range(3):
            for k in range(10):
                #vvvvvvvvvv 10 kHz vvvvvvvvvv#

                #compute angular rate at current time
                gyro = Vector3(sin(t*coning_frequency_hz*2*pi), cos(t*coning_frequency_hz*2*pi),0)*coning_magnitude_rad_s

                #integrate reference attitude
                reference_attitude.rotate((gyro+last_gyro_10000) * dt_10000 * 0.5)

                #increment time
                t += dt_10000
                last_gyro_10000 = gyro

            #vvvvvvvvvv 1 kHz vvvvvvvvvv#

            #update filter for sim 1
            filtered_gyro = filter2p_1khz_30hz_vector3(gyro, "1")

            #update integrator for sim 2
            accumulated_delta_angle += (gyro+last_gyro_1000) * dt_1000 * 0.5

            #update integrator for sim 3
            corrected_attitude_integrator.add_sample(gyro, dt_1000)

            #update integrator for sim 4
            corrected_attitude_integrator_combined.add_sample(gyro, dt_1000)

            last_gyro_1000 = gyro

        #vvvvvvvvvv 333 Hz vvvvvvvvvv#

        #update sim 1 (leftmost)
        uncorrected_attitude_low.rotate((filtered_gyro+last_filtered_gyro_333) * dt_333 * 0.5)

        #update sim 2
        uncorrected_attitude_high.rotate(accumulated_delta_angle)
        accumulated_delta_angle.zero()

        #update sim 3
        corrected_attitude.rotate(corrected_attitude_integrator.pop_delta_angles())

        #update sim 4 (rightmost)
        corrected_attitude_combined.rotate(corrected_attitude_integrator_combined.pop_delta_angles())

        last_filtered_gyro_333 = filtered_gyro

    #vvvvvvvvvv 66 Hz vvvvvvvvvv#
    reference_attitude.update_arrows()
    corrected_attitude.update_arrows()
    corrected_attitude_combined.update_arrows()
    uncorrected_attitude_low.update_arrows()
    uncorrected_attitude_high.update_arrows()
