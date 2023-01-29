// Basic matrix library copied from pymavlink again
// https://github.com/ArduPilot/pymavlink/blob/master/rotmat.py

function Matrix3() {

    this.a = {}
    this.a.x = 0
    this.a.y = 0
    this.a.z = 0

    this.b = {}
    this.b.x = 0
    this.b.y = 0
    this.b.z = 0

    this.c = {}
    this.c.x = 0
    this.c.y = 0
    this.c.z = 0


    this.from_euler = function(roll, pitch, yaw) {
        // fill the matrix from Euler angles in radians
        const cp = Math.cos(pitch)
        const sp = Math.sin(pitch)
        const sr = Math.sin(roll)
        const cr = Math.cos(roll)
        const sy = Math.sin(yaw)
        const cy = Math.cos(yaw)

        this.a.x = cp * cy
        this.a.y = (sr * sp * cy) - (cr * sy)
        this.a.z = (cr * sp * cy) + (sr * sy)
        this.b.x = cp * sy
        this.b.y = (sr * sp * sy) + (cr * cy)
        this.b.z = (cr * sp * sy) - (sr * cy)
        this.c.x = -sp
        this.c.y = sr * cp
        this.c.z = cr * cp
    }

    this.from_euler_transposed = function(roll, pitch, yaw) {
        // fill the matrix from Euler angles in radians
        const cp = Math.cos(pitch)
        const sp = Math.sin(pitch)
        const sr = Math.sin(roll)
        const cr = Math.cos(roll)
        const sy = Math.sin(yaw)
        const cy = Math.cos(yaw)

        this.a.x = cp * cy
        this.b.x = (sr * sp * cy) - (cr * sy)
        this.c.x = (cr * sp * cy) + (sr * sy)
        this.a.y = cp * sy
        this.b.y = (sr * sp * sy) + (cr * cy)
        this.c.y = (cr * sp * sy) - (sr * cy)
        this.a.z = -sp
        this.b.z = sr * cp
        this.c.z = cr * cp
    }


    this.mul = function(v) {
        ret = {}
        ret.x = this.a.x * v.x + this.a.y * v.y + this.a.z * v.z
        ret.y = this.b.x * v.x + this.b.y * v.y + this.b.z * v.z
        ret.z = this.c.x * v.x + this.c.y * v.y + this.c.z * v.z
        return ret
    }

    return this
}