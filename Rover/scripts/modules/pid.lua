-- Check if the script is being run directly by APM, if so exit
if ... == nil then
  return
end

local PID = {}
PID.__index = PID

function PID:new(p_gain, i_gain, d_gain, i_max, i_min, pid_max, pid_min)
  local obj = {
      P = p_gain or 0,
      I = i_gain or 0,
      D = d_gain or 0,
      integrator = 0,
      last_error = 0,
      i_max = i_max or 0,
      i_min = i_min or 0,
      pid_max = pid_max or 1,
      pid_min = pid_min or -1
  }
  setmetatable(obj, self)
  -- self.__index = self

  return obj
end

function PID:compute(setpoint, pv, dt)
  local error = setpoint - pv
  local deriv = (error - self.last_error)/dt
  self.integrator = self.integrator + error*dt
  self.integrator = math.max(math.min(self.integrator, self.i_max), self.i_min)

  local output = math.min(math.max(self.P*error + self.I*self.integrator + self.D*deriv, self.pid_min), self.pid_max)
  self.last_error = error

  return output
end

return PID
