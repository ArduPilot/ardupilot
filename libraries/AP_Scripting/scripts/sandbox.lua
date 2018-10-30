-- sample sandbox environment
function get_sandbox_env ()
  return {
          ipairs = ipairs,
          next = next,
          pairs = pairs,
          pcall = pcall,
          tonumber = tonumber,
          tostring = tostring,
          type = type,
          unpack = unpack,
          string = { byte = string.byte, char = string.char, find = string.find, 
                     format = string.format, gmatch = string.gmatch, gsub = string.gsub, 
                     len = string.len, lower = string.lower, match = string.match, 
                     rep = string.rep, reverse = string.reverse, sub = string.sub, 
                     upper = string.upper },
          math = { abs = math.abs, acos = math.acos, asin = math.asin, 
                   atan = math.atan, atan2 = math.atan2, ceil = math.ceil, cos = math.cos, 
                   cosh = math.cosh, deg = math.deg, exp = math.exp, floor = math.floor, 
                   fmod = math.fmod, frexp = math.frexp, huge = math.huge, 
                   ldexp = math.ldexp, log = math.log, log10 = math.log10, max = math.max, 
                   min = math.min, modf = math.modf, pi = math.pi, pow = math.pow, 
                   rad = math.rad, random = math.random, sin = math.sin, sinh = math.sinh, 
                   sqrt = math.sqrt, tan = math.tan, tanh = math.tanh },
          table = { insert = table.insert, maxn = table.maxn, remove = table.remove, 
                    sort = table.sort },
          utf8 = { char = utf8.char, charpattern = utf8.charpattern, codes = utf8.codes,
                   codepoint = utf8.codepoint, len = utf8.len, offsets = utf8.offsets},
          os = { clock = os.clock, difftime = os.difftime, time = os.time },

          -- ArduPilot specific
          gcs = { send_text = gcs.send_text},
          servo = { set_output_pwm = servo.set_output_pwm},
        }
end
