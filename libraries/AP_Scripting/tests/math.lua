-- $Id: math.lua,v 1.77 2016/06/23 15:17:20 roberto Exp roberto $

--[[
*****************************************************************************
* Copyright (C) 1994-2016 Lua.org, PUC-Rio.
*
* Permission is hereby granted, free of charge, to any person obtaining
* a copy of this software and associated documentation files (the
* "Software"), to deal in the Software without restriction, including
* without limitation the rights to use, copy, modify, merge, publish,
* distribute, sublicense, and/or sell copies of the Software, and to
* permit persons to whom the Software is furnished to do so, subject to
* the following conditions:
*
* The above copyright notice and this permission notice shall be
* included in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
* CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
* TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
* SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*****************************************************************************
]]

-- This code is copied from https://github.com/lua/tests and slightly modified to work within ArduPilot

gcs:send_text(6, "testing numbers and math lib")

local minint = math.mininteger
local maxint = math.maxinteger

local intbits = math.floor(math.log(maxint, 2) + 0.5) + 1
assert((1 << intbits) == 0)

assert(minint == 1 << (intbits - 1))
assert(maxint == minint - 1)

-- number of bits in the mantissa of a floating-point number
local floatbits = 24
do
  local p = 2.0^floatbits
  while p < p + 1.0 do
    p = p * 2.0
    floatbits = floatbits + 1
  end
end

local function isNaN (x)
  return (x ~= x)
end

assert(isNaN(0/0))
assert(not isNaN(1/0))


do
  local x = 2.0^floatbits
  assert(x > x - 1.0 and x == x + 1.0)

  gcs:send_text(6, string.format("%d-bit integers, %d-bit (mantissa) floats",
                       intbits, floatbits))
end

assert(math.type(0) == "integer" and math.type(0.0) == "float"
       and math.type("10") == nil)


local function checkerror (msg, f, ...)
  local s, err = pcall(f, ...)
  assert(not s and string.find(err, msg))
end

local msgf2i = "number.* has no integer representation"

-- float equality
function eq (a,b,limit)
  if not limit then
    if floatbits >= 50 then limit = 1E-11
    else limit = 1E-5
    end
  end
  -- a == b needed for +inf/-inf
  return a == b or math.abs(a-b) <= limit
end


-- equality with types
function eqT (a,b)
  return a == b and math.type(a) == math.type(b)
end


-- basic float notation
assert(0e12 == 0 and .0 == 0 and 0. == 0 and .2e2 == 20 and 2.E-1 == 0.2)

do
  local a,b,c = "2", " 3e0 ", " 10  "
  assert(a+b == 5 and -b == -3 and b+"2" == 5 and "10"-c == 0)
  assert(type(a) == 'string' and type(b) == 'string' and type(c) == 'string')
  assert(a == "2" and b == " 3e0 " and c == " 10  " and -c == -"  10 ")
  assert(c%a == 0 and a^b == 08)
  a = 0
  assert(a == -a and 0 == -0)
end

do
  local x = -1
  local mz = 0/x   -- minus zero
  t = {[0] = 10, 20, 30, 40, 50}
  assert(t[mz] == t[0] and t[-0] == t[0])
end

do   -- tests for 'modf'
  local a,b = math.modf(3.5)
  assert(a == 3.0 and b == 0.5)
  a,b = math.modf(-2.5)
  assert(a == -2.0 and b == -0.5)
  a,b = math.modf(-3e23)
  assert(a == -3e23 and b == 0.0)
  a,b = math.modf(3e35)
  assert(a == 3e35 and b == 0.0)
  a,b = math.modf(-1/0)   -- -inf
  assert(a == -1/0 and b == 0.0)
  a,b = math.modf(1/0)   -- inf
  assert(a == 1/0 and b == 0.0)
  a,b = math.modf(0/0)   -- NaN
  assert(isNaN(a) and isNaN(b))
  a,b = math.modf(3)  -- integer argument
  assert(eqT(a, 3) and eqT(b, 0.0))
  a,b = math.modf(minint)
  assert(eqT(a, minint) and eqT(b, 0.0))
end

assert(math.huge > 10e30)
assert(-math.huge < -10e30)


-- integer arithmetic
assert(minint < minint + 1)
assert(maxint - 1 < maxint)
assert(0 - minint == minint)
assert(minint * minint == 0)
assert(maxint * maxint * maxint == maxint)


-- testing floor division and conversions

for _, i in pairs{-16, -15, -3, -2, -1, 0, 1, 2, 3, 15} do
  for _, j in pairs{-16, -15, -3, -2, -1, 1, 2, 3, 15} do
    for _, ti in pairs{0, 0.0} do     -- try 'i' as integer and as float
      for _, tj in pairs{0, 0.0} do   -- try 'j' as integer and as float
        local x = i + ti
        local y = j + tj
          assert(i//j == math.floor(i/j))
      end
    end
  end
end

assert(1//0.0 == 1/0)
assert(-1 // 0.0 == -1/0)
assert(eqT(3.5 // 1.5, 2.0))
assert(eqT(3.5 // -1.5, -3.0))

assert(maxint // maxint == 1)
assert(maxint // 1 == maxint)
assert((maxint - 1) // maxint == 0)
assert(maxint // (maxint - 1) == 1)
assert(minint // minint == 1)
assert(minint // minint == 1)
assert((minint + 1) // minint == 0)
assert(minint // (minint + 1) == 1)
assert(minint // 1 == minint)

assert(minint // -1 == -minint)
assert(minint // -2 == 2^(intbits - 2))
assert(maxint // -1 == -maxint)


-- negative exponents
do
  assert(2^-3 == 1 / 2^3)
  assert(eq((-3)^-3, 1 / (-3)^3))
  for i = -3, 3 do    -- variables avoid constant folding
      for j = -3, 3 do
        -- domain errors (0^(-n)) are not portable
        if not _port or i ~= 0 or j > 0 then
          assert(eq(i^j, 1 / i^(-j)))
       end
    end
  end
end

-- comparison between floats and integers (border cases)
if floatbits < intbits then
  assert(2.0^floatbits == (1 << floatbits))
  assert(2.0^floatbits - 1.0 == (1 << floatbits) - 1.0)
  assert(2.0^floatbits - 1.0 ~= (1 << floatbits))
  -- float is rounded, int is not
  assert(2.0^floatbits + 1.0 ~= (1 << floatbits) + 1)
else   -- floats can express all integers with full accuracy
  assert(maxint == maxint + 0.0)
  assert(maxint - 1 == maxint - 1.0)
  assert(minint + 1 == minint + 1.0)
  assert(maxint ~= maxint - 1.0)
end
assert(maxint + 0.0 == 2.0^(intbits - 1) - 1.0)
assert(minint + 0.0 == minint)
assert(minint + 0.0 == -2.0^(intbits - 1))


-- order between floats and integers
assert(1 < 1.1); assert(not (1 < 0.9))
assert(1 <= 1.1); assert(not (1 <= 0.9))
assert(-1 < -0.9); assert(not (-1 < -1.1))
assert(1 <= 1.1); assert(not (-1 <= -1.1))
assert(-1 < -0.9); assert(not (-1 < -1.1))
assert(-1 <= -0.9); assert(not (-1 <= -1.1))
assert(minint <= minint + 0.0)
assert(minint + 0.0 <= minint)
assert(not (minint < minint + 0.0))
assert(not (minint + 0.0 < minint))
assert(maxint < minint * -1.0)
assert(maxint <= minint * -1.0)

do
  local fmaxi1 = 2^(intbits - 1)
  assert(maxint < fmaxi1)
  assert(maxint <= fmaxi1)
  assert(not (fmaxi1 <= maxint))
  assert(minint <= -2^(intbits - 1))
  assert(-2^(intbits - 1) <= minint)
end

if floatbits < intbits then
  gcs:send_text(6, "testing order (floats can't represent all int)")
  local fmax = 2^floatbits
  local ifmax = fmax | 0
  assert(fmax < ifmax + 1)
  assert(fmax - 1 < ifmax)
  assert(-(fmax - 1) > -ifmax)
  assert(not (fmax <= ifmax - 1))
  assert(-fmax > -(ifmax + 1))
  assert(not (-fmax >= -(ifmax - 1)))

  assert(fmax/2 - 0.5 < ifmax//2)
  assert(-(fmax/2 - 0.5) > -ifmax//2)

  assert(maxint < 2^intbits)
  assert(minint > -2^intbits)
  assert(maxint <= 2^intbits)
  assert(minint >= -2^intbits)
else
  gcs:send_text(6, "testing order (floats can represent all ints)")
  assert(maxint < maxint + 1.0)
  assert(maxint < maxint + 0.5)
  assert(maxint - 1.0 < maxint)
  assert(maxint - 0.5 < maxint)
  assert(not (maxint + 0.0 < maxint))
  assert(maxint + 0.0 <= maxint)
  assert(not (maxint < maxint + 0.0))
  assert(maxint + 0.0 <= maxint)
  assert(maxint <= maxint + 0.0)
  assert(not (maxint + 1.0 <= maxint))
  assert(not (maxint + 0.5 <= maxint))
  assert(not (maxint <= maxint - 1.0))
  assert(not (maxint <= maxint - 0.5))

  assert(minint < minint + 1.0)
  assert(minint < minint + 0.5)
  assert(minint <= minint + 0.5)
  assert(minint - 1.0 < minint)
  assert(minint - 1.0 <= minint)
  assert(not (minint + 0.0 < minint))
  assert(not (minint + 0.5 < minint))
  assert(not (minint < minint + 0.0))
  assert(minint + 0.0 <= minint)
  assert(minint <= minint + 0.0)
  assert(not (minint + 1.0 <= minint))
  assert(not (minint + 0.5 <= minint))
  assert(not (minint <= minint - 1.0))
end

do
  local NaN = 0/0
  assert(not (NaN < 0))
  assert(not (NaN > minint))
  assert(not (NaN <= -9))
  assert(not (NaN <= maxint))
  assert(not (NaN < maxint))
  assert(not (minint <= NaN))
  assert(not (minint < NaN))
end


-- avoiding errors at compile time
--local function checkcompt (msg, code)
--  checkerror(msg, assert(load(code)))
--end
--checkcompt("divide by zero", "return 2 // 0")
--checkcompt(msgf2i, "return 2.3 >> 0")
--checkcompt(msgf2i, ("return 2.0^%d & 1"):format(intbits - 1))
--checkcompt("field 'huge'", "return math.huge << 1")
--checkcompt(msgf2i, ("return 1 | 2.0^%d"):format(intbits - 1))
--checkcompt(msgf2i, "return 2.3 ~ '0.0'")


-- testing overflow errors when converting from float to integer (runtime)
local function f2i (x) return x | x end
checkerror(msgf2i, f2i, math.huge)     -- +inf
checkerror(msgf2i, f2i, -math.huge)    -- -inf
checkerror(msgf2i, f2i, 0/0)           -- NaN

if floatbits < intbits then
  -- conversion tests when float cannot represent all integers
  assert(maxint + 1.0 == maxint + 0.0)
  assert(minint - 1.0 == minint + 0.0)
  checkerror(msgf2i, f2i, maxint + 0.0)
  assert(f2i(2.0^(intbits - 2)) == 1 << (intbits - 2))
  assert(f2i(-2.0^(intbits - 2)) == -(1 << (intbits - 2)))
  assert((2.0^(floatbits - 1) + 1.0) // 1 == (1 << (floatbits - 1)) + 1)
  -- maximum integer representable as a float
  local mf = maxint - (1 << (floatbits - intbits)) + 1
  assert(f2i(mf + 0.0) == mf)  -- OK up to here
  mf = mf + 1
  assert(f2i(mf + 0.0) ~= mf)   -- no more representable
else
  -- conversion tests when float can represent all integers
  assert(maxint + 1.0 > maxint)
  assert(minint - 1.0 < minint)
  assert(f2i(maxint + 0.0) == maxint)
  checkerror("no integer rep", f2i, maxint + 1.0)
  checkerror("no integer rep", f2i, minint - 1.0)
end

-- 'minint' should be representable as a float no matter the precision
assert(f2i(minint + 0.0) == minint)


-- testing numeric strings

assert("2" + 1 == 3)
assert("2 " + 1 == 3)
assert(" -2 " + 1 == -1)
assert(" -0xa " + 1 == -9)


-- Literal integer Overflows (new behavior in 5.3.3)
do
  -- no overflows
  assert(eqT(tonumber(tostring(maxint)), maxint))
  assert(eqT(tonumber(tostring(minint)), minint))

  -- add 1 to last digit as a string (it cannot be 9...)
  local function incd (n)
    local s = string.format("%d", n)
    s = string.gsub(s, "%d$", function (d)
          assert(d ~= '9')
          return string.char(string.byte(d) + 1)
        end)
    return s
  end

  -- 'tonumber' with overflow by 1
  assert(eqT(tonumber(incd(maxint)), maxint + 1.0))
  assert(eqT(tonumber(incd(minint)), minint - 1.0))

  -- large numbers
  assert(eqT(tonumber("1"..string.rep("0", 30)), 1e30))
  assert(eqT(tonumber("-1"..string.rep("0", 30)), -1e30))

  -- hexa format still wraps around
  assert(eqT(tonumber("0x1"..string.rep("0", 30)), 0))

  -- lexer in the limits
  --assert(minint == load("return " .. minint)())
  --assert(eqT(maxint, load("return " .. maxint)()))

  assert(eqT(10000000000000000000000.0, 10000000000000000000000))
  assert(eqT(-10000000000000000000000.0, -10000000000000000000000))
end


-- testing 'tonumber'

-- 'tonumber' with numbers
assert(tonumber(3.4) == 3.4)
assert(eqT(tonumber(3), 3))
assert(eqT(tonumber(maxint), maxint) and eqT(tonumber(minint), minint))
assert(tonumber(1/0) == 1/0)

-- 'tonumber' with strings
assert(tonumber("0") == 0)
assert(tonumber("") == nil)
assert(tonumber("  ") == nil)
assert(tonumber("-") == nil)
assert(tonumber("  -0x ") == nil)
assert(tonumber{} == nil)
assert(tonumber'+0.01' == 1/100 and tonumber'+.01' == 0.01 and
       tonumber'.01' == 0.01    and tonumber'-1.' == -1 and
       tonumber'+1.' == 1)
assert(tonumber'+ 0.01' == nil and tonumber'+.e1' == nil and
       tonumber'1e' == nil     and tonumber'1.0e+' == nil and
       tonumber'.' == nil)
assert(tonumber('-012') == -010-2)
assert(tonumber('-1.2e2') == - - -120)

assert(tonumber("0xffffffffffff") == (1 << (4*12)) - 1)
assert(tonumber("0x"..string.rep("f", (intbits//4))) == -1)
assert(tonumber("-0x"..string.rep("f", (intbits//4))) == 1)

-- testing 'tonumber' with base
assert(tonumber('  001010  ', 2) == 10)
assert(tonumber('  001010  ', 10) == 001010)
assert(tonumber('  -1010  ', 2) == -10)
assert(tonumber('10', 36) == 36)
assert(tonumber('  -10  ', 36) == -36)
assert(tonumber('  +1Z  ', 36) == 36 + 35)
assert(tonumber('  -1z  ', 36) == -36 + -35)
assert(tonumber('-fFfa', 16) == -(10+(16*(15+(16*(15+(16*15)))))))
assert(tonumber(string.rep('1', (intbits - 2)), 2) + 1 == 2^(intbits - 2))
assert(tonumber('ffffFFFF', 16)+1 == (1 << 32))
assert(tonumber('0ffffFFFF', 16)+1 == (1 << 32))
assert(tonumber('-0ffffffFFFF', 16) - 1 == -(1 << 40))
for i = 2,36 do
  local i2 = i * i
  local i10 = i2 * i2 * i2 * i2 * i2      -- i^10
  assert(tonumber('\t10000000000\t', i) == i10)
end

if not _soft then
  -- tests with very long numerals
  assert(tonumber("0x"..string.rep("f", 13)..".0") == 2.0^(4*13) - 1)
  assert(tonumber("0x"..string.rep("f", 150)..".0") == 2.0^(4*150) - 1)
  assert(tonumber("0x"..string.rep("f", 300)..".0") == 2.0^(4*300) - 1)
  assert(tonumber("0x"..string.rep("f", 500)..".0") == 2.0^(4*500) - 1)
  assert(tonumber('0x3.' .. string.rep('0', 1000)) == 3)
  assert(tonumber('0x' .. string.rep('0', 1000) .. 'a') == 10)
  assert(tonumber('0x0.' .. string.rep('0', 13).."1") == 2.0^(-4*14))
  assert(tonumber('0x0.' .. string.rep('0', 150).."1") == 2.0^(-4*151))
  assert(tonumber('0x0.' .. string.rep('0', 300).."1") == 2.0^(-4*301))
  assert(tonumber('0x0.' .. string.rep('0', 500).."1") == 2.0^(-4*501))

  assert(tonumber('0xe03' .. string.rep('0', 1000) .. 'p-4000') == 3587.0)
  assert(tonumber('0x.' .. string.rep('0', 1000) .. '74p4004') == 0x7.4)
end

-- testing 'tonumber' for invalid formats
--[[
local function f (...)
  if select('#', ...) == 1 then
    return (...)
  else
    return "***"
  end
end

assert(f(tonumber('fFfa', 15)) == nil)
assert(f(tonumber('099', 8)) == nil)
assert(f(tonumber('1\0', 2)) == nil)
assert(f(tonumber('', 8)) == nil)
assert(f(tonumber('  ', 9)) == nil)
assert(f(tonumber('  ', 9)) == nil)
assert(f(tonumber('0xf', 10)) == nil)

assert(f(tonumber('inf')) == nil)
assert(f(tonumber(' INF ')) == nil)
assert(f(tonumber('Nan')) == nil)
assert(f(tonumber('nan')) == nil)

assert(f(tonumber('  ')) == nil)
assert(f(tonumber('')) == nil)
assert(f(tonumber('1  a')) == nil)
assert(f(tonumber('1  a', 2)) == nil)
assert(f(tonumber('1\0')) == nil)
assert(f(tonumber('1 \0')) == nil)
assert(f(tonumber('1\0 ')) == nil)
assert(f(tonumber('e1')) == nil)
assert(f(tonumber('e  1')) == nil)
assert(f(tonumber(' 3.4.5 ')) == nil)
]]

-- testing 'tonumber' for invalid hexadecimal formats

assert(tonumber('0x') == nil)
assert(tonumber('x') == nil)
assert(tonumber('x3') == nil)
assert(tonumber('0x3.3.3') == nil)   -- two decimal points
assert(tonumber('00x2') == nil)
assert(tonumber('0x 2') == nil)
assert(tonumber('0 x2') == nil)
assert(tonumber('23x') == nil)
assert(tonumber('- 0xaa') == nil)
assert(tonumber('-0xaaP ') == nil)   -- no exponent
assert(tonumber('0x0.51p') == nil)
assert(tonumber('0x5p+-2') == nil)


-- testing hexadecimal numerals

assert(0x10 == 16 and 0xfff == 2^12 - 1 and 0XFB == 251)
assert(0x0p12 == 0 and 0x.0p-3 == 0)
assert(0xFFFFFFFF == (1 << 32) - 1)
assert(tonumber('+0x2') == 2)
assert(tonumber('-0xaA') == -170)
assert(tonumber('-0xffFFFfff') == -(1 << 32) + 1)

-- possible confusion with decimal exponent
assert(0E+1 == 0 and 0xE+1 == 15 and 0xe-1 == 13)


-- floating hexas

assert(tonumber('  0x2.5  ') == 0x25/16)
assert(tonumber('  -0x2.5  ') == -0x25/16)
assert(tonumber('  +0x0.51p+8  ') == 0x51)
assert(0x.FfffFFFF == 1 - '0x.00000001')
assert('0xA.a' + 0 == 10 + 10/16)
assert(0xa.aP4 == 0XAA)
assert(0x4P-2 == 1)
assert(0x1.1 == '0x1.' + '+0x.1')
assert(0Xabcdef.0 == 0x.ABCDEFp+24)


assert(1.1 == 1.+.1)
assert(100.0 == 1E2 and .01 == 1e-2)
assert(1111111111 - 1111111110 == 1000.00e-03)
assert(1.1 == '1.'+'.1')
assert(tonumber'1111111111' - tonumber'1111111110' ==
       tonumber"  +0.001e+3 \n\t")

assert(0.1e-30 > 0.9E-31 and 0.9E30 < 0.1e31)

assert(0.123456 > 0.123455)

assert(tonumber('+1.23E18') == 1.23*10.0^18)

-- testing order operators
assert(not(1<1) and (1<2) and not(2<1))
assert(not('a'<'a') and ('a'<'b') and not('b'<'a'))
assert((1<=1) and (1<=2) and not(2<=1))
assert(('a'<='a') and ('a'<='b') and not('b'<='a'))
assert(not(1>1) and not(1>2) and (2>1))
assert(not('a'>'a') and not('a'>'b') and ('b'>'a'))
assert((1>=1) and not(1>=2) and (2>=1))
assert(('a'>='a') and not('a'>='b') and ('b'>='a'))
assert(1.3 < 1.4 and 1.3 <= 1.4 and not (1.3 < 1.3) and 1.3 <= 1.3)

-- testing mod operator
assert(eqT(-4 % 3, 2))
assert(eqT(4 % -3, -2))
assert(eqT(-4.0 % 3, 2.0))
assert(eqT(4 % -3.0, -2.0))
assert(math.pi - math.pi % 1 == 3)
assert(math.pi - math.pi % 0.001 == 3.141)

assert(eqT(minint % minint, 0))
assert(eqT(maxint % maxint, 0))
assert((minint + 1) % minint == minint + 1)
assert((maxint - 1) % maxint == maxint - 1)
assert(minint % maxint == maxint - 1)

assert(minint % -1 == 0)
assert(minint % -2 == 0)
assert(maxint % -2 == -1)

-- testing unsigned comparisons
assert(math.ult(3, 4))
assert(not math.ult(4, 4))
assert(math.ult(-2, -1))
assert(math.ult(2, -1))
assert(not math.ult(-2, -2))
assert(math.ult(maxint, minint))
assert(not math.ult(minint, maxint))


assert(eq(math.sin(-9.8)^2 + math.cos(-9.8)^2, 1))
assert(eq(math.tan(math.pi/4), 1))
assert(eq(math.sin(math.pi/2), 1) and eq(math.cos(math.pi/2), 0))
assert(eq(math.atan(1), math.pi/4) and eq(math.acos(0), math.pi/2) and
       eq(math.asin(1), math.pi/2))
assert(eq(math.deg(math.pi/2), 90) and eq(math.rad(90), math.pi/2))
assert(math.abs(-10.43) == 10.43)
assert(eqT(math.abs(minint), minint))
assert(eqT(math.abs(maxint), maxint))
assert(eqT(math.abs(-maxint), maxint))
assert(eq(math.atan(1,0), math.pi/2))
assert(math.fmod(10,3) == 1)
assert(eq(math.sqrt(10)^2, 10))
assert(eq(math.log(2, 10), math.log(2)/math.log(10)))
assert(eq(math.log(2, 2), 1))
assert(eq(math.log(9, 3), 2))
assert(eq(math.exp(0), 1))
assert(eq(math.sin(10), math.sin(10%(2*math.pi))))


assert(tonumber(' 1.3e-2 ') == 1.3e-2)
assert(tonumber(' -1.00000000000001 ') == -1.00000000000001)

-- testing constant limits
-- 2^23 = 8388608
assert(8388609 + -8388609 == 0)
assert(8388608 + -8388608 == 0)
assert(8388607 + -8388607 == 0)



do   -- testing floor & ceil
  assert(eqT(math.floor(3.4), 3))
  assert(eqT(math.ceil(3.4), 4))
  assert(eqT(math.floor(-3.4), -4))
  assert(eqT(math.ceil(-3.4), -3))
  assert(eqT(math.floor(maxint), maxint))
  assert(eqT(math.ceil(maxint), maxint))
  assert(eqT(math.floor(minint), minint))
  assert(eqT(math.floor(minint + 0.0), minint))
  assert(eqT(math.ceil(minint), minint))
  assert(eqT(math.ceil(minint + 0.0), minint))
  assert(math.floor(1e50) == 1e50)
  assert(math.ceil(1e50) == 1e50)
  assert(math.floor(-1e50) == -1e50)
  assert(math.ceil(-1e50) == -1e50)
  for _, p in pairs{31,32,63,64} do
    assert(math.floor(2^p) == 2^p)
    assert(math.floor(2^p + 0.5) == 2^p)
    assert(math.ceil(2^p) == 2^p)
    assert(math.ceil(2^p - 0.5) == 2^p)
  end
  checkerror("number expected", math.floor, {})
  checkerror("number expected", math.ceil, print)
  assert(eqT(math.tointeger(minint), minint))
  assert(eqT(math.tointeger(minint .. ""), minint))
  assert(eqT(math.tointeger(maxint), maxint))
  assert(eqT(math.tointeger(maxint .. ""), maxint))
  assert(eqT(math.tointeger(minint + 0.0), minint))
  assert(math.tointeger(0.0 - minint) == nil)
  assert(math.tointeger(math.pi) == nil)
  assert(math.tointeger(-math.pi) == nil)
  assert(math.floor(math.huge) == math.huge)
  assert(math.ceil(math.huge) == math.huge)
  assert(math.tointeger(math.huge) == nil)
  assert(math.floor(-math.huge) == -math.huge)
  assert(math.ceil(-math.huge) == -math.huge)
  assert(math.tointeger(-math.huge) == nil)
  assert(math.tointeger("34.0") == 34)
  assert(math.tointeger("34.3") == nil)
  assert(math.tointeger({}) == nil)
  assert(math.tointeger(0/0) == nil)    -- NaN
end


-- testing fmod for integers
for i = -6, 6 do
  for j = -6, 6 do
    if j ~= 0 then
      local mi = math.fmod(i, j)
      local mf = math.fmod(i + 0.0, j)
      assert(mi == mf)
      assert(math.type(mi) == 'integer' and math.type(mf) == 'float')
      if (i >= 0 and j >= 0) or (i <= 0 and j <= 0) or mi == 0 then
        assert(eqT(mi, i % j))
      end
    end
  end
end
assert(eqT(math.fmod(minint, minint), 0))
assert(eqT(math.fmod(maxint, maxint), 0))
assert(eqT(math.fmod(minint + 1, minint), minint + 1))
assert(eqT(math.fmod(maxint - 1, maxint), maxint - 1))

checkerror("zero", math.fmod, 3, 0)


do    -- testing max/min
  checkerror("value expected", math.max)
  checkerror("value expected", math.min)
  assert(eqT(math.max(3), 3))
  assert(eqT(math.max(3, 5, 9, 1), 9))
  assert(math.max(maxint, 10e60) == 10e60)
  assert(eqT(math.max(minint, minint + 1), minint + 1))
  assert(eqT(math.min(3), 3))
  assert(eqT(math.min(3, 5, 9, 1), 1))
  assert(math.min(3.2, 5.9, -9.2, 1.1) == -9.2)
  assert(math.min(1.9, 1.7, 1.72) == 1.7)
  assert(math.min(-10e60, minint) == -10e60)
  assert(eqT(math.min(maxint, maxint - 1), maxint - 1))
  assert(eqT(math.min(maxint - 2, maxint, maxint - 1), maxint - 2))
end
-- testing implicit convertions

local a,b = '10', '20'
assert(a*b == 200 and a+b == 30 and a-b == -10 and a/b == 0.5 and -b == -20)
assert(a == '10' and b == '20')


do
  gcs:send_text(6, "testing -0 and NaN")
  local mz, z = -0.0, 0.0
  assert(mz == z)
  assert(1/mz < 0 and 0 < 1/z)
  local a = {[mz] = 1}
  assert(a[z] == 1 and a[mz] == 1)
  a[z] = 2
  assert(a[z] == 2 and a[mz] == 2)
  local inf = math.huge * 2 + 1
  mz, z = -1/inf, 1/inf
  assert(mz == z)
  assert(1/mz < 0 and 0 < 1/z)
  local NaN = inf - inf
  assert(NaN ~= NaN)
  assert(not (NaN < NaN))
  assert(not (NaN <= NaN))
  assert(not (NaN > NaN))
  assert(not (NaN >= NaN))
  assert(not (0 < NaN) and not (NaN < 0))
  local NaN1 = 0/0
  assert(NaN ~= NaN1 and not (NaN <= NaN1) and not (NaN1 <= NaN))
  local a = {}
  assert(not pcall(rawset, a, NaN, 1))
  assert(a[NaN] == nil)
  a[1] = 1
  assert(not pcall(rawset, a, NaN, 1))
  assert(a[NaN] == nil)
  -- strings with same binary representation as 0.0 (might create problems
  -- for constant manipulation in the pre-compiler)
  local a1, a2, a3, a4, a5 = 0, 0, "\0\0\0\0\0\0\0\0", 0, "\0\0\0\0\0\0\0\0"
  assert(a1 == a2 and a2 == a4 and a1 ~= a3)
  assert(a3 == a5)
end


gcs:send_text(6, "testing 'math.random'")
math.randomseed(0)

do   -- test random for floats
  local max = -math.huge
  local min = math.huge
  for i = 0, 20000 do
    local t = math.random()
    assert(0 <= t and t < 1)
    max = math.max(max, t)
    min = math.min(min, t)
    if eq(max, 1, 0.001) and eq(min, 0, 0.001) then
      goto ok
    end
  end
  -- loop ended without satisfing condition
  assert(false)
 ::ok::
end

do
  local function aux (p, lim)   -- test random for small intervals
    local x1, x2
    if #p == 1 then x1 = 1; x2 = p[1]
    else x1 = p[1]; x2 = p[2]
    end
    local mark = {}; local count = 0   -- to check that all values appeared
    for i = 0, lim or 2000 do
      local t = math.random(table.unpack(p))
      assert(x1 <= t and t <= x2)
      if not mark[t] then  -- new value
        mark[t] = true
        count = count + 1
      end
      if count == x2 - x1 + 1 then   -- all values appeared; OK
        goto ok
      end
    end
    -- loop ended without satisfing condition
    assert(false)
   ::ok::
  end

  aux({-10,0})
  aux({6})
  aux({-10, 10})
  aux({minint, minint})
  aux({maxint, maxint})
  aux({minint, minint + 9})
  aux({maxint - 3, maxint})
end

do
  local function aux(p1, p2)       -- test random for large intervals
    local max = minint
    local min = maxint
    local n = 200
    local mark = {}; local count = 0   -- to count how many different values
    for _ = 1, n do
      local t = math.random(p1, p2)
      max = math.max(max, t)
      min = math.min(min, t)
      if not mark[t] then  -- new value
        mark[t] = true
        count = count + 1
      end
    end
    -- at least 80% of values are different
    assert(count >= n * 0.8)
    -- min and max not too far from formal min and max
    local diff = (p2 - p1) // 8
    assert(min < p1 + diff and max > p2 - diff)
  end
  aux(0, maxint)
  aux(1, maxint)
  aux(minint, -1)
  aux(minint // 2, maxint // 2)
end

for i=1,100 do
  assert(math.random(maxint) > 0)
  assert(math.random(minint, -1) < 0)
end

assert(not pcall(math.random, 1, 2, 3))    -- too many arguments

-- empty interval
assert(not pcall(math.random, minint + 1, minint))
assert(not pcall(math.random, maxint, maxint - 1))
assert(not pcall(math.random, maxint, minint))

-- interval too large
assert(not pcall(math.random, minint, 0))
assert(not pcall(math.random, -1, maxint))
assert(not pcall(math.random, minint // 2, maxint // 2 + 1))

function update()
  gcs:send_text(6, 'Math tests passed')
  return update, 1000
end

return update()
