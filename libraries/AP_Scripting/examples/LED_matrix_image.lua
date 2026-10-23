--[[
Script to control LED strips based on the roll of the aircraft. This is an example to demonstrate
the LED interface for WS2812 LEDs
--]]

--[[
for this demo we will use a single strip with 30 LEDs
--]]
local matrix_x = 7
local matrix_y = 7

-- matrix to convert from x y pos to location in the strip
local id = {}
-- because my strips go diagonally to get the led's closer together this is a odd ordering
id[1] = {}
id[1][1] = 21
id[1][2] = 20
id[1][3] = 10
id[1][4] = 9
id[1][5] = 3
id[1][6] = 2
id[1][7] = 0

id[2] = {}
id[2][1] = 33
id[2][2] = 22
id[2][3] = 19
id[2][4] = 11
id[2][5] = 8
id[2][6] = 4
id[2][7] = 1

id[3] = {}
id[3][1] = 34
id[3][2] = 32
id[3][3] = 23
id[3][4] = 18
id[3][5] = 12
id[3][6] = 7
id[3][7] = 5

id[4] = {}
id[4][1] = 42
id[4][2] = 35
id[4][3] = 31
id[4][4] = 24
id[4][5] = 17
id[4][6] = 13
id[4][7] = 6

id[5] = {}
id[5][1] = 43
id[5][2] = 41
id[5][3] = 36
id[5][4] = 30
id[5][5] = 25
id[5][6] = 16
id[5][7] = 14

id[6] = {}
id[6][1] = 47
id[6][2] = 44
id[6][3] = 40
id[6][4] = 37
id[6][5] = 29
id[6][6] = 26
id[6][7] = 15

id[7] = {}
id[7][1] = 48
id[7][2] = 46
id[7][3] = 45
id[7][4] = 39
id[7][5] = 38
id[7][6] = 28
id[7][7] = 27

-- ArduPilot logo 7 x 48, RGB
local image = {}
image[1] = {}
image[2] = {}
image[3] = {}
image[4] = {}
image[5] = {}
image[6] = {}
image[7] = {}

image[1][1] = {0, 0, 0}
image[2][1] = {0, 0, 0}
image[3][1] = {0, 0, 0}
image[4][1] = {0, 0, 0}
image[5][1] = {0, 0, 0}
image[6][1] = {0, 0, 0}
image[7][1] = {0, 0, 0}

image[1][2] = {0, 0, 0}
image[2][2] = {0, 0, 0}
image[3][2] = {0, 0, 0}
image[4][2] = {0, 0, 0}
image[5][2] = {0, 0, 0}
image[6][2] = {0, 0, 0}
image[7][2] = {191, 191, 191}

image[1][3] = {0, 0, 0}
image[2][3] = {0, 0, 0}
image[3][3] = {0, 0, 0}
image[4][3] = {0, 0, 0}
image[5][3] = {0, 0, 0}
image[6][3] = {191, 191, 191}
image[7][3] = {0, 0, 0}

image[1][4] = {0, 0, 0}
image[2][4] = {0, 0, 0}
image[3][4] = {0, 0, 0}
image[4][4] = {196, 196, 196}
image[5][4] = {191, 191, 191}
image[6][4] = {0, 0, 0}
image[7][4] = {191, 191, 191}

image[1][5] = {0, 0, 0}
image[2][5] = {0, 0, 0}
image[3][5] = {0, 0, 0}
image[4][5] = {191, 191, 191}
image[5][5] = {0, 0, 0}
image[6][5] = {0, 0, 0}
image[7][5] = {191, 191, 191}

image[1][6] = {0, 0, 0}
image[2][6] = {0, 0, 0}
image[3][6] = {191, 191, 191}
image[4][6] = {0, 0, 0}
image[5][6] = {191, 191, 191}
image[6][6] = {0, 0, 0}
image[7][6] = {191, 191, 191}

image[1][7] = {0, 0, 0}
image[2][7] = {191, 191, 191}
image[3][7] = {0, 0, 0}
image[4][7] = {0, 0, 0}
image[5][7] = {191, 191, 191}
image[6][7] = {0, 0, 0}
image[7][7] = {191, 191, 191}

image[1][8] = {191, 191, 191}
image[2][8] = {191, 191, 191}
image[3][8] = {191, 191, 191}
image[4][8] = {191, 191, 191}
image[5][8] = {191, 191, 191}
image[6][8] = {191, 191, 191}
image[7][8] = {191, 191, 191}

image[1][9] = {0, 0, 0}
image[2][9] = {0, 0, 0}
image[3][9] = {0, 0, 0}
image[4][9] = {0, 0, 0}
image[5][9] = {0, 0, 0}
image[6][9] = {0, 0, 0}
image[7][9] = {191, 191, 191}

image[1][10] = {0, 0, 0}
image[2][10] = {191, 191, 191}
image[3][10] = {191, 191, 191}
image[4][10] = {191, 191, 191}
image[5][10] = {191, 191, 191}
image[6][10] = {191, 191, 191}
image[7][10] = {191, 191, 191}

image[1][11] = {191, 191, 191}
image[2][11] = {190, 190, 190}
image[3][11] = {0, 0, 0}
image[4][11] = {0, 0, 0}
image[5][11] = {0, 0, 0}
image[6][11] = {0, 0, 0}
image[7][11] = {191, 191, 191}

image[1][12] = {191, 191, 191}
image[2][12] = {192, 192, 192}
image[3][12] = {0, 0, 0}
image[4][12] = {191, 191, 191}
image[5][12] = {0, 0, 0}
image[6][12] = {0, 0, 0}
image[7][12] = {191, 191, 191}

image[1][13] = {191, 191, 191}
image[2][13] = {191, 191, 191}
image[3][13] = {191, 191, 191}
image[4][13] = {191, 191, 191}
image[5][13] = {193, 193, 193}
image[6][13] = {0, 0, 0}
image[7][13] = {191, 191, 191}

image[1][14] = {0, 0, 0}
image[2][14] = {191, 191, 191}
image[3][14] = {191, 191, 191}
image[4][14] = {0, 0, 0}
image[5][14] = {191, 191, 191}
image[6][14] = {187, 187, 187}
image[7][14] = {191, 191, 191}

image[1][15] = {0, 0, 0}
image[2][15] = {191, 191, 191}
image[3][15] = {191, 191, 191}
image[4][15] = {191, 191, 191}
image[5][15] = {191, 191, 191}
image[6][15] = {191, 191, 191}
image[7][15] = {191, 191, 191}

image[1][16] = {195, 195, 195}
image[2][16] = {191, 191, 191}
image[3][16] = {191, 191, 191}
image[4][16] = {191, 191, 191}
image[5][16] = {191, 191, 191}
image[6][16] = {191, 191, 191}
image[7][16] = {191, 191, 191}

image[1][17] = {191, 191, 191}
image[2][17] = {190, 190, 190}
image[3][17] = {0, 0, 0}
image[4][17] = {0, 0, 0}
image[5][17] = {190, 190, 190}
image[6][17] = {191, 191, 191}
image[7][17] = {191, 191, 191}

image[1][18] = {191, 191, 191}
image[2][18] = {191, 191, 191}
image[3][18] = {0, 0, 0}
image[4][18] = {0, 0, 0}
image[5][18] = {191, 191, 191}
image[6][18] = {191, 191, 191}
image[7][18] = {191, 191, 191}

image[1][19] = {0, 0, 0}
image[2][19] = {191, 191, 191}
image[3][19] = {191, 191, 191}
image[4][19] = {191, 191, 191}
image[5][19] = {191, 191, 191}
image[6][19] = {0, 0, 0}
image[7][19] = {191, 191, 191}

image[1][20] = {0, 0, 0}
image[2][20] = {0, 0, 0}
image[3][20] = {0, 0, 0}
image[4][20] = {0, 0, 0}
image[5][20] = {0, 0, 0}
image[6][20] = {0, 0, 0}
image[7][20] = {191, 191, 191}

image[1][21] = {0, 0, 0}
image[2][21] = {191, 191, 191}
image[3][21] = {191, 191, 191}
image[4][21] = {191, 191, 191}
image[5][21] = {191, 191, 191}
image[6][21] = {0, 0, 0}
image[7][21] = {191, 191, 191}

image[1][22] = {191, 191, 191}
image[2][22] = {192, 192, 192}
image[3][22] = {192, 192, 192}
image[4][22] = {192, 192, 192}
image[5][22] = {191, 191, 191}
image[6][22] = {191, 191, 191}
image[7][22] = {191, 191, 191}

image[1][23] = {0, 0, 0}
image[2][23] = {0, 0, 0}
image[3][23] = {0, 0, 0}
image[4][23] = {0, 0, 0}
image[5][23] = {191, 191, 191}
image[6][23] = {191, 191, 191}
image[7][23] = {191, 191, 191}

image[1][24] = {191, 191, 191}
image[2][24] = {191, 191, 191}
image[3][24] = {191, 191, 191}
image[4][24] = {191, 191, 191}
image[5][24] = {191, 191, 191}
image[6][24] = {0, 0, 0}
image[7][24] = {191, 191, 191}

image[1][25] = {192, 192, 192}
image[2][25] = {192, 192, 192}
image[3][25] = {192, 192, 192}
image[4][25] = {192, 192, 192}
image[5][25] = {0, 0, 0}
image[6][25] = {0, 0, 0}
image[7][25] = {191, 191, 191}

image[1][26] = {0, 0, 0}
image[2][26] = {254, 210, 15}
image[3][26] = {251, 195, 20}
image[4][26] = {249, 179, 23}
image[5][26] = {249, 163, 26}
image[6][26] = {244, 150, 28}
image[7][26] = {191, 191, 191}

image[1][27] = {255, 223, 11}
image[2][27] = {254, 211, 18}
image[3][27] = {244, 196, 36}
image[4][27] = {242, 181, 41}
image[5][27] = {240, 166, 41}
image[6][27] = {237, 152, 46}
image[7][27] = {191, 191, 191}

image[1][28] = {255, 221, 12}
image[2][28] = {253, 210, 20}
image[3][28] = {0, 0, 0}
image[4][28] = {249, 179, 23}
image[5][28] = {0, 0, 0}
image[6][28] = {0, 0, 0}
image[7][28] = {191, 191, 191}

image[1][29] = {252, 222, 12}
image[2][29] = {253, 210, 18}
image[3][29] = {252, 195, 20}
image[4][29] = {249, 179, 23}
image[5][29] = {0, 0, 0}
image[6][29] = {0, 0, 0}
image[7][29] = {191, 191, 191}

image[1][30] = {0, 0, 0}
image[2][30] = {253, 210, 18}
image[3][30] = {251, 195, 20}
image[4][30] = {248, 179, 23}
image[5][30] = {0, 0, 0}
image[6][30] = {0, 0, 0}
image[7][30] = {191, 191, 191}

image[1][31] = {0, 0, 0}
image[2][31] = {0, 0, 0}
image[3][31] = {0, 0, 0}
image[4][31] = {0, 0, 0}
image[5][31] = {0, 0, 0}
image[6][31] = {0, 0, 0}
image[7][31] = {191, 191, 191}

image[1][32] = {0, 0, 0}
image[2][32] = {253, 210, 18}
image[3][32] = {251, 195, 20}
image[4][32] = {249, 179, 23}
image[5][32] = {249, 163, 26}
image[6][32] = {244, 150, 28}
image[7][32] = {191, 191, 191}

image[1][33] = {0, 0, 0}
image[2][33] = {0, 0, 0}
image[3][33] = {0, 0, 0}
image[4][33] = {0, 0, 0}
image[5][33] = {0, 0, 0}
image[6][33] = {0, 0, 0}
image[7][33] = {191, 191, 191}

image[1][34] = {0, 0, 0}
image[2][34] = {253, 210, 17}
image[3][34] = {251, 195, 20}
image[4][34] = {249, 179, 23}
image[5][34] = {249, 163, 26}
image[6][34] = {244, 150, 28}
image[7][34] = {191, 191, 191}

image[1][35] = {0, 0, 0}
image[2][35] = {0, 0, 0}
image[3][35] = {0, 0, 0}
image[4][35] = {0, 0, 0}
image[5][35] = {250, 162, 26}
image[6][35] = {244, 150, 28}
image[7][35] = {191, 191, 191}

image[1][36] = {0, 0, 0}
image[2][36] = {0, 0, 0}
image[3][36] = {0, 0, 0}
image[4][36] = {0, 0, 0}
image[5][36] = {249, 163, 26}
image[6][36] = {244, 150, 28}
image[7][36] = {191, 191, 191}

image[1][37] = {0, 0, 0}
image[2][37] = {0, 0, 0}
image[3][37] = {252, 196, 21}
image[4][37] = {248, 179, 23}
image[5][37] = {0, 0, 0}
image[6][37] = {0, 0, 0}
image[7][37] = {191, 191, 191}

image[1][38] = {0, 0, 0}
image[2][38] = {253, 210, 18}
image[3][38] = {0, 0, 0}
image[4][38] = {0, 0, 0}
image[5][38] = {249, 163, 26}
image[6][38] = {0, 0, 0}
image[7][38] = {191, 191, 191}

image[1][39] = {249, 223, 14}
image[2][39] = {0, 0, 0}
image[3][39] = {244, 193, 22}
image[4][39] = {247, 187, 41}
image[5][39] = {0, 0, 0}
image[6][39] = {245, 149, 28}
image[7][39] = {191, 191, 191}

image[1][40] = {252, 222, 14}
image[2][40] = {0, 0, 0}
image[3][40] = {249, 196, 18}
image[4][40] = {249, 179, 19}
image[5][40] = {0, 0, 0}
image[6][40] = {0, 0, 0}
image[7][40] = {191, 191, 191}

image[1][41] = {255, 222, 13}
image[2][41] = {250, 214, 18}
image[3][41] = {0, 0, 0}
image[4][41] = {0, 0, 0}
image[5][41] = {247, 164, 22}
image[6][41] = {244, 147, 32}
image[7][41] = {191, 191, 191}

image[1][42] = {0, 0, 0}
image[2][42] = {255, 209, 17}
image[3][42] = {251, 195, 20}
image[4][42] = {249, 179, 23}
image[5][42] = {247, 164, 26}
image[6][42] = {0, 0, 0}
image[7][42] = {191, 191, 191}

image[1][43] = {0, 0, 0}
image[2][43] = {249, 213, 15}
image[3][43] = {0, 0, 0}
image[4][43] = {0, 0, 0}
image[5][43] = {0, 0, 0}
image[6][43] = {0, 0, 0}
image[7][43] = {191, 191, 191}

image[1][44] = {249, 221, 15}
image[2][44] = {244, 211, 18}
image[3][44] = {0, 0, 0}
image[4][44] = {0, 0, 0}
image[5][44] = {0, 0, 0}
image[6][44] = {0, 0, 0}
image[7][44] = {191, 191, 191}

image[1][45] = {255, 221, 12}
image[2][45] = {253, 210, 18}
image[3][45] = {251, 195, 20}
image[4][45] = {249, 179, 23}
image[5][45] = {249, 163, 26}
image[6][45] = {245, 149, 26}
image[7][45] = {191, 191, 191}

image[1][46] = {255, 221, 12}
image[2][46] = {251, 211, 17}
image[3][46] = {0, 0, 0}
image[4][46] = {0, 0, 0}
image[5][46] = {0, 0, 0}
image[6][46] = {0, 0, 0}
image[7][46] = {191, 191, 191}

image[1][47] = {255, 221, 12}
image[2][47] = {0, 0, 0}
image[3][47] = {0, 0, 0}
image[4][47] = {0, 0, 0}
image[5][47] = {0, 0, 0}
image[6][47] = {0, 0, 0}
image[7][47] = {0, 0, 0}

image[1][48] = {0, 0, 0}
image[2][48] = {0, 0, 0}
image[3][48] = {0, 0, 0}
image[4][48] = {0, 0, 0}
image[5][48] = {0, 0, 0}
image[6][48] = {0, 0, 0}
image[7][48] = {0, 0, 0}

--[[
 use SERVOn_FUNCTION 94 for LED. We can control up to 16 separate strips of LEDs
 by putting them on different channels
--]]
local chan = SRV_Channels:find_channel(94)

if not chan then
    gcs:send_text(6, "LEDs: channel not set")
    return
end

-- find_channel returns 0 to 15, convert to 1 to 16
chan = chan + 1

gcs:send_text(6, "LEDs: chan=" .. tostring(chan))

-- initialisation code
--serialLED:set_num_neopixel(chan,  matrix_x * matrix_y)
serialLED:set_num_profiled(chan,  matrix_x * matrix_y)

local offset = 8;

local function display_image(image_in,offset_in,brightness_in)
    local im_offset = 0
    if offset_in then
        im_offset = offset_in
    end
    local brightness = 1
    if brightness_in then
        brightness = brightness_in
    end

    for i = 1, 48 do
        local x_index = i + im_offset
        if x_index >= 1 and x_index <= matrix_x then
            for j = 1, matrix_y do
                serialLED:set_RGB(chan, id[j][x_index], math.floor(image_in[j][i][1]*brightness), math.floor(image_in[j][i][2]*brightness), math.floor(image_in[j][i][3]*brightness))
            end
        end
    end
    
end


function update_LEDs()

  serialLED:set_RGB(chan, -1, 0, 0, 0)

  display_image(image,offset,0.05) 

  serialLED:send(chan)

  offset = offset - 1

 -- scroll until it is off the left edge
  if offset < - 48 - 8 then
    -- start with the stuff off the right edge of the display
    offset = 8
  end


  return update_LEDs, 100
end

return update_LEDs, 1000
