#!/usr/bin/env python
'''
setup board.h for esp32 derived from chibios script.
'''

import argparse
import sys
import fnmatch
import os
import dma_resolver
import shlex
import pickle
import re
import shutil
import filecmp

#parser = argparse.ArgumentParser("chibios_pins.py")
#parser = argparse.ArgumentParser("chibios_hwdef.py")
parser = argparse.ArgumentParser("esp32_hwdef.py")
parser.add_argument(
    '-D', '--outdir', type=str, default=None, help='Output directory')
parser.add_argument(
    '--bootloader', action='store_true', default=False, help='configure for bootloader')
#/home/buzz2/ardupilot/libraries/AP_HAL_ESP32/hwdef/esp32s3buzz/hwdef.dat
parser.add_argument(
    'hwdef', type=str, nargs='+', default='None', help='hardware definition file')
parser.add_argument(
    '--params', type=str, default=None, help='user default params path')

args = parser.parse_args()

# output variables for each pin
f4f7_vtypes = ['MODER', 'OTYPER', 'OSPEEDR', 'PUPDR', 'ODR', 'AFRL', 'AFRH']
#f1_vtypes = ['CRL', 'CRH', 'ODR']
#f1_input_sigs = ['RX', 'MISO', 'CTS']
#f1_output_sigs = ['TX', 'MOSI', 'SCK', 'RTS', 'CH1', 'CH2', 'CH3', 'CH4']
af_labels = ['USART', 'UART', 'SPI', 'I2C', 'SDIO', 'SDMMC', 'OTG', 'JT', 'TIM', 'CAN', 'QUADSPI']

default_gpio = ['INPUT', 'FLOATING']

vtypes = []

# boolean indicating whether we have read and processed hwdef yet
processed_hwdefs = False

#classic_esp32 full pis list
classic_esp32_allpins = [
    0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,8,19, 
     21,22,23, 
     25,26,27, 
     32,33,34,35,36,37,38,39
     ]

classic_esp32_allowedoutpins = [
     0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,8,19,  
     21,22,23, 
     25,26,27,
     32,33
    ]
classic_esp32_inonlypins = [
34,35,36,37,38,39
]

# according to 'Figure 7' in 'classic' esp32 ref man, the connection from peripheral/s to digital pads is:
# peripheral (eg uart,i2c,pwm, led etc)[162 possible signals] -> gpio_matrix
# gpio_matrix[34 gpios] -> io_mux
# io_mux[pad control signals] -> digital pads
#..
# or for high-speed idrect stuff[jtag,uart,sdio,spi,ethernet]:
# direct-io -> -> io_mux
# io_mux[pad control signals] -> digital pads
#..
# RTC stuff is different and has its own mux

#. The IO_MUX contains one register per GPIO pad. 
# Each pad can be configured to perform a âGPIOâ function
#(when connected to the GPIO Matrix) or a direct function (bypassing the GPIO Matrix). 
# Some high-speed digital functions (Ethernet, SDIO, SPI, JTAG, UART) can bypass the GPIO Matrix for better high-frequency
#digital performance. In this case, the IO_MUX is used to connect these pads directly to the peripheral.)


# esp32 'classic' gpio matrix...
# For input to the chip: Each of the 162 internal peripheral inputs can select any GPIO pad as the input source
#
# For output from the chip: The output signal of each of the 34 GPIO pads can be from one of the 176
#  peripheral output signals.


# table 17 in ref man
#Signal Input_Signal Output_Signal Direct_I/O_in_IO_MUX
classic_esp32__gpio_matrix = {
'0': ['SPICLK_in' ,'SPICLK_out'             ,'YES'],
'1': ['SPIQ_in' ,'SPIQ_out'                 ,'YES'],
'2': ['SPID_in' ,'SPID_out'                 ,'YES'],
'3': ['SPIHD_in' ,'SPIHD_out'               ,'YES'],
'4': ['SPIWP_in' ,'SPIWP_out'               ,'YES'],
'5': ['SPICS0_in' ,'SPICS0_out'             ,'YES'],
'6': ['SPICS1_in' ,'SPICS1_out'    ,'-'],
'7': ['SPICS2_in' ,'SPICS2_out'    ,'-'],
'8': ['HSPICLK_in' ,'HSPICLK_out'           ,'YES'],
'9': ['HSPIQ_in' ,'HSPIQ_out'               ,'YES'],
'10':['HSPID_in' ,'HSPID_out'               ,'YES'],
'11':['HSPICS0_in' ,'HSPICS0_out'           ,'YES'],
'12':['HSPIHD_in' ,'HSPIHD_out'             ,'YES'],
'13':['HSPIWP_in' ,'HSPIWP_out'             ,'YES'],
'14':['U0RXD_in' ,'U0TXD_out'               ,'YES'],
'15':['U0CTS_in' ,'U0RTS_out'               ,'YES'],
'16':['U0DSR_in' ,'U0DTR_out'      ,'-'],
'17':['U1RXD_in' ,'U1TXD_out'               ,'YES'],
'18':['U1CTS_in' ,'U1RTS_out'               ,'YES'],
'23':['I2S0O_BCK_in' ,'I2S0O_BCK_out'  ,'-'],
'24':['I2S1O_BCK_in' ,'I2S1O_BCK_out'  ,'-'],
'25':['I2S0O_WS_in' ,'I2S0O_WS_out'    ,'-'],
'26':['I2S1O_WS_in' ,'I2S1O_WS_out'    ,'-'],
'27':['I2S0I_BCK_in' ,'I2S0I_BCK_out'  ,'-'],
'28':['I2S0I_WS_in' ,'I2S0I_WS_out'    ,'-'],
'29':['I2CEXT0_SCL_in' ,'I2CEXT0_SCL_out' ,'-'],
'30':['I2CEXT0_SDA_in' ,'I2CEXT0_SDA_out' ,'-'],
'31':['pwm0_sync0_in' ,'sdio_tohost_int_out' ,'-'],
'32':['pwm0_sync1_in' ,'pwm0_out0a' ,'-'],
'33':['pwm0_sync2_in' ,'pwm0_out0b' ,'-'],
'34':['pwm0_f0_in' ,'pwm0_out1a' ,'-'],
'35':['pwm0_f1_in' ,'pwm0_out1b' ,'-'],
'36':['pwm0_f2_in' ,'pwm0_out2a' ,'-'],
'37':['-' ,'pwm0_out2b' ,'-'],
'39':['pcnt_sig_ch0_in0' ,'-' ,'-'],
'40':['pcnt_sig_ch1_in0' ,'-' ,'-'],
'41':['pcnt_ctrl_ch0_in0' ,'-' ,'-'],
'42':['pcnt_ctrl_ch1_in0' ,'-' ,'-'],
'43':['pcnt_sig_ch0_in1' ,'-' ,'-'],
'44':['pcnt_sig_ch1_in1' ,'-' ,'-'],
'45':['pcnt_ctrl_ch0_in1' ,'-' ,'-'],
'46':['pcnt_ctrl_ch1_in1' ,'-' ,'-'],
'47':['pcnt_sig_ch0_in2' ,'-' ,'-'],
'48':['pcnt_sig_ch1_in2' ,'-' ,'-'],
'49':['pcnt_ctrl_ch0_in2' ,'-' ,'-'],
'50':['pcnt_ctrl_ch1_in2' ,'-' ,'-'],
'51':['pcnt_sig_ch0_in3' ,'-' ,'-'],
'52':['pcnt_sig_ch1_in3' ,'-' ,'-'],
'53':['pcnt_ctrl_ch0_in3' ,'-' ,'-'],
'54':['pcnt_ctrl_ch1_in3' ,'-' ,'-'],
'55':['pcnt_sig_ch0_in4' ,'-' ,'-'],
'56':['pcnt_sig_ch1_in4' ,'-' ,'-'],
'57':['pcnt_ctrl_ch0_in4' ,'-' ,'-'],
'58':['pcnt_ctrl_ch1_in4' ,'-' ,'-'],
'61':['HSPICS1_in' ,'HSPICS1_out' ,'-'],
'62':['HSPICS2_in' ,'HSPICS2_out' ,'-'],
'63':['VSPICLK_in' ,'VSPICLK_out_mux'            ,'YES'],
'64':['VSPIQ_in' ,'VSPIQ_out'                    ,'YES'],
'65':['VSPID_in' ,'VSPID_out'                    ,'YES'],
'66':['VSPIHD_in' ,'VSPIHD_out'                  ,'YES'],
'67':['VSPIWP_in' ,'VSPIWP_out'                  ,'YES'],
'68':['VSPICS0_in' ,'VSPICS0_out'                ,'YES'],
'69':['VSPICS1_in' ,'VSPICS1_out'            ,'-'],
'70':['VSPICS2_in' ,'VSPICS2_out'            ,'-'],
'71':['pcnt_sig_ch0_in5' ,'ledc_hs_sig_out0' ,'-'],
'72':['pcnt_sig_ch1_in5' ,'ledc_hs_sig_out1' ,'-'],
'73':['pcnt_ctrl_ch0_in5' ,'ledc_hs_sig_out2' ,'-'],
'74':['pcnt_ctrl_ch1_in5' ,'ledc_hs_sig_out3' ,'-'],
'75':['pcnt_sig_ch0_in6' ,'ledc_hs_sig_out4' ,'-'],
'76':['pcnt_sig_ch1_in6' ,'ledc_hs_sig_out5' ,'-'],
'77':['pcnt_ctrl_ch0_in6' ,'ledc_hs_sig_out6' ,'-'],
'78':['pcnt_ctrl_ch1_in6' ,'ledc_hs_sig_out7' ,'-'],
'79':['pcnt_sig_ch0_in7' ,'ledc_ls_sig_out0' ,'-'],
'80':['pcnt_sig_ch1_in7' ,'ledc_ls_sig_out1' ,'-'],
'81':['pcnt_ctrl_ch0_in7' ,'ledc_ls_sig_out2' ,'-'],
'82':['pcnt_ctrl_ch1_in7' ,'ledc_ls_sig_out3' ,'-'],
'83':['rmt_sig_in0' ,'ledc_ls_sig_out4' ,'-'],
'84':['rmt_sig_in1' ,'ledc_ls_sig_out5' ,'-'],
'85':['rmt_sig_in2' ,'ledc_ls_sig_out6' ,'-'],
'86':['rmt_sig_in3' ,'ledc_ls_sig_out7' ,'-'],
'87':['rmt_sig_in4' ,'rmt_sig_out0' ,'-'],
'88':['rmt_sig_in5' ,'rmt_sig_out1' ,'-'],
'89':['rmt_sig_in6' ,'rmt_sig_out2' ,'-'],
'90':['rmt_sig_in7' ,'rmt_sig_out3' ,'-'],
'91':['-' ,'rmt_sig_out4' ,'-'],
'92':['-' ,'rmt_sig_out5' ,'-'],
'93':['-' ,'rmt_sig_out6' ,'-'],
'94':['-' ,'rmt_sig_out7' ,'-'],
'95':['I2CEXT1_SCL_in' ,'I2CEXT1_SCL_out' ,'-'],
'96':['I2CEXT1_SDA_in' ,'I2CEXT1_SDA_out' ,'-'],
'97':['host_card_detect_n_1' ,'host_ccmd_od_pullup_en_n' ,'-'],
'98':['host_card_detect_n_2' ,'host_rst_n_1' ,'-'],
'99':['host_card_write_prt_1' ,'host_rst_n_2' ,'-'],
'100':['host_card_write_prt_2' ,'gpio_sd0_out' ,'-'],
'101':['host_card_int_n_1' ,'gpio_sd1_out' ,'-'],
'102':['host_card_int_n_2' ,'gpio_sd2_out' ,'-'],
'103':['pwm1_sync0_in' ,'gpio_sd3_out' ,'-'],
'104':['pwm1_sync1_in' ,'gpio_sd4_out' ,'-'],
'105':['pwm1_sync2_in' ,'gpio_sd5_out' ,'-'],
'106':['pwm1_f0_in' ,'gpio_sd6_out' ,'-'],
'107':['pwm1_f1_in' ,'gpio_sd7_out' ,'-'],
'108':['pwm1_f2_in' ,'pwm1_out0a' ,'-'],
'109':['pwm0_cap0_in' ,'pwm1_out0b' ,'-'],
'110':['pwm0_cap1_in' ,'pwm1_out1a' ,'-'],
'111':['pwm0_cap2_in' ,'pwm1_out1b' ,'-'],
'112':['pwm1_cap0_in' ,'pwm1_out2a' ,'-'],
'113':['pwm1_cap1_in' ,'pwm1_out2b' ,'-'],
'114':['pwm1_cap2_in' ,'-' ,'-'],
'115':['-' ,'-' ,'-'],
'116':['-' ,'-' ,'-'],
'117':['-' ,'-' ,'-'],
'118':['-' ,'-' ,'-'],
'119':['-' ,'-' ,'-'],
'120':['-' ,'-' ,'-'],
'121':['-' ,'-' ,'-'],
'122':['-' ,'-' ,'-'],
'123':['-' ,'-' ,'-'],
'124':['-' ,'-' ,'-'],
'140':['I2S0I_DATA_in0' ,'I2S0O_DATA_out0' ,'-'],
'141':['I2S0I_DATA_in1' ,'I2S0O_DATA_out1' ,'-'],
'142':['I2S0I_DATA_in2' ,'I2S0O_DATA_out2' ,'-'],
'143':['I2S0I_DATA_in3' ,'I2S0O_DATA_out3' ,'-'],
'144':['I2S0I_DATA_in4' ,'I2S0O_DATA_out4' ,'-'],
'145':['I2S0I_DATA_in5' ,'I2S0O_DATA_out5' ,'-'],
'146':['I2S0I_DATA_in6' ,'I2S0O_DATA_out6' ,'-'],
'147':['I2S0I_DATA_in7' ,'I2S0O_DATA_out7' ,'-'],
'148':['I2S0I_DATA_in8' ,'I2S0O_DATA_out8' ,'-'],
'149':['I2S0I_DATA_in9' ,'I2S0O_DATA_out9' ,'-'],
'150':['I2S0I_DATA_in10' ,'I2S0O_DATA_out10' ,'-'],
'151':['I2S0I_DATA_in11' ,'I2S0O_DATA_out11' ,'-'],
'152':['I2S0I_DATA_in12' ,'I2S0O_DATA_out12' ,'-'],
'153':['I2S0I_DATA_in13' ,'I2S0O_DATA_out13' ,'-'],
'154':['I2S0I_DATA_in14' ,'I2S0O_DATA_out14' ,'-'],
'155':['I2S0I_DATA_in15' ,'I2S0O_DATA_out15' ,'-'],
'156':['-' ,'I2S0O_DATA_out16' ,'-'],
'157':['-' ,'I2S0O_DATA_out17' ,'-'],
'158':['-' ,'I2S0O_DATA_out18' ,'-'],
'159':['-' ,'I2S0O_DATA_out19' ,'-'],
'160':['-' ,'I2S0O_DATA_out20' ,'-'],
'161':['-' ,'I2S0O_DATA_out21' ,'-'],
'162':['-' ,'I2S0O_DATA_out22' ,'-'],
'163':['-' ,'I2S0O_DATA_out23' ,'-'],
'164':['I2S1I_BCK_in' ,'I2S1I_BCK_out' ,'-'],
'165':['I2S1I_WS_in' ,'I2S1I_WS_out' ,'-'],
'166':['I2S1I_DATA_in0' ,'I2S1O_DATA_out0' ,'-'],
'167':['I2S1I_DATA_in1' ,'I2S1O_DATA_out1' ,'-'],
'168':['I2S1I_DATA_in2' ,'I2S1O_DATA_out2' ,'-'],
'169':['I2S1I_DATA_in3' ,'I2S1O_DATA_out3' ,'-'],
'170':['I2S1I_DATA_in4' ,'I2S1O_DATA_out4' ,'-'],
'171':['I2S1I_DATA_in5' ,'I2S1O_DATA_out5' ,'-'],
'172':['I2S1I_DATA_in6' ,'I2S1O_DATA_out6' ,'-'],
'173':['I2S1I_DATA_in7' ,'I2S1O_DATA_out7' ,'-'],
'174':['I2S1I_DATA_in8' ,'I2S1O_DATA_out8' ,'-'],
'175':['I2S1I_DATA_in9' ,'I2S1O_DATA_out9' ,'-'],
'176':['I2S1I_DATA_in10' ,'I2S1O_DATA_out10' ,'-'],
'177':['I2S1I_DATA_in11' ,'I2S1O_DATA_out11' ,'-'],
'178':['I2S1I_DATA_in12' ,'I2S1O_DATA_out12' ,'-'],
'179':['I2S1I_DATA_in13' ,'I2S1O_DATA_out13' ,'-'],
'180':['I2S1I_DATA_in14' ,'I2S1O_DATA_out14' ,'-'],
'181':['I2S1I_DATA_in15' ,'I2S1O_DATA_out15' ,'-'],
'182':['-' ,'I2S1O_DATA_out16' ,'-'],
'183':['-' ,'I2S1O_DATA_out17' ,'-'],
'184':['-' ,'I2S1O_DATA_out18' ,'-'],
'185':['-' ,'I2S1O_DATA_out19' ,'-'],
'186':['-' ,'I2S1O_DATA_out20' ,'-'],
'187':['-' ,'I2S1O_DATA_out21' ,'-'],
'188':['-' ,'I2S1O_DATA_out22' ,'-'],
'189':['-' ,'I2S1O_DATA_out23' ,'-'],
'190':['I2S0I_H_SYNC' ,'-' ,'-'],
'191':['I2S0I_V_SYNC' ,'-' ,'-'],
'192':['I2S0I_H_ENABLE' ,'-' ,'-'],
'193':['I2S1I_H_SYNC' ,'-' ,'-'],
'194':['I2S1I_V_SYNC' ,'-' ,'-'],
'195':['I2S1I_H_ENABLE' ,'-' ,'-'],
'196':['-' ,'-' ,'-'],
'197':['-' ,'-' ,'-'],
'198':['U2RXD_in' ,'U2TXD_out'                 ,'YES'],
'199':['U2CTS_in' ,'U2RTS_out'                 ,'YES'],
'200':['emac_mdc_i' ,'emac_mdc_o' ,'-'],
'201':['emac_mdi_i' ,'emac_mdo_o' ,'-'],
'202':['emac_crs_i' ,'emac_crs_o' ,'-'],
'203':['emac_col_i' ,'emac_col_o' ,'-'],
'204':['pcmfsync_in' ,'bt_audio0_irq' ,'-'],
'205':['pcmclk_in' ,'bt_audio1_irq' ,'-'],
'206':['pcmdin' ,'bt_audio2_irq' ,'-'],
'207':['-' ,'ble_audio0_irq' ,'-'],
'208':['-' ,'ble_audio1_irq' ,'-'],
'209':['-' ,'ble_audio2_irq' ,'-'],
'210':['-' ,'pcmfsync_out' ,'-'],
'211':['-' ,'pcmclk_out' ,'-'],
'212':['-' ,'pcmdout' ,'-'],
'213':['-' ,'ble_audio_sync0_p' ,'-'],
'214':['-' ,'ble_audio_sync1_p' ,'-'],
'215':['-' ,'ble_audio_sync2_p' ,'-'],
'224':['-' ,'sig_in_func224' ,'-'],
'225':['-' ,'sig_in_func225' ,'-'],
'226':['-' ,'sig_in_func226' ,'-'],
'227':['-' ,'sig_in_func227' ,'-'],
'228':['-' ,'sig_in_func228', '-']
}

# tbl 18 in 'classic' esp32 ref man  has mux-pad-summary.

#GPIO Pad_Name Function_0 Function_1 Function_2 Function_3 Function_4 Function_5 Reset Notes
# 'Reset' column shows each pads default configurations after reset:
#  0=input-disabled
#  1=input-enabled
#  2=input-enabled-pull-down
#  3=input-enabled-pull-up
# "Notes" : R => RTC/analog functions via RTC_MUX. ; I => Pad can only be configured as input GPIO,  no pullup/down avail.

classic_esp32_io_mux_pad_list = {

#GPIO Pad_Name   Function_0 Function_1 Function_2 Function_3 Function_4   Function_5    Reset Notes
# esp32 doesn't really have a concept of a 'port' like stm32, its got pads and pad labels, so we'll group them in 8's and call them ports where we have to
# "virtual port A":                                                                                  #our virtual pin name
'0':['GPIO0'     ,'GPIO0'  ,'CLK_OUT1','GPIO0'   ,'-'         ,'-'       ,'EMAC_TX_CLK' ,'3','R'],    # PA0
'1':['U0TXD'     ,'U0TXD'  ,'CLK_OUT3','GPIO1'   ,'-'         ,'-'       ,'EMAC_RXD2'   ,'3','-'],    # PA1
'2':['GPIO2'     ,'GPIO2'  ,'HSPIWP'  ,'GPIO2'   ,'HS2_DATA0' ,'SD_DATA0','-'           ,'2','R'],    # PA2
'3':['U0RXD'     ,'U0RXD'  ,'CLK_OUT2','GPIO3'   ,'-'         ,'-'       ,'-'           ,'3','-'],    # PA3
'4':['GPIO4'     ,'GPIO4'   ,'HSPIHD' ,'GPIO4'   ,'HS2_DATA1' ,'SD_DATA1','EMAC_TX_ER'  ,'2','R'],    # PA4
'5':['GPIO5'     ,'GPIO5'   ,'VSPICS0','GPIO5'   ,'HS1_DATA6' ,'-'       ,'EMAC_RX_CLK' ,'3','-'],    # PA5
'6':['SD_CLK'    ,'SD_CLK'  ,'SPICLK' ,'GPIO6'   ,'HS1_CLK'   ,'U1CTS'   ,'-'           ,'3','-'],    # PA6
'7':['SD_DATA_0' ,'SD_DATA0','SPIQ'   ,'GPIO7'   ,'HS1_DATA0' ,'U2RTS'   ,'-'           ,'3','-'],    # PA7
# "virtual port B":
'8':['SD_DATA_1' ,'SD_DATA1','SPID'   ,'GPIO8'   ,'HS1_DATA1' ,'U2CTS'   ,'-'           ,'3','-'],    # PB0
'9':['SD_DATA_2' ,'SD_DATA2','SPIHD'  ,'GPIO9'   ,'HS1_DATA2' ,'U1RXD'   ,'-'           ,'3','-'],    # PB1
'10':['SD_DATA_3','SD_DATA3','SPIWP'  ,'GPIO10'  ,'HS1_DATA3' ,'U1TXD'   ,'-'           ,'3','-'],    # PB2
'11':['SD_CMD'   ,'SD_CMD'  ,'SPICS0' ,'GPIO11'  ,'HS1_CMD'   ,'U1RTS'   ,'-'           ,'3','-'],    # PB3
'12':['MTDI'     ,'MTDI'    ,'HSPIQ'  ,'GPIO12'  ,'HS2_DATA2' ,'SD_DATA2','EMAC_TXD3'   ,'2','R'],    # PB4
'13':['MTCK'     ,'MTCK'    ,'HSPID'  ,'GPIO13'  , 'HS2_DATA3' ,'SD_DATA3','EMAC_RX_ER'  ,'2','R'],   # PB5
'14':['MTMS'     ,'MTMS'    ,'HSPICLK','GPIO14'  ,'HS2_CLK'   ,'SD_CLK'  ,'EMAC_TXD2'   ,'3','R'],    # PB6
'15':['MTDO'     ,'MTDO'    ,'HSPICS0','GPIO15'  ,'HS2_CMD'   ,'SD_CMD'  ,'EMAC_RXD3'   ,'3','R'],    # PB7
# "virtual port C":
'16':['GPIO16'   ,'GPIO16'  ,'-'      ,'GPIO16'  ,'HS1_DATA4' ,'U2RXD'   ,'EMAC_CLK_OUT','1','-'],    # PC0
'17':['GPIO17'   ,'GPIO17'  ,'-'      ,'GPIO17'  ,'HS1_DATA5' ,'U2TXD'   ,'EMAC_CLK_180','1','-'],    # PC1
'18':['GPIO18'   ,'GPIO18'  ,'VSPICLK','GPIO18'  ,'HS1_DATA7' ,'-'       ,'-'           ,'1','-'],    # PC2
'19':['GPIO19'   ,'GPIO19'  ,'VSPIQ'  ,'GPIO19'  ,'U0CTS'     ,'-'       ,'EMAC_TXD0'   ,'1','-'],    # PC3
'21':['GPIO21'   ,'GPIO21'  ,'VSPIHD' ,'GPIO21'  ,'-'         ,'-'       ,'EMAC_TX_EN'  ,'1','-'],    # PC4
'22':['GPIO22'   ,'GPIO22'  ,'VSPIWP' ,'GPIO22'  ,'U0RTS'     ,'-'       ,'EMAC_TXD1'   ,'1','-'],    # PC5
'23':['GPIO23'   ,'GPIO23'  ,'VSPID'  ,'GPIO23'  ,'HS1_STROBE','-'       ,'-'           ,'1','-'],    # PC6
'25':['GPIO25'   ,'GPIO25'  ,'-'      ,'GPIO25'  ,'-'         ,'-'       ,'EMAC_RXD0'   ,'0','R'],    # PC7
# "virtual port D":
'26':['GPIO26'   ,'GPIO26'  ,'-'      ,'GPIO26'  ,'-'         ,'-'       ,'EMAC_RXD1'   ,'0','R'],    # PD0
'27':['GPIO27'   ,'GPIO27'  ,'-'      ,'GPIO27'  ,'-'         ,'-'       ,'EMAC_RX_DV'  ,'0','R'],    # PD1
'28': [], # not in table
'29': [], # not in table
'30': [], # not in table
'31': [], # not in table
# there is 28,29,30,31 in this table, but they are implied to be part of port D just unused.
# "virtual port E":
'32':['32K_XP'   ,'GPIO32'  ,'-'      ,'GPIO32'  ,'-'         ,'-'       ,'-'           ,'0','R'],         # PE0
'33':['32K_XN'   ,'GPIO33'  ,'-'      ,'GPIO33'  ,'-'         ,'-'       ,'-'           ,'0','R'],         # PE1
'34':['VDET_1'   ,'GPIO34'  ,'-'      ,'GPIO34'  ,'-'         ,'-'       ,'-'           ,'0','R,','I'],    # PE2
'35':['VDET_2'   ,'GPIO35'  ,'-'      ,'GPIO35'  ,'-'         ,'-'       ,'-'           ,'0','R,','I'],    # PE3
'36':['SENSOR_VP','GPIO36'  ,'-'      ,'GPIO36'  ,'-'         ,'-'       ,'-'           ,'0','R,','I'],    # PE4
'37':['SENSOR_CAPP','GPIO37','-'      ,'GPIO37'  ,'-'         ,'-'       ,'-'           ,'0','R,','I'],    # PE5
'38':['SENSOR_CAPN','GPIO38','-'      ,'GPIO38'  ,'-'         ,'-'       ,'-'           ,'0','R,','I'],    # PE6
'39':['SENSOR_VN','GPIO39'  ,'-'      ,'GPIO39'  ,'-'         ,'-'       ,'-'           ,'0','R,','I'],    # PE7

# this is a S3 only pin, so shouldnt be here, but we havnt separated the s3 to using a different table yet - this lets a hwdef.dat use , say 'PF7 CAN1_TX CAN1' on a s3 without error for pin 47
# "virtual port F":
'40':['GPIO40','','','','','','','',''],    # PF0
'41':['GPIO41','','','','','','','',''],    # PF1
'42':['GPIO42','','','','','','','',''],    # PF2
'43':['GPIO43','','','','','','','',''],    # PF3
'44':['GPIO44','','','','','','','',''],    # PF4
'45':['GPIO45','','','','','','','',''],    # PF5
'46':['GPIO46','','','','','','','',''],    # PF6
'47':['GPIO47','','','','','','','',''],    # PF7

# todo 48-52 on s3 , not added here as we dont use them yet.

}

# PA, PB, PC, PD, PE
classic_esp32_virtual_ports = {
    'A': [0,1,2,3,4,5,6,7],
    'B': [8,9,10,11,12,13,14,15],
    'C': [16,17,18,19,21,22,23,25],
    'D': [26,27,28,29,30,31],
    'E': [32,33,34,35,36,37,38,39],
    'F': [40,41,42,43,44,45,46,47], # S3 only, todo separate
}


# s3 manual/s:
# https://www.espressif.com/sites/default/files/documentation/esp32-s3_technical_reference_manual_en.pdf
# https://www.espressif.com/sites/default/files/documentation/esp32-s3_datasheet_en.pdf

# TODO - how do we handle pins on S3 with numbers 40,41,42,45,46,47,48 ? 
# warning pins 39 and 40 behave as jtag pins on boot.?
# use any of these pins as normal in idf, and the jtag will simply be disabled on first use.

# derived from Appendix A – ESP32-S3   's3' datasheet 
#

# colnames: 
#Pin Pin   Pin Pin-Providing Pin-Settings Pin-Settings  RTC-Function    Analog-Function    ---------------IOMUX-Function----------------
#No. Name Type Power         At-Reset     After-Reset     0   3             0   1          0 Type    1 Type    2 Type    3 Type    4 Type

# esp32s3_colnames = ['Pin-No.','Pin-Name','Pin-Type', 'Pin-Providing-Power', 'Pin-Settings-At-Reset', 'Pin-Settings-After-Reset', 
#                                 'RTC-Function-0', 'RTC-Function-3', 'Analog-Function-0', 'Analog-Function-1',
#                                   'IOMUX-F-0', 'IOMUX-F-0-Type','IOMUX-F-1', 'IOMUX-F-1-Type','IOMUX-F-2', 'IOMUX-F-2-Type',
#                                   'IOMUX-F-3', 'IOMUX-F-3-Type','IOMUX-F-4', 'IOMUX-F-4-Type']

# esp32s3_consolidated_pin_overview = { 
# #Appendix A – ESP32-S3 Consolidated Pin Overview
# # Pin  Pin           Pin      Pin-Providing     Pin-Settings Pin-Settings -RTC-Function---------   Analog-Function----   -IO-MUX-Function-----------------------------------------------------------------------------
# # No.  Name          Type     Power              At-Reset    After-Reset   0          3               0         1          0         0-Type     1     1-Type     2       2-Type      3        3-Type     4      4-Type
# '1': ['LNA_IN',     'Analog'],
# '2': ['VDD3P3',     'Power'],
# '3': ['VDD3P3',     'Power'],
# '4': ['CHIP_PU',    'Analog','VDD3P3_RTC'],
# '5': ['GPIO0',      'IO',    'VDD3P3_RTC',         'IE-WPU',  'IE-WPU','RTC_GPIO0','sar_i2c_scl_0','GPIO0',  'I/O/T',   'GPIO0',   'I/O/T'],
# '6': ['GPIO1',      'IO',    'VDD3P3_RTC',         'IE',      'IE',    'RTC_GPIO1','sar_i2c_sda_0','TOUCH1', 'ADC1_CH0','GPIO1',   'I/O/T', 'GPIO1', 'I/O/T'],
# '7': ['GPIO2',      'IO',    'VDD3P3_RTC',         'IE',      'IE',    'RTC_GPIO2','sar_i2c_scl_1','TOUCH2', 'ADC1_CH1','GPIO2',   'I/O/T', 'GPIO2', 'I/O/T'],
# '8': ['GPIO3',      'IO',    'VDD3P3_RTC',         'IE',      'IE',    'RTC_GPIO3','sar_i2c_sda_1','TOUCH3', 'ADC1_CH2','GPIO3',   'I/O/T', 'GPIO3', 'I/O/T'],
# '9': ['GPIO4',      'IO',    'VDD3P3_RTC',         '',        '',      'RTC_GPIO4', '',            'TOUCH4', 'ADC1_CH3','GPIO4',   'I/O/T', 'GPIO4', 'I/O/T'],
# '10':['GPIO5',      'IO',    'VDD3P3_RTC',         '',        '',      'RTC_GPIO5', '',            'TOUCH5', 'ADC1_CH4','GPIO5',   'I/O/T', 'GPIO5', 'I/O/T'],
# '11':['GPIO6',      'IO',    'VDD3P3_RTC',         '',        '',      'RTC_GPIO6', '',            'TOUCH6', 'ADC1_CH5','GPIO6',   'I/O/T', 'GPIO6', 'I/O/T'],
# '12':['GPIO7',      'IO',    'VDD3P3_RTC',         '',        '',      'RTC_GPIO7', '',            'TOUCH7', 'ADC1_CH6','GPIO7',   'I/O/T', 'GPIO7', 'I/O/T'],
# '13':['GPIO8',      'IO',    'VDD3P3_RTC',         '',        '',      'RTC_GPIO8', '',            'TOUCH8', 'ADC1_CH7','GPIO8',   'I/O/T', 'GPIO8', 'I/O/T','',        '',      'SUBSPICS1','O/T'],
# '14':['GPIO9',      'IO',    'VDD3P3_RTC',         '',        'IE',    'RTC_GPIO9', '',            'TOUCH9', 'ADC1_CH8','GPIO9',   'I/O/T', 'GPIO9', 'I/O/T','',        '',      'SUBSPIHD', 'I1/O/T','FSPIHD', 'I1/O/T'],
# '15':['GPIO10',     'IO',    'VDD3P3_RTC',         '',        'IE',    'RTC_GPIO10','',            'TOUCH10','ADC1_CH9','GPIO10',  'I/O/T', 'GPIO10','I/O/T','FSPIIO4', 'I1/O/T','SUBSPICS0','O/T',   'FSPICS0','I1/O/T'],
# '16':['GPIO11',     'IO',    'VDD3P3_RTC',         '',        'IE',    'RTC_GPIO11','',            'TOUCH11','ADC2_CH0','GPIO11',  'I/O/T', 'GPIO11','I/O/T','FSPIIO5', 'I1/O/T','SUBSPID',  'I1/O/T','FSPID',  'I1/O/T'],
# '17':['GPIO12',     'IO',    'VDD3P3_RTC',         '',        'IE',    'RTC_GPIO12','',            'TOUCH12','ADC2_CH1','GPIO12',  'I/O/T', 'GPIO12','I/O/T','FSPIIO6', 'I1/O/T','SUBSPICLK','O/T',   'FSPICLK','I1/O/T'],
# '18':['GPIO13',     'IO',    'VDD3P3_RTC',         '',        'IE',    'RTC_GPIO13','',            'TOUCH13','ADC2_CH2','GPIO13',  'I/O/T', 'GPIO13','I/O/T','FSPIIO7', 'I1/O/T','SUBSPIQ',  'I1/O/T','FSPIQ',  'I1/O/T'],
# '19':['GPIO14',     'IO',    'VDD3P3_RTC',         '',        'IE',    'RTC_GPIO14','',            'TOUCH14','ADC2_CH3','GPIO14',  'I/O/T', 'GPIO14','I/O/T','FSPIDQS', 'O/T',   'SUBSPIWP', 'I1/O/T','FSPIWP', 'I1/O/T'],
# '20':['VDD3P3_RTC', 'Power'],
# '21':['XTAL_32K_P', 'IO',    'VDD3P3_RTC',         '',        '',      'RTC_GPIO15','',         'XTAL_32K_P','ADC2_CH4','GPIO15',  'I/O/T', 'GPIO15','I/O/T','U0RTS',   'O'],
# '22':['XTAL_32K_N', 'IO',    'VDD3P3_RTC',         '',        '',      'RTC_GPIO16','',         'XTAL_32K_N','ADC2_CH5','GPIO16',  'I/O/T', 'GPIO16','I/O/T','U0CTS',   'I1'],
# '23':['GPIO17',     'IO',    'VDD3P3_RTC',         '',        'IE',    'RTC_GPIO17','',            '',       'ADC2_CH6','GPIO17',  'I/O/T', 'GPIO17','I/O/T','U1TXD',   'O'],
# '24':['GPIO18',     'IO',    'VDD3P3_RTC',         '',        'IE',    'RTC_GPIO18','',            '',       'ADC2_CH7','GPIO18',  'I/O/T', 'GPIO18','I/O/T','U1RXD',   'I1',    'CLK_OUT3', 'O'],
# '25':['GPIO19',     'IO',    'VDD3P3_RTC',         '',        '',      'RTC_GPIO19','',            'USB_D-', 'ADC2_CH8','GPIO19',  'I/O/T', 'GPIO19','I/O/T','U1RTS',   'O',     'CLK_OUT2', 'O'],
# '26':['GPIO20',     'IO',    'VDD3P3_RTC',         'USB_PU',  'USB_PU','RTC_GPIO20','',            'USB_D+', 'ADC2_CH9','GPIO20',  'I/O/T', 'GPIO20','I/O/T','U1CTS',   'I1',    'CLK_OUT1', 'O'],
# '27':['GPIO21',     'IO',    'VDD3P3_RTC',         '',        '',      'RTC_GPIO21','',            '',       '',        'GPIO21',  'I/O/T', 'GPIO21','I/O/T'],
# '28':['SPICS1',     'IO',    'VDD_SPI',            'IE-WPU',  'IE-WPU','',          '',            '',       '',        'SPICS1',  'O/T',   'GPIO26','I/O/T'],
# '29':['VDD_SPI',    'Power'],
# '30':['SPIHD',      'IO',    'VDD_SPI',            'IE-WPU',  'IE-WPU','',          '',            '',       '',        'SPIHD',   'I1/O/T','GPIO27','I/O/T'],
# '31':['SPIWP',      'IO',    'VDD_SPI',            'IE-WPU',  'IE-WPU','',          '',            '',       '',        'SPIWP',   'I1/O/T','GPIO28','I/O/T'],
# '32':['SPICS0',     'IO',    'VDD_SPI',            'IE-WPU',  'IE-WPU','',          '',            '',       '',        'SPICS0',  'O/T',   'GPIO29','I/O/T'],
# '33':['SPICLK',     'IO',    'VDD_SPI',            'IE-WPU',  'IE-WPU','',          '',            '',       '',        'SPICLK',  'O/T',   'GPIO30','I/O/T'],
# '34':['SPIQ',       'IO',    'VDD_SPI',            'IE-WPU',  'IE-WPU','',          '',            '',       '',        'SPIQ',    'I1/O/T','GPIO31','I/O/T'],
# '35':['SPID',       'IO',    'VDD_SPI',            'IE-WPU',  'IE-WPU','',          '',            '',       '',        'SPID',    'I1/O/T','GPIO32','I/O/T'],
# '36':['SPICLK_N',   'IO',    'VDD_SPI/VDD3P3_CPU', 'IE',      'IE',    '',          '',            '',       '',  'SPI_CLK_N_DIFF','O/T',   'GPIO48','I/O/T','SUBSPI',  'CLK_N_DIFF','O/T'],
# '37':['SPICLK_P',   'IO',    'VDD_SPI/VDD3P3_CPU', 'IE',      'IE',    '',          '',            '',       '',  'SPI_CLK_P_DIFF','O/T',   'GPIO47','I/O/T','SUBSPI',  'CLK_P_DIFF','O/T'],
# '38':['GPIO33',     'IO',    'VDD_SPI/VDD3P3_CPU', '',        'IE',    '',          '',            '',       '',        'GPIO33',  'I/O/T', 'GPIO33','I/O/T','FSPIHD',  'I1/O/T',   'SUBSPIHD', 'I1/O/T','SPIIO4','I1/O/T'],
# '39':['GPIO34',     'IO',    'VDD_SPI/VDD3P3_CPU', '',        'IE',    '',          '',            '',       '',        'GPIO34',  'I/O/T', 'GPIO34','I/O/T','FSPICS0', 'I1/O/T',   'SUBSPICS0','O/T',   'SPIIO5','I1/O/T'],
# '40':['GPIO35',     'IO',    'VDD_SPI/VDD3P3_CPU', '',        'IE',    '',          '',            '',       '',        'GPIO35',  'I/O/T', 'GPIO35','I/O/T','FSPID',   'I1/O/T',   'SUBSPID',  'I1/O/T','SPIIO6','I1/O/T'],
# '41':['GPIO36',     'IO',    'VDD_SPI/VDD3P3_CPU', '',        'IE',    '',          '',            '',       '',        'GPIO36',  'I/O/T', 'GPIO36','I/O/T','FSPICLK', 'I1/O/T',   'SUBSPICLK','O/T',   'SPIIO7','I1/O/T'],
# '42':['GPIO37',     'IO',    'VDD_SPI/VDD3P3_CPU', '',        'IE',    '',          '',            '',       '',        'GPIO37',  'I/O/T', 'GPIO37','I/O/T','FSPIQ',   'I1/O/T',   'SUBSPIQ',  'I1/O/T','SPIDQS','I0/O/T'],
# '43':['GPIO38',     'IO',    'VDD3P3_CPU',         '',        'IE',    '',          '',            '',       '',        'GPIO38',  'I/O/T', 'GPIO38','I/O/T','FSPIWP',  'I1/O/T',   'SUBSPIWP', 'I1/O/T'],
# '44':['MTCK',       'IO',    'VDD3P3_CPU',         '',        'IE*',   '',          '',            '',       '',        'MTCK',    'I1',    'GPIO39','I/O/T','CLK_OUT3','O',        'SUBSPICS1','O/T'],
# '45':['MTDO',       'IO',    'VDD3P3_CPU',         '',        'IE',    '',          '',            '',       '',        'MTDO',    'O/T',   'GPIO40','I/O/T','CLK_OUT2','O'],
# '46':['VDD3P3_CPU', 'Power'],
# '47':['MTDI',       'IO',    'VDD3P3_CPU',         '',        'IE',    '',          '',            '',       '',        'MTDI',    'I1',    'GPIO41','I/O/T','CLK_OUT1','O'],
# '48':['MTMS',       'IO',    'VDD3P3_CPU',         '',        'IE',    '',          '',            '',       '',        'MTMS',    'I1',    'GPIO42','I/O/T'],
# '49':['U0TXD',      'IO',    'VDD3P3_CPU',         'IE-WPU',  'IE-WPU','',          '',            '',       '',        'U0TXD',   'O',     'GPIO43','I/O/T','CLK_OUT1','O'],
# '50':['U0RXD',      'IO',    'VDD3P3_CPU',         'IE-WPU',  'IE-WPU','',          '',            '',       '',        'U0RXD',   'I1',    'GPIO44','I/O/T','CLK_OUT2','O'],
# '51':['GPIO45',     'IO',    'VDD3P3_CPU',         'IE-WPD',  'IE-WPD','',          '',            '',       '',        'GPIO45',  'I/O/T', 'GPIO45','I/O/T'],
# '52':['GPIO46',     'IO',    'VDD3P3_CPU',         'IE-WPD',  'IE-WPD','',          '',            '',       '',        'GPIO46',  'I/O/T', 'GPIO46','I/O/T'],
# '53':['XTAL_N',     'Analog'],
# '54':['XTAL_P',     'Analog'],
# '55':['VDDA',       'Power'],
# '56':['VDDA',       'Power'],
# '57':['GND',        'Power'],
# }


# pass in a number ranging from 0-39 and used as index here: classic_esp32_io_mux_pad_list
def from_mux_pad_idx_to_compat_name(pidx):
    '''return the compat name for a given pad index'''
     # we gonna scan all the sub values in classic_esp32_virtual_ports and do a reverse look from there with two loops
    for p in classic_esp32_virtual_ports:
        for pnum in classic_esp32_virtual_ports[p]:
            if pnum == pidx:
                return p

def from_mux_pad_idx_to_compat_short_idx(pidx):
    '''return the compat idx 0-7 for a given pad index 0-39'''
     # we gonna scan all the sub values in classic_esp32_virtual_ports and do a reverse look from there with two loops
    for p in classic_esp32_virtual_ports:
        shrtidx=0
        for pnum in classic_esp32_virtual_ports[p]:
            if pnum == pidx:
                return str(shrtidx)
            shrtidx+=1

def get_port_pins(port):
    '''return the pins for a given port'''
    return classic_esp32_virtual_ports[port]

# x  = PA , PB etc
esp32_mux_pad_lookup = {}
def port_lookup(pname) :
    for pnum in classic_esp32_virtual_ports[pname]:
         #print ("Zport_lookup:%s" % pnum)
         # compat_port_name => A0 
         compat_port_name = pname+str(pnum)
               #for pidx,mux_pad_info in classic_esp32_io_mux_pad_list.items():
         mux_pad_info = classic_esp32_io_mux_pad_list[str(pnum)] # go from classic name to esp idx and get pad_name and functions etc
         if len(mux_pad_info) == 0:
                continue   # skips empty entries 
         Pad_Name=mux_pad_info[0]
         Function_0=mux_pad_info[1]
         Function_1=mux_pad_info[2]
         Function_2=mux_pad_info[3]
         Function_3=mux_pad_info[4]
         Function_4=mux_pad_info[5]
         Function_5=mux_pad_info[6]
         Reset=mux_pad_info[7]
         Notes=mux_pad_info[8]
         # we make all the printed values fixed-width
         #print("compat_name:%-5s \tPad_Name:%-10s \tFunction_0:%-10s \tFunction_1:%-10s \tFunction_2:%-10s \tFunction_3:%-10s \tFunction_4:%-10s \tFunction_5:%-10s \tReset:%-10s \tNotes:%-10s" % (compat_port_name,Pad_Name,Function_0,Function_1,Function_2,Function_3, Function_4,Function_5,Reset,Notes))
         # save the data in a dictionary
         esp32_mux_pad_lookup[compat_port_name]= [Pad_Name,Function_0,Function_1,Function_2,Function_3,Function_4,Function_5,Reset,Notes]
    # implicit return value is in esp32_mux_pad_lookup

# ref tbl 19 is RTC_MUX not included here yet - appears to be mostly to do with ADC's, xtals ?

esp32_alt_functions = {}
# build a dictionary of alternate functions kinda like above but for alternate functions
def esp32_altfunc_lookup():
    for pidx,mux_pad_info in classic_esp32_io_mux_pad_list.items():
        if len(mux_pad_info) == 0:
            continue   # skips empty entries 
        compat_port_name =  from_mux_pad_idx_to_compat_name(int(pidx))
        compat_short_idx =  from_mux_pad_idx_to_compat_short_idx(int(pidx)) #0-7
        zzidx = 'P'+compat_port_name+compat_short_idx
        esp32_alt_functions[zzidx] = {}
        Pad_Name=mux_pad_info[0]
        Function_0=mux_pad_info[1]
        Function_1=mux_pad_info[2]
        Function_2=mux_pad_info[3]
        Function_3=mux_pad_info[4]
        Function_4=mux_pad_info[5]
        Function_5=mux_pad_info[6]
        Reset=mux_pad_info[7]
        Notes=mux_pad_info[8]
        # we make all the printed values fixed-width
        #print("espidx:%-5s compat_name:P%-5s \tPad_Name:%-10s \tFunction_0:%-10s \tFunction_1:%-10s \tFunction_2:%-10s \tFunction_3:%-10s \tFunction_4:%-10s \tFunction_5:%-10s \tReset:%-10s \tNotes:%-10s" % (pidx,compat_port_name,Pad_Name,Function_0,Function_1,Function_2,Function_3, Function_4,Function_5,Reset,Notes))

        #iterate over the function_0 to function_4 , not using a range, but just a list of values
        fnidx=0
        for fname in [Function_0,Function_1,Function_2,Function_3,Function_4,Function_5]:
            newidx = 'F'+str(fnidx)
            esp32_alt_functions[zzidx][newidx]= fname
            #print("esp32_altfunc_lookup:%s => %s => %s" % (zzidx,newidx,fname))
            fnidx+=1
    # implicit return value is in esp32_mux_pad_lookup


# esp32 classic
# number of pins in each port, lets call them A and B due to no better names
# GPIO_OUT_REG GPIO 0-31 output register 0x3FF44004 R/W - classic
# GPIO_OUT1_REG GPIO 32-39 output register 0x3FF44010 R/W - classic
pincount = {
    'A': 8,  #0-7   of 31
    'B': 8,  #8-15  of 31
    'C': 8,  #16-25 of 31
    'D': 6,  #26-31 of 31
    'E': 8,  #32-39
}
# so this gives us GPIOA and GPIOB as possible ports
#  we make them all 8 pins wide and call the GPIOA,GPIOB,GPIOC,GPIOD,GPIOE for no better names 

# from a 

ports = pincount.keys()

# pre-read all teh data into esp32_mux_pad_lookup
for p in ports: 
    port_lookup(p) # p= 'A', 'B', 'C', 'D', 'E' etc 
#print ("esp32_mux_pad_lookup:%s" % esp32_mux_pad_lookup)

# now do the same for alternate functions - see results in esp32_alt_functions
esp32_altfunc_lookup()
#print ("esp32_alt_functions:%s" % esp32_alt_functions)
#exit(0)

portmap = {}

# dictionary of all config lines, indexed by first word
config = {}

# alternate pin mappings
altmap = {}

# list of all pins in config file order
allpins = []

# list of configs by type
bytype = {}

# list of alt configs by type
alttype = {}

# list of configs by label
bylabel = {}

# list of alt configs by label
altlabel = {}

# list of SPI devices
spidev = []

# list of QSPI devices
qspidev = []

# dictionary of ROMFS files
romfs = {}

# SPI bus list
spi_list = []

# list of QSPI devices
qspi_list = []

# all config lines in order
alllines = []

# allow for extra env vars
env_vars = {}

# build flags for ESP32 makefiles
build_flags = []

# sensor lists
imu_list = []
compass_list = []
baro_list = []
airspeed_list = []

all_lines = []

dma_exclude_pattern = []

# map from uart names to SERIALn numbers
uart_serial_num = {}

mcu_type = None
dual_USB_enabled = False

# list of device patterns that can't be shared
dma_noshare = []

# integer defines
intdefines = {}

def is_int(str):
    '''check if a string is an integer'''
    try:
        int(str)
    except Exception:
        return False
    return True


def error(str):
    '''show an error and exit'''
    print("Error: " + str)
    #sys.exit(1)

# MCU parameters - this is BS on the esp32, and just a placeholder for now
_mcu = {
    # ram map, as list of (address, size-kb, flags)
    # flags of 1 means DMA-capable
    # flags of 2 means faster memory for CPU intensive work
    'RAM_MAP' : [
        (0x20000000, 192, 1), # main memory, DMA safe
        (0x10000000,  64, 2), # CCM memory, faster, but not DMA safe
    ],

    'EXPECTED_CLOCK' : 168000000
}


def get_mcu_lib(mcu):
    '''get library file for the chosen MCU'''

    lib = {}
    lib['AltFunction_map'] = esp32_alt_functions
    lib['mcu'] = _mcu  # a global dict with MCU parameters
    # add more here as needed
    return lib
    # we don't really use the esp32-xxxxx.py yet, so lets just stick to esp32_s3.py till we need more than 1
    #mcu = 'esp32-classic'
    #import importlib
    #try:
    #    #print("MCU type:",mcu)
    #    lib = importlib.import_module(mcu)
    #    return lib
    #except ImportError:
    #    error("MCULIB Unable to find module for MCU %s" % mcu)

# flag inputs as INPUT,FLOATING by default
def setup_mcu_type_defaults():
    '''setup defaults for given mcu type'''
    global  pincount, portmap, vtypes, mcu_type
    lib = get_mcu_lib(mcu_type)
    vtypes = f4f7_vtypes
    ports = pincount.keys()
    #setup default as input pins
    for port in ports:
        portmap[port] = []
        esppins = get_port_pins(port)     #esppins ranges from x-to-y , not starting at zero.
        for pinidx in range(pincount[port]): #pinidx ranges from 0-7
            intype = default_gpio[0]
            inextra = default_gpio[1:]
            portmap[port].append(generic_pin(port, pinidx, None, intype, inextra))
            esppin = esppins[pinidx]
            snme = "P%s%u" % (port, pinidx)
            #print("defaults setting snme: %s port %s esppin %u pinidx %u type:%s extra:%s" % (snme,port, esppin, pinidx,intype,inextra))



def get_alt_function(mcu, pin, function):
    '''return alternative function number for a pin'''
    lib = get_mcu_lib(mcu)

    if function.endswith('_TXINV') or function.endswith('_RXINV'):
        # RXINV and TXINV are special labels for inversion pins, not alt-functions
        return None

    if hasattr(lib, "AltFunction_map"):
        alt_map = lib.AltFunction_map
    else:
        # just check if Alt Func is available or not
        for l in af_labels:
            if function.startswith(l):
                return 0
        return None

    if function and function.endswith("_RTS") and (
            function.startswith('USART') or function.startswith('UART')):
        # we do software RTS
        return None

    for l in af_labels:
        if function.startswith(l):
            s = pin + ":" + function
            if s not in alt_map:
                error("Unknown pin function %s for MCU %s" % (s, mcu))
                #error("Unknown pin function %s for MCU %s with alt_map:%s" % (s, mcu,alt_map))
            return alt_map[s]
    return None


def have_type_prefix(ptype):
    '''return True if we have a peripheral starting with the given peripheral type'''
    for t in list(bytype.keys()) + list(alttype.keys()):
        if t.startswith(ptype):
            return True
    return False


def get_ADC1_chan(mcu, pin):
    '''return ADC1 channel for an analog pin'''
    #mcu = 'esp32-classic'
    import importlib
    try:
        lib = importlib.import_module(mcu)
        ADC1_map = lib.ADC1_map
    except ImportError:
        error("Unable to find ADC1_Map for MCU %s" % mcu)

    if pin not in ADC1_map:
        error("Unable to find ADC1 channel for pin %s" % pin)
    return ADC1_map[pin]


class generic_pin(object):
    '''class to hold pin definition'''

    def __init__(self, port, pin, label, type, extra):
        global mcu_series
        self.portpin = "P%s%u" % (port, pin)
        self.port = port
        self.pin = pin
        self.label = label
        self.type = type
        self.extra = extra
        self.af = None
        if type == 'OUTPUT':
            self.sig_dir = 'OUTPUT'
        else:
            self.sig_dir = 'INPUT'

        # check that labels and pin types are consistent
        for prefix in ['USART', 'UART', 'TIM']:
            if label is None or type is None:
                continue
            if type.startswith(prefix):
                a1 = label.split('_')
                a2 = type.split('_')
                if a1[0] != a2[0]:
                    error("Peripheral prefix mismatch for %s %s %s" % (self.portpin, label, type))

    def f1_pin_setup(self):
        for label in af_labels:
            if self.label.startswith(label):
                if self.label.endswith(tuple(f1_input_sigs)):
                    self.sig_dir = 'INPUT'
                    self.extra.append('FLOATING')
                elif self.label.endswith(tuple(f1_output_sigs)):
                    self.sig_dir = 'OUTPUT'
                elif label == 'I2C':
                    self.sig_dir = 'OUTPUT'
                elif label == 'OTG':
                    self.sig_dir = 'OUTPUT'
                else:
                    error("Unknown signal type %s:%s for %s!" % (self.portpin, self.label, mcu_type))

    def has_extra(self, v):
        '''return true if we have the given extra token'''
        return v in self.extra

    def extra_prefix(self, prefix):
        '''find an extra token starting with the given prefix'''
        for e in self.extra:
            if e.startswith(prefix):
                return e
        return None

    def extra_value(self, name, type=None, default=None):
        '''find an extra value of given type'''
        v = self.extra_prefix(name)
        if v is None:
            return default
        if v[len(name)] != '(' or v[-1] != ')':
            error("Badly formed value for %s: %s\n" % (name, v))
        ret = v[len(name) + 1:-1]
        if type is not None:
            try:
                ret = type(ret)
            except Exception:
                error("Badly formed value for %s: %s\n" % (name, ret))
        return ret

    def is_RTS(self):
        '''return true if this is a RTS pin'''
        if self.label and self.label.endswith("_RTS") and (
                self.type.startswith('USART') or self.type.startswith('UART')):
            return True
        return False

    def is_CS(self):
        '''return true if this is a CS pin'''
        return self.has_extra("CS") or self.type == "CS"

    def get_MODER_value(self):
        '''return one of ALTERNATE, OUTPUT, ANALOG, INPUT'''
        if self.af is not None:
            v = "ALTERNATE"
        elif self.type == 'OUTPUT':
            v = "OUTPUT"
        elif self.type.startswith('ADC'):
            v = "ANALOG"
        elif self.is_CS():
            v = "OUTPUT"
        elif self.is_RTS():
            v = "OUTPUT"
        else:
            v = "INPUT"
        return v

    def get_MODER(self):
        '''return one of ALTERNATE, OUTPUT, ANALOG, INPUT'''
        return "PIN_MODE_%s(%uU)" % (self.get_MODER_value(), self.pin)

    def get_OTYPER_value(self):
        '''return one of PUSHPULL, OPENDRAIN'''
        v = 'PUSHPULL'
        if self.type.startswith('I2C'):
            # default I2C to OPENDRAIN
            v = 'OPENDRAIN'
        values = ['PUSHPULL', 'OPENDRAIN']
        for e in self.extra:
            if e in values:
                v = e
        return v

    def get_OTYPER(self):
        '''return one of PUSHPULL, OPENDRAIN'''
        return "PIN_OTYPE_%s(%uU)" % (self.get_OTYPER_value(), self.pin)

    def get_OSPEEDR_value(self):
        '''return one of SPEED_VERYLOW, SPEED_LOW, SPEED_MEDIUM, SPEED_HIGH'''
        # on STM32F4 these speeds correspond to 2MHz, 25MHz, 50MHz and 100MHz
        values = ['SPEED_VERYLOW', 'SPEED_LOW', 'SPEED_MEDIUM', 'SPEED_HIGH']
        v = 'SPEED_MEDIUM'
        for e in self.extra:
            if e in values:
                v = e
        return v

    def get_OSPEEDR_int(self):
        '''return value from 0 to 3 for speed'''
        values = ['SPEED_VERYLOW', 'SPEED_LOW', 'SPEED_MEDIUM', 'SPEED_HIGH']
        v = self.get_OSPEEDR_value()
        if v not in values:
            error("Bad OSPEED %s" % v)
        return values.index(v)

    def get_OSPEEDR(self):
        '''return one of SPEED_VERYLOW, SPEED_LOW, SPEED_MEDIUM, SPEED_HIGH'''
        return "PIN_O%s(%uU)" % (self.get_OSPEEDR_value(), self.pin)

    def get_PUPDR_value(self):
        '''return one of FLOATING, PULLUP, PULLDOWN'''
        values = ['FLOATING', 'PULLUP', 'PULLDOWN']
        v = 'FLOATING'
        if self.is_CS():
            v = "PULLUP"
        # generate pullups for UARTs
        if (self.type.startswith('USART') or
            self.type.startswith('UART')) and (
            (self.label.endswith('_TX') or
             self.label.endswith('_RX') or
             self.label.endswith('_CTS') or
             self.label.endswith('_RTS'))):
            v = "PULLUP"

        if (self.type.startswith('SWD') and
            'SWDIO' in self.label):
            v = "PULLUP"

        if (self.type.startswith('SWD') and
            'SWCLK' in self.label):
            v = "PULLDOWN"

        # generate pullups for SDIO and SDMMC
        if (self.type.startswith('SDIO') or
            self.type.startswith('SDMMC')) and (
            (self.label.endswith('_D0') or
             self.label.endswith('_D1') or
             self.label.endswith('_D2') or
             self.label.endswith('_D3') or
             self.label.endswith('_CMD'))):
            v = "PULLUP"
        for e in self.extra:
            if e in values:
                v = e
        return v

    def get_PUPDR(self):
        '''return one of FLOATING, PULLUP, PULLDOWN wrapped in PIN_PUPDR_ macro'''
        return "PIN_PUPDR_%s(%uU)" % (self.get_PUPDR_value(), self.pin)

    def get_ODR_value(self):
        '''return one of LOW, HIGH'''
        values = ['LOW', 'HIGH']
        v = 'HIGH'
        for e in self.extra:
            if e in values:
                v = e
        return v

    def get_ODR(self):
        '''return one of LOW, HIGH wrapped in PIN_ODR macro'''
        return "PIN_ODR_%s(%uU)" % (self.get_ODR_value(), self.pin)

    def get_AFIO_value(self):
        '''return AFIO'''
        af = self.af
        if af is None:
            af = 0
        return af

    def get_AFIO(self):
        '''return AFIO wrapped in PIN_AFIO_AF macro'''
        return "PIN_AFIO_AF(%uU, %uU)" % (self.pin, self.get_AFIO_value())

    def get_AFRL(self):
        '''return AFIO low 8'''
        if self.pin >= 8:
            return None
        return self.get_AFIO()

    def get_AFRH(self):
        '''return AFIO high 8'''
        if self.pin < 8:
            return None
        return self.get_AFIO()

    def get_CR(self):
        '''return CR FLAGS'''
        if self.sig_dir != "INPUT":
            speed_values = ['SPEED_LOW', 'SPEED_MEDIUM', 'SPEED_HIGH']
            v = 'SPEED_MEDIUM'
            for e in self.extra:
                if e in speed_values:
                    v = e
            speed_str = "PIN_%s(%uU) |" % (v, self.pin)
        else:
            speed_str = ""
        # Check Alternate function
        if self.type.startswith('I2C'):
            v = "AF_OD"
        elif self.sig_dir == 'OUTPUT':
            if self.af is not None:
                v = "AF_PP"
            else:
                v = "OUTPUT_PP"
        elif self.type.startswith('ADC'):
            v = "ANALOG"
        elif self.is_CS():
            v = "OUTPUT_PP"
        elif self.is_RTS():
            v = "OUTPUT_PP"
        else:
            v = "PUD"
            if 'FLOATING' in self.extra:
                v = "NOPULL"
        mode_str = "PIN_MODE_%s(%uU)" % (v, self.pin)
        return "%s %s" % (speed_str, mode_str)

    def get_CRH(self):
        if self.pin < 8:
            return None
        return self.get_CR()

    def get_CRL(self):
        if self.pin >= 8:
            return None
        return self.get_CR()

    def pal_modeline(self):
        '''return a mode line suitable for palSetModeLine()'''
        # MODER, OTYPER, OSPEEDR, PUPDR, ODR, AFRL, AFRH
        ret = 'PAL_STM32_MODE_' + self.get_MODER_value()
        ret += '|PAL_STM32_OTYPE_' + self.get_OTYPER_value()
        ret += '|PAL_STM32_SPEED(%u)' % self.get_OSPEEDR_int()
        ret += '|PAL_STM32_PUPDR_' + self.get_PUPDR_value()
        af = self.get_AFIO_value()
        if af != 0:
            ret += '|PAL_STM32_ALTERNATE(%u)' % af

        return ret

    def periph_type(self):
        '''return peripheral type from GPIO_PIN_TYPE class'''
        patterns = {
            'USART*RX' : 'PERIPH_TYPE::UART_RX',
            'UART*RX' : 'PERIPH_TYPE::UART_RX',
            'USART*TX' : 'PERIPH_TYPE::UART_TX',
            'UART*TX' : 'PERIPH_TYPE::UART_TX',
            'I2C*SDA' : 'PERIPH_TYPE::I2C_SDA',
            'I2C*SCL' : 'PERIPH_TYPE::I2C_SCL',
            'EXTERN_GPIO*' : 'PERIPH_TYPE::GPIO',
        }
        for k in patterns.keys():
            if fnmatch.fnmatch(self.label, k):
                return patterns[k]
        return 'PERIPH_TYPE::OTHER'

    def periph_instance(self):
        '''return peripheral instance'''
        if self.periph_type() == 'PERIPH_TYPE::GPIO':
            result = re.match(r'[A-Z_]*([0-9]+)', self.label)
        else:
            result = re.match(r'[A-Z_]*([0-9]+)', self.type)
        if result:
            return int(result.group(1))
        return 0

    def __str__(self):
        str = ''
        if self.af is not None:
            str += " AF%u" % self.af
        if self.type.startswith('ADC1'):
            str += " ADC1_IN%u" % get_ADC1_chan(mcu_type, self.portpin)
        if self.extra_value('PWM', type=int):
            str += " PWM%u" % self.extra_value('PWM', type=int)
        return "P%s%u %s %s%s" % (self.port, self.pin, self.label, self.type,
                                  str)


def get_config(name, column=0, required=True, default=None, type=None, spaces=False, aslist=False):
    '''get a value from config dictionary'''
    if name not in config:
        if required and default is None:
            error("missing required value %s in hwdef.dat" % name)
        return default
    if aslist:
        return config[name]
    if len(config[name]) < column + 1:
        if not required:
            return None
        error("missing required value %s in hwdef.dat (column %u)" % (name,
                                                                      column))
    if spaces:
        ret = ' '.join(config[name][column:])
    else:
        ret = config[name][column]

    if type is not None:
        if type == int and ret.startswith('0x'):
            try:
                ret = int(ret, 16)
            except Exception:
                error("Badly formed config value %s (got %s)" % (name, ret))
        else:
            try:
                ret = type(ret)
            except Exception:
                error("Badly formed config value %s (got %s)" % (name, ret))
    return ret


def get_mcu_config(name, required=False):
    '''get a value from the mcu dictionary'''
    lib = get_mcu_lib(mcu_type)
    #print ("get_mcu_config:%s" % lib)
    #import pprint
    #pprint.pprint(lib)
    if 'mcu' not in lib:
        error("Missing mcu config for %s" % mcu_type)
    if name not in lib['mcu']:
        if required:
            error("Missing required mcu config %s for %s" % (name, mcu_type))
        return None
    return lib['mcu'][name]


def get_ram_reserve_start():
    '''get amount of memory to reserve for bootloader comms'''
    ram_reserve_start = get_config('RAM_RESERVE_START', default=0, type=int)
    if ram_reserve_start == 0 and is_periph_fw():
        ram_reserve_start = 256
    return ram_reserve_start


def make_line(label):
    '''return a line for a label'''
    if label in bylabel:
        p = bylabel[label]
        line = 'PAL_LINE(GPIO%s,%uU)' % (p.port, p.pin)
    else:
        line = "0"
    return line


def enable_can(f):
    '''setup for a CAN enabled board'''
    global mcu_series
    # if mcu_series.startswith("STM32H7") or mcu_series.startswith("STM32G4"):
    #     prefix = "FDCAN"
    #     cast = "CanType"
    # else:
    if 1:
        prefix = "CAN"
        cast = "bxcan::CanType"

    # allow for optional CAN_ORDER option giving bus order
    can_order_str = get_config('CAN_ORDER', required=False, aslist=True)
    if can_order_str:
        can_order = [int(s) for s in can_order_str]
    else:
        can_order = []
        for i in range(1,3):
            if 'CAN%u' % i in bytype or (i == 1 and 'CAN' in bytype):
                can_order.append(i)

    base_list = [0]
    # for i in can_order:
    #     base_list.append("reinterpret_cast<%s*>(uintptr_t(%s%s_BASE))" % (cast, prefix, i))
    #     f.write("#define HAL_CAN_IFACE%u_ENABLE\n" % i)

    # can_rev_order = [-1]*3
    # for i in range(len(can_order)):
    #     can_rev_order[can_order[i]-1] = i

    # f.write('#define HAL_CAN_INTERFACE_LIST %s\n' % ','.join([str(i-1) for i in can_order]))
    # f.write('#define HAL_CAN_INTERFACE_REV_LIST %s\n' % ','.join([str(i) for i in can_rev_order]))
    # f.write('#define HAL_CAN_BASE_LIST %s\n' % ','.join(base_list))
    f.write('#define HAL_NUM_CAN_IFACES %d\n' % len(base_list))
    global mcu_type
    if 'CAN' in bytype and mcu_type.startswith("STM32F3"):
        f.write('#define CAN1_BASE CAN_BASE\n')
    env_vars['HAL_NUM_CAN_IFACES'] = 1
    #str(len(base_list))


def has_sdcard_spi():
    '''check for sdcard connected to spi bus'''
    for dev in spidev:
        if(dev[0] == 'sdcard'):
            return True
    return False


def get_ram_map():
    '''get RAM_MAP. May be different for bootloader'''
    if args.bootloader:
        ram_map = get_mcu_config('RAM_MAP_BOOTLOADER', False)
        if ram_map is not None:
            return ram_map
    return get_mcu_config('RAM_MAP', True)

def get_flash_pages_sizes():
    global mcu_series
    global mcu_type
    if mcu_type.startswith('classic'):
        if get_config('FLASH_SIZE_KB', type=int) == 2048:
            return [ 16, 16, 16, 16, 64, 128, 128, 128,
                     128, 128, 128, 128,
                     128, 128, 128, 128,
                     128, 128, 128, 128,
                     128, 128, 128, 128 ]
        # elif get_config('FLASH_SIZE_KB', type=int) == 4096:
        #     return [ 16, 16, 16, 16, 64, 128, 128, 128,
        #              128, 128, 128, 128,
        #              128, 128, 128, 128,
        #              128, 128, 128, 128,
        #              128, 128, 128, 128 ]
        else:
            raise Exception("Unsupported flash size %u" % get_config('FLASH_SIZE_KB', type=int))


def get_flash_npages():
    page_sizes = get_flash_pages_sizes()
    total_size = sum(pages)
    if total_size != get_config('FLASH_SIZE_KB',type=int):
        raise Exception("Invalid flash size MCU %s" % mcu_series)
    return len(pages)

def get_flash_page_offset_kb(sector):
    '''return the offset in flash of a page number'''
    pages = get_flash_pages_sizes()
    offset = 0
    for i in range(sector):
        offset += pages[i]
    return offset

def get_storage_flash_page():
    '''get STORAGE_FLASH_PAGE either from this hwdef or from hwdef.dat
       in the same directory if this is a bootloader
    '''
    storage_flash_page = get_config('STORAGE_FLASH_PAGE', default=None, type=int, required=False)
    if storage_flash_page is not None:
        return storage_flash_page
    if args.bootloader and args.hwdef[0].find("-bl") != -1:
        hwdefdat = args.hwdef[0].replace("-bl", "")
        if os.path.exists(hwdefdat):
            ret = None
            lines = open(hwdefdat,'r').readlines()
            for line in lines:
                result = re.match(r'STORAGE_FLASH_PAGE\s*([0-9]+)', line)
                if result:
                    ret = int(result.group(1))
            return ret
    return None

def validate_flash_storage_size():
    '''check there is room for storage with HAL_STORAGE_SIZE'''
    if intdefines.get('HAL_WITH_RAMTRON',0) == 1:
        # no check for RAMTRON storage
        return
    storage_flash_page = get_storage_flash_page()
    pages = get_flash_pages_sizes()
    page_size = pages[storage_flash_page] * 1024
    storage_size = intdefines.get('HAL_STORAGE_SIZE', None)
    if storage_size is None:
        error('Need HAL_STORAGE_SIZE define')
    if storage_size >= page_size:
        error("HAL_STORAGE_SIZE too large %u %u" % (storage_size, page_size))
    if page_size == 16384 and storage_size > 15360:
        error("HAL_STORAGE_SIZE invalid, needs to be 15360")

def write_mcu_config(f):
    '''write MCU config defines'''
    f.write('// MCU type (ESP32 define)\n')
    f.write('#define %s_MCUCONF\n' % get_config('MCU'))
    mcu_subtype = get_config('MCU', 1)
    if mcu_subtype.endswith('xx'):
        f.write('#define %s_MCUCONF\n\n' % mcu_subtype[:-2])
    f.write('#define %s\n\n' % mcu_subtype)
    f.write('// crystal frequency\n')
    #f.write('#define STM32_HSECLK %sU\n\n' % get_config('OSCILLATOR_HZ'))
    f.write('// UART used for stdout (printf)\n')
    if get_config('STDOUT_SERIAL', required=False):
        f.write('#define HAL_STDOUT_SERIAL %s\n\n' % get_config('STDOUT_SERIAL'))
        f.write('// baudrate used for stdout (printf)\n')
        f.write('#define HAL_STDOUT_BAUDRATE %u\n\n' % get_config('STDOUT_BAUDRATE', type=int))
    if have_type_prefix('SDIO'):
        f.write('// SDIO available, enable POSIX filesystem support\n')
        f.write('#define USE_POSIX\n\n')
        f.write('#define HAL_USE_SDC TRUE\n')
        build_flags.append('USE_FATFS=yes')
        env_vars['WITH_FATFS'] = "1"
    elif have_type_prefix('SDMMC2'):
        f.write('// SDMMC2 available, enable POSIX filesystem support\n')
        f.write('#define USE_POSIX\n\n')
        f.write('#define HAL_USE_SDC TRUE\n')
        #f.write('#define STM32_SDC_USE_SDMMC2 FALSE\n')
        build_flags.append('USE_FATFS=yes')
        env_vars['WITH_FATFS'] = "1"
    elif have_type_prefix('SDMMC'):
        f.write('// SDMMC available, enable POSIX filesystem support\n')
        f.write('#define USE_POSIX\n\n')
        f.write('#define HAL_USE_SDC TRUE\n')
        #f.write('#define STM32_SDC_USE_SDMMC1 FALSE\n')
        build_flags.append('USE_FATFS=yes')
        env_vars['WITH_FATFS'] = "1"
    elif has_sdcard_spi():
        f.write('// MMC via SPI available, enable POSIX filesystem support\n')
        f.write('#define USE_POSIX\n\n')
        f.write('#define HAL_USE_MMC_SPI TRUE\n')
        f.write('#define HAL_USE_SDC FALSE\n')
        f.write('#define HAL_SDCARD_SPI_HOOK TRUE\n')
        build_flags.append('USE_FATFS=yes')
        env_vars['WITH_FATFS'] = "1"
    else:
        f.write('#define HAL_USE_SDC FALSE\n')
        build_flags.append('USE_FATFS=no')
        env_vars['DISABLE_SCRIPTING'] = True
    if 'OTG1' in bytype:
        #f.write('#define STM32_USB_USE_OTG1                  FALSE\n')
        f.write('#define HAL_USE_USB TRUE\n')
        f.write('#define HAL_USE_SERIAL_USB TRUE\n')
    if 'OTG2' in bytype:
        #f.write('#define STM32_USB_USE_OTG2                  FALSE\n')
        pass

    defines = get_mcu_config('DEFINES', False)
    if defines is not None:
        for d in defines.keys():
            v = defines[d]
            print("BUZZ ESP HWDEF PY gegnerator adding DEFINES:",v,defines[d])
            f.write("#ifndef %s\n#define %s %s\n#endif\n" % (d, d, v))
    else:
        defines = {}
    # enable RNG for all H7 chips
    # if mcu_series.startswith("STM32H7") and 'HAL_USE_HW_RNG' not in defines.keys():
    #     f.write("#define HAL_USE_HW_RNG TRUE\n")
    if 'HAL_USE_HW_RNG' not in defines.keys():
        f.write("#define HAL_USE_HW_RNG FALSE\n")

    if get_config('PROCESS_STACK', required=False):
        env_vars['PROCESS_STACK'] = get_config('PROCESS_STACK')
    else:
        env_vars['PROCESS_STACK'] = "0x1C00"

    f.write('#define HAL_PROCESS_STACK_SIZE %s\n' % env_vars['PROCESS_STACK'])
    # MAIN_STACK is location of initial stack on startup and is also the stack
    # used for slow interrupts. It needs to be big enough for maximum interrupt
    # nesting
    if get_config('MAIN_STACK', required=False):
        env_vars['MAIN_STACK'] = get_config('MAIN_STACK')
    else:
        env_vars['MAIN_STACK'] = "0x600"

    if get_config('IOMCU_FW', required=False):
        env_vars['IOMCU_FW'] = get_config('IOMCU_FW')
    else:
        env_vars['IOMCU_FW'] = 0

    if get_config('PERIPH_FW', required=False):
        #print("BUZZ EEP------------------------------------------------yy")
        env_vars['PERIPH_FW'] = 1
        # get_config('PERIPH_FW')
    else:
        #print("BUZZ EEP2-------------------------------------------------xx")
        env_vars['PERIPH_FW'] = 0

    # write any custom STM32 defines
    using_chibios_can = False
    for d in alllines:
        if d.startswith('STM32_'):
            f.write('#define %s\n' % d)
        if d.startswith('define '):
            if 'HAL_USE_CAN' in d:
                using_chibios_can = True
            f.write('#define %s\n' % d[7:])
            # extract numerical defines for processing by other parts of the script
            result = re.match(r'define\s*([A-Z_]+)\s*([0-9]+)', d)
            if result:
                intdefines[result.group(1)] = int(result.group(2))

    #if have_type_prefix('CAN') and not using_chibios_can:
    #    enable_can(f)
    flash_size = get_config('FLASH_SIZE_KB', type=int)
    f.write('#define BOARD_FLASH_SIZE %u\n' % flash_size)
    env_vars['BOARD_FLASH_SIZE'] = flash_size
    f.write('#define CRT1_AREAS_NUMBER 1\n')

    flash_reserve_start = get_config(
        'FLASH_RESERVE_START_KB', default=16, type=int)
    f.write('\n// location of loaded firmware\n')
    f.write('#define FLASH_LOAD_ADDRESS 0x%08x\n' % (0x08000000 + flash_reserve_start*1024))
    f.write('#define EXTERNAL_PROG_FLASH_MB %u\n' % get_config('EXTERNAL_PROG_FLASH_MB', default=0, type=int))

    env_vars['EXTERNAL_PROG_FLASH_MB'] = get_config('EXTERNAL_PROG_FLASH_MB', default=0, type=int)

    if env_vars['EXTERNAL_PROG_FLASH_MB'] and not args.bootloader:
        f.write('#define CRT1_RAMFUNC_ENABLE TRUE\n') # this will enable loading program sections to RAM
        f.write('#define __RAMFUNC__ __attribute__ ((long_call, __section__(".ramfunc")))\n')
        f.write('#define PORT_IRQ_ATTRIBUTES __RAMFUNC__')
    else:
        f.write('#define CRT1_RAMFUNC_ENABLE FALSE\n')

    storage_flash_page = get_storage_flash_page()
    if storage_flash_page is not None:
        if not args.bootloader:
            f.write('#define STORAGE_FLASH_PAGE %u\n' % storage_flash_page)
            validate_flash_storage_size()
        else:
            # ensure the flash page leaves room for bootloader
            offset = get_flash_page_offset_kb(storage_flash_page)

    if flash_size >= 2048 and not args.bootloader:
        # lets pick a flash sector for Crash log
        f.write('#define HAL_CRASHDUMP_ENABLE 1\n')
        env_vars['ENABLE_CRASHDUMP'] = 1
    else:
        f.write('#define HAL_CRASHDUMP_ENABLE 0\n')
        env_vars['ENABLE_CRASHDUMP'] = 0

    if args.bootloader:
        if env_vars['EXTERNAL_PROG_FLASH_MB']:
            f.write('#define APP_START_ADDRESS 0x90000000\n')
            f.write('#define BOOT_FROM_EXT_FLASH 1\n')
        f.write('#define FLASH_BOOTLOADER_LOAD_KB %u\n' % get_config('FLASH_BOOTLOADER_LOAD_KB', type=int))
        f.write('#define FLASH_RESERVE_END_KB %u\n' % get_config('FLASH_RESERVE_END_KB', default=0, type=int))
        f.write('#define APP_START_OFFSET_KB %u\n' % get_config('APP_START_OFFSET_KB', default=0, type=int))
    f.write('\n')

    ram_map = get_ram_map()
    f.write('// memory regions\n')
    regions = []
    cc_regions = []
    total_memory = 0
    for (address, size, flags) in ram_map:
        regions.append('{(void*)0x%08x, 0x%08x, 0x%02x }' % (address, size*1024, flags))
        cc_regions.append('{0x%08x, 0x%08x, CRASH_CATCHER_BYTE }' % (address, address + size*1024))
        total_memory += size
    f.write('#define HAL_MEMORY_REGIONS %s\n' % ', '.join(regions))
    f.write('#define HAL_CC_MEMORY_REGIONS %s\n' % ', '.join(cc_regions))
    f.write('#define HAL_MEMORY_TOTAL_KB %u\n' % total_memory)

    f.write('#define HAL_RAM0_START 0x%08x\n' % ram_map[0][0])
    ram_reserve_start = get_ram_reserve_start()
    if ram_reserve_start > 0:
        f.write('#define HAL_RAM_RESERVE_START 0x%08x\n' % ram_reserve_start)

    f.write('\n// CPU serial number (12 bytes)\n')
    udid_start = get_mcu_config('UDID_START')
    if udid_start is None:
        f.write('#define UDID_START UID_BASE\n\n')
    else:
        f.write('#define UDID_START 0x%08x\n\n' % udid_start)

    f.write('\n// APJ board ID (for bootloaders)\n')
    f.write('#define APJ_BOARD_ID %s\n' % get_config('APJ_BOARD_ID'))

    f.write('''
#ifndef HAL_ENABLE_THREAD_STATISTICS
#define HAL_ENABLE_THREAD_STATISTICS FALSE
#endif
    ''')

    #lib = get_mcu_lib(mcu_type)
    #build_info = lib['build']
    #
    #if get_mcu_config('CPU_FLAGS') and get_mcu_config('CORTEX'):
    #    # CPU flags specified in mcu file
    #    cortex = get_mcu_config('CORTEX')
    #    env_vars['CPU_FLAGS'] = get_mcu_config('CPU_FLAGS').split()
    #    build_info['MCU'] = cortex
    #    print("MCU Flags: %s %s" % (cortex, env_vars['CPU_FLAGS']))
    #else:
    #    cortex = "cortex-m4"
    #    env_vars['CPU_FLAGS'] = ["-mcpu=%s" % cortex, "-mfpu=fpv4-sp-d16", "-mfloat-abi=hard"]
    #    build_info['MCU'] = cortex

    f.write('''
#ifndef HAL_HAVE_HARDWARE_DOUBLE
#define HAL_HAVE_HARDWARE_DOUBLE 0
#endif
''')

    if get_mcu_config('EXPECTED_CLOCK', required=True):
        f.write('#define HAL_EXPECTED_SYSCLOCK %u\n' % get_mcu_config('EXPECTED_CLOCK'))

    #env_vars['CORTEX'] = cortex
    #
    #if not args.bootloader:
    #    if cortex == 'cortex-m4':
    #        env_vars['CPU_FLAGS'].append('-DARM_MATH_CM4')
    #    elif cortex == 'cortex-m7':
    #        env_vars['CPU_FLAGS'].append('-DARM_MATH_CM7')

    # setup build variables
    #for v in build_info.keys():
    #    build_flags.append('%s=%s' % (v, build_info[v]))

    # setup for bootloader build
    if args.bootloader:
        if get_config('FULL_CHIBIOS_BOOTLOADER', required=False, default=False):
            # we got enough space to fit everything so we enable almost everything
            f.write('''
#define HAL_BOOTLOADER_BUILD TRUE
#define HAL_USE_ADC FALSE
#define HAL_USE_EXT FALSE
#define HAL_USE_I2C FALSE
#define HAL_USE_PWM FALSE
#define HAL_NO_UARTDRIVER
#define CH_CFG_USE_DYNAMIC FALSE
#define HAL_USE_EMPTY_STORAGE 1
#ifndef HAL_STORAGE_SIZE
#define HAL_STORAGE_SIZE 16384
#endif
''')
        else:
            f.write('''
#define HAL_BOOTLOADER_BUILD TRUE
#define HAL_USE_ADC FALSE
#define HAL_USE_EXT FALSE
#define HAL_NO_UARTDRIVER
#define HAL_NO_PRINTF
#define HAL_NO_CCM
#define HAL_USE_I2C FALSE
#define HAL_USE_PWM FALSE
#define CH_DBG_ENABLE_STACK_CHECK FALSE
#define CH_CFG_USE_DYNAMIC FALSE
// avoid timer and RCIN threads to save memory
#define HAL_NO_TIMER_THREAD
#define HAL_NO_RCOUT_THREAD
#define HAL_NO_RCIN_THREAD
#define HAL_NO_SHARED_DMA FALSE
#define HAL_NO_ROMFS_SUPPORT TRUE
#define CH_CFG_USE_TM FALSE
#define CH_CFG_USE_REGISTRY FALSE
#define CH_CFG_USE_WAITEXIT FALSE
#define CH_CFG_USE_MEMPOOLS FALSE
#define CH_DBG_FILL_THREADS FALSE
#define CH_CFG_USE_MUTEXES FALSE
#define CH_CFG_USE_EVENTS FALSE
#define CH_CFG_USE_EVENTS_TIMEOUT FALSE
#define HAL_USE_EMPTY_STORAGE 1
#ifndef HAL_STORAGE_SIZE
#define HAL_STORAGE_SIZE 16384
#endif
#define HAL_USE_RTC FALSE
#define DISABLE_SERIAL_ESC_COMM TRUE
''')
        if not env_vars['EXTERNAL_PROG_FLASH_MB']:
            f.write('''
#define CH_CFG_USE_HEAP FALSE
#define CH_CFG_USE_SEMAPHORES FALSE
#define CH_CFG_USE_MEMCORE FALSE
''')
    if env_vars.get('ROMFS_UNCOMPRESSED', False):
        f.write('#define HAL_ROMFS_UNCOMPRESSED\n')

    # if not args.bootloader:
    #     f.write('''#define STM32_DMA_REQUIRED FALSE\n\n''')

def write_ldscript(fname):
    '''write ldscript.ld for this board'''
    flash_size = get_config('FLASH_USE_MAX_KB', type=int, default=0)
    if flash_size == 0:
        flash_size = get_config('FLASH_SIZE_KB', type=int)

    # space to reserve for bootloader and storage at start of flash
    flash_reserve_start = get_config(
        'FLASH_RESERVE_START_KB', default=16, type=int)

    storage_flash_page = get_storage_flash_page()
    if storage_flash_page is not None:
        offset = get_flash_page_offset_kb(storage_flash_page)
        if offset > flash_reserve_start:
            # storage is after flash, need to ensure flash doesn't encroach on it
            flash_size = min(flash_size, offset)
        else:
            # storage is before flash, need to ensure storage fits
            offset2 = get_flash_page_offset_kb(storage_flash_page+2)
            if flash_reserve_start < offset2:
                error("Storage overlaps flash")

    env_vars['FLASH_RESERVE_START_KB'] = str(flash_reserve_start)

    # space to reserve for storage at end of flash
    flash_reserve_end = get_config('FLASH_RESERVE_END_KB', default=0, type=int)

    # ram layout
    ram_map = get_ram_map()
    instruction_ram = get_mcu_config('INSTRUCTION_RAM', False)

    flash_base = 0x08000000 + flash_reserve_start * 1024
    ext_flash_base = 0x90000000
    if instruction_ram is not None:
        instruction_ram_base = instruction_ram[0]
        instruction_ram_length = instruction_ram[1]

    if not args.bootloader:
        flash_length = flash_size - (flash_reserve_start + flash_reserve_end)
    else:
        flash_length = min(flash_size, get_config('FLASH_BOOTLOADER_LOAD_KB', type=int))

    env_vars['FLASH_TOTAL'] = flash_length * 1024

    print("Generating ldscript.ld")
    f = open(fname, 'w')
    ram0_start = ram_map[0][0]
    ram0_len = ram_map[0][1] * 1024

    # possibly reserve some memory for app/bootloader comms
    ram_reserve_start = get_ram_reserve_start()
    ram0_start += ram_reserve_start
    ram0_len -= ram_reserve_start
    ext_flash_length = get_config('EXTERNAL_PROG_FLASH_MB', default=0, type=int)
    if ext_flash_length == 0 or args.bootloader:
        env_vars['HAS_EXTERNAL_FLASH_SECTIONS'] = 0
        f.write('''/* generated ldscript.ld */
MEMORY
{
    flash : org = 0x%08x, len = %uK
    ram0  : org = 0x%08x, len = %u
}

INCLUDE common.ld
''' % (flash_base, flash_length, ram0_start, ram0_len))
    else:
        if ext_flash_length > 32:
            error("We only support 24bit addressing over external flash")
        env_vars['HAS_EXTERNAL_FLASH_SECTIONS'] = 1
        f.write('''/* generated ldscript.ld */
MEMORY
{
    default_flash : org = 0x%08x, len = %uM
    instram : org = 0x%08x, len = %uK
    ram0  : org = 0x%08x, len = %u
}

INCLUDE common_extf.ld
''' % (ext_flash_base, ext_flash_length,
       instruction_ram_base, instruction_ram_length,
       ram0_start, ram0_len))

def copy_common_linkerscript(outdir):
    dirpath = os.path.dirname(os.path.realpath(__file__))
    # shutil.copy(os.path.join(dirpath, "../common/common.ld"),
    #             os.path.join(outdir, "common.ld"))
    pass


def get_USB_IDs():
    '''return tuple of USB VID/PID'''

    global dual_USB_enabled
    if dual_USB_enabled:
        # use pidcodes allocated ID
        default_vid = 0x1209
        default_pid = 0x5740
    else:
        default_vid = 0x1209
        default_pid = 0x5741
    return (get_config('USB_VENDOR', type=int, default=default_vid), get_config('USB_PRODUCT', type=int, default=default_pid))

def write_USB_config(f):
    '''write USB config defines'''
    if not have_type_prefix('OTG'):
        return
    f.write('// USB configuration\n')
    (USB_VID, USB_PID) = get_USB_IDs()
    f.write('#define HAL_USB_VENDOR_ID 0x%04x\n' % int(USB_VID))
    f.write('#define HAL_USB_PRODUCT_ID 0x%04x\n' % int(USB_PID))
    f.write('#define HAL_USB_STRING_MANUFACTURER %s\n' % get_config("USB_STRING_MANUFACTURER", default="\"ArduPilot\""))
    default_product = "%BOARD%"
    if args.bootloader:
        default_product += "-BL"
    f.write('#define HAL_USB_STRING_PRODUCT %s\n' % get_config("USB_STRING_PRODUCT", default="\"%s\""%default_product))
    f.write('#define HAL_USB_STRING_SERIAL %s\n' % get_config("USB_STRING_SERIAL", default="\"%SERIAL%\""))

    f.write('\n\n')


def write_SPI_table(f):
    '''write SPI device table'''
    f.write('\n// SPI device table\n')
    devlist = []
    for dev in spidev:
        if len(dev) != 7:
            print("Badly formed SPIDEV line %s" % dev)
        name = '"' + dev[0] + '"'
        bus = dev[1]
        devid = dev[2]
        cs = dev[3]
        mode = dev[4]
        lowspeed = dev[5]
        highspeed = dev[6]
        if not bus.startswith('SPI') or bus not in spi_list:
            error("Bad SPI bus in SPIDEV line dev:%s bus:%s spibuslist:%s" % (dev,bus,spi_list))
        if not devid.startswith('DEVID') or not is_int(devid[5:]):
            error("Bad DEVID in SPIDEV line %s" % dev)
        if cs not in bylabel or not bylabel[cs].is_CS():
            error("Bad CS pin in SPIDEV line %s" % dev)
        if mode not in ['MODE0', 'MODE1', 'MODE2', 'MODE3']:
            error("Bad MODE in SPIDEV line %s" % dev)
        if not lowspeed.endswith('*MHZ') and not lowspeed.endswith('*KHZ'):
            error("Bad lowspeed value %s in SPIDEV line %s" % (lowspeed, dev))
        if not highspeed.endswith('*MHZ') and not highspeed.endswith('*KHZ'):
            error("Bad highspeed value %s in SPIDEV line %s" % (highspeed,
                                                                dev))
        cs_pin = bylabel[cs]
        pal_line = 'PAL_LINE(GPIO%s,%uU)' % (cs_pin.port, cs_pin.pin)
        devidx = len(devlist)
        f.write(
            '#define HAL_SPI_DEVICE%-2u SPIDesc(%-17s, %2u, %2u, %-19s, SPIDEV_%s, %7s, %7s)\n'
            % (devidx, name, spi_list.index(bus), int(devid[5:]), pal_line,
               mode, lowspeed, highspeed))
        devlist.append('HAL_SPI_DEVICE%u' % devidx)
    f.write('#define HAL_SPI_DEVICE_LIST %s\n\n' % ','.join(devlist))
    for dev in spidev:
        f.write("#define HAL_WITH_SPI_%s 1\n" % dev[0].upper().replace("-","_"))
    f.write("\n")


def write_SPI_config(f):
    '''write SPI config defines'''
    global spi_list
    for t in list(bytype.keys()) + list(alttype.keys()):
        if t.startswith('SPI'):
            spi_list.append(t)
    spi_list = sorted(spi_list)
    if len(spi_list) == 0:
        f.write('#define HAL_USE_SPI FALSE\n')
        return
    devlist = []
    for dev in spi_list:
        n = int(dev[3:])
        devlist.append('HAL_SPI%u_CONFIG' % n)
        #f.write(
        #    '#define HAL_SPI%u_CONFIG { &SPID%u, %u, STM32_SPI_SPI%u_DMA_STREAMS }\n'
        #    % (n, n, n, n))
    f.write('#define HAL_SPI_BUS_LIST %s\n\n' % ','.join(devlist))
    write_SPI_table(f)


def write_QSPI_table(f):
    '''write SPI device table'''
    f.write('\n// QSPI device table\n')
    devlist = []
    for dev in qspidev:
        if len(dev) != 6:
            print("Badly formed QSPIDEV line %s" % dev)
        name = '"' + dev[0] + '"'
        bus = dev[1]
        mode = dev[2]
        speed = dev[3]
        size_pow2 = dev[4]
        ncs_clk_delay = dev[5]
        if not bus.startswith('QUADSPI') or bus not in qspi_list:
            error("Bad QUADSPI bus in QSPIDEV line %s" % dev)
        if mode not in ['MODE1', 'MODE3']:
            error("Bad MODE in QSPIDEV line %s" % dev)
        if not speed.endswith('*MHZ') and not speed.endswith('*KHZ'):
            error("Bad speed value %s in SPIDEV line %s" % (speed, dev))

        devidx = len(devlist)
        f.write(
            '#define HAL_QSPI_DEVICE%-2u QSPIDesc(%-17s, %2u, QSPIDEV_%s, %7s, %2u, %2u)\n'
            % (devidx, name, qspi_list.index(bus), mode, speed, int(size_pow2), int(ncs_clk_delay)))
        devlist.append('HAL_QSPI_DEVICE%u' % devidx)
    f.write('#define HAL_QSPI_DEVICE_LIST %s\n\n' % ','.join(devlist))
    for dev in qspidev:
        f.write("#define HAL_HAS_WSPI_%s 1\n" % dev[0].upper().replace("-", "_"))
        f.write("#define HAL_QSPI%d_CLK (%s)" % (int(bus[7:]), speed))
    f.write("\n")


def write_QSPI_config(f):
    '''write SPI config defines'''
    global qspi_list
    if len(qspidev) == 0:
        # nothing to do
        return
    for t in list(bytype.keys()) + list(alttype.keys()):
        if t.startswith('QUADSPI'):
            qspi_list.append(t)
    qspi_list = sorted(qspi_list)
    if len(qspi_list) == 0:
        return
    f.write('#define HAL_USE_WSPI TRUE\n')
    devlist = []
    for dev in qspi_list:
        n = int(dev[7:])
        devlist.append('HAL_QSPI%u_CONFIG' % n)
        f.write(
            '#define HAL_QSPI%u_CONFIG { &WSPID%u, %u}\n'
            % (n, n, n))
    f.write('#define HAL_QSPI_BUS_LIST %s\n\n' % ','.join(devlist))
    write_QSPI_table(f)


def parse_spi_device(dev):
    '''parse a SPI:xxx device item'''
    a = dev.split(':')
    if len(a) != 2:
        error("Bad SPI device: %s" % dev)
    return 'hal.spi->get_device("%s")' % a[1]


def parse_i2c_device(dev):
    '''parse a I2C:xxx:xxx device item'''
    a = dev.split(':')
    if len(a) != 3:
        error("Bad I2C device: %s" % dev)
    busaddr = int(a[2], base=0)
    if a[1] == 'ALL_EXTERNAL':
        return ('FOREACH_I2C_EXTERNAL(b)', 'GET_I2C_DEVICE(b,0x%02x)' % (busaddr))
    elif a[1] == 'ALL_INTERNAL':
        return ('FOREACH_I2C_INTERNAL(b)', 'GET_I2C_DEVICE(b,0x%02x)' % (busaddr))
    elif a[1] == 'ALL':
        return ('FOREACH_I2C(b)', 'GET_I2C_DEVICE(b,0x%02x)' % (busaddr))
    busnum = int(a[1])
    return ('', 'GET_I2C_DEVICE(%u,0x%02x)' % (busnum, busaddr))


def seen_str(dev):
    '''return string representation of device for checking for duplicates'''
    return str(dev[:2])


def write_IMU_config(f):
    '''write IMU config defines'''
    global imu_list
    devlist = []
    wrapper = ''
    seen = set()
    for dev in imu_list:
        if seen_str(dev) in seen:
            error("Duplicate IMU: %s" % seen_str(dev))
        seen.add(seen_str(dev))
        driver = dev[0]
        for i in range(1, len(dev)):
            if dev[i].startswith("SPI:"):
                dev[i] = parse_spi_device(dev[i])
            elif dev[i].startswith("I2C:"):
                (wrapper, dev[i]) = parse_i2c_device(dev[i])
        n = len(devlist)+1
        devlist.append('HAL_INS_PROBE%u' % n)
        f.write(
            '#define HAL_INS_PROBE%u %s ADD_BACKEND(AP_InertialSensor_%s::probe(*this,%s))\n'
            % (n, wrapper, driver, ','.join(dev[1:])))
    if len(devlist) > 0:
        if len(devlist) < 3:
            f.write('#define INS_MAX_INSTANCES %u\n' % len(devlist))
        f.write('#define HAL_INS_PROBE_LIST %s\n\n' % ';'.join(devlist))
    #exit(1) # buzz hack


def write_MAG_config(f):
    '''write MAG config defines'''
    global compass_list
    devlist = []
    seen = set()
    for dev in compass_list:
        if seen_str(dev) in seen:
            error("Duplicate MAG: %s" % seen_str(dev))
        seen.add(seen_str(dev))
        driver = dev[0]
        probe = 'probe'
        wrapper = ''
        a = driver.split(':')
        driver = a[0]
        if len(a) > 1 and a[1].startswith('probe'):
            probe = a[1]
        for i in range(1, len(dev)):
            if dev[i].startswith("SPI:"):
                dev[i] = parse_spi_device(dev[i])
            elif dev[i].startswith("I2C:"):
                (wrapper, dev[i]) = parse_i2c_device(dev[i])
        n = len(devlist)+1
        devlist.append('HAL_MAG_PROBE%u' % n)
        f.write(
            '#define HAL_MAG_PROBE%u %s ADD_BACKEND(DRIVER_%s, AP_Compass_%s::%s(%s))\n'
            % (n, wrapper, driver, driver, probe, ','.join(dev[1:])))
    if len(devlist) > 0:
        f.write('#define HAL_MAG_PROBE_LIST %s\n\n' % ';'.join(devlist))


def write_BARO_config(f):
    '''write barometer config defines'''
    global baro_list
    devlist = []
    seen = set()
    for dev in baro_list:
        if seen_str(dev) in seen:
            error("Duplicate BARO: %s" % seen_str(dev))
        seen.add(seen_str(dev))
        driver = dev[0]
        probe = 'probe'
        wrapper = ''
        a = driver.split(':')
        driver = a[0]
        if len(a) > 1 and a[1].startswith('probe'):
            probe = a[1]
        for i in range(1, len(dev)):
            if dev[i].startswith("SPI:"):
                dev[i] = parse_spi_device(dev[i])
            elif dev[i].startswith("I2C:"):
                (wrapper, dev[i]) = parse_i2c_device(dev[i])
                if dev[i].startswith('hal.i2c_mgr'):
                    dev[i] = 'std::move(%s)' % dev[i]
        n = len(devlist)+1
        devlist.append('HAL_BARO_PROBE%u' % n)
        args = ['*this'] + dev[1:]
        f.write(
            '#define HAL_BARO_PROBE%u %s ADD_BACKEND(AP_Baro_%s::%s(%s))\n'
            % (n, wrapper, driver, probe, ','.join(args)))
    if len(devlist) > 0:
        f.write('#define HAL_BARO_PROBE_LIST %s\n\n' % ';'.join(devlist))

def write_AIRSPEED_config(f):
    '''write airspeed config defines'''
    global airspeed_list
    devlist = []
    seen = set()
    idx = 0
    for dev in airspeed_list:
        if seen_str(dev) in seen:
            error("Duplicate AIRSPEED: %s" % seen_str(dev))
        seen.add(seen_str(dev))
        driver = dev[0]
        wrapper = ''
        a = driver.split(':')
        driver = a[0]
        for i in range(1, len(dev)):
            if dev[i].startswith("SPI:"):
                dev[i] = parse_spi_device(dev[i])
            elif dev[i].startswith("I2C:"):
                (wrapper, dev[i]) = parse_i2c_device(dev[i])
                if dev[i].startswith('hal.i2c_mgr'):
                    dev[i] = 'std::move(%s)' % dev[i]
        n = len(devlist)+1
        devlist.append('HAL_AIRSPEED_PROBE%u' % n)
        args = ['*this', str(idx)] + dev[1:]
        f.write(
            '#define HAL_AIRSPEED_PROBE%u %s ADD_BACKEND(AP_Airspeed_%s::probe(%s))\n'
            % (n, wrapper, driver, ','.join(args)))
        idx += 1
    if len(devlist) > 0:
        f.write('#define HAL_AIRSPEED_PROBE_LIST %s\n\n' % ';'.join(devlist))
        
def write_board_validate_macro(f):
    '''write board validation macro'''
    global config
    validate_string = ''
    validate_dict = {}
    if 'BOARD_VALIDATE' in config:
        for check in config['BOARD_VALIDATE']:
            check_name = check
            check_string = check
            while True:
                def substitute_alias(m):
                    return '(' + get_config(m.group(1), spaces=True) + ')'
                output = re.sub(r'\$(\w+|\{([^}]*)\})', substitute_alias, check_string)
                if (output == check_string):
                    break
                check_string = output
            validate_dict[check_name] = check_string
        # Finally create check conditional
        for check_name in validate_dict:
            validate_string += "!" + validate_dict[check_name] + "?" + "\"" + check_name + "\"" + ":"
        validate_string += "nullptr"
        f.write('#define HAL_VALIDATE_BOARD (%s)\n\n' % validate_string) 

def get_gpio_bylabel(label):
    '''get GPIO(n) setting on a pin label, or -1'''
    p = bylabel.get(label)
    if p is None:
        return -1
    return p.extra_value('GPIO', type=int, default=-1)


def get_extra_bylabel(label, name, default=None):
    '''get extra setting for a label by name'''
    p = bylabel.get(label)
    if p is None:
        return default
    return p.extra_value(name, type=str, default=default)

def get_UART_ORDER():
    '''get UART_ORDER from SERIAL_ORDER option'''
    if get_config('UART_ORDER', required=False, aslist=True) is not None:
        error('Please convert UART_ORDER to SERIAL_ORDER')
    serial_order = get_config('SERIAL_ORDER', required=False, aslist=True)
    if serial_order is None:
        return None
    if args.bootloader:
        # in bootloader SERIAL_ORDER is treated the same as UART_ORDER
        return serial_order
    map = [ 0, 3, 1, 2, 4, 5, 6, 7, 8, 9, 10, 11, 12 ]
    while len(serial_order) < 4:
        serial_order += ['EMPTY']
    uart_order = []
    global uart_serial_num
    for i in range(len(serial_order)):
        uart_order.append(serial_order[map[i]])
        uart_serial_num[serial_order[i]] = i
    return uart_order

def write_UART_config(f):
    '''write UART config defines'''
    global dual_USB_enabled
    uart_list = get_UART_ORDER()
    if uart_list is None:
        return
    f.write('\n// UART configuration\n')

    # write out driver declarations for HAL_ChibOS_Class.cpp
    devnames = "ABCDEFGHIJ"
    sdev = 0
    idx = 0
    for dev in uart_list:
        if dev == 'EMPTY':
            f.write('#define HAL_UART%s_DRIVER Empty::UARTDriver uart%sDriver\n' %
                    (devnames[idx], devnames[idx]))
            sdev += 1
        else:
            f.write(
                '#define HAL_UART%s_DRIVER ESP32::UARTDriver uart%sDriver(%u)\n'
                % (devnames[idx], devnames[idx], sdev))
            sdev += 1
        idx += 1
    for idx in range(len(uart_list), len(devnames)):
        f.write('#define HAL_UART%s_DRIVER Empty::UARTDriver uart%sDriver\n' %
                (devnames[idx], devnames[idx]))

    # if 'IOMCU_UART' in config:
    #     if not 'io_firmware.bin' in romfs:
    #         error("Need io_firmware.bin in ROMFS for IOMCU")

    #     f.write('#define HAL_WITH_IO_MCU 1\n')
    #     idx = len(uart_list)
    #     f.write('#define HAL_UART_IOMCU_IDX %u\n' % idx)
    #     f.write(
    #         '#define HAL_UART_IO_DRIVER ESP32::UARTDriver uart_io(HAL_UART_IOMCU_IDX)\n'
    #     )
    #     uart_list.append(config['IOMCU_UART'][0])
    #     f.write('#define HAL_HAVE_SERVO_VOLTAGE 1\n') # make the assumption that IO gurantees servo monitoring
    #     # all IOMCU capable boards have SBUS out
    #     f.write('#define AP_FEATURE_SBUS_OUT 1\n')
    # else:
    f.write('#define HAL_WITH_IO_MCU 0\n')
    #f.write('\n')

    need_uart_driver = False
    OTG2_index = None
    devlist = []
    have_rts_cts = False
    crash_uart = None

    # write config for CrashCatcher UART
    # if not uart_list[0].startswith('OTG') and not uart_list[0].startswith('EMPTY'):
    #     crash_uart = uart_list[0]
    # elif not uart_list[2].startswith('OTG') and not uart_list[2].startswith('EMPTY'):
    #     crash_uart = uart_list[2]

    if crash_uart is not None and get_config('FLASH_SIZE_KB', type=int) >= 2048:
        f.write('#define HAL_CRASH_SERIAL_PORT %s\n' % crash_uart)
        f.write('#define IRQ_DISABLE_HAL_CRASH_SERIAL_PORT() nvicDisableVector(STM32_%s_NUMBER)\n' % crash_uart)
        f.write('#define RCC_RESET_HAL_CRASH_SERIAL_PORT() rccReset%s(); rccEnable%s(true)\n' % (crash_uart, crash_uart))
        #f.write('#define HAL_CRASH_SERIAL_PORT_CLOCK STM32_%sCLK\n' % crash_uart)
    for dev in uart_list:
        if dev.startswith('UART'):
            n = int(dev[4:])
        elif dev.startswith('USART'):
            n = int(dev[5:])
        elif dev.startswith('OTG'):
            n = int(dev[3:])
        elif dev.startswith('EMPTY'):
            devlist.append('{}')
            continue
        else:
            error("Invalid element %s in UART_ORDER" % dev)
        devlist.append('HAL_%s_CONFIG' % dev)
        tx_line = make_line(dev + '_TX')
        rx_line = make_line(dev + '_RX')
        rts_line = make_line(dev + '_RTS')
        cts_line = make_line(dev + '_CTS')
        if rts_line != "0":
            have_rts_cts = True
            f.write('#define HAL_HAVE_RTSCTS_SERIAL%u\n' % uart_serial_num[dev])

        if dev.startswith('OTG2'):
            f.write(
                '#define HAL_%s_CONFIG {(BaseSequentialStream*) &SDU2, 2, true, false, 0, 0, false, 0, 0, 2}\n'
                % dev)
            OTG2_index = uart_list.index(dev)
            dual_USB_enabled = True
        elif dev.startswith('OTG'):
            f.write(
                '#define HAL_%s_CONFIG {(BaseSequentialStream*) &SDU1, 1, true, false, 0, 0, false, 0, 0, 0}\n'
                % dev)
        else:
            need_uart_driver = True
            f.write(
                "#define HAL_%s_CONFIG { (BaseSequentialStream*) &SD%u, %u, false, "
                % (dev, n, n))
            #f.write("STM32_%s_RX_DMA_CONFIG, STM32_%s_TX_DMA_CONFIG, %s, %s, %s, %s, " %
            #        (dev, dev, tx_line, rx_line, rts_line, cts_line))

            # add inversion pins, if any
            f.write("%d, " % get_gpio_bylabel(dev + "_RXINV"))
            f.write("%s, " % get_extra_bylabel(dev + "_RXINV", "POL", "0"))
            f.write("%d, " % get_gpio_bylabel(dev + "_TXINV"))
            f.write("%s, 0}\n" % get_extra_bylabel(dev + "_TXINV", "POL", "0"))
    if have_rts_cts:
        f.write('#define AP_FEATURE_RTSCTS 1\n')
    if OTG2_index is not None:
        f.write('#define HAL_OTG2_UART_INDEX %d\n' % OTG2_index)
        f.write('#define HAL_HAVE_DUAL_USB_CDC 1\n')
        if not is_periph_fw():
            f.write('''
#if defined(HAL_NUM_CAN_IFACES) && HAL_NUM_CAN_IFACES
#ifndef HAL_OTG2_PROTOCOL
#define HAL_OTG2_PROTOCOL SerialProtocol_SLCAN
#endif
#define DEFAULT_SERIAL%d_PROTOCOL HAL_OTG2_PROTOCOL
#define HAL_SERIAL%d_BAUD 115200
#endif
''' % (OTG2_index, OTG2_index))

    f.write('#define HAL_UART_DEVICE_LIST %s\n\n' % ','.join(devlist))
    if not need_uart_driver and not args.bootloader:
        f.write('''
#ifndef HAL_USE_SERIAL
#define HAL_USE_SERIAL HAL_USE_SERIAL_USB
#endif
''')
    num_uarts = len(devlist)
    if 'IOMCU_UART' in config:
        num_uarts -= 1
    #if 'IOMCU_UART' in config:
    num_uarts += 2  # the tcp and the udp driver are two additional virtual uarts
    if num_uarts > 10:
        error("Exceeded max num UARTs of 10 (%u)" % num_uarts)
    f.write('#define HAL_UART_NUM_SERIAL_PORTS %u\n' % num_uarts)


def write_UART_config_bootloader(f):
    '''write UART config defines'''
    uart_list = get_UART_ORDER()
    if uart_list is None:
        return
    f.write('\n// UART configuration\n')
    devlist = []
    have_uart = False
    OTG2_index = None
    for u in uart_list:
        if u.startswith('OTG2'):
            devlist.append('(BaseChannel *)&SDU2')
            OTG2_index = uart_list.index(u)
        elif u.startswith('OTG'):
            devlist.append('(BaseChannel *)&SDU1')
        else:
            unum = int(u[-1])
            devlist.append('(BaseChannel *)&SD%u' % unum)
            have_uart = True
    if len(devlist) > 0:
        f.write('#define BOOTLOADER_DEV_LIST %s\n' % ','.join(devlist))
    if OTG2_index is not None:
        f.write('#define HAL_OTG2_UART_INDEX %d\n' % OTG2_index)
    if not have_uart:
        f.write('''
#ifndef HAL_USE_SERIAL
#define HAL_USE_SERIAL FALSE
#endif
''')


def write_I2C_config(f):
    '''write I2C config defines'''
    if not have_type_prefix('I2C'):
        print("No I2C peripherals")
        f.write('''
#ifndef HAL_USE_I2C
#define HAL_USE_I2C FALSE
#endif
''')
        return
    if 'I2C_ORDER' not in config:
        print("Missing I2C_ORDER config")
        return
    i2c_list = config['I2C_ORDER']
    f.write('// I2C configuration\n')
    if len(i2c_list) == 0:
        error("I2C_ORDER invalid")
    devlist = []

    # write out config structures
    for dev in i2c_list:
        if not dev.startswith('I2C') or dev[3] not in "1234":
            error("Bad I2C_ORDER element %s" % dev)
        n = int(dev[3:])
        devlist.append('HAL_I2C%u_CONFIG' % n)
        sda_line = make_line('I2C%u_SDA' % n)
        scl_line = make_line('I2C%u_SCL' % n)
        #f.write('''
#if defined(STM32_I2C_I2C%u_RX_DMA_STREAM) && defined(STM32_I2C_I2C%u_TX_DMA_STREAM)
#define HAL_I2C%u_CONFIG { &I2CD%u, %u, STM32_I2C_I2C%u_RX_DMA_STREAM, STM32_I2C_I2C%u_TX_DMA_STREAM, %s, %s }
#else
#define HAL_I2C%u_CONFIG { &I2CD%u, %u, SHARED_DMA_NONE, SHARED_DMA_NONE, %s, %s }
#endif
#'''
#                % (n, n, n, n, n, n, n, scl_line, sda_line, n, n, n, scl_line, sda_line))
    f.write('\n#define HAL_I2C_DEVICE_LIST %s\n\n' % ','.join(devlist))


def parse_timer(str):
    '''parse timer channel string, i.e TIM8_CH2N'''
    result = re.match(r'TIM([0-9]*)_CH([1234])(N?)', str)
    if result:
        tim = int(result.group(1))
        chan = int(result.group(2))
        compl = result.group(3) == 'N'
        if tim < 1 or tim > 17:
            error("Bad timer number %s in %s" % (tim, str))
        return (tim, chan, compl)
    else:
        error("Bad timer definition %s" % str)


def write_PWM_config(f, ordered_timers):
    '''write PWM config defines'''
    rc_in = None
    rc_in_int = None
    alarm = None
    bidir = None
    pwm_out = []
    # start with the ordered list from the dma resolver
    pwm_timers = ordered_timers
    has_bidir = False
    for l in bylabel.keys():
        p = bylabel[l]
        if p.type.startswith('TIM'):
            if p.has_extra('RCIN'):
                rc_in = p
            elif p.has_extra('RCININT'):
                rc_in_int = p
            elif p.has_extra('ALARM'):
                alarm = p
            else:
                if p.extra_value('PWM', type=int) is not None:
                    pwm_out.append(p)
                if p.has_extra('BIDIR'):
                    bidir = p
                if p.type not in pwm_timers:
                    pwm_timers.append(p.type)


    f.write('#define HAL_PWM_COUNT %u\n' % len(pwm_out))
    if not pwm_out and not alarm:
        print("No PWM output defined")
        f.write('''
#ifndef HAL_USE_PWM
#define HAL_USE_PWM FALSE
#endif
''')

    if rc_in is not None:
        (n, chan, compl) = parse_timer(rc_in.label)
        if compl:
            # it is an inverted channel
            f.write('#define HAL_RCIN_IS_INVERTED\n')
        if chan not in [1, 2]:
            error(
                "Bad channel number, only channel 1 and 2 supported for RCIN")
        f.write('// RC input config\n')
        f.write('#define HAL_USE_ICU TRUE\n')
        #f.write('#define STM32_ICU_USE_TIM%u TRUE\n' % n)
        f.write('#define RCIN_ICU_TIMER ICUD%u\n' % n)
        f.write('#define RCIN_ICU_CHANNEL ICU_CHANNEL_%u\n' % chan)
        #f.write('#define STM32_RCIN_DMA_STREAM STM32_TIM_TIM%u_CH%u_DMA_STREAM\n' % (n, chan))
        #f.write('#define STM32_RCIN_DMA_CHANNEL STM32_TIM_TIM%u_CH%u_DMA_CHAN\n' % (n, chan))
        f.write('\n')

    if rc_in_int is not None:
        (n, chan, compl) = parse_timer(rc_in_int.label)
        if compl:
            error('Complementary channel is not supported for RCININT %s' % rc_in_int.label)
        f.write('// RC input config\n')
        f.write('#define HAL_USE_EICU TRUE\n')
        #f.write('#define STM32_EICU_USE_TIM%u TRUE\n' % n)
        f.write('#define RCININT_EICU_TIMER EICUD%u\n' % n)
        f.write('#define RCININT_EICU_CHANNEL EICU_CHANNEL_%u\n' % chan)
        f.write('\n')

    if alarm is not None:
        (n, chan, compl) = parse_timer(alarm.label)
        if compl:
            error("Complementary channel is not supported for ALARM %s" % alarm.label)
        f.write('\n')
        f.write('// Alarm PWM output config\n')
        #f.write('#define STM32_PWM_USE_TIM%u TRUE\n' % n)
        #f.write('#define STM32_TIM%u_SUPPRESS_ISR\n' % n)

        chan_mode = [
            'PWM_OUTPUT_DISABLED', 'PWM_OUTPUT_DISABLED',
            'PWM_OUTPUT_DISABLED', 'PWM_OUTPUT_DISABLED'
        ]
        chan_mode[chan - 1] = 'PWM_OUTPUT_ACTIVE_HIGH'

        pwm_clock = 1000000
        period = 1000

        #f.write('''#define HAL_PWM_ALARM 0''') # buzz disabld alarm on periph as we don't have hardwar eworking forit yet

        # f.write('''#define HAL_PWM_ALARM \\
        # { /* pwmGroup */ \\
        #   %u,  /* Timer channel */ \\
        #   { /* PWMConfig */ \\
        #     %u,    /* PWM clock frequency. */ \\
        #     %u,    /* Initial PWM period 20ms. */ \\
        #     NULL,  /* no callback */ \\
        #     { /* Channel Config */ \\
        #      {%s, NULL}, \\
        #      {%s, NULL}, \\
        #      {%s, NULL}, \\
        #      {%s, NULL}  \\
        #     }, \\
        #     0, 0 \\
        #   }, \\
        #   &PWMD%u /* PWMDriver* */ \\
        # }\n''' %
        #         (chan-1, pwm_clock, period, chan_mode[0],
        #          chan_mode[1], chan_mode[2], chan_mode[3], n))
    #else:
        f.write('\n')
        f.write('// No Alarm output pin defined\n')
        f.write('#undef HAL_PWM_ALARM\n')
    f.write('\n')

    f.write('// PWM timer config\n')
    if bidir is not None:
        f.write('#define HAL_WITH_BIDIR_DSHOT\n')
    for t in pwm_timers:
        n = int(t[3:])
        #f.write('#define STM32_PWM_USE_TIM%u TRUE\n' % n)
        #f.write('#define STM32_TIM%u_SUPPRESS_ISR\n' % n)
    f.write('\n')
    f.write('// PWM output config\n')
    groups = []
    have_complementary = False

    for t in pwm_timers:
        group = len(groups) + 1
        n = int(t[3:])
        chan_list = [255, 255, 255, 255]
        chan_mode = [
            'PWM_OUTPUT_DISABLED', 'PWM_OUTPUT_DISABLED',
            'PWM_OUTPUT_DISABLED', 'PWM_OUTPUT_DISABLED'
        ]
        alt_functions = [0, 0, 0, 0]
        pal_lines = ['0', '0', '0', '0']
        for p in pwm_out:
            if p.type != t:
                continue
            (n, chan, compl) = parse_timer(p.label)
            pwm = p.extra_value('PWM', type=int)
            chan_list[chan - 1] = pwm - 1
            if compl:
                chan_mode[chan - 1] = 'PWM_COMPLEMENTARY_OUTPUT_ACTIVE_HIGH'
                have_complementary = True
            else:
                chan_mode[chan - 1] = 'PWM_OUTPUT_ACTIVE_HIGH'
            alt_functions[chan - 1] = p.af
            pal_lines[chan - 1] = 'PAL_LINE(GPIO%s,%uU)' % (p.port, p.pin)
        groups.append('HAL_PWM_GROUP%u' % group)
        if n in [1, 8]:
            # only the advanced timers do 8MHz clocks
            advanced_timer = 'true'
        else:
            advanced_timer = 'false'
        pwm_clock = 1000000
        period = 20000 * pwm_clock / 1000000
        hal_icu_def = ''
        hal_icu_cfg = ''
        if bidir is not None:
            hal_icu_cfg = '\n          {'
            hal_icu_def = '\n'
            for i in range(1,5):
                hal_icu_cfg += '{HAL_IC%u_CH%u_DMA_CONFIG},' % (n, i)
                hal_icu_def +='''#if defined(STM32_TIM_TIM%u_CH%u_DMA_STREAM) && defined(STM32_TIM_TIM%u_CH%u_DMA_CHAN)
# define HAL_IC%u_CH%u_DMA_CONFIG true, STM32_TIM_TIM%u_CH%u_DMA_STREAM, STM32_TIM_TIM%u_CH%u_DMA_CHAN
#else
# define HAL_IC%u_CH%u_DMA_CONFIG false, 0, 0
#endif
''' % (n, i, n, i, n, i, n, i, n, i, n, i)
            hal_icu_cfg += '},  \\'


        f.write('''#if defined(STM32_TIM_TIM%u_UP_DMA_STREAM) && defined(STM32_TIM_TIM%u_UP_DMA_CHAN)
# define HAL_PWM%u_DMA_CONFIG true, STM32_TIM_TIM%u_UP_DMA_STREAM, STM32_TIM_TIM%u_UP_DMA_CHAN
#else
# define HAL_PWM%u_DMA_CONFIG false, 0, 0
#endif\n%s''' % (n, n, n, n, n, n, hal_icu_def))
        f.write('''#define HAL_PWM_GROUP%u { %s, \\
        {%u, %u, %u, %u}, \\
        /* Group Initial Config */ \\
        { \\
          %u,  /* PWM clock frequency. */ \\
          %u,   /* Initial PWM period 20ms. */ \\
          NULL,     /* no callback */ \\
          { \\
           /* Channel Config */ \\
           {%s, NULL}, \\
           {%s, NULL}, \\
           {%s, NULL}, \\
           {%s, NULL}  \\
          }, 0, 0}, &PWMD%u, \\
          HAL_PWM%u_DMA_CONFIG, \\%s
          { %u, %u, %u, %u }, \\
          { %s, %s, %s, %s }}\n''' %
                (group, advanced_timer,
                 chan_list[0], chan_list[1], chan_list[2], chan_list[3],
                 pwm_clock, period,
                 chan_mode[0], chan_mode[1], chan_mode[2], chan_mode[3],
                 n, n, hal_icu_cfg,
                 alt_functions[0], alt_functions[1], alt_functions[2], alt_functions[3],
                 pal_lines[0], pal_lines[1], pal_lines[2], pal_lines[3]))
    f.write('#define HAL_PWM_GROUPS %s\n\n' % ','.join(groups))
    if have_complementary:
        #f.write('#define STM32_PWM_USE_ADVANCED TRUE\n')
        pass


def write_ADC_config(f):
    '''write ADC config defines'''
    f.write('// ADC config\n')
    adc_chans = []
    for l in bylabel:
        p = bylabel[l]
        if not p.type.startswith('ADC'):
            continue
        chan = get_ADC1_chan(mcu_type, p.portpin)
        scale = p.extra_value('SCALE', default=None)
        if p.label == 'VDD_5V_SENS':
            f.write('#define ANALOG_VCC_5V_PIN %u\n' % chan)
            f.write('#define HAL_HAVE_BOARD_VOLTAGE 1\n')
        if p.label == 'FMU_SERVORAIL_VCC_SENS':
            f.write('#define FMU_SERVORAIL_ADC_CHAN %u\n' % chan)
            f.write('#define HAL_HAVE_SERVO_VOLTAGE 1\n')
        adc_chans.append((chan, scale, p.label, p.portpin))
    adc_chans = sorted(adc_chans)
    vdd = get_config('STM32_VDD', default='330U')
    if vdd[-1] == 'U':
        vdd = vdd[:-1]
    vdd = float(vdd) * 0.01
    f.write('#define HAL_USE_ADC TRUE \n')
    f.write('#define HAL_ANALOG_PINS { \\\n')
    for (chan, scale, label, portpin) in adc_chans:
        scale_str = '%.2f/4096' % vdd
        if scale is not None and scale != '1':
            scale_str = scale + '*' + scale_str
        f.write('{ %2u, %12s }, /* %s %s */ \\\n' % (chan, scale_str, portpin,
                                                     label))
    f.write('}\n\n')


def write_GPIO_config(f):
    '''write GPIO config defines'''
    f.write('// GPIO config\n')
    gpios = []
    gpioset = set()
    for l in bylabel:
        p = bylabel[l]
        gpio = p.extra_value('GPIO', type=int)
        if gpio is None:
            continue
        if gpio in gpioset:
            error("Duplicate GPIO value %u" % gpio)
        gpioset.add(gpio)
        # see if it is also a PWM pin
        pwm = p.extra_value('PWM', type=int, default=0)
        port = p.port
        pin = p.pin
        # default config always enabled
        gpios.append((gpio, pwm, port, pin, p, 'true'))
    for alt in altmap.keys():
        for pp in altmap[alt].keys():
            p = altmap[alt][pp]
            gpio = p.extra_value('GPIO', type=int)
            if gpio is None:
                continue
            if gpio in gpioset:
                # check existing entry
                existing_gpio = [item for item in gpios if item[0] == gpio]
                if (existing_gpio[0][4].label == p.label) and (existing_gpio[0][3] == p.pin) and (existing_gpio[0][2] == p.port):
                    # alt item is identical to exiting, do not add again
                    continue
                error("Duplicate GPIO value %u, %s != %s" % (gpio, p, existing_gpio[0][4]))
            pwm = p.extra_value('PWM', type=int, default=0)
            if pwm != 0:
                error("PWM not supported for alt config: %s" % p)
            gpioset.add(gpio)
            port = p.port
            pin = p.pin
            # aux config disabled by defualt
            gpios.append((gpio, pwm, port, pin, p, 'false'))
    gpios = sorted(gpios)
    for (gpio, pwm, port, pin, p, enabled) in gpios:
        f.write('#define HAL_GPIO_LINE_GPIO%u PAL_LINE(GPIO%s,%uU)\n' % (gpio, port, pin))
    f.write('#define HAL_GPIO_PINS { \\\n')
    for (gpio, pwm, port, pin, p, enabled) in gpios:
        f.write('{ %3u, %s, %2u, PAL_LINE(GPIO%s,%uU)}, /* %s */ \\\n' %
                (gpio, enabled, pwm, port, pin, p))
    # and write #defines for use by config code
    f.write('}\n\n')
    f.write('// full pin define list\n')
    last_label = None
    for l in sorted(list(set(bylabel.keys()))):
        p = bylabel[l]
        label = p.label
        label = label.replace('-', '_')
        if label == last_label:
            continue
        last_label = label
        f.write('//todo define HAL_GPIO_PIN_%-20s PAL_LINE(GPIO%s,%uU)\n' %
                (label, p.port, p.pin))
    f.write('\n')


def bootloader_path():
    # always embed a bootloader if it is available
    this_dir = os.path.realpath(__file__)
    rootdir = os.path.relpath(os.path.join(this_dir, "../../../../.."))
    hwdef_dirname = os.path.basename(os.path.dirname(args.hwdef[0]))
    bootloader_filename = "%s_bl.bin" % (hwdef_dirname,)
    bootloader_path = os.path.join(rootdir,
                                   "Tools",
                                   "bootloaders",
                                   bootloader_filename)
    if os.path.exists(bootloader_path):
        return os.path.realpath(bootloader_path)

    return None


def add_bootloader():
    # '''added bootloader to ROMFS'''
    # bp = bootloader_path()
    # if bp is not None and int(get_config('BOOTLOADER_EMBED', required=False, default='1')):
    #     romfs["bootloader.bin"] = bp
    #     env_vars['BOOTLOADER_EMBED'] = 1
    # else:
    env_vars['BOOTLOADER_EMBED'] = 0



def write_ROMFS(outdir):
    '''create ROMFS embedded header'''
    romfs_list = []
    for k in romfs.keys():
        romfs_list.append((k, romfs[k]))
    nodupes = list(set(romfs_list))
    env_vars['ROMFS_FILES'] = nodupes


def setup_apj_IDs():
    '''setup the APJ board IDs'''
    env_vars['APJ_BOARD_ID'] = get_config('APJ_BOARD_ID')
    env_vars['APJ_BOARD_TYPE'] = get_config('APJ_BOARD_TYPE', default=mcu_type)
    (USB_VID, USB_PID) = get_USB_IDs()
    env_vars['USBID'] = '0x%04x/0x%04x' % (USB_VID, USB_PID)


def write_peripheral_enable(f):
    '''write peripheral enable lines'''
    f.write('// peripherals enabled\n')
    for type in sorted(list(bytype.keys()) + list(alttype.keys())):
        if type.startswith('USART') or type.startswith('UART'):
            #dstr = 'STM32_SERIAL_USE_%-6s' % type
            #f.write('#ifndef %s\n' % dstr)
            #f.write('#define %s TRUE\n' % dstr)
            #f.write('#endif\n')
            pass
        # if type.startswith('SPI'):
        #     f.write('#define STM32_SPI_USE_%s                  TRUE\n' % type)
        # if type.startswith('OTG'):
        #     f.write('#define STM32_USB_USE_%s                  TRUE\n' % type)
        # if type.startswith('I2C'):
        #     f.write('#define STM32_I2C_USE_%s                  TRUE\n' % type)
        # if type.startswith('QUADSPI'):
        #     f.write('#define STM32_WSPI_USE_%s                 TRUE\n' % type)


def get_dma_exclude(periph_list):
    '''return list of DMA devices to exclude from DMA'''
    dma_exclude = set()
    for p in dma_exclude_pattern:
        for periph in periph_list:
            if fnmatch.fnmatch(periph, p):
                dma_exclude.add(periph)

    for periph in periph_list:
        if periph in bylabel:
            p = bylabel[periph]
            if p.has_extra('NODMA'):
                dma_exclude.add(periph)
        if periph in altlabel:
            p = altlabel[periph]
            if p.has_extra('NODMA'):
                dma_exclude.add(periph)
    return list(dma_exclude)


def write_alt_config(f):
    '''write out alternate config settings'''
    if len(altmap.keys()) == 0:
        # no alt configs
        return
    f.write('''
/* alternative configurations */
//#define PAL_STM32_SPEED(n) ((n&3U)<<3U)
//#define PAL_STM32_HIGH     0x8000U

#define HAL_PIN_ALT_CONFIG { \\
''')
    for alt in altmap.keys():
        for pp in altmap[alt].keys():
            p = altmap[alt][pp]
            f.write("    { %u, %s, PAL_LINE(GPIO%s,%uU), %s, %u}, /* %s */ \\\n" % (alt, p.pal_modeline(), p.port, p.pin, p.periph_type(), p.periph_instance(), str(p)))
    f.write('}\n\n')

def write_all_lines(hwdat):
    f = open(hwdat, 'w')
    f.write('\n'.join(all_lines))
    f.close()
    if not 'AP_PERIPH' in env_vars:
        romfs["hwdef.dat"] = hwdat

def write_hwdef_header(outfilename):
  '''write hwdef header file'''

  print("Writing hwdef setup in %s" % outfilename)
  tmpfile = outfilename + ".tmp"
  f = open(tmpfile, 'w')

  f.write('''/*
 generated hardware definitions from hwdef.dat or similar - DO NOT EDIT
  more details below...
*/
#pragma once  
#ifndef TRUE
#define TRUE 1
#endif
#ifndef FALSE
#define FALSE 0
#endif
#ifndef MHZ
#define MHZ (1000U*1000U)
#endif
#ifndef KHZ
#define KHZ (1000U)
#endif
''')


  #First look for the old style manually curated .h and use that if it exists..
  # if we don't have a xxxx/hwdef.dat, fallback to the old method
  this_dir = os.path.realpath(__file__)
  rootdir = os.path.relpath(os.path.join(this_dir, "../../../../.."))
  hwdef_dirname = os.path.basename(os.path.dirname(args.hwdef[0]))
  dot_h_filename = "%s.h" % (hwdef_dirname,)
  dot_h_path = os.path.join(rootdir,
                            "libraries",
                            "AP_HAL_ESP32",
                            "boards")
  dot_h_result = os.path.join(dot_h_path, dot_h_filename)
  if os.path.exists(dot_h_result):
      print("//---------------------------------------")
      print("// Using old style ",dot_h_filename," for config...")
      print("//  ...please migrate to a hwdef.dat from this file and retry for new process.")
      print("//---------------------------------------")
      lines = ''
      with open(dot_h_result) as z:
        lines = z.read()
      f.write("// --------------------------------------------------------------------------------\n")
      f.write("// This file contains OLD-STYLE static content from libraries/AP_HAL_ESP32/boards/%s\n" % dot_h_filename)
      f.write("// This file contains OLD-STYLE static content from libraries/AP_HAL_ESP32/boards/%s\n" % dot_h_filename)
      f.write("// --------------------------------------------------------------------------------\n")
      f.write("#define ESP_BUILD_TYPE 1\n")
      f.write(lines)
      f.close()
      os.rename(tmpfile, outfilename)
      return 'old'
  else:
    print("//---------------------------------------")
    print("// Using NEW style xxx/hwdef.dat for config...")
    print("//---------------------------------------")

    f.write("// --------------------------------------------------------------------------------\n")
    f.write("// This file contains NEW-STYLE content from  libraries/AP_HAL_ESP32/hwdef/%s/hwdef.dat\n" % hwdef_dirname)
    f.write("// This file contains NEW-STYLE content from  libraries/AP_HAL_ESP32/hwdef/%s/hwdef.dat\n" % hwdef_dirname)
    f.write("// --------------------------------------------------------------------------------\n")
    f.write("#define ESP_BUILD_TYPE 2\n")

    dma_noshare.extend(get_config('DMA_NOSHARE', default='', aslist=True))

    write_mcu_config(f)
    write_SPI_config(f)
    write_QSPI_config(f)
    write_ADC_config(f)
    write_GPIO_config(f)
    write_IMU_config(f)
    write_MAG_config(f)
    write_BARO_config(f)
    write_AIRSPEED_config(f)
    write_board_validate_macro(f)
    add_apperiph_defaults(f)

    write_peripheral_enable(f)

    # if mcu_series.startswith("STM32H7"):
    #     # add in ADC3 on H7 to get MCU temperature and reference voltage
    #     periph_list.append('ADC3')

    dma_unassigned, ordered_timers = dma_resolver.write_dma_header(f, periph_list, mcu_type,
                                                   dma_exclude=get_dma_exclude(periph_list),
                                                   dma_priority=get_config('DMA_PRIORITY', default='TIM* SPI*', spaces=True),
                                                   dma_noshare=dma_noshare)

    if not args.bootloader:
        write_PWM_config(f, ordered_timers)
        write_I2C_config(f)
        write_UART_config(f)
    else:
        write_UART_config_bootloader(f)

    setup_apj_IDs()
    write_USB_config(f)

    add_bootloader()

    if len(romfs) > 0:
        f.write('#define HAL_HAVE_AP_ROMFS_EMBEDDED_H 1\n')

    f.write('''
/*
* I/O ports initial setup, this configuration is established soon after reset
* in the initialization code.
* Please refer to the STM32 Reference Manual for details.
*/
#define PIN_MODE_INPUT(n)           (0U << ((n) * 2U))
#define PIN_MODE_OUTPUT(n)          (1U << ((n) * 2U))
#define PIN_MODE_ALTERNATE(n)       (2U << ((n) * 2U))
#define PIN_MODE_ANALOG(n)          (3U << ((n) * 2U))
#define PIN_ODR_LOW(n)              (0U << (n))
#define PIN_ODR_HIGH(n)             (1U << (n))
#define PIN_OTYPE_PUSHPULL(n)       (0U << (n))
#define PIN_OTYPE_OPENDRAIN(n)      (1U << (n))
#define PIN_OSPEED_VERYLOW(n)       (0U << ((n) * 2U))
#define PIN_OSPEED_LOW(n)           (1U << ((n) * 2U))
#define PIN_OSPEED_MEDIUM(n)        (2U << ((n) * 2U))
#define PIN_OSPEED_HIGH(n)          (3U << ((n) * 2U))
#define PIN_PUPDR_FLOATING(n)       (0U << ((n) * 2U))
#define PIN_PUPDR_PULLUP(n)         (1U << ((n) * 2U))
#define PIN_PUPDR_PULLDOWN(n)       (2U << ((n) * 2U))
#define PIN_AFIO_AF(n, v)           ((v) << (((n) % 8U) * 4U))

''')

    for port in sorted(ports):
        f.write("/* PORT%s:\n" % port)
        for pin in range(pincount[port]):
            p = portmap[port][pin]
            if p.label is not None:
                f.write(" %s\n" % p)
        f.write("*/\n\n")

        if pincount[port] == 0:
            # handle blank ports
            for vtype in vtypes:
                f.write("#define VAL_GPIO%s_%-7s             0x0\n" % (port,
                                                                       vtype))
            f.write("\n\n\n")
            continue

        for vtype in vtypes:
            f.write("#define VAL_GPIO%s_%-7s (" % (p.port, vtype))
            first = True
            for pin in range(pincount[port]):
                p = portmap[port][pin]
                modefunc = getattr(p, "get_" + vtype)
                v = modefunc()
                if v is None:
                    continue
                if not first:
                    f.write(" | \\\n                           ")
                f.write(v)
                first = False
            if first:
                # there were no pin definitions, use 0
                f.write("0")
            f.write(")\n\n")
    write_alt_config(f)

    #if not mcu_series.startswith("STM32F1"):
    dma_required = ['SPI*', 'ADC*']
    if 'IOMCU_UART' in config:
        dma_required.append(config['IOMCU_UART'][0] + '*')
    for d in dma_unassigned:
        for r in dma_required:
            if fnmatch.fnmatch(d, r):
                error("Missing required DMA for %s" % d)

    f.close()
    # see if we ended up with the same file, on an unnecessary reconfigure
    try:
        if filecmp.cmp(outfilename, tmpfile):
            print("No change in hwdef.h")
            os.unlink(tmpfile)
            return
    except Exception:
        pass
    try:
        os.unlink(outfilename)
    except Exception:
        pass
    os.rename(tmpfile, outfilename)
    return 'new'


def build_peripheral_list():
    '''build a list of peripherals for DMA resolver to work on'''
    peripherals = []
    done = set()
    prefixes = ['SPI', 'USART', 'UART', 'I2C']
    periph_pins = allpins[:]
    for alt in altmap.keys():
        for p in altmap[alt].keys():
            periph_pins.append(altmap[alt][p])
    for p in periph_pins:
        type = p.type
        if type.startswith('TIM'):
            # we need to independently demand DMA for each channel
            type = p.label
        if type in done:
            continue
        for prefix in prefixes:
            if type.startswith(prefix):
                ptx = type + "_TX"
                prx = type + "_RX"
                if prefix in ['SPI', 'I2C']:
                    # in DMA map I2C and SPI has RX and TX suffix
                    if ptx not in bylabel:
                        bylabel[ptx] = p
                    if prx not in bylabel:
                        bylabel[prx] = p
                if prx in bylabel or prx in altlabel:
                    peripherals.append(prx)
                if ptx in bylabel or ptx in altlabel:
                    peripherals.append(ptx)

        if type.startswith('ADC'):
            peripherals.append(type)
        if type.startswith('SDIO') or type.startswith('SDMMC'):
            #if not mcu_series.startswith("STM32H7"):
            peripherals.append(type)
        if type.startswith('TIM'):
            if p.has_extra('RCIN'):
                label = p.label
                if label[-1] == 'N':
                    label = label[:-1]
                peripherals.append(label)
                # RCIN DMA channel cannot be shared as it is running all the time
                dma_noshare.append(label)
            elif not p.has_extra('ALARM') and not p.has_extra('RCININT'):
                # get the TIMn_UP DMA channels for DShot
                label = p.type + '_UP'
                if label not in peripherals and not p.has_extra('NODMA'):
                    peripherals.append(label)
                ch_label = type
                (_, _, compl) = parse_timer(ch_label)
                if ch_label not in peripherals and p.has_extra('BIDIR') and not compl:
                    peripherals.append(ch_label)
        done.add(type)
    return peripherals


def write_env_py(filename):
    '''write out env.py for environment variables to control the build process'''

    # see if board has a defaults.parm file or a --default-parameters file was specified
    defaults_filename = os.path.join(os.path.dirname(args.hwdef[0]), 'defaults.parm')
    #defaults_path = os.path.join(os.path.dirname(args.hwdef[0]), args.params)

    # if not args.bootloader:
    #     if os.path.exists(defaults_path):
    #         env_vars['DEFAULT_PARAMETERS'] = os.path.abspath(defaults_path)
    #         print("Default parameters path from command line: %s" % defaults_path)
    #     elif os.path.exists(defaults_filename):
    #         env_vars['DEFAULT_PARAMETERS'] = os.path.abspath(defaults_filename)
    #         print("Default parameters path from hwdef: %s" % defaults_filename)
    #     else:
    #         print("No default parameter file found")

    # CHIBIOS_BUILD_FLAGS is passed to the ESP32 makefile
    #env_vars['CHIBIOS_BUILD_FLAGS'] = ' '.join(build_flags)
    pickle.dump(env_vars, open(filename, "wb"))


def romfs_add(romfs_filename, filename):
    '''add a file to ROMFS'''
    romfs[romfs_filename] = filename


def romfs_wildcard(pattern):
    '''add a set of files to ROMFS by wildcard'''
    base_path = os.path.join(os.path.dirname(__file__), '..', '..', '..', '..')
    (pattern_dir, pattern) = os.path.split(pattern)
    for f in os.listdir(os.path.join(base_path, pattern_dir)):
        if fnmatch.fnmatch(f, pattern):
            romfs[f] = os.path.join(pattern_dir, f)

def romfs_add_dir(subdirs):
    '''add a filesystem directory to ROMFS'''
    for dirname in subdirs:
        romfs_dir = os.path.join(os.path.dirname(args.hwdef[0]), dirname)
        if not args.bootloader and os.path.exists(romfs_dir):
            for root, d, files in os.walk(romfs_dir):
                for f in files:
                    if fnmatch.fnmatch(f, '*~'):
                        # skip editor backup files
                        continue
                    fullpath = os.path.join(root, f)
                    relpath = os.path.normpath(os.path.join(dirname, os.path.relpath(root, romfs_dir), f))
                    romfs[relpath] = fullpath

def valid_type(ptype, label):
    '''check type of a pin line is valid'''
    patterns = [ 'INPUT', 'OUTPUT', 'TIM\\d+', 'USART\\d+', 'UART\\d+', 'ADC\\d+',
                'SPI\\d+', 'OTG\\d+', 'SWD', 'CAN\\d?', 'I2C\\d+', 'CS',
                'SDMMC\\d+', 'SDIO', 'QUADSPI\\d' ]
    matches = False
    for p in patterns:
        if re.match(p, ptype):
            matches = True
            break
    if not matches:
        return False
    # special checks for common errors
    m1 = re.match('TIM(\\d+)', ptype)
    m2 = re.match('TIM(\\d+)_CH\\d+', label)
    if (m1 and not m2) or (m2 and not m1) or (m1 and m1.group(1) != m2.group(1)):
        '''timer numbers need to match'''
        return False
    m1 = re.match('CAN(\\d+)', ptype)
    m2 = re.match('CAN(\\d+)_(RX|TX)', label)
    if (m1 and not m2) or (m2 and not m1) or (m1 and m1.group(1) != m2.group(1)):
        '''CAN numbers need to match'''
        return False
    if ptype == 'OUTPUT' and re.match('US?ART\\d+_(TXINV|RXINV)', label):
        return True
    m1 = re.match('USART(\\d+)', ptype)
    m2 = re.match('USART(\\d+)_(RX|TX|CTS|RTS)', label)
    if (m1 and not m2) or (m2 and not m1) or (m1 and m1.group(1) != m2.group(1)):
        '''usart numbers need to match'''
        return False
    m1 = re.match('UART(\\d+)', ptype)
    m2 = re.match('UART(\\d+)_(RX|TX|CTS|RTS)', label)
    if (m1 and not m2) or (m2 and not m1) or (m1 and m1.group(1) != m2.group(1)):
        '''uart numbers need to match'''
        return False
    return True

def process_line(line):
    '''process one line of pin definition file'''
    global portmap,allpins, imu_list, compass_list, baro_list, airspeed_list
    global mcu_type, mcu_series, default_gpio
    all_lines.append(line)
    a = shlex.split(line, posix=False)
    # keep all config lines for later use
    alllines.append(line)

    #print("Processing line: ",line)

    p = None
    if a[0].startswith('P') and a[0][1] in ports:
        # it is a port/pin definition
        try:
            port = a[0][1]
            pin = int(a[0][2:])
            label = a[1]
            type = a[2]
            extra = a[3:]
        except Exception:
            error("Bad pin line: %s" % a)
            return

        if not valid_type(type, label):
            error("bad type on line: %s" % a)

        p = generic_pin(port, pin, label, type, extra)
        af = get_alt_function(mcu_type, a[0], label)
        if af is not None:
            p.af = af

        alt = p.extra_value("ALT", type=int, default=0)
        if alt != 0:

            if alt not in altmap:
                altmap[alt] = {}
            if p.portpin in altmap[alt]:
                error("Pin %s ALT(%u) redefined" % (p.portpin, alt))
            altmap[alt][p.portpin] = p
            # we need to add alt pins into bytype[] so they are enabled in chibios config
            if type not in alttype:
                alttype[type] = []
            alttype[type].append(p)
            altlabel[label] = p
            return

        if a[0] in config:
            error("Pin %s redefined" % a[0])

    if p is None and line.find('ALT(') != -1:
        error("ALT() invalid for %s" % a[0])

    if a[0] == 'DEFAULTGPIO':
        default_gpio = a[1:]
        return

    if a[0] == 'NODMA':
        dma_exclude_pattern.extend(a[1:])
        return
    
    config[a[0]] = a[1:]
    if p is not None:
        # add to set of pins for primary config
        #print("port,pin,p",port,'--',pin,'--',p)
        portmap[port][pin] = p
        allpins.append(p)
        if type not in bytype:
            bytype[type] = []
        bytype[type].append(p)
        bylabel[label] = p
    elif a[0] == 'MCU':
        mcu_type = a[2]
        mcu_series = a[1]
        print("MCU series:",mcu_series," type:",mcu_type)
        setup_mcu_type_defaults()
    elif a[0] == 'SPIDEV':
        spidev.append(a[1:])
    elif a[0] == 'QSPIDEV':
        qspidev.append(a[1:])
    elif a[0] == 'IMU':
        imu_list.append(a[1:])
    elif a[0] == 'COMPASS':
        compass_list.append(a[1:])
    elif a[0] == 'BARO':
        baro_list.append(a[1:])
    elif a[0] == 'AIRSPEED':
        airspeed_list.append(a[1:])
    elif a[0] == 'ROMFS':
        romfs_add(a[1], a[2])
    elif a[0] == 'ROMFS_WILDCARD':
        romfs_wildcard(a[1])
    elif a[0] == 'undef':
        for u in a[1:]:
            print("Removing %s" % u)
            config.pop(u, '')
            bytype.pop(u, '')
            bylabel.pop(u, '')
            alttype.pop(u, '')
            altlabel.pop(u, '')
            # also remove all occurences of defines in previous lines if any
            for line in alllines[:]:
                if line.startswith('define') and u == line.split()[1]:
                    alllines.remove(line)
            newpins = []
            for pin in allpins:
                if pin.type == u or pin.label == u or pin.portpin == u:
                    if pin.label is not None:
                        bylabel.pop(pin.label, '')
                    portmap[pin.port][pin.pin] = generic_pin(pin.port, pin.pin, None, 'INPUT', [])
                    continue
                newpins.append(pin)
            allpins = newpins
            if u == 'IMU':
                imu_list = []
            if u == 'COMPASS':
                compass_list = []
            if u == 'BARO':
                baro_list = []
            if u == 'AIRSPEED':
                airspeed_list = []
    elif a[0] == 'env':
        print("Adding environment %s" % ' '.join(a[1:]))
        if len(a[1:]) < 2:
            error("Bad env line for %s" % a[0])
        env_vars[a[1]] = ' '.join(a[2:])


def process_file(filename):
    '''process a hwdef.dat file'''
    try:
        f = open(filename, "r")
    except Exception:
        error("Unable to open file %s" % filename)
    for line in f.readlines():
        line = line.split('#')[0] # ensure we discard the comments
        line = line.strip()
        if len(line) == 0 or line[0] == '#':
            continue
        a = shlex.split(line)
        if a[0] == "include" and len(a) > 1:
            include_file = a[1]
            if include_file[0] != '/':
                dir = os.path.dirname(filename)
                include_file = os.path.normpath(
                    os.path.join(dir, include_file))
            print("Including %s" % include_file)
            process_file(include_file)
        else:
            process_line(line)

def is_periph_fw():
        if not processed_hwdefs:
            raise ValueError("Need to process_hwdefs() first")
        return int(env_vars.get('AP_PERIPH', 0)) != 0

def add_apperiph_defaults(f):
    '''add default defines for peripherals'''
    if not is_periph_fw():
        # not AP_Periph
        return

    if not args.bootloader:
        # use the app descriptor needed by MissionPlanner for CAN upload
        env_vars['APP_DESCRIPTOR'] = 'MissionPlanner'

    print("Setting up as AP_Periph")
    #periph isn't much good without CAN
    enable_can(f)
    # and the rest..
    f.write('''
#ifndef HAL_SCHEDULER_ENABLED
#define HAL_SCHEDULER_ENABLED 0
#endif
#ifndef HAL_LOGGING_ENABLED
#define HAL_LOGGING_ENABLED 0
#endif
#ifndef HAL_GCS_ENABLED
#define HAL_GCS_ENABLED 0
#endif

// enables ADC subsystem, to do the opposive, define HAL_DISABLE_ADC_DRIVER 1 instead
#define TRUE 1
#define HAL_USE_ADC TRUE

#define HAL_ESP32_SDMMC 1
#define HAVE_FILESYSTEM_SUPPORT 1
#define HAL_ESP32_SDCARD 1
#define LOGGER_MAVLINK_SUPPORT 1
#define HAL_OS_POSIX_IO 1

#define HAL_NO_ROMFS_SUPPORT TRUE


// this becomes the default value for the ardupilot param LOG_BACKEND_TYPE, which most ppl want to be 1, for log-to-flash
// setting to 2 means log-over-mavlink to a companion computer etc.
#define HAL_LOGGING_BACKENDS_DEFAULT 1

// default to no protocols, AP_Periph enables with params
#define DEFAULT_SERIAL1_PROTOCOL -1
#define DEFAULT_SERIAL2_PROTOCOL -1
#define DEFAULT_SERIAL3_PROTOCOL -1
#define DEFAULT_SERIAL4_PROTOCOL -1

#ifndef HAL_LOGGING_MAVLINK_ENABLED
#define HAL_LOGGING_MAVLINK_ENABLED 0
#endif
#ifndef HAL_MISSION_ENABLED
#define HAL_MISSION_ENABLED 0
#endif
#ifndef HAL_RALLY_ENABLED
#define HAL_RALLY_ENABLED 0
#endif
#ifndef HAL_CAN_DEFAULT_NODE_ID
#define HAL_CAN_DEFAULT_NODE_ID 0
#endif
#define PERIPH_FW TRUE
#define HAL_BUILD_AP_PERIPH
#ifndef HAL_WATCHDOG_ENABLED_DEFAULT
#define HAL_WATCHDOG_ENABLED_DEFAULT true
#endif

#ifndef AP_FETTEC_ONEWIRE_ENABLED
#define AP_FETTEC_ONEWIRE_ENABLED 0
#endif
#ifndef HAL_BARO_WIND_COMP_ENABLED
#define HAL_BARO_WIND_COMP_ENABLED 0

#ifndef HAL_UART_STATS_ENABLED
#define HAL_UART_STATS_ENABLED (HAL_GCS_ENABLED || HAL_LOGGING_ENABLED)
#endif

#endif
''')


# process input file
for fname in args.hwdef:
    process_file(fname)
processed_hwdefs = True

outdir = args.outdir
if outdir is None:
    outdir = '/tmp'

mcu_type = get_config('MCU', 1)

if "MCU" not in config:
    error("Missing MCU type in config - old style config")
    mcu_type = 'esp32'

print("Setup for MCU %s" % mcu_type)

# build a list for peripherals for DMA resolver
periph_list = build_peripheral_list()

# write out hw.dat for ROMFS
write_all_lines(os.path.join(outdir, "hw.dat"))

# write out hwdef.h
style = write_hwdef_header(os.path.join(outdir, "hwdef.h"))

if style == 'new':

    # write out ldscript.ld
    write_ldscript(os.path.join(outdir, "ldscript.ld"))

    romfs_add_dir(['scripts'])

    write_ROMFS(outdir)

    # copy the shared linker script into the build directory; it must
    # exist in the same directory as the ldscript.ld file we generate.
    copy_common_linkerscript(outdir)

write_env_py(os.path.join(outdir, "env.py"))
