# Options hwdef ESP32 ArduPilot

Ce document decrit les options que l'on peut utiliser dans un `hwdef.dat` pour une cible ESP32 dans ArduPilot.

Point important:

- Cette liste est **exhaustive pour les directives que le parser reconnait explicitement**.
- Pour les lignes `define ...`, le parser accepte presque n'importe quelle macro C/C++, donc il n'existe pas de liste "fermee" cote parser.
- La partie `define` ci-dessous recense donc les **macros reelles observees** dans les `hwdef.dat` ESP32 du depot, classees par famille.

## 1. Fichiers de reference

Le comportement vient principalement de:

- `libraries/AP_HAL_ESP32/hwdef/scripts/esp32_hwdef.py`
- `libraries/AP_HAL/hwdef/scripts/hwdef.py`

Exemples utiles dans le depot:

- `libraries/AP_HAL_ESP32/hwdef/esp32empty/hwdef.dat`
- `libraries/AP_HAL_ESP32/hwdef/esp32s3empty/hwdef.dat`
- `libraries/AP_HAL_ESP32/hwdef/esp32s3devkit/hwdef.dat`
- `libraries/AP_HAL_ESP32/hwdef/esp32s3mowgli/hwdef.dat`
- `libraries/AP_HAL_ESP32/hwdef/esp32nick/hwdef.dat`

## 2. Directives parser "generiques"

Ces directives sont reconnues par le parser commun `hwdef.py`.

### `include`

Inclut un autre fichier hwdef.

```text
include ../esp32empty/hwdef.dat
```

Usage:

- pratique pour heriter d'une base commune
- le chemin peut etre relatif au fichier courant

### `define`

Ajoute une macro qui sera emise telle quelle dans `hwdef.h`.

```text
define HAL_ESP32_BOARD_NAME "esp32s2mowgli"
define HAL_ESP32_WIFI 0
define DEFAULT_SERIAL4_PROTOCOL SerialProtocol_MAVLink2
```

Usage:

- c'est le mecanisme principal pour activer/desactiver des fonctions
- le parser ne limite pas la liste des macros
- la validite depend ensuite du code C/C++ qui consomme ces macros

### `undef`

Supprime une ou plusieurs macros deja definies. Peut aussi vider les listes capteurs.

```text
undef HAL_ESP32_WIFI
undef IMU
undef COMPASS
undef BARO
```

Effets speciaux:

- `undef IMU` vide toute la liste des IMU declarees
- `undef COMPASS` vide toute la liste des compas declares
- `undef BARO` vide toute la liste des baros declares

### `env`

Ajoute une variable d'environnement pour le build waf/IDF.

```text
env AP_PERIPH 1
env SOME_FLAG yes
```

Usage:

- plutot rare
- utile pour influencer le build sans coder une nouvelle macro

## 3. Directives ESP32 natives

Ce sont les directives explicitement gerees par `esp32_hwdef.py`.

## 3.1 `ESP32_I2CBUS`

Syntaxe:

```text
ESP32_I2CBUS <port> <sda> <scl> <speed> <internal> <soft>
```

Exemple:

```text
ESP32_I2CBUS I2C_NUM_0 GPIO_NUM_13 GPIO_NUM_12 400*KHZ true false
```

Signification:

- `port` : ex. `I2C_NUM_0`
- `sda` : pin SDA
- `scl` : pin SCL
- `speed` : ex. `100*KHZ`, `400*KHZ`
- `internal` : `true` ou `false`
- `soft` : `true` ou `false`

## 3.2 `ESP32_SPIBUS`

Syntaxe:

```text
ESP32_SPIBUS <host> <dma_ch> <mosi> <miso> <sclk>
```

Exemple:

```text
ESP32_SPIBUS SPI3_HOST SPI_DMA_CH_AUTO GPIO_NUM_36 GPIO_NUM_37 GPIO_NUM_35
```

Signification:

- `host` : ex. `SPI2_HOST`, `SPI3_HOST`
- `dma_ch` : ex. `SPI_DMA_CH_AUTO`
- `mosi`, `miso`, `sclk` : pins du bus

## 3.3 `ESP32_SPIDEV`

Syntaxe:

```text
ESP32_SPIDEV <name> <bus> <device> <cs> <mode> <lowspeed> <highspeed>
```

Exemple:

```text
ESP32_SPIDEV mpu9250 0 0 GPIO_NUM_10 0 2*MHZ 8*MHZ
```

Contraintes du parser:

- `bus` doit etre numerique
- `device` doit etre numerique
- `cs` doit etre un identifiant simple
- `mode` doit etre numerique
- `lowspeed` et `highspeed` doivent finir par `*MHZ` ou `*KHZ`

Usage:

- declare un peripherique SPI nomme
- ce nom est ensuite reutilise dans `IMU ... SPI:<name>` ou `BARO ... SPI:<name>`

## 3.4 `ESP32_SERIAL`

Syntaxe:

```text
ESP32_SERIAL <port> <rxpin> <txpin>
```

Exemple:

```text
ESP32_SERIAL UART_NUM_0 GPIO_NUM_44 GPIO_NUM_43
ESP32_SERIAL UART_NUM_1 GPIO_NUM_16 GPIO_NUM_17
```

Usage:

- decrit les UART materiels exposes par le HAL
- ne configure pas a lui seul le protocole MAVLink/GPS/etc.
- le protocole vient ensuite des macros `DEFAULT_SERIALx_PROTOCOL`

## 3.5 `ESP32_RCOUT`

Syntaxe:

```text
ESP32_RCOUT <gpio>
```

Exemple:

```text
ESP32_RCOUT GPIO_NUM_9
ESP32_RCOUT GPIO_NUM_11
ESP32_RCOUT GPIO_NUM_5
```

Usage:

- une ligne par sortie PWM/RC

## 3.6 `ESP32_ADC_PIN`

Syntaxe:

```text
ESP32_ADC_PIN <channel> <gain> <ardupilotpin>
```

Exemple:

```text
ESP32_ADC_PIN ADC_CHANNEL_0 11 1
ESP32_ADC_PIN ADC_CHANNEL_1 11 2
```

Usage:

- relie un canal ADC ESP32 a un index ADC ArduPilot
- `gain` sert a l'echelle appliquee

## 3.7 `ESP32_SDSPI`

Syntaxe:

```text
ESP32_SDSPI <host> <dma_ch> <mosi> <miso> <sclk> <cs>
```

Exemple:

```text
ESP32_SDSPI SPI2_HOST SPI_DMA_CH_AUTO GPIO_NUM_11 GPIO_NUM_13 GPIO_NUM_12 GPIO_NUM_10
```

Usage:

- decrit une carte SD en mode SPI

## 4. Directives capteurs generiques

Elles viennent du parser commun, mais sont tres importantes dans un hwdef ESP32.

## 4.1 `IMU`

Forme generale:

```text
IMU <Driver> <bus/device...> <rotation> [INSTANCE:n] [AUX:n] [BOARD_MATCH(...)]
```

Exemples:

```text
IMU Invensense SPI:mpu9250 ROTATION_NONE
IMU BMI270 I2C:0:0x68 ROTATION_YAW_90
IMU ICM20948 I2C:ALL_EXTERNAL:0x68 ROTATION_NONE
```

Formats de bus supportes:

- `SPI:<name>`
- `I2C:<bus>:<addr>`
- `I2C:ALL:<addr>`
- `I2C:ALL_INTERNAL:<addr>`
- `I2C:ALL_EXTERNAL:<addr>`

Options de fin supportees par le parser:

- `INSTANCE:n`
- `AUX:n`
- `BOARD_MATCH(...)`

Notes:

- `INSTANCE:n` force l'index d'instance IMU
- `AUX:n` cree un backend auxiliaire
- `BOARD_MATCH(...)` conditionne l'activation a une detection carte

## 4.2 `COMPASS`

Forme generale:

```text
COMPASS <Driver[:probe_fn]> <device/probe args...> [force_external] <rotation>
```

Exemples:

```text
COMPASS AK8963:probe_mpu9250 0 ROTATION_NONE
COMPASS IST8310 I2C:0:0x0E true ROTATION_NONE
COMPASS AK09916 I2C:ALL_EXTERNAL:0x0C ROTATION_NONE
```

Particularites:

- certains drivers utilisent une fonction `probe_...` speciale
- certains backends prennent `force_external`
- la rotation est la derniere valeur

## 4.3 `BARO`

Forme generale:

```text
BARO <Driver[:probe_fn]> <device...>
```

Exemples:

```text
BARO BMP280 I2C:0:0x77
BARO BMP280 SPI:bmp280
BARO LPS2XH:probe_InvensenseIMU SPI:imu 0x5c
```

Formats observes:

- baro I2C
- baro SPI
- certains cas speciaux avec adresse supplementaire

## 5. Familles de macros `define` courantes

Le parser ne limite pas ces macros. La liste ci-dessous recense celles observees dans les `hwdef.dat` ESP32 du depot.

## 5.1 Identite de carte

```text
define HAL_ESP32_BOARD_NAME "esp32s2mowgli"
```

Macros observees:

- `HAL_ESP32_BOARD_NAME`

## 5.2 USB / serial natif S3

Macros observees:

- `HAL_ESP32_S3_USB_SERIAL4_ENABLED`
- `HAL_ESP32_S3_USB_SERIAL_JTAG_BACKEND`

Usage:

- surtout utile sur ESP32-S3
- permet d'exposer `SERIAL4` sur l'USB natif selon le backend choisi

## 5.3 Configuration par defaut des ports serie

Macros observees:

- `DEFAULT_SERIAL2_PROTOCOL`
- `DEFAULT_SERIAL2_BAUD`
- `DEFAULT_SERIAL3_PROTOCOL`
- `DEFAULT_SERIAL3_BAUD`
- `DEFAULT_SERIAL4_PROTOCOL`
- `DEFAULT_SERIAL4_BAUD`
- `DEFAULT_SERIAL5_PROTOCOL`
- `DEFAULT_SERIAL5_BAUD`
- `DEFAULT_SERIAL6_PROTOCOL`
- `DEFAULT_SERIAL6_BAUD`
- `DEFAULT_SERIAL7_PROTOCOL`
- `DEFAULT_SERIAL7_BAUD`
- `DEFAULT_SERIAL8_PROTOCOL`
- `DEFAULT_SERIAL8_BAUD`
- `DEFAULT_SERIAL9_PROTOCOL`
- `DEFAULT_SERIAL9_BAUD`
- `HAL_SERIAL9_BAUD`

Usage:

- protocole par defaut d'un lien serie (`MAVLink2`, `GPS`, `None`, etc.)
- debit par defaut

## 5.4 WiFi

Macros observees:

- `HAL_ESP32_WIFI`
- `WIFI_SSID`
- `WIFI_PWD`

Usage:

- active/desactive le backend WiFi
- configure SSID et mot de passe

## 5.5 CAN / TWAI

Macros observees:

- `HAL_ESP32_CAN_TX_PIN`
- `HAL_ESP32_CAN_RX_PIN`
- `HAL_ESP32_CAN_SELFTEST`

## 5.6 RC input / RMT

Macros observees:

- `HAL_ESP32_RMT_RX_PIN_NUMBER`
- `HAL_ESP32_RCIN`

Usage:

- reception RC via RMT / pin d'entree associee

## 5.7 Capteurs / INS / compas / baro

Macros observees:

- `HAL_INS_DEFAULT`
- `INS_MAX_INSTANCES`
- `HAL_INS_MPU9250_NAME`
- `HAL_INS_MPU60x0_NAME`
- `HAL_INS_ICM20XXX_I2C_BUS`
- `HAL_INS_ICM20XXX_I2C_ADDR`
- `AP_COMPASS_AK8963_ENABLED`
- `AP_COMPASS_ICM20948_ENABLED`
- `AP_COMPASS_ENABLE_DEFAULT`
- `AP_COMPASS_PROBING_ENABLED`
- `HAL_COMPASS_AK09916_I2C_BUS`
- `HAL_COMPASS_AK09916_I2C_ADDR`
- `HAL_COMPASS_ICM20948_I2C_ADDR`
- `HAL_COMPASS_MAX_SENSORS`
- `HAL_BARO_ALLOW_INIT_NO_BARO`
- `HAL_PROBE_EXTERNAL_I2C_BAROS`
- `ALLOW_ARM_NO_COMPASS`
- `AP_AIRSPEED_ENABLED`
- `AP_AIRSPEED_ANALOG_ENABLED`
- `AP_AIRSPEED_BACKEND_DEFAULT_ENABLED`

Usage:

- choix du backend INS principal
- nombre max d'instances
- activation forcee de certains compas
- autorisation de boot sans baro/compas

## 5.8 Logging / stockage / filesystem

Macros observees:

- `HAL_LOGGING_BACKENDS_DEFAULT`
- `HAL_LOGGING_FILESYSTEM_ENABLED`
- `HAL_LOGGING_DATAFLASH_ENABLED`
- `HAL_LOGGING_MAVLINK_ENABLED`
- `HAL_LOGGING_STACK_SIZE`
- `LOGGER_MAVLINK_SUPPORT`
- `AP_FILESYSTEM_ESP32_ENABLED`
- `HAVE_FILESYSTEM_SUPPORT`
- `HAL_OS_POSIX_IO`
- `HAL_USE_EMPTY_STORAGE`

Usage:

- selection des backends de logs
- activation du filesystem
- mode bring-up sans vrai stockage

## 5.9 Carte SD / stockage externe

Macros observees:

- `HAL_ESP32_SDCARD`
- `HAL_ESP32_SDMMC`

Et cote directives:

- `ESP32_SDSPI ...`

Usage:

- `SDMMC` pour le mode natif SD/MMC
- `SDSPI` pour une carte SD sur bus SPI

## 5.10 Batterie / ADC / monitoring

Macros observees:

- `HAL_BATT_VOLT_PIN`
- `HAL_BATT_VOLT_SCALE`
- `HAL_BATT_CURR_PIN`
- `HAL_BATT_CURR_SCALE`
- `AP_BATTERY_INA3221_ENABLED`
- `HAL_BATTMON_INA3221_AVG_MODE_SEL`
- `HAL_BATTMON_INA3221_BUS_CONV_TIME_SEL`
- `HAL_BATTMON_INA3221_SHUNT_CONV_TIME_SEL`
- `HAL_BATTMON_INA3221_SHUNT_OHMS`

Usage:

- tension/courant analogique
- backend INA3221

## 5.11 Notifications / LED / sorties annexes

Macros observees:

- `DEFAULT_NTF_LED_TYPES`
- `BUILD_DEFAULT_LED_TYPE`
- `HAL_ESP32_WENC_LEFT_A`
- `HAL_ESP32_WENC_LEFT_B`
- `HAL_ESP32_WENC_RIGHT_A`
- `HAL_ESP32_WENC_RIGHT_B`

Usage:

- LED par defaut
- encodeurs de roues sur certaines cartes Rover

## 5.12 Reduction de taille / bring-up

Macros observees:

- `AP_RCPROTOCOL_ENABLED`
- `AP_SCRIPTING_ENABLED`
- `AP_MOTORS_FRAME_DEFAULT_ENABLED`
- `AP_MOTORS_FRAME_QUAD_ENABLED`

Usage:

- couper des fonctions gourmandes
- utile pour les petites cartes ou les profils de debug

## 6. Macros obsoletes que le parser signale explicitement

Le parser connait quelques anciennes macros a eviter. Si tu les ajoutes, il peut lever une erreur.

Exemples:

- `HAL_COMPASS_DEFAULT`
- `HAL_NO_GCS`
- `HAL_NO_LOGGING`
- `HAL_NO_UARTDRIVER`
- `HAL_DISABLE_LOOP_DELAY`
- `HAL_EXTERNAL_AHRS_ENABLED`
- `HAL_PROBE_EXTERNAL_I2C_COMPASSES`
- `HAL_SKIP_AUTO_INTERNAL_I2C_PROBE`
- `HAL_COMPASS_DISABLE_IST8310_INTERNAL_PROBE`
- `BOARD_PWM_COUNT_DEFAULT`
- `HAL_PICCOLO_CAN_ENABLE`
- `HAL_PERIPH_ENABLE_NETWORKING`
- `HAL_PROBE_EXTERNAL_I2C_BAROS`

Conclusion pratique:

- si une vieille macro est reprise d'un ancien board, il faut souvent la remplacer par son equivalent moderne

## 7. Exemple de squelette minimal

```text
define HAL_ESP32_BOARD_NAME "esp32s2mowgli"

define HAL_ESP32_WIFI 0
define HAL_BARO_ALLOW_INIT_NO_BARO 1
define AP_COMPASS_ENABLE_DEFAULT 0
define ALLOW_ARM_NO_COMPASS

ESP32_SERIAL UART_NUM_0 GPIO_NUM_44 GPIO_NUM_43
ESP32_SERIAL UART_NUM_1 GPIO_NUM_16 GPIO_NUM_17

ESP32_I2CBUS I2C_NUM_0 GPIO_NUM_8 GPIO_NUM_9 400*KHZ true false

ESP32_RCOUT GPIO_NUM_5
ESP32_RCOUT GPIO_NUM_6

define DEFAULT_SERIAL3_PROTOCOL SerialProtocol_MAVLink2
define DEFAULT_SERIAL3_BAUD 921
```

Puis tu ajoutes au besoin:

- un bus SPI avec `ESP32_SPIBUS`
- des peripheriques SPI avec `ESP32_SPIDEV`
- `IMU`, `COMPASS`, `BARO`
- SD/MMC ou SDSPI
- WiFi, CAN, ADC, logging

## 8. Conseils pratiques

1. Commencer par un board minimal qui boote.
2. Ajouter les bus (`I2C`, `SPI`, `SERIAL`) avant les capteurs.
3. Ajouter ensuite `IMU`, puis `COMPASS`, puis `BARO`.
4. N'activer le filesystem, la SD, le WiFi et le logging qu'une fois la base stable.
5. Utiliser `esp32empty` et `esp32s3empty` comme reference pour un bring-up simple.
6. Utiliser `esp32s3devkit` comme reference pour une cible S3 plus complete.

## 9. Resume court

En pratique, un `hwdef.dat` ESP32 est compose de 3 couches:

1. des directives parser communes:
   `include`, `define`, `undef`, `env`, `IMU`, `COMPASS`, `BARO`
2. des directives ESP32 natives:
   `ESP32_I2CBUS`, `ESP32_SPIBUS`, `ESP32_SPIDEV`, `ESP32_SERIAL`, `ESP32_RCOUT`, `ESP32_ADC_PIN`, `ESP32_SDSPI`
3. des macros `define` qui activent ou desactivent les fonctions compile-time du HAL et d'ArduPilot

Si tu veux un document "100 % exhaustif" au sens strict:

- il est possible pour les directives parser
- il ne l'est pas pour `define`, car toute macro C/C++ consommee ailleurs dans le code peut etre injectee ici

Ce fichier fournit donc:

- l'exhaustif de la syntaxe `hwdef`
- et l'inventaire pratique des options ESP32 deja utilisees dans le depot
