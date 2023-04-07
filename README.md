# Projet Long n°2
## _EI2I4 HF_ | _Groupe 2_

[![Build Status](https://travis-ci.org/joemccann/dillinger.svg?branch=master)](https://travis-ci.org/joemccann/dillinger)

The goal of this project is to develop a complete device that can measure several physical datas and to make them available through a Human Machine Interface (HMI) on an internet application. This project will allow to recover and format the data measured by the sensors and then send them to a server (Cloud) thanks to the long range and low power technology (LPWAN).

## Features
- Autonomous: Rechargeable LiPo battery and indoor solar panels as power source
- Communication of information by LPWAN network (LoRAWAN): Data sending every 10 minutes (configurable)
- On/off button : Allows an instantaneous measurement
- LED that lights up for a few seconds when the system is started.
- Data display in graphical form on an internet application: Ubidots STEM during the development phase and an internal Polytech website for the final rendering of the project.

## Plugins
In order to realize this project, we had to find components that best respond to the imposed specifications by focusing on the main constraint which is the low consumption of the system:

| Components | Functions | Price |
| ------ | ------ | ------ |
| [E-Ink Screen](https://www.digikey.fr/en/products/detail/adafruit-industries-llc/4197/10060730) | | 21.10€ |
| [Indoor solar kit](https://www.mouser.fr/ProductDetail/PowerFilm/DEV-BASIC?qs=BJlw7L4Cy7%2Fw4dHFXPm5kg%3D%3D) | | 55.97€ |
| [CO2, T°, Humidity Sensor](https://www.digikey.fr/fr/products/detail/seeed-technology-co.,-ltd/101020952/14672116?utm_adgroup=&utm_source=google&utm_medium=cpc&utm_campaign=PMAX%20Shopping_Product_Development%20Board&utm_term=&productid=14672116&gclid=CjwKCAiA3KefBhByEiwAi2LDHGC3sSf3_wtaYPH630ib0XKAaLsoO_e8GBNamO-FCIEC66xhhMLAGhoCcFoQAvD_BwE) | | 48.36€ |
| [Noise Sensor](https://www.gotronic.fr/art-module-micro-mems-fermion-sen0487-34167.htm) | | 3.50€ |
| [Light Sensor](https://www.digikey.fr/fr/products/detail/adafruit-industries-llc/5378/16056942?utm_adgroup=&utm_source=google&utm_medium=cpc&utm_campaign=PMAX%20Shopping_Product_Development%20Board&utm_term=&productid=16056942&gclid=CjwKCAiAl9efBhAkEiwA4ToriqFnO5TcQVJoOHX-VCiYtwIO2bZSYyVDAaRXxzhDUZyRP_28fs59SxoCnOUQAvD_BwE) | | 4.64€ |
| [Battery](https://www.gotronic.fr/art-accu-lipo-3-7-vcc-1000-mah-pr523450-5813.htm) | | 9.90€ |

For a total cost of 143.47€, we created [insert project name]. Responding to the specifications imposed, this energy autonomous system allows to manage the constants of a room in order to respect the sanitary conditions imposed by the school, or by any other organization.

## Installation
In order to make this project work, you need to gather the same components as listed above. 

In case you don't have the same components, you will just have to modify the code at the location of the different sensors you want to use.

Otherwise you will need to install the following libraries in arduino or on their respective repositories:

```sh
SensirionI2CScd4x v0.4.0: CO2, T° & Humidity sensor
```
```sh
Sensirion Core v0.6.0: Sensirion sensors core.
```
```sh
Adafruit_VEML7700 v2.1.2 : VEML7700 sensors in the Adafruit shop
```
```sh
Wire
```
```sh
Adafruit_GFX v1.11.4:
```
```sh
Adafruit_EPD v4.5.1:
```
```sh
MKRWAN v1.1.0 : Provides APIs to communicate with LoRaWAN networks.
```
```sh
ArduinoLowPower v1.2.2
```

To format some data you will also need:
```sh
avr/dtostrf
```

```sh
stdlib
```
