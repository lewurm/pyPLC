
# For serial (including USB-to-serial) interfaces:
# https://pyserial.readthedocs.io/en/latest/pyserial.html
# Install pyserial library:
#   python -m pip install pyserial
# List ports:
#   python -m serial.tools.list_ports

import serial # the pyserial
from serial.tools.list_ports import comports
from time import sleep, time
from helpers import currentMillis
from configmodule import getConfigValue, getConfigValueBool
import sys # For exit_on_session_end hack
import subprocess
import requests
import os
import threading
import struct

import pymodbus.client as ModbusClient
from pymodbus.payload import BinaryPayloadDecoder
from pymodbus.constants import Endian

PinCp = "P8_18"
PinPowerRelay = "P8_16"



###### DIY DC Wallbox with Kostal Plenticore inverter #####

# BCM (WiringPi) numbering
# out
KICp = 26 # (25)
KIContactorPrecharge = 19 # (24)
KIContactorDCMinus = 13 # (23)
KIContactorDCPlus = 6 # (22)
KIPWMStepup = 18 # (1)

# in
KIContactorPrechargeAUX = None
KIContactorDCMinusAUX = 8 # (10)
KIContactorDCPlusAUX = None

KIInverterSerial = "/dev/ttyACM0"
KIInverterBaud = 56000

KIState0Idle = 0
KIState0IdlePrime = 1
KIState1Negative = 2
KIState1NegativeConfirm = 3
KIState2Precharge = 4
KIState2PrechargeConfirm = 5
KIState3Positive = 6
KIState3PositiveConfirm = 7
KIState4Operate = 8

RXframeCyclicData      = bytearray([0x09, 0x62, 0xFF, 0x02, 0xFF, 0x29, 0x4A, 0x04, 0x27, 0x00])
RXframeBatteryInfo     = bytearray([0x09, 0x62, 0xFF, 0x02, 0xFF, 0x29, 0x4A, 0x08, 0x23, 0x00])
RXframeErrorState      = bytearray([0x09, 0x62, 0xFF, 0x02, 0xFF, 0x29, 0x53, 0x03, 0x1F, 0x00])
RXframeRestartFrame    = bytearray([0x09, 0x62, 0xFF, 0x02, 0xFF, 0x29, 0x5E, 0xFF, 0x17, 0x00])
RXframeCloseContactors = bytearray([0x07, 0x63, 0xFF, 0x02, 0xFF, 0x29, 0x5E, 0x02, 0x16, 0x00])

# maybe a hint to restart the state machine?
RXframeUnknownFrame1   = bytearray([0x09, 0x63, 0xff, 0x02, 0xff, 0x29, 0x5e, 0x04, 0x12, 0x00])
# ???
RXframeUnknownFrame2   = bytearray([0x09, 0x62, 0xff, 0x02, 0xff, 0x09, 0x62, 0xff, 0x12, 0xff])

# ??? seen in a long session after 5745596ms.  maybe fucked up byte, and should be RXframeErrorState?
RXframeUnknownFrame2   = bytearray([0x09, 0x62, 0xff, 0x02, 0x29, 0x53, 0x03, 0x1f, 0x00, 0x00])




shared_data = {
    "battery_voltage": 0.0,
    "battery_current": 0.0,
    "inverter_status": 0,
    "haas_dcwb_allow_charging": False
}

def _read_inverter_data(mbClient, tracer):
    while True:
        shared_data['battery_voltage'] = _inverter_battery_voltage(mbClient, tracer)
        shared_data['battery_current'] = _inverter_battery_current(mbClient, tracer)
        shared_data['inverter_status'] = _inverter_status(mbClient, tracer)
        _haas_dcwb_allow_charging(tracer)
        sleep(0.01)

def _inverter_battery_current(mbClient, tracer):
    try:
        start_ts = currentMillis()
        ret = _inverter_float32(mbClient, 200)
        #tracer(f"ModBus battery current read took {currentMillis() - start_ts}ms")
        return float(ret)
    except:
        return 0.01

def _inverter_battery_voltage(mbClient, tracer):
    try:
        start_ts = currentMillis()
        ret = _inverter_float32(mbClient, 216)
        #tracer(f"ModBus battery voltage read took {currentMillis() - start_ts}ms")
        return float(ret)
    except:
        return 5.5

def _inverter_status(mbClient, tracer):
    try:
        start_ts = currentMillis()
        ret = _inverter_uint32(mbClient, 56)
        #tracer(f"ModBus inverter status read took {currentMillis() - start_ts}ms")
        return int(ret)
    except:
        return 0

def _haas_dcwb_allow_charging(tracer):
    try:
        haasURL = getConfigValue("homeassistant_url")
        haasToken = getConfigValue("homeassistant_token")

        headers = {"Authorization": f"Bearer {haasToken}",
                   "Content-Type": "application/json"}

        url = f"{haasURL}/api/states/input_select.dcwb_allow_charging"
        response = requests.get(url, headers=headers, timeout=1)
        if response.status_code == 200:
            if response.json()['state'] == 'no':
                shared_data['haas_dcwb_allow_charging'] = False
            elif response.json()['state'] == 'yes':
                shared_data['haas_dcwb_allow_charging'] = currentMillis()
            else:
                # do not update
                pass
        else:
            # do not update
            pass
    except:
        tracer("Warning: haasDcwbAllowCharging failed!!!!")
        # do not update
        pass

def _inverter_float32(mbClient, reg):
    rr = mbClient.read_holding_registers(reg, count=2, slave=71)
    decoder = BinaryPayloadDecoder.fromRegisters(rr.registers, byteorder=Endian.Big, wordorder=Endian.Little)
    return decoder.decode_32bit_float()

def _inverter_uint32(mbClient, reg):
    rr = mbClient.read_holding_registers(reg, count=2, slave=71)
    decoder = BinaryPayloadDecoder.fromRegisters(rr.registers, byteorder=Endian.Big, wordorder=Endian.Little)
    return decoder.decode_32bit_uint()


if (getConfigValue("digital_output_device")=="beaglebone"):
    # In case we run on beaglebone, we want to use GPIO ports.
    import Adafruit_BBIO.GPIO as GPIO

if (getConfigValue("charge_parameter_backend")=="chademo"):
    # In case we use the CHAdeMO backend, we want to use CAN
    import can

if (getConfigValue("digital_output_device")=="kostalinverter"):
    import RPi.GPIO as GPIO

class hardwareInterface():
    def needsSerial(self):
        # Find out, whether we need a serial port. This depends on several configuration items.
        if (getConfigValueBool("display_via_serial")):
            return True # a display is expected to be connected to serial port.
        if (getConfigValue("digital_output_device")=="dieter"):
            return True # a "dieter" output device is expected to be connected on serial port.
        if (getConfigValue("analog_input_device")=="dieter"):
            return True # a "dieter" input device is expected to be connected on serial port.
        if (getConfigValue("digital_output_device")=="celeron55device"):
            return True
        if (getConfigValue("analog_input_device")=="celeron55device"):
            return True
        return False # non of the functionalities need a serial port.
        
    def findSerialPort(self):
        if not self.needsSerial():
            return

        baud = int(getConfigValue("serial_baud"))
        if (getConfigValue("serial_port")!="auto"):
            port = getConfigValue("serial_port")
            try:
                self.addToTrace("Using serial port " + port)
                self.ser = serial.Serial(port, baud, timeout=0)
                self.isSerialInterfaceOk = True
            except:
                if (self.needsSerial()):
                    self.addToTrace("ERROR: Could not open serial port.")
                else:
                    self.addToTrace("Could not open serial port, but also do not need it. Ok.")
                self.ser = None
                self.isSerialInterfaceOk = False
            return

        ports = []
        self.addToTrace('Auto detection of serial ports. Available serial ports:')
        for n, (port, desc, hwid) in enumerate(sorted(comports()), 1):
            if (port=="/dev/ttyAMA0"):
                self.addToTrace("ignoring /dev/ttyAMA0, because this is not an USB serial port")
            else:
                self.addToTrace('{:2}: {:20} {!r}'.format(n, port, desc))
                ports.append(port)
        if (len(ports)<1):
            if (self.needsSerial()):
                self.addToTrace("ERROR: No serial ports found. No hardware interaction possible.")
                self.ser = None
                self.isSerialInterfaceOk = False
            else:
                self.addToTrace("We found no serial port, but also do not need it. No problem.")
                self.ser = None
                self.isSerialInterfaceOk = False
        else:
            # only open serial if actually needed
            if self.needsSerial():
                self.addToTrace("ok, we take the first port, " + ports[0])
                try:
                    self.ser = serial.Serial(ports[0], baud, timeout=0)
                    self.isSerialInterfaceOk = True
                except:
                    self.addToTrace("ERROR: Could not open serial port.")
                    self.ser = None
                    self.isSerialInterfaceOk = False

    def addToTrace(self, s):
        self.callbackAddToTrace("[HARDWAREINTERFACE] " + s)            

    def setStateB(self):
        self.addToTrace("Setting CP line into state B.")
        if (getConfigValue("digital_output_device")=="beaglebone"):
            GPIO.output(PinCp, GPIO.LOW)
        if (getConfigValue("digital_output_device")=="celeron55device"):
            self.ser.write(bytes("cp=0\n", "utf-8"))
        self.outvalue &= ~1
        
    def setStateC(self):
        self.addToTrace("Setting CP line into state C.")
        if (getConfigValue("digital_output_device")=="beaglebone"):
            GPIO.output(PinCp, GPIO.HIGH)
        if (getConfigValue("digital_output_device")=="celeron55device"):
            self.ser.write(bytes("cp=1\n", "utf-8"))
        self.outvalue |= 1
        
    def setPowerRelayOn(self):
        self.addToTrace("Switching PowerRelay ON.")
        if (getConfigValue("digital_output_device")=="beaglebone"):
            GPIO.output(PinPowerRelay, GPIO.HIGH)
        if (getConfigValue("digital_output_device")=="celeron55device"):
            self.ser.write(bytes("contactor=1\n", "utf-8"))
        self.outvalue |= 2

    def setPowerRelayOff(self):
        self.addToTrace("Switching PowerRelay OFF.")
        if (getConfigValue("digital_output_device")=="beaglebone"):
            GPIO.output(PinPowerRelay, GPIO.LOW)
        if (getConfigValue("digital_output_device")=="celeron55device"):
            self.ser.write(bytes("contactor=0\n", "utf-8"))
        self.outvalue &= ~2

    def setRelay2On(self):
        self.addToTrace("Switching Relay2 ON.")
        self.outvalue |= 4

    def setRelay2Off(self):
        self.addToTrace("Switching Relay2 OFF.")
        self.outvalue &= ~4
        
    def getPowerRelayConfirmation(self):
        if (getConfigValue("digital_output_device")=="celeron55device"):
            return self.contactor_confirmed
        return 1 # todo: self.contactor_confirmed
        
    def triggerConnectorLocking(self):
        self.addToTrace("Locking the connector")
        if (getConfigValue("digital_output_device")=="celeron55device"):
            self.ser.write(bytes("lock\n", "utf-8"))
        # todo control the lock motor into lock direction until the end (time based or current based stopping?)

    def triggerConnectorUnlocking(self):
        self.addToTrace("Unocking the connector")
        if (getConfigValue("digital_output_device")=="celeron55device"):
            self.ser.write(bytes("unlock\n", "utf-8"))
        # todo control the lock motor into unlock direction until the end (time based or current based stopping?)

    def isConnectorLocked(self):
        # TODO: Read the lock= value from the hardware so that this works
        #if (getConfigValue("digital_output_device")=="celeron55device"):
        #    return self.lock_confirmed
        return 1 # todo: use the real connector lock feedback
        
    def setChargerParameters(self, maxVoltage, maxCurrent):
        self.maxChargerVoltage = int(maxVoltage)
        self.maxChargerCurrent = int(maxCurrent)
        
    def setChargerVoltageAndCurrent(self, voltageNow, currentNow):
        self.chargerVoltage = int(voltageNow)
        self.chargerCurrent = int(currentNow)
        
    def _hia4v1_precharge(self, targetVoltage):
        # Modded HIA4V1
        # https://github.com/dalathegreat/Battery-Emulator/wiki/High-Voltage-source

        self.lastTargetVoltage = targetVoltage
        if currentMillis() - self.lastPrechargeVoltage > 500:
            # bei 5.1V Eingangsspannung (USB Netzteil)
            freqSettings = [
                      25, #kHz ~298V
                      55, #kHz ~355V
                      65, #kHz ~369V
                      80, #kHz ~380V
                     110, #kHz ~388V
                     150, #kHz ~401V
                     180, #kHz ~407V
                     210, #kHz ~411V
                     300, #kHz ~415V
                     400, #kHz ~420V
                     600, #kHz ~425V
                    2000, #kHz ~431V
                    ]
            freq = freqSettings[self.stepUpPWMIndex] * 1000
            self.stepUpPWM.ChangeFrequency(freq)
            self.stepUpPWM.start(0.5)
            self.addToTrace("<< hw pwr supply, set freq to "+str(freq/1000)+"kHz for targetVoltage="+str(targetVoltage)+"V")
            self.stepUpPWMIndex = (self.stepUpPWMIndex + 1) % len(freqSettings)
            self.lastPrechargeVoltage = currentMillis()

    def stopPrecharge(self):
        self.stepUpPWM.stop()

    def setPowerSupplyVoltageAndCurrent(self, targetVoltage, targetCurrent):
        if (getConfigValue("digital_output_device")=="kostalinverter"):
            self._hia4v1_precharge(targetVoltage)
        else:
            # if we are the charger, and have a real power supply which we want to control, we do it here
            self.homeplughandler.sendSpecialMessageToControlThePowerSupply(targetVoltage, targetCurrent)

    def getInletVoltage(self):
        # uncomment this line, to take the simulated inlet voltage instead of the really measured
        # self.inletVoltage = self.simulatedInletVoltage
        return self.inletVoltage
        
    def getAccuVoltage(self):
        if (getConfigValue("digital_output_device")=="celeron55device"):
            return self.accuVoltage
        elif getConfigValue("charge_parameter_backend")=="chademo":
           return self.accuVoltage
        #todo: get real measured voltage from the accu
        self.accuVoltage = 230
        return self.accuVoltage

    def getAccuMaxCurrent(self):
        if (getConfigValue("digital_output_device")=="celeron55device"):
            # The overall current limit is currently hardcoded in
            # OpenV2Gx/src/test/main_commandlineinterface.c
            EVMaximumCurrentLimit = 250
            if self.accuMaxCurrent >= EVMaximumCurrentLimit:
                return EVMaximumCurrentLimit
            return self.accuMaxCurrent
        elif getConfigValue("charge_parameter_backend")=="chademo":
            return self.accuMaxCurrent #set by CAN        
        #todo: get max charging current from the BMS
        self.accuMaxCurrent = 10
        return self.accuMaxCurrent

    def getAccuMaxVoltage(self):
        if getConfigValue("charge_parameter_backend")=="chademo":
            return self.accuMaxVoltage #set by CAN
        elif getConfigValue("charge_target_voltage"):
            self.accuMaxVoltage = getConfigValue("charge_target_voltage")            
        else:
            #todo: get max charging voltage from the BMS
            self.accuMaxVoltage = 230
        return self.accuMaxVoltage

    def getIsAccuFull(self):
        if (getConfigValue("digital_output_device")=="celeron55device"):
            self.IsAccuFull = (self.soc_percent >= 98)
        else:
            #todo: get "full" indication from the BMS
            self.IsAccuFull = (self.simulatedSoc >= 98)
        return self.IsAccuFull

    def getSoc(self):
        if self.callbackShowStatus:
            self.callbackShowStatus(format(self.soc_percent,".1f"), "soc")
        if (getConfigValue("digital_output_device")=="celeron55device"):
            return self.soc_percent
        #todo: get SOC from the BMS
        self.callbackShowStatus(format(self.simulatedSoc,".1f"), "soc")
        return self.simulatedSoc

    def isUserAuthenticated(self):
        # If the user needs to authorize, fill this function in a way that it returns False as long as
        # we shall wait for the users authorization, and returns True if the authentication was successfull.
        # Discussing here: https://github.com/uhi22/pyPLC/issues/28#issuecomment-2230656379
        # For testing purposes, we just use a counter to decide that we return
        # once "ongoing" and then "finished".
        if (self.demoAuthenticationCounter<1):
            self.demoAuthenticationCounter += 1
            return False
        else:
            return True

    def initPorts(self):
        if (getConfigValue("charge_parameter_backend") == "chademo"):
            filters = [
               {"can_id": 0x100, "can_mask": 0x7FF, "extended": False},
               {"can_id": 0x101, "can_mask": 0x7FF, "extended": False},
               {"can_id": 0x102, "can_mask": 0x7FF, "extended": False}]
            self.canbus = can.interface.Bus(bustype='socketcan', channel="can0", can_filters = filters)
    
        if (getConfigValue("digital_output_device") == "beaglebone"):
            # Port configuration according to https://github.com/jsphuebner/pyPLC/commit/475f7fe9f3a67da3d4bd9e6e16dfb668d0ddb1d6
            GPIO.setup(PinPowerRelay, GPIO.OUT) #output for port relays
            GPIO.setup(PinCp, GPIO.OUT) #output for CP

        if (getConfigValue("digital_output_device")=="kostalinverter"):
            GPIO.setmode(GPIO.BCM)

            for outpin in [KIPWMStepup]:
                if outpin is not None:
                    GPIO.setup(outpin, GPIO.OUT, initial=GPIO.LOW)
                    self.stepUpPWM = GPIO.PWM(18, 10)

            # HAT relais are active-low
            for outpin in [KICp, KIContactorPrecharge, KIContactorDCMinus, KIContactorDCPlus]:
                if outpin is not None:
                    GPIO.setup(outpin, GPIO.OUT, initial=GPIO.HIGH)

            for inpin in [KIContactorPrechargeAUX, KIContactorDCMinusAUX, KIContactorDCPlusAUX]:
                if inpin is not None:
                    GPIO.setup(inpin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

            self.mbClient = ModbusClient.ModbusTcpClient(getConfigValue("kostalinverter_url"), port = "1502", timeout=2.00)
            self.mbClient.connect()

            threading.Thread(target=_read_inverter_data, args=(self.mbClient, self.addToTrace), daemon=True).start()

    def __init__(self, callbackAddToTrace=None, callbackShowStatus=None, homeplughandler=None, inverterComm=None):
        self.callbackAddToTrace = callbackAddToTrace
        self.callbackShowStatus = callbackShowStatus
        self.homeplughandler = homeplughandler
        if inverterComm is None:
            if (getConfigValue("digital_output_device")=="kostalinverter"):
                # self.inverterComm = serial.Serial(KIInverterSerial, baudrate=KIInverterBaud, timeout=0.2, write_timeout=0)
                self.inverterComm = serial.Serial(KIInverterSerial, baudrate=KIInverterBaud, write_timeout=0)
        else:
            self.inverterComm = inverterComm

        self.loopcounter = 0
        self.outvalue = 0
        self.simulatedSoc = 20.0 # percent
        self.demoAuthenticationCounter = 0

        self.inletVoltage = 0.0 # volts
        self.accuVoltage = 0.0
        self.lock_confirmed = False  # Confirmation from hardware
        self.cp_pwm = 0.0
        self.soc_percent = 0.0
        self.capacity = 0.0
        self.accuMaxVoltage = 0.0
        self.accuMaxCurrent = 0.0
        self.contactor_confirmed = False  # Confirmation from hardware
        self.plugged_in = None  # None means "not known yet"
        self.lastReceptionTime = 0

        self.maxChargerVoltage = 0
        self.maxChargerCurrent = 10
        self.chargerVoltage = 0
        self.chargerCurrent = 0

        self.logged_inlet_voltage = None
        self.logged_dc_link_voltage = None
        self.logged_cp_pwm = None
        self.logged_max_charge_a = None
        self.logged_soc_percent = None
        self.logged_contactor_confirmed = None
        self.logged_plugged_in = None

        self.rxbuffer = ""

        self.errorState = None
        self.kState = KIState0Idle
        self.kLastStateTransition = currentMillis()
        self.currentRX = bytearray(10)
        self.RXIndex = 0
        self.allowInverterCommunication = False
        self.startInverterCommunication = False
        self.startInverterCommunicationTimeStamp = None
        self.mbClient = None
        self.lastPrechargeVoltage = currentMillis()
        self.stepUpPWMIndex = 0
        self.stepUpPWM = None
        self.lastTargetVoltage = 44.0
        self._inverterConfigureBattery(False)

        self.closedCp = False
        self.closedDCMinus = False
        self.closedDCPlus = False
        self.closedPrecharge = False

        self.isSerialInterfaceOk = None
        self.findSerialPort()
        self.initPorts()
        
    def resetSimulation(self):
        self.simulatedInletVoltage = 0.0 # volts
        self.simulatedSoc = 20.0 # percent
        self.demoAuthenticationCounter = 0
        
    def simulatePreCharge(self):
        if (self.simulatedInletVoltage<230):
            self.simulatedInletVoltage = self.simulatedInletVoltage + 1.0 # simulate increasing voltage during PreCharge

    def close(self):
        if (self.isSerialInterfaceOk):        
            self.ser.close()
        if (getConfigValue("digital_output_device")=="kostalinverter"):
            self.stopDCSoft(setState=False)
            sleep(0.05)
            if shared_data['battery_current'] > 0.2:
                # one more try...
                self.stopDCSoft(setState=False)
                sleep(0.051)
            # stopDCHard() not required, it's implicitly done by GPIO.cleanup()
            GPIO.cleanup()
            self._inverterConfigureBattery(False)

    def evaluateReceivedData_dieter(self, s):
        self.rxbuffer += s
        x=self.rxbuffer.find("A0=")
        if (x>=0):
            s = self.rxbuffer[x+3:x+7]
            if (len(s)==4):
                try:
                    self.inletVoltage = int(s) / 1024.0 * 1.08 * (6250) / (4.7+4.7)
                    if (getConfigValue("analog_input_device")=="dieter"):
                        self.callbackShowStatus(format(self.inletVoltage,".1f"), "uInlet")
                except:
                    # keep last known value, if nothing new valid was received.
                    pass
                #self.addToTrace("RX data ok " + s)
                self.rxbuffer = self.rxbuffer[x+3:] # consume the receive buffer entry

    def evaluateReceivedData_celeron55device(self, s):
        self.rxbuffer += s
        while True:
            x = self.rxbuffer.find("\n")
            if x < 0:
                break
            line = self.rxbuffer[0:x].strip()
            self.rxbuffer = self.rxbuffer[x+1:]
            #self.addToTrace("Received line: \""+line+"\"")
            if line.startswith("inlet_v="):
                self.inletVoltage = int(line[8:])
                if self.logged_inlet_voltage != self.inletVoltage:
                    self.logged_inlet_voltage = self.inletVoltage
                    self.addToTrace("<< inlet_voltage="+str(self.inletVoltage))
                if self.callbackShowStatus:
                    self.callbackShowStatus(format(self.inletVoltage,".1f"), "uInlet")
            elif line.startswith("dc_link_v="):
                self.accuVoltage = int(line[10:])
                if self.logged_dc_link_voltage != self.accuVoltage:
                    self.logged_dc_link_voltage = self.accuVoltage
                    self.addToTrace("<< dc_link_voltage="+str(self.accuVoltage))
            elif line.startswith("cp_pwm="):
                self.cp_pwm = int(line[7:])
                if self.logged_cp_pwm != self.cp_pwm:
                    self.logged_cp_pwm = self.cp_pwm
                    self.addToTrace("<< cp_pwm="+str(self.cp_pwm))
            elif line.startswith("cp_output_state="):
                state = int(line[len("cp_output_state="):])
                if bool(state) == ((self.outvalue & 1)!=0):
                    self.addToTrace("<< CP state confirmed")
                else:
                    self.addToTrace("<< CP state MISMATCH")
            elif line.startswith("ccs_contactor_wanted_closed="):
                state = int(line[len("ccs_contactor_wanted_closed="):])
                if bool(state) == ((self.outvalue & 2)!=0):
                    self.addToTrace("<< Contactor request confirmed")
                else:
                    self.addToTrace("<< Contactor request MISMATCH")
            elif line.startswith("max_charge_a="):
                self.accuMaxCurrent = int(line[13:])
                if self.logged_max_charge_a != self.accuMaxCurrent:
                    self.logged_max_charge_a = self.accuMaxCurrent
                    self.addToTrace("<< max_charge_a="+str(self.accuMaxCurrent))
            elif line.startswith("soc_percent="):
                self.soc_percent = int(line[12:])
                if self.logged_soc_percent != self.soc_percent:
                    self.logged_soc_percent = self.soc_percent
                    self.addToTrace("<< soc_percent="+str(self.soc_percent))
            elif line.startswith("contactor_confirmed="):
                self.contactor_confirmed = bool(int(line[20:]))
                if self.logged_contactor_confirmed != self.contactor_confirmed:
                    self.logged_contactor_confirmed = self.contactor_confirmed
                    self.addToTrace("<< contactor_confirmed="+str(self.contactor_confirmed))
            elif line.startswith("plugged_in="):
                self.plugged_in = bool(int(line[11:]))
                if self.logged_plugged_in != self.plugged_in:
                    self.logged_plugged_in = self.plugged_in
                    self.addToTrace("<< plugged_in="+str(self.plugged_in))
            else:
                self.addToTrace("Received unknown line: \""+line+"\"")

    def showOnDisplay(self, s1, s2, s3):
        # show the given string s on the display which is connected to the serial port
        if (getConfigValueBool("display_via_serial") and self.isSerialInterfaceOk):
            if (getConfigValue("digital_output_device")=="celeron55device"):
                s = "disp0=" + s1 + "\n" + "disp1=" + s2 + "\n" + "disp2=" + s3 + "\n"
                self.ser.write(bytes(s, "utf-8"))
            else:
                s = "lc" + s1 + "\n" + "lc" + s2 + "\n" + "lc" + s3 + "\n"
                self.ser.write(bytes(s, "utf-8"))
        
    def mainfunction(self):
        if (getConfigValueBool("soc_simulation")):
            if (self.simulatedSoc<100):
                if ((self.outvalue & 2)!=0):
                    # while the relay is closed, simulate increasing SOC
                    deltaSoc = 0.5 # how fast the simulated SOC shall rise.
                    # Examples:
                    #  0.01 charging needs some minutes, good for light bulb tests
                    #  0.5 charging needs ~8s, good for automatic test case runs.
                    self.simulatedSoc = self.simulatedSoc + deltaSoc
                
        if (getConfigValue("charge_parameter_backend")=="chademo"):
           self.mainfunction_chademo()
        
        if (getConfigValue("digital_output_device")=="dieter"):
            self.mainfunction_dieter()

        if (getConfigValue("digital_output_device")=="celeron55device"):
            self.mainfunction_celeron55device()

        if (getConfigValue("digital_output_device")=="kostalinverter"):
            self.mainfunction_kostalinverter()

        if getConfigValueBool("exit_on_session_end"):
            # TODO: This is a hack. Do this in fsmPev instead and publish some
            # of these values into there if needed.
            if (self.plugged_in is not None and self.plugged_in == False and
                    self.inletVoltage < 50):
                sys.exit(0)

    def mainfunction_dieter(self):
        self.loopcounter+=1
        if (self.isSerialInterfaceOk):
            if (self.loopcounter>15):
                self.loopcounter=0
                # self.ser.write(b'hello world\n')
                s = "000" + str(self.outvalue)
                self.ser.write(bytes("do"+s+"\n", "utf-8")) # set outputs of dieter, see https://github.com/uhi22/dieter
            s = self.ser.read(100)
            if (len(s)>0):
                try:
                    s = str(s, 'utf-8').strip()
                except:
                    s = "" # for the case we received corrupted data (not convertable as utf-8)
                self.addToTrace(str(len(s)) + " bytes received: " + s)
                self.evaluateReceivedData_dieter(s)

    def mainfunction_celeron55device(self):
        if (self.isSerialInterfaceOk):
            s = self.ser.read(100)
            if (len(s)>0):
                try:
                    s = str(s, 'utf-8')
                except:
                    s = "" # for the case we received corrupted data (not convertable as utf-8)
                #self.addToTrace(str(len(s)) + " bytes received: " + s)
                self.evaluateReceivedData_celeron55device(s)
                
    def mainfunction_chademo(self):
       message = self.canbus.recv(0)
       
       if message:
          if message.arbitration_id == 0x100:
             vtg = (message.data[1] << 8) + message.data[0]
             if self.accuVoltage != vtg:
                 self.addToTrace("CHAdeMO: Set battery voltage to %d V" % vtg)
             self.accuVoltage = vtg
             if self.capacity != message.data[6]:
                 self.addToTrace("CHAdeMO: Set capacity to %d" % message.data[6])
             self.capacity = message.data[6]
             
             msg = can.Message(arbitration_id=0x108, data=[ 0, self.maxChargerVoltage & 0xFF, self.maxChargerVoltage >> 8, self.maxChargerCurrent, 0, 0, 0, 0], is_extended_id=False)
             self.canbus.send(msg)
             #Report unspecified version 10, this makes our custom implementation send the momentary
             #battery voltage in 0x100 bytes 0 and 1
             status = 4 if self.maxChargerVoltage > 0 else 0  #report connector locked
             msg = can.Message(arbitration_id=0x109, data=[ 10, self.chargerVoltage & 0xFF, self.chargerVoltage >> 8, self.chargerCurrent, 0, status, 0, 0], is_extended_id=False)
             self.canbus.send(msg)
             
          if message.arbitration_id == 0x102:
             vtg = (message.data[2] << 8) + message.data[1]
             if self.accuMaxVoltage != vtg:
                 self.addToTrace("CHAdeMO: Set target voltage to %d V" % vtg)
             self.accuMaxVoltage = vtg
             
             if self.accuMaxCurrent != message.data[3]:
                 self.addToTrace("CHAdeMO: Set current request to %d A" % message.data[3])
             self.accuMaxCurrent = message.data[3]
             self.lastReceptionTime = time()
             
             if self.capacity > 0:
                 soc = message.data[6] / self.capacity * 100
                 if self.simulatedSoc != soc:
                     self.addToTrace("CHAdeMO: Set SoC to %d %%" % soc)
                 self.simulatedSoc = soc
       #if nothing was received for over a second, time out        
       if self.lastReceptionTime < (time() - 1):
           if self.accuMaxCurrent != 0:
              self.addToTrace("CHAdeMO: No current limit update for over 1s, setting current to 0")
           self.accuMaxCurrent = 0

    def _calcFrameCRC(self, frame, length):
        assert length < len(frame)
        assert frame[0] == 0x00

        crc = 0
        for i in range(length):
            crc += frame[i]

        return (-crc & 0xff);

    def _scrambleFrame(self, frame):
        assert frame[0] == 0x00
        lastIndex = 0

        for i in range(len(frame)):
            if i == 0:
                continue

            if frame[i] == 0x00:
                frame[lastIndex] = i - lastIndex
                lastIndex = i

    def _sendFrameToInverter(self, frame):
        self._scrambleFrame(frame)

        hex_string = " ".join(f"0x{b:02x}" for b in frame)
        self.addToTrace(f"TX frame: {hex_string}")
        self.inverterComm.write(frame)
        if self.startInverterCommunicationTimeStamp is None:
            self.startInverterCommunicationTimeStamp = currentMillis()

    def _resetInverterStateMachine(self):
        batCur = shared_data['battery_current']
        if not (batCur > -0.01 and batCur < 0.01):
            self.setError(f"wanted to reset inverter state machine, but batCur={batCur}")
            return
        self.addToTrace("reset inverter state machine")
        self._setKState(KIState0Idle)
        if self.closedDCPlus or self.closedPrecharge:
            self._setPrecharge(True)
            sleep(0.03) # 30ms
        self._setDCPlus(False)
        sleep(0.03) # 30ms
        self._setPrecharge(False)
        self._setDCMinus(False)
        self.inverterComm.reset_input_buffer()
        self.startInverterCommunicationTimeStamp = None

    # kostalinverter config
    def mainfunction_kostalinverter(self):
        if self.hasError():
            return

        if not self.startInverterCommunication:
            return

        #  70s worked.
        # 140s worked.
        if self.startInverterCommunicationTimeStamp is not None and currentMillis() - self.startInverterCommunicationTimeStamp > 70 * 1000:
            if shared_data['inverter_status'] == 0 or (not self.closedDCPlus and not self.closedPrecharge):
                self._resetInverterStateMachine()
                return

        frame = None
        if self.inverterComm.in_waiting > 0:
            rxByte = int(self.inverterComm.read()[0])
            if self.RXIndex >= len(self.currentRX):
                hex_string = " ".join(f"0x{b:02x}" for b in self.currentRX)
                # TODO: make it an error?
                self.addToTrace(f"invalid RX frame: {hex_string}, self.RXIndex={self.RXIndex} self.currentRX={self.currentRX}")
                self.currentRX = bytearray(10)
                self.RXIndex = 0
                self.inverterComm.reset_input_buffer()
                return
            elif rxByte == 0:
                self.currentRX[self.RXIndex] = 0
                frame = bytearray(self.currentRX)
                self.currentRX = bytearray(10)
                self.RXIndex = 0
            else:
                # self.addToTrace(f"rxByte={rxByte}")
                # self.addToTrace(f"self.RXIndex={self.RXIndex}")
                # self.addToTrace(f"self.currentRX={self.currentRX}")
                self.currentRX[self.RXIndex] = rxByte
                self.RXIndex += 1
                return
        else:
            return

        if frame is None:
            hex_string = " ".join(f"0x{b:02x}" for b in self.currentRX)
            self.setError(f"wtf RX frame: {hex_string}")
            return

        isCyclicData = frame == RXframeCyclicData
        isBatteryInfo = frame == RXframeBatteryInfo
        isErrorState = frame == RXframeErrorState
        isRestartFrame = frame == RXframeRestartFrame
        isCloseContactors = frame == RXframeCloseContactors
        isUnknownFrame1 = frame == RXframeUnknownFrame1

        rxStr = "CyclicData" if isCyclicData else "BatteryInfo" if isBatteryInfo else "ErrorState" if isErrorState else "RestartFrame" if isRestartFrame else "CloseContactors" if isCloseContactors else "UnknownFrame1" if isUnknownFrame1 else "???"
        self.addToTrace(f"RX: {rxStr}")
        if rxStr == "???":
            hex_string = " ".join(f"0x{b:02x}" for b in frame)
            # TODO: make it an error?
            self.addToTrace(f"unknown RX frame: {hex_string}")
            self.currentRX = bytearray(10)
            self.RXIndex = 0
            self.inverterComm.reset_input_buffer()
            return

        if isBatteryInfo:
            outframe = bytearray(40)
            # header
            outframe[0:6] = bytearray([0x00, 0xE2, 0xFF, 0x02, 0xFF, 0x29])
            # nominal voltage
            outframe[6:10] = struct.pack('f', 380.5)
            # outframe[6:10] = bytearray([0x00, 0x00, 0xa2, 0x43]) # with that the checksum should be 0x2b

            # manufacture date ?
            outframe[10:14] = bytearray([0xE4, 0x70, 0x8A, 0x5C])
            # serial number ?
            outframe[14:18] = bytearray([0xB5, 0x00, 0xD3, 0x00])
            # 0x10b4
            outframe[18:22] = bytearray([0x00, 0x00, 0xC8, 0x41])
            # battery firmware
            outframe[22:24] = bytearray([0xC2, 0x18])
            # ???
            outframe[24:28] = bytearray([0x00, 0x00, 0x59, 0x42])
            # ???
            outframe[28:32] = bytearray([0x00, 0x00, 0x00, 0x00])
            # ???
            outframe[32:38] = bytearray([0x05, 0x00, 0xA0, 0x00, 0x00, 0x00])
            # CRC
            outframe[38] = self._calcFrameCRC(outframe, 38)
            # terminating NUL byte
            outframe[39] = 0x00
            self._sendFrameToInverter(outframe)
            return

        if isErrorState:
            self._sendFrameToInverter(bytearray([0x00, 0xE2, 0xFF, 0x02, 0xFF, 0x29, 0x06, 0xEF, 0x00]))
            return

        if isRestartFrame:
            self.setError("Restart Frame received, do not know what to do. PANIC")
            return

        if isCloseContactors:
            self._sendFrameToInverter(bytearray([0x00, 0xE3, 0xFF, 0x02, 0xFF, 0x29, 0xF4, 0x00]))
            if self.kState != KIState1NegativeConfirm:
                self.addToTrace(f"expected state machine to be in KIState1NegativeConfirm, but is in kState={self.kState}")
                return
            self._contactorsStateMachine()
            return

        if isUnknownFrame1:
            # ACK it
            self._sendFrameToInverter(bytearray([0x00, 0xE3, 0xFF, 0x02, 0xFF, 0x29, 0xF4, 0x00]))
            return

        if isCyclicData:
            outframe = bytearray(64)
            # frame header
            outframe[0:6] = bytearray([0x00, 0xE2, 0xFF, 0x02, 0xFF, 0x29])
            # current voltage
            outframe[6:10] = struct.pack('f', self.lastTargetVoltage)
            # max voltage
            outframe[10:14] = struct.pack('f', 422.2)
            # battery temp
            outframe[14:18] = struct.pack('f', 14.0)
            # peak current
            outframe[18:22] = struct.pack('f', 1.0)
            # avg current
            outframe[22:26] = struct.pack('f', 0.5)
            # max discharge current
            outframe[26:30] = struct.pack('f', 13.0 if self.kState > KIState1NegativeConfirm else 0.0)
            # something capacity related
            outframe[30:34] = bytearray([0x00, 0x00, 0xC8, 0x41])
            # max charge current
            outframe[34:38] = struct.pack('f', 13.0 if self.kState > KIState1NegativeConfirm else 0.0)

            # max cell temp
            outframe[38:42] = struct.pack('f', 22.0 if self.kState > KIState0IdlePrime else 0.0)
            # min cell temp
            outframe[42:46] = struct.pack('f', 11.0 if self.kState > KIState0IdlePrime else 0.0)

            # max cell voltage
            outframe[46:50] = struct.pack('f', 3.9 if self.kState > KIState0IdlePrime else 0.0)
            # min cell voltage
            outframe[50:54] = struct.pack('f', 3.5 if self.kState > KIState0IdlePrime else 0.0)

            # cycle count
            outframe[54:56] = bytearray([0x39, 0x05]) # if self.kState != KIState0Idle else bytearray([0x00, 0x00])

            # ModBus "battery ready" flag
            outframe[56] = 0x01 if self.kState > KIState2PrechargeConfirm else 0x00

            # TODO: should be 0x40 if SoC is at 100%
            outframe[57] = 0x00

            # SoC (TODO)
            outframe[58] = 55

            # "operate flag"?
            outframe[59] = 0x00 if self.kState != KIState4Operate else 0x02

            # always NUL byte
            outframe[60] = 0x00

            # only set on the very first time this frame is sent
            outframe[61] = 0x00 if self.kState > KIState0IdlePrime else 0x01

            outframe[62] = self._calcFrameCRC(outframe, 62)
            # terminating NUL byte
            outframe[63] = 0x00
            self._sendFrameToInverter(outframe)

            # progress in state machine
            if self.kState == KIState0Idle or self.kState == KIState0IdlePrime or self.kState == KIState1Negative:
                # let's close DC-
                self._contactorsStateMachine()
            elif self.kState == KIState1NegativeConfirm:
                # this must be continued by `isCloseContactors`
                pass
            elif self.kState == KIState2Precharge or self.kState == KIState2PrechargeConfirm:
                # let's precharge
                self._contactorsStateMachine()
            elif self.kState == KIState3Positive or self.kState == KIState3PositiveConfirm:
                # let's close DC+
                self._contactorsStateMachine()
            elif self.kState == KIState4Operate:
                # business as usual!
                self._contactorsStateMachine()
            else:
                self.setError(f"weird state in isCyclicData: kState={self.kState}")

            return

        hex_string = " ".join(f"0x{b:02x}" for b in frame)
        self.setError(f"should not reach here, missing handling of RX frame: {hex_string}")

    def _contactorsStateMachine(self):
        if not self.allowInverterCommunication:
            if self.kState > KIState0IdlePrime:
                self.setError("inverter communication disabled and but state machine not in init")
            return

        iKnowWhatIAmDoing = True

        if self.kState == KIState0Idle:
            if self.closedDCMinus:
                self.setError("DC- is closed at KIState0Idle")
                return

            if self.closedDCPlus:
                self.setError("DC+ is closed at KIState0Idle")
                return

            if self.closedPrecharge:
                self.setError("Precharge is closed at KIState0Idle")
                return

            if self._readDCMinusAUX() is True:
                self.setError("DC- AUX is closed at KIState0Idle (WELDED CONTACTOR?!)")
                return

            self._setKState(KIState1Negative)
        elif self.kState == KIState0IdlePrime:
            batCur = shared_data['battery_current']
            if batCur > 0.1 or batCur < -0.1:
                self.setError("There is current during KIState0IdlePrime flowing, should not happen " + str(batCur) + "A")
                return

            self._setDCMinus(False)
            self._setDCPlus(False)
            self._setDCPrecharge(False)

            self._setKState(KIState1Negative)
        elif self.kState == KIState1Negative:
            self._setDCMinus(True)
            self._setDCPlus(False)
            self._setPrecharge(False)
            self._setKState(KIState1NegativeConfirm)
        elif self.kState == KIState1NegativeConfirm:
            if not self.closedDCMinus:
                self.setError("DC- is not closed at KIState1NegativeConfirm")
                return
            if not self._readDCMinusAUX() is True:
                self.setError("DC- AUX is not closed at KIState1NegativeConfirm")
                return

            batVol = shared_data['battery_voltage']
            if batVol > 150 and not iKnowWhatIAmDoing:
                self.setError("There is voltage during KIState1NegativeConfirm, either DC+ or Prechare welded?! " + str(batVol) + "V")
                return

            batCur = shared_data['battery_current']
            if batCur > 0.1 or batCur < -0.1:
                self.setError("There is current during KIState1NegativeConfirm flowing, should not happen " + str(batCur) + "A")
                return

            self._setKState(KIState2Precharge)
        elif self.kState == KIState2Precharge:
            self._setPrecharge(True)
            self._setKState(KIState2PrechargeConfirm)
        elif self.kState == KIState2PrechargeConfirm:
            if not self.closedPrecharge:
                self.setError("Precharge is not closed at KIState2PrechargeConfirm")
                return

            batVol = shared_data['battery_voltage']
            if batVol < 150 and not iKnowWhatIAmDoing:
                self.setError("There is no voltage during KIState2PrechargeConfirm, circuit breaker to inverter not closed? " + str(batVol) + "V")
                return

            batCur = shared_data['battery_current']
            if batCur > 0.1 or batCur < -0.1:
                self.setError("There is current during KIState2PrechargeConfirm flowing, should not happen " + str(batCur) + "A")
                return

            self._setKState(KIState3Positive)
        elif self.kState == KIState3Positive:
            self._setDCPlus(True)
            self._setKState(KIState3PositiveConfirm)
        elif self.kState == KIState3PositiveConfirm:
            self._setPrecharge(False)
            if not self.closedDCPlus:
                self.setError("DC+ is not closed at KIState3PositiveConfirm")
                return

            batVol = shared_data['battery_voltage']
            if batVol < 10 and not isinstance(self.inverterComm, FakeKostalInverter):
                self.setError("There is no voltage during KIState3PositiveConfirm. but voltage seen during precharge, weird? " + str(batVol) + "V")
                return

            # batCur = self.inverter_battery_current()
            # if batCur > 0.1 or batCur < -0.1:
            #     self.setError("There is current during KIState3PositiveConfirm flowing, should not happen " + str(batCur) + "A")
            #     return

            self._setKState(KIState4Operate)
        elif self.kState == KIState4Operate:
            if not self.closedDCPlus:
                self.setError("DC+ is not closed at KIState4Operate")
                return
            if not self.closedDCMinus:
                self.setError("DC- is not closed at KIState4Operate")
                return
            if self.closedPrecharge:
                self.setError("Precharge is closed at KIState4Operate")
                return
        else:
            self.setError("invalid state: " + str(self.kState))
            return

    def _setKState(self, next_state):
        self.kLastStateTransition = currentMillis()
        self.addToTrace(f"kState: from={self.kState} to next={next_state}")
        self.kState = next_state

    def hasError(self):
        return self.errorState is not None

    def setError(self, errmsg):
        self.errorState = errmsg
        self.addToTrace("ERROR: " + str(errmsg))
        self.haasDcwbEvseState("error")

    def opInverter(self):
        if not self.allowInverterCommunication:
            self.startInverterCommunication = True
            self.inverterComm.reset_input_buffer()
            self._inverterConfigureBattery(True)
        self.allowInverterCommunication = True
        self.haasDcwbEvseState("operating")

    def _disableInverterCommunication(self):
        if self.allowInverterCommunication:
            # reset state
            self._setKState(KIState0IdlePrime)
        self.allowInverterCommunication = False

    def _inverterConfigureBattery(self, enabled):
        kostalIP = getConfigValue("kostalinverter_url")
        kostalPassword = getConfigValue("kostalinverter_password")
        kostalServicecode = getConfigValue("kostalinverter_servicecode")

        subprocess.Popen([
            'pykoplenti',
            '--host', kostalIP,
            '--password', kostalPassword,
            '--service-code', kostalServicecode,
            'write-settings',
            'devices:local/Battery:Type=' + ('4' if enabled else '0')
            ])

    def haasDcwbEvseState(self, next_state):
        if (getConfigValue("digital_output_device")!="kostalinverter"):
            return

        e_disconnected = "disconnected"
        e_stop_session = "stop_session"
        e_start_session = "start_session"
        e_operating = "operating"
        e_maybe_timeout = "maybe_timeout"
        e_error = "error"

        if next_state != e_disconnected and next_state != e_stop_session and next_state != e_start_session and next_state != e_operating and next_state != e_maybe_timeout and next_state != e_error:
            self.errorState = "haas invalid evse state: " + str(next_state)
            next_state = e_error

        try:
            haasURL = getConfigValue("homeassistant_url")
            haasToken = getConfigValue("homeassistant_token")

            with open(os.devnull, 'w') as devnull:
                subprocess.Popen(["curl", '-H',
                    f"Authorization: Bearer {haasToken}",
                    "-H",
                    "Content-Type: application/json",
                    f"{haasURL}/api/services/input_select/select_option",
                    "-d",
                    '{ "entity_id": "input_select.dcwb_evse_state", "option": "' + next_state + '" }'],
                    stdout=devnull,
                    stderr=devnull)
        except:
            self.addToTrace("Warning: haasDcwbEvseState failed!!!!")
            self.stopDCSoft(setState=False)
            return False

    def haasDcwbAllowCharging(self):
        if (getConfigValue("digital_output_device")!="kostalinverter"):
            return

        val = shared_data['haas_dcwb_allow_charging']

        if val is False:
            return False

        if currentMillis() - val < 35 * 1000:
            # allow up to 35s, could be a HAAS restart
            return True
        return False

    def stopDCSoft(self, setState=True):
        if (getConfigValue("digital_output_device")!="kostalinverter"):
            return

        self.addToTrace("stopDCSoft()")
        self._disableInverterCommunication()
        self.stopPrecharge()
        self._setPrecharge(True)
        # command inverter via ModBus to draw zero power from battery
        kostalIP = getConfigValue("kostalinverter_url")
        # really is reg=1034, off-by-one via `mbpoll`
        subprocess.Popen(["mbpoll", "-p", "1502", str(kostalIP), "-1", "-a", "71", "-t", "4:float", "-r", "1035", "--", "0"])
        if setState:
            self.haasDcwbEvseState("maybe_timeout")

    def stopDCHard(self):
        if (getConfigValue("digital_output_device")!="kostalinverter"):
            return

        self.addToTrace("stopDCHard()")
        if self.closedDCPlus or self.closedPrecharge:
            self._setDCPlus(False)
            sleep(0.03) # 30ms
        else:
            self._setDCPlus(False)
        self._setPrecharge(False)
        self._setDCMinus(False)
        self.stopPrecharge()
        self._disableInverterCommunication()

    def _readDCMinusAUX(self):
        if KIContactorDCMinusAUX is None:
            return None
        # pull-up is configured. so if active, GND.  if not active, V3.3
        return GPIO.input(KIContactorDCMinusAUX) == 0

    def _setDCMinus(self, val):
        # Note 20ms are enough to see AUX applied, good.
        GPIO.output(KIContactorDCMinus, GPIO.LOW if val is True else GPIO.HIGH)
        self.addToTrace(f"relay: DC- from {self.closedDCMinus} to {val}")
        self.closedDCMinus = val

    def _setDCPlus(self, val):
        GPIO.output(KIContactorDCPlus, GPIO.LOW if val is True else GPIO.HIGH)
        self.addToTrace(f"relay: DC+ from {self.closedDCPlus} to {val}")
        self.closedDCPlus = val

    def _setPrecharge(self, val):
        GPIO.output(KIContactorPrecharge, GPIO.LOW if val is True else GPIO.HIGH)
        self.addToTrace(f"relay: Pre from {self.closedPrecharge} to {val}")
        self.closedPrecharge = val

    def _setCp(self, val):
        # TODO: maybe inverted?
        GPIO.output(KICp, GPIO.LOW if val is True else GPIO.HIGH)
        shouldWait = False
        if self.closedCp != val:
            self.addToTrace(f"relay: CP from {self.closedCp} to {val}")
        if not self.closedCp and val:
            shouldWait = True
        self.closedCp = val
        if shouldWait:
            sleep(1)
        
def myPrintfunction(s):
    print("myprint " + s)

class FakeKostalInverter:
    def __init__(self):
        self.index = 0
        self.state = 1
        self.lastFrameSent = currentMillis()
        self.currentTX = bytearray(64)
        self.TXIndex = 0
        self.lastContactorsClosedSent = currentMillis()

    def reset_input_buffer(self):
        # just a stub to mimic serial interface
        pass

    @property
    def in_waiting(self):
        if currentMillis() - self.lastFrameSent > 1000:
            return 1
        return 0

    def write(self, txBytes):
        assert isinstance(txBytes, bytearray)
        for b in txBytes:
            self._write(b)

    def _write(self, txByte):
        if txByte == 0:
            self.currentTX[self.TXIndex] = 0
            hex_string = " ".join(f"0x{b:02x}" for b in self.currentTX)
            print(f"[fake] TX frame: {hex_string}")
            self.currentTX = bytearray(64)
            self.TXIndex = 0
        elif self.TXIndex > len(self.currentTX) - 1:
            hex_string = " ".join(f"0x{b:02x}" for b in self.currentTX)
            print(f"[fake] INVALID frame: {hex_string}")
            self.currentTX = bytearray(64)
            self.TXIndex = 0
        else:
            self.currentTX[self.TXIndex] = txByte
            self.TXIndex += 1

    def read(self):
        frameToSend = None
        if self.state == 1:
            frameToSend = RXframeBatteryInfo
        elif self.state == 2:
            frameToSend = RXframeCyclicData
        elif self.state == 3:
            frameToSend = RXframeErrorState
        elif self.state == 4:
            frameToSend = RXframeCloseContactors
        else:
            assert False, "not implemented yet"

        ret = bytearray([frameToSend[self.index]])

        self.index += 1

        if len(frameToSend) == self.index:
            self.lastFrameSent = currentMillis()
            self.index = 0
            if self.state == 1:
                self.state = 2
            else:
                if currentMillis() - self.lastContactorsClosedSent > 20 * 1000:
                    self.state = 4
                    self.lastContactorsClosedSent = currentMillis()
                elif self.state == 2:
                    self.state = 3
                elif self.state == 3 or self.state == 4:
                    self.state = 2
                else:
                    assert False, "not implemented yet"

        return ret

if __name__ == "__main__":
    print("Testing hardwareInterface...")
    if (getConfigValue("digital_output_device")!="kostalinverter"):
        hw = hardwareInterface(myPrintfunction)
        for i in range(0, 350):
            hw.mainfunction()
            if (i==20):
                hw.showOnDisplay("Hello", "A DEMO", "321.0V")
            if (i==50):
                hw.setStateC()
            if (i==100):
                hw.setStateB()
            if (i==150):
                hw.setStateC()
                hw.setPowerRelayOn()
                hw.showOnDisplay("", "..middle..", "")
            if (i==200):
                hw.setStateB()
                hw.setPowerRelayOff()
            if (i==250):
                hw.setRelay2On()
            if (i==300):
                hw.setRelay2Off()
            if (i==320):
                hw.showOnDisplay("This", "...is...", "DONE :-)")
            sleep(0.03)
        hw.close()
        print("finished.")
    else:
        def myPrint(s):
            dT_ms = currentMillis()
            print("[" + str(dT_ms) + "ms] " + s)
        hw = hardwareInterface(myPrint, inverterComm=FakeKostalInverter())
        print("hi.")
        hw.mainfunction()
        hw.haasDcwbEvseState("disconnected")

        if False:
            timeout = 0.20

            hw._setPrecharge(True)
            print("state DC-AUX: " + str(hw._readDCMinusAUX()))
            assert hw._readDCMinusAUX() is False
            sleep(timeout)

            hw._setDCMinus(True)
            print("enable DC- contactor")
            sleep(timeout)

            print("state DC-AUX: " + str(hw._readDCMinusAUX()))
            assert hw._readDCMinusAUX() is True
            sleep(timeout)
            hw._setDCPlus(True)

            hw._setDCMinus(False)
            print("disable DC- contactor")
            sleep(timeout)

            print("state DC-AUX: " + str(hw._readDCMinusAUX()))
            assert hw._readDCMinusAUX() is False
        else:
            byte_array = []
            startStamp = currentMillis()
            print("allow: " + str(hw.haasDcwbAllowCharging()))
            print(f"battery_amp:  {shared_data['battery_current']:.1f}A")
            print(f"battery_volt: {shared_data['battery_voltage']:.1f}V")
            hw.opInverter()
            while True:
                hw.mainfunction()

                # if hw.inverterComm.in_waiting > 0:  # Check if there is data available to read
                #     hw.inverterComm.write(0x32)
                #     data = hw.inverterComm.read()  # Read a single byte
                #     byte_array.append(data)
                # if len(byte_array) >= 10:
                #     hw.inverterComm.write(0x00)

                #     # hex_string = " ".join(f"0x{b:x}" for b in byte_array)
                #     hex_string = " ".join(f"0x{b:02x}" for b in byte_array)
                #     print(f"received frame: {hex_string}")
                #     byte_array = []


        hw.close()
