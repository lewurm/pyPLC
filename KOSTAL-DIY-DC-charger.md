## Parts list

| Price | Description | Potential source |
--------|-------------|------------------|
| €220 |	CCS2 Plug with 5m cable rated for 80A 	| That was in 2023, offer on Aliexpress not available anymore.  I recommend to get something with a rating of minimum 50A	Maybe that? https://aliexpress.com/item/1005006261361557.html |
| €150	| 3x contactors from TE, rated for HV and 50A.  If you like, get one with additional AUX lines, that gives you additional safety to determine if a contactor is welded. 12V or 24V are available. Checkout these covers (thank you Josef ❤️): https://www.printables.com/model/1227300-te-eck-series-high-voltage-dc-contactor-cover 	| https://www.te.com/en/product-CAT-P23-ECK.html?d=674019 |
| €63	| Junction box	| https://www.amazon.de/dp/B018TYPFH6 |
| €60	| Raspberry Pi 4 4gb |	- |
| €18	| 4 Channels Relay HAT	| https://www.amazon.de/dp/B0BP1YVDZ4 |
| €33	| 5x terminals	| https://www.amazon.de/dp/B009923XUY (I recommend to get different colors) |
| €16	| RS485 to USB dongle	| https://www.amazon.de/dp/B0B87D9LNC |
| €27	| 24V power supply	| https://www.amazon.de/gp/product/B09QMQS6MR |
| €20	| 4x DC fuses	| https://www.amazon.de/dp/B0B81XSKMJ |
| €13	| 1A fuses for step-up converter	| https://www.amazon.de/dp/B0B7ML9HBB |
| €14	| 15A fuses for inverter	| https://www.amazon.de/dp/B0B7N173SG |
| €30	| Arduino UNO R4 | - |
| €8	| Arduino Prototype Shield	| https://www.amazon.de/dp/B0BN1M2SNG |
| €11	| USB power supply	| https://www.amazon.de/dp/B0C2336R7J |
| €14	| 400 Ohm precharge resistor	| https://www.amazon.de/dp/B07HMRVR6V |
| €20	| misc parts for Arduino CP handling, step-up converter mod and tp link mod |
| €40	| 2x6mm H1Z2Z2-K solar cable, roughly 4EUR/m, assuming 10m |
| €10	| shielded CAT7 cable (for inverter communication), assuming 10m |
| €15	| 6mm cable with M5 ring clamp (fit for contactors)	| https://www.amazon.de/dp/B0CM9BLX1S |
| €7	| jumper wires	| https://www.amazon.de/dp/B074P726ZR |
| €8	| HIA4V1 step-up converter	| https://de.aliexpress.com/item/32913619861.html |
| €15	| TPlink TL-PA4010P v5.0 | (used)  |
| €78	| optional: protective gear	| https://www.amazon.de/dp/B0CH38SG3M & | https://www.amazon.de/gp/product/B0973X468X
| €19	| optional: NVMe USB3.1 adapter	| https://www.amazon.de/dp/B07TXCMQ8B
| €34	| optional: NVMe 500GB	| https://www.amazon.de/dp/B0B25LQQPC



### Services
Since the involved components (car and inverter) are not designed for a this kind of usage, I came up with a few convenience automation via Home Assistant.

A few scenarios that are covered:
- When a charge session ends, the Raspberry Pi in the Wallbox will shut down.  Home Assistant fully disconnects it from the 230V supply for a few seconds and the turns on the wallbox again.  This helps with starting from a known state (e.g. the PLC modem will be resetted too).
- On garage entrance there is a phyiscal button (hooked up to Home Assistant) that will initate a "safe shutdown" (setting `dcwb_allow_charging` set to `no`) on double click.  Once done, it will also unlock the charge port via an API call via BLE (see https://github.com/yoziru/esphome-tesla-ble ).
- When the garage is left (single click) the garage door is closed and it's checked if the car is there (via BLE), and if so, `dcwb_allow_charging` will be set to `yes`.
- Before a charging session starts, PP is shortly disconnected to simulate a replug.  This helps with resetting the internal BMS state machine e.g. for the 15Ah discharge limit.
- Notification when the inverter is in a weird state and requires manual intervention (happens roughly every two weeks).


Install service via systemd:

```sh
$ systemctl --user enable tmux.service                                                                                                                                                    │
$ systemctl --user start tmux.service                                                                                                                                                     │
$ ~/.config/systemd/user/tmux.service tmux.service
```

Requires the following home assistant `input_select` entities, with the given options:

```
input_select.dcwb_allow_charging: yes, no, please_restart_no, please_restart_yes, please_shutdown_no, please_shutdown_yes

input_select.dcwb_allow_inverter: yes, no

input_select.dcwb_evse_state: disconnect, stop_session, start_session, operating, error, maybe_timeout

```

virtual button:

```
input_button.dcwb_stop_session
```



### Automation for restarting wallbox

Assuming `switch.dcwb_plug` is a smart switch that is hooked up between the 230V supply and the wallbox.
It basically interacts with `startup.sh`. For example, if `please_shutdown_no` is choosen in Home Assistant,
the script will turn off the RaspberryPi, and shortly after Home Assistant will disconnect it from the 230V
supply via `switch.dcwb_plug`. The `_no` suffix here means that `dwcb_allow_charging` will be set to `no`,
meaning that the `startup.sh` script will wait until flipped to `yes`, and only then start a pyPLC
session.



```yaml
alias: "dcwb: restart box"
description: ""
triggers:
  - trigger: state
    entity_id:
      - input_select.dcwb_allow_charging
    from: null
    to: please_restart_no
    for:
      hours: 0
      minutes: 0
      seconds: 8
    id: reason_no
  - trigger: state
    entity_id:
      - input_select.dcwb_allow_charging
    from: null
    to: please_restart_yes
    for:
      hours: 0
      minutes: 0
      seconds: 8
    id: reason_yes
  - trigger: state
    entity_id:
      - input_select.dcwb_allow_charging
    from: null
    to: please_shutdown_no
    for:
      hours: 0
      minutes: 0
      seconds: 8
    id: reason_no
  - trigger: state
    entity_id:
      - input_select.dcwb_allow_charging
    from: null
    to: please_shutdown_yes
    for:
      hours: 0
      minutes: 0
      seconds: 8
    id: reason_yes
conditions: []
actions:
  - action: switch.turn_off
    metadata: {}
    data: {}
    target:
      entity_id: switch.dcwb_plug
  - delay:
      hours: 0
      minutes: 0
      seconds: 8
    enabled: false
  - delay:
      hours: 0
      minutes: 0
      seconds: 8
    enabled: false
  - delay:
      hours: 0
      minutes: 0
      seconds: 8
    enabled: false
  - delay:
      hours: 0
      minutes: 0
      seconds: 8
  - if:
      - condition: or
        conditions:
          - condition: state
            entity_id: input_select.dcwb_allow_charging
            state: please_restart_no
          - condition: state
            entity_id: input_select.dcwb_allow_charging
            state: please_restart_yes
    then:
      - action: switch.turn_on
        metadata: {}
        data: {}
        target:
          entity_id: switch.dcwb_plug
  - choose:
      - conditions:
          - condition: trigger
            id:
              - reason_no
        sequence:
          - action: input_select.select_option
            metadata: {}
            data:
              option: "no"
            target:
              entity_id: input_select.dcwb_allow_charging
      - conditions:
          - condition: trigger
            id:
              - reason_yes
        sequence:
          - action: input_select.select_option
            metadata: {}
            data:
              option: "yes"
            target:
              entity_id: input_select.dcwb_allow_charging
          - action: switch.toggle
            metadata: {}
            data: {}
            target:
              entity_id: switch.tesla_ble_charge_port_switch
          - delay:
              hours: 0
              minutes: 0
              seconds: 2
              milliseconds: 0
          - action: switch.turn_on
            metadata: {}
            data: {}
            target:
              entity_id: switch.tesla_ble_charge_port_switch
mode: single
```


References to `switch.tesla_ble*` are coming from https://github.com/yoziru/esphome-tesla-ble


### Hook for stopping DCWB session

Automation for button (can be any button in Home Assistant, in this case it's some Zigbee Button exposed via Zigbee2MQTT):

```yaml
alias: "Garage button (double): stop DC Wallbox"
description: ""
triggers:
  - domain: mqtt
    device_id: 5366b78e29a305a69e28c7f2dfa73aa5
    type: action
    subtype: double
    trigger: device
conditions: []
actions:
  - action: input_button.press
    metadata: {}
    data: {}
    target:
      entity_id: input_button.dcwb_stop_session
mode: single
```


Then the actual logic is done with PyScript in Home Assistant:

```python
@state_trigger("input_button.dcwb_stop_session")
def trigger_stop_session():
    if pyscript.dcwb_button_stop_pressed == False:
        nlog("DCWB: trigger stop session already in progress", True)
        service.call(domain="modbus", name="write_register", address="1034", slave="71", hub="kostalplenticoreplus", value=float_to_32bit_parts(float(0)))
        return

    pyscript.dcwb_button_stop_pressed = True

    # nlog("dcwb stop step1", True)
    service.call(domain="modbus", name="write_register", address="1034", slave="71", hub="kostalplenticoreplus", value=float_to_32bit_parts(float(0)))
    time.sleep(1)

    service.call(domain="modbus", name="write_register", address="1034", slave="71", hub="kostalplenticoreplus", value=float_to_32bit_parts(float(0)))
    service.call(domain="input_select", name="select_option", option="no", entity_id="input_select.dcwb_allow_charging")
    time.sleep(4)

    nlog("dcwb stop button done", True)
    service.call(domain="switch", name="turn_on", entity_id="switch.tesla_ble_charge_port_switch")

    pyscript.dcwb_button_stop_pressed = False
```

### Automation for closing garage


```yaml
alias: Garage button Tor auf (single)
description: ""
triggers:
  - domain: mqtt
    device_id: 5366b78e29a305a69e28c7f2dfa73aa5
    type: action
    subtype: single
    trigger: device
conditions: []
actions:
  - type: turn_on
    device_id: c928de9e4d96ad49b56e97539b94f5e3
    entity_id: 534799022a4783f26d12a5ad157c29dd
    domain: switch
  - if:
      - condition: state
        entity_id: input_select.dcwb_allow_charging
        state: "no"
      - condition: not
        conditions:
          - condition: or
            conditions:
              - condition: state
                entity_id: sensor.tesla_ble_ble_signal
                state: unavailable
              - condition: state
                entity_id: sensor.tesla_ble_ble_signal
                state: unknown
    then:
      - action: input_select.select_option
        metadata: {}
        data:
          option: "yes"
        target:
          entity_id: input_select.dcwb_allow_charging
      - action: notify.lewurm
        metadata: {}
        data:
          message: "garage button: turning on DCWB station"
    else:
      - action: notify.lewurm
        metadata: {}
        data:
          message: "garage button: not turning on DCWB station"
mode: single
```


### Virtual counters

aka. Utility Meter in Home Assistant.

* `sensor.dcwb_session_charge` -> `kostal-sbc Battery Charge Total`
* `sensor.dcwb_session_discharge` -> `kostal-pc10 Battery Discharge Total`

```yaml
alias: "dcwb: reset charge/discharge counters"
description: ""
triggers:
  - trigger: state
    entity_id:
      - input_select.dcwb_evse_state
    from: null
    to: operating
conditions: []
actions:
  - action: utility_meter.reset
    metadata: {}
    data: {}
    target:
      entity_id:
        - sensor.dcwb_session_charge
        - sensor.dcwb_session_discharge
mode: single

```

### Notification on weird inverter state

```yaml
alias: "dcwb: inverter standby-hang notification"
description: ""
triggers:
  - trigger: state
    entity_id:
      - sensor.scb_kostal_inverter_operating_state
    from: null
    to: STANDBY
    for:
      hours: 0
      minutes: 0
      seconds: 50
conditions:
  - condition: state
    entity_id: input_select.dcwb_evse_state
    state: operating
actions:
  - action: notify.lewurm
    metadata: {}
    data:
      message: >-
        dcwb: inverter stuck in standby state, needs physical interaction.  1.
        DC switch off, 2. cut DC lines, 3. AC off, 4. wait 20s, 5. AC on, 6.
        wait until booted, 7. enable DC lines, 8. enable DC switch
mode: single

```


#### setup notes


```
$ sudo setcap cap_net_raw,cap_net_admin=eip `realpath $(which python3)`
$ sudo apt install mbpoll

$ nmcli device set eth0 managed no

# dedicated vlan for PLC stuff
$ nmcli connection add type vlan con-name CCS dev eth0 id 87
$ nmcli connection modify CCS ipv4.method disabled
$ nmcli connection modify CCS ipv6.method auto

# vlan for IoT stuff
$ nmcli connection add type vlan con-name vlan44 dev eth0 id 45
$ nmcli connection modify vlan44 ipv4.method auto

$ mncli connection up vlan44
$ mncli connection up CCS

$ nmcli connection
NAME           UUID                                  TYPE      DEVICE
CCS            2dc72a2f-b215-47e3-ad34-edf7e2100a43  vlan      eth0.87
vlan44         81eac5f2-3661-4bd6-8b4d-b0b1552b61e4  vlan      eth0.44
preconfigured  3ca5d0f3-bca2-4e1c-a62f-b4e2f4c2b2a7  wifi      wlan0
lo             0a38a526-b617-43a7-a081-6827b8f96dcd  loopback  lo

$ sudo sysctl net.ipv6.conf.eth0/87.keep_addr_on_down=1
$ sudo sysctl -w net.ipv6.conf.eth0/87.accept_ra=1
```
