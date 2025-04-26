#!/bin/bash
set -x

URL=`cd ~/private/pyPLC && python3 configmodule.py homeassistant_url`
TOKEN=`cd ~/private/pyPLC && python3 configmodule.py homeassistant_token`
PLCMAC='28:EE:52:E3:4E:FF'

while sleep 2; do
    ctrl=`curl -H "Authorization: Bearer $TOKEN" -H "Content-Type: application/json" $URL/api/states/input_select.dcwb_allow_charging | jq '.state == "yes"'`
    if test xtrue '==' x"$ctrl"; then
        (cd ~/private/open-plc-utils && sudo ./plc/plctool -ieth0 -R $PLCMAC ) &
        python break-pp.py
        (cd ~/private/pyPLC && bash -x ./starter.sh)
        (cd ~/private/open-plc-utils && sudo ./plc/plctool -ieth0 -R $PLCMAC ) &
        sleep 10
        # (cd ~/private/open-plc-utils && sudo ./plc/plctool -ieth0 -P ../evse-.pib EC:08:6B:8B:DC:D6) &
        # sleep 10
        #
        # trigger home assistant to turn off/on AC plug for wallbox LV DC supply.
        ctrl=`curl -H "Authorization: Bearer $TOKEN" -H "Content-Type: application/json" $URL/api/states/input_select.dcwb_allow_charging | jq '.state == "yes"'`
        if test xtrue '==' x"$ctrl"; then
            curl -H "Authorization: Bearer $TOKEN" -H "Content-Type: application/json" $URL/api/services/input_select/select_option -d '{ "entity_id": "input_select.dcwb_allow_charging", "option": "please_restart_yes" }'
        else
            curl -H "Authorization: Bearer $TOKEN" -H "Content-Type: application/json" $URL/api/services/input_select/select_option -d '{ "entity_id": "input_select.dcwb_allow_charging", "option": "please_restart_no" }'
        fi
        sudo shutdown -h now
    fi

    ctrl=`curl -H "Authorization: Bearer $TOKEN" -H "Content-Type: application/json" $URL/api/states/input_select.dcwb_allow_charging | jq '.state == "please_shutdown_yes"'`
    if test xtrue '==' x"$ctrl"; then
        curl -H "Authorization: Bearer $TOKEN" -H "Content-Type: application/json" $URL/api/services/input_select/select_option -d '{ "entity_id": "input_select.dcwb_allow_charging", "option": "please_shutdown_yes" }'
        sudo shutdown -h now
    fi
    ctrl=`curl -H "Authorization: Bearer $TOKEN" -H "Content-Type: application/json" $URL/api/states/input_select.dcwb_allow_charging | jq '.state == "please_shutdown_no"'`
    if test xtrue '==' x"$ctrl"; then
        curl -H "Authorization: Bearer $TOKEN" -H "Content-Type: application/json" $URL/api/services/input_select/select_option -d '{ "entity_id": "input_select.dcwb_allow_charging", "option": "please_shutdown_no" }'
        sudo shutdown -h now
    fi
done
