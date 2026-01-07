#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_FILE="$SCRIPT_DIR/sensett.conf"
SERVICE_FILE="/etc/systemd/system/sensett.service"
PYTHON_EXEC="/usr/bin/python3"
SCRIPT_DIR="$(dirname "$(realpath "$0")")"
HARDWARE_PY_PATH="hardware.py"
HARDWARE_PY_MODULE="hardware"

clear

print_header() {
    echo " ███████╗███████╗███╗   ██╗███████╗███████╗████████╗████████╗ "
    echo " ██╔════╝██╔════╝████╗  ██║██╔════╝██╔════╝╚══██╔══╝╚══██╔══╝ "
    echo " ███████╗█████╗  ██╔██╗ ██║███████╗█████╗     ██║      ██║    "
    echo " ╚════██║██╔══╝  ██║╚██╗██║╚════██║██╔══╝     ██║      ██║    "
    echo " ███████║███████╗██║ ╚████║███████║███████╗   ██║      ██║    "
    echo " ╚══════╝╚══════╝╚═╝  ╚═══╝╚══════╝╚══════╝   ╚═╝      ╚═╝    "
    echo "                                                               "
    echo ""
}

read_config() {
    if [ -f "$CONFIG_FILE" ]; then
        echo "Reading configuration..."
        CONFIG=$(cat "$CONFIG_FILE")
        if jq -e . >/dev/null 2>&1 <<<"$CONFIG"; then
            echo "Configuration loaded and validated."
        else
            echo "Error reading JSON file: $(jq . <<<"$CONFIG" 2>&1)"
            CONFIG='{"HARDWARE_SENSORS": [], "MQTT_SENSORS": [], "FAN": {}}'
        fi
    else
        echo "No configuration found. Creating a new one..."
        CONFIG='{"HARDWARE_SENSORS": [], "MQTT_SENSORS": [], "FAN": {}}'
    fi
}

write_config() {
    echo "Saving configuration..."
    echo "$CONFIG" | jq '.' > "$CONFIG_FILE"
    if [ $? -eq 0 ]; then
        echo "Configuration saved successfully."
    else
        echo "Error saving configuration."
    fi
}

print_current_config() {
    echo "----------------------------------------"
    echo "|        CURRENT SENSOR CONFIG         |"
    echo "----------------------------------------"
    echo "| Hardware                             |"
    echo "----------------------------------------"

    hardware_count=$(echo "$CONFIG" | jq '.HARDWARE_SENSORS | length' 2>/dev/null)
    mqtt_count=$(echo "$CONFIG" | jq '.MQTT_SENSORS | length' 2>/dev/null)

    if [[ -z "$hardware_count" || "$hardware_count" -eq 0 ]]; then
        echo "|    No hardware configured...         |"
    else
        echo "$CONFIG" | jq -r '.HARDWARE_SENSORS[] | "|   Sensor: \(.name) (\(.type)) - I2C Bus: \(.i2c_bus)"'
        if [[ "$(echo "$CONFIG" | jq '.FAN | length' 2>/dev/null)" -gt 0 ]]; then
            echo "$CONFIG" | jq -r '.FAN | "|   Fan: \(.name)\n|    Trigger Sensor: \(.trigger_sensor)\n|    Trigger Value: \(.trigger_value)\n|    Trigger on Above: \(.trigger_on_above)"'
        fi
    fi

    echo "----------------------------------------"
    echo "| MQTT Sensors                         |"
    echo "----------------------------------------"

    if [[ -z "$mqtt_count" || "$mqtt_count" -eq 0 ]]; then
        echo "|    No MQTT Sensors configured...     |"
    else
        echo "$CONFIG" | jq -r '.MQTT_SENSORS[] | "|  \(.name): \n|    Hardware Sensors: \(.hardware_sensors | join(", "))"'
    fi
    echo "----------------------------------------"
}

install_dependencies() {
    echo "Updating package lists..."
    sudo apt-get update

    echo "Installing required packages..."
    sudo apt-get install -y python3 python3-pip python3-smbus python3-paho-mqtt python3-smbus2 i2c-tools jq lsof python3-numpy python3-requests mosquitto mosquitto-clients
}

check_dependencies() {
    print_header
    echo "Checking dependencies..."
    if ! dpkg -s python3 python3-pip python3-smbus python3-smbus2 python3-paho-mqtt i2c-tools jq lsof python3-numpy python3-requests mosquitto mosquitto-clients >/dev/null 2>&1 ; then
        clear
        print_header
        echo "Dependencies are not fully installed. Would you like to install them? (y/n)"
        while true; do
            read install_deps
            case $install_deps in
                [Yy]* ) install_dependencies; break;;
                [Nn]* ) echo "Dependencies are not installed."; break;;
                * ) echo "Please answer yes (y) or no (n).";;
            esac
        done
    else
        echo "Dependencies installed & up-to-date!"
    fi
    echo ""
}

create_service() {
    local SERVICE_FILE_PATH="/etc/systemd/system/sensett.service"

    # Prefer the user who invoked sudo; fallback to current user.
    local INSTALL_USER="${SUDO_USER:-$USER}"

    # Ensure these are absolute paths when your script sets them.
    # PYTHON_EXEC="/usr/bin/python3"
    # SCRIPT_DIR="/home/whatever/sensett"

    sudo tee "$SERVICE_FILE_PATH" > /dev/null <<EOF
[Unit]
Description=Sensett Service
Wants=network-online.target
After=network-online.target
# If using a local broker, uncomment the next two lines:
# Wants=mosquitto.service
# After=mosquitto.service

[Service]
Type=simple
User=${INSTALL_USER}
WorkingDirectory=${SCRIPT_DIR}
Environment=PYTHONUNBUFFERED=1
ExecStart=${PYTHON_EXEC} ${SCRIPT_DIR}/main.py
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF

    sudo systemctl daemon-reload
    sudo systemctl enable --now sensett.service
}


check_service() {
    echo "Checking for Sensett service..."
    if ! systemctl is-active --quiet sensett.service; then
        echo "Sensett background service is not running"
        echo ""
        echo "Checking for conflicting service on port 1883..."
        
        if sudo lsof -i :1883 | grep LISTEN; then
            echo ""
            echo "Another service is running on port 1883."
            echo "Running two services on the same port may cause conflicts"
            echo "Would you like to stop this service? (y/n)"
            while true; do
                read stop_service
                case $stop_service in
                    [Yy]* )
                        service_name=$(sudo lsof -i :1883 | grep LISTEN | awk '{print $1}' | uniq)
                        sudo systemctl stop $service_name
                        echo "$service_name stopped."
                        break;;
                    [Nn]* )
                        echo ""
                        echo "Would you like to delete this service? (y/n)"
                        while true; do
                            read delete_service
                            case $delete_service in
                                [Yy]* )
                                    service_name=$(sudo lsof -i :1883 | grep LISTEN | awk '{print $1}' | uniq)
                                    sudo systemctl stop $service_name
                                    sudo systemctl disable $service_name
                                    sudo rm /etc/systemd/system/$service_name.service
                                    sudo systemctl daemon-reload
                                    echo "$service_name deleted."
                                    break;;
                                [Nn]* )
                                    echo ""
                                    echo "Service on port 1883 not modified."
                                    break;;
                                * ) echo "Please answer yes (y) or no (n).";;
                            esac
                        done
                        break;;
                    * ) echo "Please answer yes (y) or no (n).";;
                esac
            done
        else
            echo "No conflicting service on port 1883"
            echo ""
        fi
        
        echo "" 
        echo "A background service will configure Sensett to run automatically on startup"
        echo "Would you like to create the sensett service? (y/n)"
        while true; do
            read create_svc
            case $create_svc in
                [Yy]* ) create_service; break;;
                [Nn]* ) echo "Service is not created"; break;;
                * ) echo "Please answer yes (y) or no (n).";;
            esac
        done
    fi
}

get_supported_sensors() {
    echo $($PYTHON_EXEC -c "import sys; sys.path.append('$SCRIPT_DIR'); from $HARDWARE_PY_MODULE import SUPPORTED_SENSORS; print(' '.join(SUPPORTED_SENSORS.keys()))")
}

get_sensor_notes() {
    echo $($PYTHON_EXEC -c "import sys; sys.path.append('$SCRIPT_DIR'); from $HARDWARE_PY_MODULE import SUPPORTED_SENSORS; import json; print(json.dumps({key: SUPPORTED_SENSORS[key]['notes'] for key in SUPPORTED_SENSORS}))")
}

get_sensor_data_types() {
    local sensor_type=$1
    echo $($PYTHON_EXEC -c "import sys; import json; sys.path.append('$SCRIPT_DIR'); from $HARDWARE_PY_MODULE import SUPPORTED_SENSORS; print(json.dumps(SUPPORTED_SENSORS['$sensor_type']['data_types']))")
}

get_sensor_data_names() {
    local sensor_type=$1
    echo $($PYTHON_EXEC -c "import sys; import json; sys.path.append('$SCRIPT_DIR'); from $HARDWARE_PY_MODULE import SUPPORTED_SENSORS; print(json.dumps(SUPPORTED_SENSORS['$sensor_type']['data_names']))")
}

get_sensor_data_units() {
    local sensor_type=$1
    echo $($PYTHON_EXEC -c "import sys; import json; sys.path.append('$SCRIPT_DIR'); from $HARDWARE_PY_MODULE import SUPPORTED_SENSORS; print(json.dumps(SUPPORTED_SENSORS['$sensor_type']['data_units']))")
}

await_input() {
    echo ""
    echo "Press enter to continue..."
    read continue
}

SUPPORTED_SENSORS=($(get_supported_sensors))
NOTES=$(get_sensor_notes)

hw_header(){
    echo "----------------------------------------"
    echo "|          CURRENT HARDWARE            |"
    echo "----------------------------------------"

    hardware_count=$(echo "$CONFIG" | jq '.HARDWARE_SENSORS | length' 2>/dev/null)

    if [[ -z "$hardware_count" || "$hardware_count" -eq 0 ]]; then
        echo "|    No hardware configured...         |"
        echo "----------------------------------------"
    else
        echo "$CONFIG" | jq -r '.HARDWARE_SENSORS[] | "| Sensor: \(.name) (\(.type)) - I2C Bus: \(.i2c_bus)"'
        echo "----------------------------------------"
        echo ""
    fi

    echo "----------------------------------------"
    echo "|            Hardware Menu             |"
    echo "----------------------------------------"
    echo "| 1. Add Hardware Sensors              |"
    echo "| 2. Remove Hardware Sensors           |"
    echo "| 3. Back                              |"
    echo "----------------------------------------"
    echo " "
}

setup_hardware_sensors() {
    while true; do
        clear
        print_header
        hw_header

        echo "Make a Selection: "
        while true; do
            read sensor_action
            clear
            case $sensor_action in
                1)
                    clear
                    print_header
                    echo "Configuring new hardware sensor:"
                    echo ""
                    echo "Select sensor type:"
                    for index in "${!SUPPORTED_SENSORS[@]}"; do
                        echo "   $((index + 1)). ${SUPPORTED_SENSORS[$index]}"
                    done
                    echo "   $(( ${#SUPPORTED_SENSORS[@]} + 1 )). Other Sensor"
                    echo "   0. Back"
                    echo ""

                    while true; do
                        read sensor_type_index
                        if [[ "$sensor_type_index" =~ ^[0-9]+$ ]] && [ "$sensor_type_index" -ge 1 ] && [ "$sensor_type_index" -le "$(( ${#SUPPORTED_SENSORS[@]} + 1 ))" ]; then
                            if [ "$sensor_type_index" -eq "$(( ${#SUPPORTED_SENSORS[@]} + 1 ))" ]; then
                                echo ""
                                echo "Other sensors are not currently supported"
                                echo "But Sensett would love to add support for new Sensors!"
                                echo "Create an issue on https://github.com/stephen-opet/sensett/issues"
                                echo "Name your sensor and provide plenty of details - we'll get on it!"
                                await_input
                                break
                            else
                                sensor_type=${SUPPORTED_SENSORS[$((sensor_type_index - 1))]}
                                break
                            fi
                        elif [ "$sensor_type_index" -eq 0 ]; then
                            break
                        else
                            clear
                            print_header
                            echo "Configuring new hardware sensor:"
                            echo ""
                            echo "Select sensor type:"
                            for index in "${!SUPPORTED_SENSORS[@]}"; do
                                echo "   $((index + 1)). ${SUPPORTED_SENSORS[$index]}"
                            done
                            echo "   $(( ${#SUPPORTED_SENSORS[@]} + 1 )). Other Sensor"
                            echo "   0. Back"
                            echo ""
                            echo "$sensor_type_index is an invalid selection. Please choose a menu item."
                            echo ""
                        fi
                    done

                    if [ "$sensor_type_index" -eq "$(( ${#SUPPORTED_SENSORS[@]} + 1 ))" ] || [ "$sensor_type_index" -eq 0 ]; then
                        break
                    fi

                    clear
                    print_header
                    echo "Configuring new hardware sensor:"
                    echo "$sensor_type Selected"
                    echo ""

                    notes=$(echo "$NOTES" | jq -r --arg type "$sensor_type" '.[$type][]')
                    echo "Notes specific to $sensor_type hardware:"
                    echo ""
                    while IFS= read -r note; do
                        echo -e "\t$note\n"
                    done <<< "$notes"

                    await_input

                    clear
                    print_header
                    echo "Configuring new hardware sensor:"
                    echo "$sensor_type Selected"
                    echo ""

                    echo "Enter sensor name:"
                        while true; do
                            read sensor_name
                            echo ""
                            if [[ -z "$sensor_name" ]]; then
                                echo "Error: Sensor name cannot be blank. Please enter a valid name."
                            elif ! [[ "$sensor_name" =~ ^[a-zA-Z0-9]+$ ]]; then
                                echo "Error: Sensor name must contain only letters and numbers. Please enter a valid name."
                            elif [[ $(echo "$CONFIG" | jq -r --arg name "$sensor_name" '.HARDWARE_SENSORS[] | select(.name == $name) | .name') == "$sensor_name" ]]; then
                                echo "Error: A sensor with the name \"$sensor_name\" already exists. Please use a different name."
                            else
                                break
                            fi
                        done

                    clear
                    print_header
                    echo "Configuring new hardware sensor:"
                    echo "$sensor_type Selected"
                    echo "Sensor Name: $sensor_name"
                    echo ""

                    echo "Enter I2C bus number:"
                    while true; do
                        read i2c_bus
                        if [[ "$i2c_bus" =~ ^[0-9]+$ ]]; then
                            break
                        else
                            echo "Invalid I2C bus number. Please enter a valid number."
                        fi
                    done

                    clear
                    print_header
                    echo "Configuring new hardware sensor:"
                    echo "$sensor_type Selected"
                    echo "Sensor Name: $sensor_name"
                    echo "I2C Bus: $i2c_bus"
                    echo ""

                    CONFIG=$(echo "$CONFIG" | jq --arg type "$sensor_type" --arg name "$sensor_name" --arg i2c_bus "$i2c_bus" \
                        '.HARDWARE_SENSORS += [{"type": $type, "name": $name, "i2c_bus": ($i2c_bus | tonumber)}]')
                    write_config
                    echo "Hardware sensor added."
                    await_input
                    break;;
                2)
                    hardware_count=$(echo "$CONFIG" | jq '.HARDWARE_SENSORS | length' 2>/dev/null)
                    if [[ -z "$hardware_count" || "$hardware_count" -eq 0 ]]; then
                        echo ""
                        echo "No hardware sensors to remove!"
                        await_input
                    else
                        print_header
                        while true; do
                            echo "Select a sensor to remove or enter 0 to return to the menu:"
                            jq -r '.HARDWARE_SENSORS[] | .name' <<< "$CONFIG" | nl
                            echo "     0. Return to hardware menu"
                            echo ""
                            read sensor_index
                            if [[ "$sensor_index" =~ ^[0-9]+$ ]] && [ "$sensor_index" -eq 0 ]; then
                                break
                            elif [[ "$sensor_index" =~ ^[0-9]+$ ]] && [ "$sensor_index" -gt 0 ] && [ "$sensor_index" -le "$(jq '.HARDWARE_SENSORS | length' <<< "$CONFIG")" ]; then
                                sensor_name=$(jq -r --argjson index "$sensor_index" '.HARDWARE_SENSORS[$index - 1].name' <<< "$CONFIG")
                                CONFIG=$(echo "$CONFIG" | jq --arg name "$sensor_name" 'del(.HARDWARE_SENSORS[] | select(.name == $name))')
                                CONFIG=$(echo "$CONFIG" | jq --arg name "$sensor_name" '
                                    .MQTT_SENSORS |= map(
                                        .hardware_sensors |= map(select(. != $name))
                                    )')

                                write_config
                                echo "Hardware sensor removed."
                                await_input
                                break
                            else
                                clear
                                print_header
                                echo "\"$sensor_index\" is an invalid selection. Please enter a menu option."
                                echo ""
                            fi
                        done
                    fi
                    break;;
                3) return;;
                *) 
                    clear
                    print_header
                    hw_header
                    echo "\"$sensor_action\" is an invalid Entry. Please select a menu option"
                    echo "";;
            esac
        done
    done
}

mqtt_header(){
    echo "----------------------------------------"
    echo "|          CURRENT MQTT SENSORS        |"
    echo "----------------------------------------"
    if jq -e '.MQTT_SENSORS | length > 0' > /dev/null 2>&1 <<< "$CONFIG"; then
        jq -r '.MQTT_SENSORS[] | "| \(.name)\n|   Hardware: \(.hardware_sensors | join(", "))\n|"' <<< "$CONFIG"
        echo "----------------------------------------"
        echo ""
    else
        echo "|    No MQTT sensors configured...     |"
        echo "----------------------------------------"
        echo ""
    fi

    echo "----------------------------------------"
    echo "|              MQTT Menu               |"
    echo "----------------------------------------"
    echo "| 1. Add MQTT Sensor                   |"
    echo "| 2. Modify MQTT Sensor                |"
    echo "| 3. Remove MQTT Sensor                |"
    echo "| 4. Back                              |"
    echo "----------------------------------------"
    echo " "
}

setup_mqtt_sensors() {
    while true; do
        clear
        print_header
        mqtt_header
        echo "Make a Selection: "
        while true; do
            read sensor_action
            case $sensor_action in
                1)
                    clear
                    print_header
                    echo "Configuring new MQTT sensor:"
                    echo "Enter MQTT sensor name:"
                    while true; do
                        read sensor_name
                        if [[ -z "$sensor_name" ]]; then
                            echo "Error: Sensor name cannot be blank. Please enter a valid name."
                        elif ! [[ "$sensor_name" =~ ^[a-zA-Z0-9]+$ ]]; then
                            echo "Error: Sensor name must contain only letters and numbers. Please enter a valid name."
                        elif jq -e --arg name "$sensor_name" '.MQTT_SENSORS[] | select(.name == $name)' > /dev/null 2>&1 <<< "$CONFIG"; then
                            echo "Error: An MQTT sensor with the name \"$sensor_name\" already exists. Please use a different name."
                        else
                            break
                        fi
                    done

                    if jq -e '.MQTT_SENSORS | length > 0' > /dev/null 2>&1 <<< "$CONFIG"; then
                        echo "Select hardware sensors to include (enter numbers separated by space):"
                        jq -r '.HARDWARE_SENSORS[] | .name' <<< "$CONFIG" | nl
                        read -a hw_sensor_indexes

                        hw_sensors=()
                        for index in "${hw_sensor_indexes[@]}"; do
                            if [[ "$index" =~ ^[0-9]+$ ]] && [ "$index" -gt 0 ] && [ "$index" -le "$(jq '.HARDWARE_SENSORS | length' <<< "$CONFIG")" ]; then
                                hw_sensors+=("$(jq -r --argjson idx "$index" '.HARDWARE_SENSORS[$idx - 1].name' <<< "$CONFIG")")
                            else
                                echo "Invalid selection: $index"
                            fi
                        done

                        if [ ${#hw_sensors[@]} -eq 0 ]; then
                            echo "No valid hardware sensors selected."
                        else
                            hw_sensors_json=$(printf '%s\n' "${hw_sensors[@]}" | jq -R . | jq -s .)
                            MQTT_SENSORS=$(jq --arg name "$sensor_name" --argjson sensors "$hw_sensors_json" \
                                '.MQTT_SENSORS += [{"name": $name, "mqtt_topic": ("/sensor/" + $name), "hardware_sensors": $sensors}]' <<< "$CONFIG")
                            CONFIG=$MQTT_SENSORS
                            write_config
                            echo "MQTT sensor added."
                        fi
                    else
                        echo "No hardware sensors configured. Please configure hardware sensors first."
                    fi
                    break;;
                2)
                    clear
                    print_header
                    if jq -e '.MQTT_SENSORS | length > 0' > /dev/null 2>&1 <<< "$CONFIG"; then
                        while true; do
                            echo "Select an MQTT sensor to modify or enter 0 to return to the menu:"
                            jq -r '.MQTT_SENSORS[] | .name' <<< "$CONFIG" | nl
                            echo "     0. Return to MQTT menu"
                            read sensor_index
                            if [[ "$sensor_index" =~ ^[0-9]+$ ]] && [ "$sensor_index" -eq 0 ]; then
                                break
                            elif [[ "$sensor_index" =~ ^[0-9]+$ ]] && [ "$sensor_index" -gt 0 ] && [ "$sensor_index" -le "$(jq '.MQTT_SENSORS | length' <<< "$CONFIG")" ]; then
                                sensor_name=$(jq -r --argjson index "$sensor_index" '.MQTT_SENSORS[$index - 1].name' <<< "$CONFIG")
                                clear
                                print_header
                                echo "Modifying MQTT sensor: $sensor_name"
                                echo "Current hardware sensors: $(jq -r --arg name "$sensor_name" '.MQTT_SENSORS[] | select(.name == $name) | .hardware_sensors | join(", ")' <<< "$CONFIG")"
                                echo ""
                                echo "Select new hardware sensors to include (enter numbers separated by space):"
                                jq -r '.HARDWARE_SENSORS[] | .name' <<< "$CONFIG" | nl
                                read -a new_hw_sensor_indexes

                                new_hw_sensors=()
                                for index in "${new_hw_sensor_indexes[@]}"; do
                                    if [[ "$index" =~ ^[0-9]+$ ]] && [ "$index" -gt 0 ] && [ "$index" -le "$(jq '.HARDWARE_SENSORS | length' <<< "$CONFIG")" ]; then
                                        new_hw_sensors+=("$(jq -r --argjson idx "$index" '.HARDWARE_SENSORS[$idx - 1].name' <<< "$CONFIG")")
                                    else
                                        echo "Invalid selection: $index"
                                    fi
                                done

                                if [ ${#new_hw_sensors[@]} -eq 0 ]; then
                                    echo "No valid hardware sensors selected."
                                else
                                    new_hw_sensors_json=$(printf '%s\n' "${new_hw_sensors[@]}" | jq -R . | jq -s .)
                                    CONFIG=$(echo "$CONFIG" | jq --arg name "$sensor_name" --argjson sensors "$new_hw_sensors_json" '
                                        .MQTT_SENSORS |= map(
                                            if .name == $name then .hardware_sensors = $sensors else . end
                                        )')
                                    write_config
                                    echo "MQTT sensor modified."
                                fi
                                break
                            else
                                clear
                                print_header
                                echo "\"$sensor_index\" is an invalid selection. Please enter a menu item."
                                echo ""
                            fi
                        done
                    else
                        echo "No MQTT sensors to modify."
                    fi
                    break;;
                3)
                    clear
                    print_header
                    if jq -e '.MQTT_SENSORS | length > 0' > /dev/null 2>&1 <<< "$CONFIG"; then
                        while true; do
                            echo "Select an MQTT sensor to remove or enter 0 to return to the menu:"
                            jq -r '.MQTT_SENSORS[] | .name' <<< "$CONFIG" | nl
                            echo "     0. Return to MQTT menu"
                            read sensor_index
                            if [[ "$sensor_index" =~ ^[0-9]+$ ]] && [ "$sensor_index" -eq 0 ]; then
                                break
                            elif [[ "$sensor_index" =~ ^[0-9]+$ ]] && [ "$sensor_index" -gt 0 ] && [ "$sensor_index" -le "$(jq '.MQTT_SENSORS | length' <<< "$CONFIG")" ]; then
                                sensor_name=$(jq -r --argjson index "$sensor_index" '.MQTT_SENSORS[$index - 1].name' <<< "$CONFIG")
                                CONFIG=$(echo "$CONFIG" | jq --arg name "$sensor_name" 'del(.MQTT_SENSORS[] | select(.name == $name))')
                                write_config
                                echo "MQTT sensor removed."
                                break
                            else
                                clear
                                print_header
                                echo "\"$sensor_index\" is an invalid selection. Please enter a menu item."
                                echo ""
                            fi
                        done
                    else
                        echo "No MQTT sensors to remove."
                    fi
                    break;;
                4) return;;
                *) 
                    clear
                    print_header
                    mqtt_header
                    echo "\"$sensor_action\" is an invalid entry. Please select a menu item.";;
            esac
        done
    done
}

fan_header(){
    echo "----------------------------------------"
    echo "|              Setup Fan               |"
    echo "----------------------------------------"
    echo ""
    echo "----------------------------------------"
    echo "|            CURRENT FAN               |"
    echo "----------------------------------------"
    if jq -e '.FAN | length > 0' > /dev/null 2>&1 <<< "$CONFIG"; then
        jq -r '.FAN | "| Fan: \(.name)\n|    Trigger Sensor: \(.trigger_sensor)\n|    Trigger Value: \(.trigger_value)\n|    Trigger On Above: \(.trigger_on_above)"' <<< "$CONFIG"
        echo "----------------------------------------"
    else
        echo "|    No fan configured...              |"
        echo "----------------------------------------"
    fi

    echo "----------------------------------------"
    echo "|               Fan Menu               |"
    echo "----------------------------------------"
    echo "| 1. Add Fan Configuration             |"
    echo "| 2. Remove Fan Configuration          |"
    echo "| 3. Back                              |"
    echo "----------------------------------------"
    echo " "
}

setup_fan() {
    while true; do
        clear
        print_header
        fan_header

        echo "Make a Selection: "
        while true; do
            read fan_action
            case $fan_action in
                1)
                    clear
                    print_header
                    echo "Configuring new fan:"
                    echo "Enter fan name:"
                    while true; do
                        read fan_name
                        if [[ -z "$fan_name" ]]; then
                            echo "Error: Fan name cannot be blank. Please enter a valid name."
                        elif ! [[ "$fan_name" =~ ^[a-zA-Z0-9]+$ ]]; then
                            echo "Error: Fan name must contain only letters and numbers. Please enter a valid name."
                        elif jq -e --arg name "$fan_name" '.FAN | select(.name == $name)' > /dev/null 2>&1 <<< "$CONFIG"; then
                            echo "Error: A fan with the name \"$fan_name\" already exists. Please use a different name."
                        else
                            break
                        fi
                    done

                    clear
                    print_header
                    echo "Configuring new fan:"
                    echo "Fan Name: $fan_name"
                    echo ""

                    echo "Select trigger sensor:"
                    jq -r '.HARDWARE_SENSORS[] | .name' <<< "$CONFIG" | nl
                    while true; do
                        read sensor_index
                        if [[ "$sensor_index" =~ ^[0-9]+$ ]] && [ "$sensor_index" -gt 0 ] && [ "$sensor_index" -le "$(jq '.HARDWARE_SENSORS | length' <<< "$CONFIG")" ]; then
                            trigger_sensor=$(jq -r --argjson index "$sensor_index" '.HARDWARE_SENSORS[$index - 1].name' <<< "$CONFIG")
                            break
                        else
                            echo "$sensor_index is not a valid input! Please select a menu item"
                            clear
                            print_header
                            echo "Configuring new fan:"
                            echo "Fan Name: $fan_name"
                            echo ""
                            echo "Select trigger sensor:"
                            jq -r '.HARDWARE_SENSORS[] | .name' <<< "$CONFIG" | nl
                        fi
                    done

                    clear
                    print_header
                    echo "Configuring new fan:"
                    echo "Fan Name: $fan_name"
                    echo "Trigger Sensor: $trigger_sensor"
                    echo ""

                    echo "Enter trigger value:"
                    while true; do
                        read trigger_value
                        if [[ "$trigger_value" =~ ^[0-9]+$ ]]; then
                            break
                        else
                            echo "$trigger_value is not a valid input! Please enter a valid number."
                        fi
                    done

                    clear
                    print_header
                    echo "Configuring new fan:"
                    echo "Fan Name: $fan_name"
                    echo "Trigger Sensor: $trigger_sensor"
                    echo "Trigger Value: $trigger_value"
                    echo ""

                    echo "Should the fan turn on above or below the trigger value?"
                    echo "1. Above"
                    echo "2. Below"
                    while true; do
                        read trigger_on_above
                        case $trigger_on_above in
                            1)
                                trigger_on_above=true
                                break
                                ;;
                            2)
                                trigger_on_above=false
                                break
                                ;;
                            *)
                                echo "$trigger_on_above is not a valid input! Please select a menu item"
                                clear
                                print_header
                                echo "Configuring new fan:"
                                echo "Fan Name: $fan_name"
                                echo "Trigger Sensor: $trigger_sensor"
                                echo "Trigger Value: $trigger_value"
                                echo ""
                                echo "Should the fan turn on above or below the trigger value?"
                                echo "1. Above"
                                echo "2. Below"
                                ;;
                        esac
                    done

                    clear
                    print_header
                    echo "Configuring new fan:"
                    echo "Fan Name: $fan_name"
                    echo "Trigger Sensor: $trigger_sensor"
                    echo "Trigger Value: $trigger_value"
                    echo "Trigger On Above: $trigger_on_above"
                    echo ""

                    FAN_CONFIG=$(jq --arg name "$fan_name" --arg sensor "$trigger_sensor" \
                        --arg trigger_value "$trigger_value" --argjson trigger_on_above "$trigger_on_above" \
                        '.FAN = {"name": $name, "trigger_sensor": $sensor, "trigger_value": ($trigger_value | tonumber), "trigger_on_above": $trigger_on_above}' <<< "$CONFIG")
                    CONFIG=$FAN_CONFIG
                    write_config
                    echo "Fan added."
                    await_input
                    break;;
                2)
                    if jq -e '.FAN | length > 0' > /dev/null 2>&1 <<< "$CONFIG"; then
                        FAN_CONFIG=$(jq 'del(.FAN)' <<< "$CONFIG")
                        CONFIG=$FAN_CONFIG
                        write_config
                        echo "Fan configuration removed."
                    else
                        echo "No fan configured to remove."
                    fi
                    await_input
                    break;;
                3) return;;
                *) 
                    clear
                    print_header
                    fan_header
                    echo "$fan_action is not a valid input! Please select a menu item"
                    ;;
            esac
        done
    done
}

print_moonraker_config() {
    clear
    print_header

    echo "In order to import data into moonraker/mainsail, the below config data must be saved in @/printer_config/moonraker.conf"
    echo "_______________________________"
    echo ""
    echo "[mqtt]"
    echo "address: localhost"
    echo "port: 1883"
    echo ""

    # Iterate through each MQTT sensor
    for sensor_name in $(jq -r '.MQTT_SENSORS[].name' <<< "$CONFIG"); do
        sensor_name_lower=${sensor_name,,}
        echo "[sensor $sensor_name_lower]"
        echo "type: mqtt"
        echo "name: $sensor_name"
        echo "state_topic: sensor/$sensor_name_lower"
        echo "state_response_template:"
        echo "  {% set data = payload|fromjson %}"

        # Collect all set_result lines
        set_result_lines=()
        parameter_lines=()
        history_lines=()

        # Process each hardware sensor in the MQTT sensor
        for hw_sensor_name in $(jq -r --arg sensor_name "$sensor_name" '.MQTT_SENSORS[] | select(.name == $sensor_name) | .hardware_sensors[]' <<< "$CONFIG"); do
            hw_sensor=$(jq -r --arg hw_sensor_name "$hw_sensor_name" '.HARDWARE_SENSORS[] | select(.name == $hw_sensor_name)' <<< "$CONFIG")

            if [[ -z "$hw_sensor" ]]; then
                echo "Warning: Hardware sensor $hw_sensor_name not found in HARDWARE_SENSORS"
                continue
            fi

            sensor_type=$(jq -r '.type' <<< "$hw_sensor")

            data_types=$(get_sensor_data_types "$sensor_type")
            data_names=$(get_sensor_data_names "$sensor_type")
            data_units=$(get_sensor_data_units "$sensor_type")

            types_array=$(echo "$data_types" | jq -r '.[]')
            names_array=$(echo "$data_names" | jq -r '.[]')
            units_array=$(echo "$data_units" | jq -r '.[]')

            # Collect set_result lines
            i=0
            while IFS= read -r type; do

                data_name=$(echo "$names_array" | sed -n "$((i + 1))p")
                format_name=$(echo "$data_name" | tr '[:upper:]' '[:lower:]' | tr ' ' '_')
                history_lines+=("history_field_$sensor_name_lower"_"$format_name:")
                history_lines+=("  parameter=$data_name")
                history_lines+=("  strategy=basic")
                if [[ -n "$type" && -n "$data_name" ]]; then
                    set_result_lines+=("{set_result(\"$data_name\", data[\"$type\"]|float)}")
                else
                    echo "Warning: Missing type or data_name for index $i in hardware sensor $hw_sensor_name"
                fi
                i=$((i + 1))
            done <<< "$types_array"

            # Collect parameter lines
            i=0
            while IFS= read -r unit; do
                if [[ -n "$unit" ]]; then
                    data_name=$(echo "$names_array" | sed -n "$((i + 1))p")
                    if [[ -n "$data_name" ]]; then
                        parameter_lines+=("parameter_$data_name:")
                        parameter_lines+=("  units=$unit")
                    else
                        echo "Warning: Missing data_name for index $i in hardware sensor $hw_sensor_name"
                    fi
                fi
                i=$((i + 1))
            done <<< "$units_array"

        done

        # Print all set_result lines
        for line in "${set_result_lines[@]}"; do
            echo "  $line"
        done

        # Print all parameter lines
        for line in "${parameter_lines[@]}"; do
            echo "$line"
        done

        for line in "${history_lines[@]}"; do
            echo "$line"
        done

        echo ""
    done
    echo "_______________________________"
    await_input
}

mm_header(){
    echo "----------------------------------------"
    echo "|               Main Menu              |"
    echo "----------------------------------------"
    echo "| 1. Setup Hardware Sensors            |"
    echo "| 2. Setup MQTT Sensors                |"
    echo "| 3. Setup Fan                         |"
    echo "| 4. View Current Settings             |"
    echo "| 5. Generate Moonraker Config         |"
    echo "| 6. Quit                              |"
    echo "----------------------------------------"
    echo " "
}

main_menu() {
    while true; do
        clear
        print_header
        mm_header

        echo "Enter your choice:"
        while true; do
            read choice
            case $choice in
                1) setup_hardware_sensors; break;;
                2) clear; print_header; setup_mqtt_sensors; break;;
                3) setup_fan; break;;
                4) clear; print_current_config; echo "Press any key to return to the main menu..."; read -n 1; break;;
                5) print_moonraker_config; break;;
                6) 
                    write_config
                    exit;;
                *) 
                    clear
                    print_header
                    mm_header
                    echo "\"$choice\" is an invalid entry! Please select a menu option:"
                    echo ;;
            esac
        done
    done
}

check_dependencies
check_service
read_config
main_menu
