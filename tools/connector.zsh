ARGS=`getopt -a -o hs:n:p:i: -l help,connect_srv:,wifi_name:,wifi_password:,provider_ip:, -- "$@"`

help_info() {
    echo "
usage: connector [-h] / [-s xxs -n xxn -p xxp -i xxi]

Request connection service.

optional arguments:
    -h, --help            show this help message and exit
    -s, --connect_srv     Current connection service name, usually in the following format:
                            /<namespace>/connect
                            Usually namespace is in the following format:'user_macaddress'
    -n, --wifi_name       The name of the currently connected wifi
    -p, --wifi_password   The password of the currently connected wifi
    -i, --provider_ip     The provider ip of the currently connected wifi

example:
    1. connector.zsh -s /robot_9d_51_e5_a4_b4_74/connect -n Redmi_41A3 -p admin123 -i 192.168.0.1
    2. connector.zsh --connect_srv /robot_9d_51_e5_a4_b4_74/connect --wifi_name Redmi_41A3 --wifi_password admin123 --provider_ip 192.168.0.1
    3. connector.zsh --connect_srv=/robot_9d_51_e5_a4_b4_74/connect --wifi_name=Redmi_41A3 --wifi_password=admin123 --provider_ip=192.168.0.1
    4. connector.zsh -s /robot_9d_51_e5_a4_b4_74/connect --wifi_name=Redmi_41A3 --wifi_password=admin123 --provider_ip=192.168.0.1
    "
}

judge_parameter() {
    key=$1
    value=`eval echo '$'"$key"`
    if [ -z "$value" ]; then
        echo "Please enter the $key parameter."
        help_info
        exit
    fi
}

[ $? -ne 0 ] && help
#set -- "${ARGS}"
eval set -- "${ARGS}"
while true
do
    case "$1" in
    -h|--help)
        help_info
        ;;
    -s|--connect_srv)
        connect_srv="$2"
        shift
        ;;
    -n|--wifi_name)
        wifi_name="$2"
        shift
        ;;
    -p|--wifi_password)
        wifi_password="$2"
        shift
        ;;
    -i|--provider_ip)
        provider_ip="$2"
        shift
        ;;
    --)
        shift
        break
        ;;
    esac
shift
done

judge_parameter connect_srv
judge_parameter wifi_name
judge_parameter wifi_password
judge_parameter provider_ip

ros2 service call $connect_srv protocol/srv/Connector "wifi_name: '$wifi_name'
wifi_password: '$wifi_password'
provider_ip: '$provider_ip'
"
