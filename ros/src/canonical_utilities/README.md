
Setting up a new robot (yarp example).

# Network configuration

- change hostname to something recognizable or you will not be able to tell one from another:
    ```
    hostnamectl set-hostname yarp5r1
    ```
    The pattern convention (hereby set) is <modelname>r<robot number> (yarp5 is the model,
    r1 is the robot with number 1) , lowercase only.
    NOTE: hostname allows only alphanumerics and hyphen (-), but hyphen cannot be used
    in ROS names for hosts, so don't use it here.
	NOTE2: maybe hyphen is allowed, seems to work sometimes.

    Also change `/etc/hosts`:
    ```
    127.0.1.1       yarp5r1 yarp5r1.local

    ```
- configure wifi to autoconnect to the robot network. In this case it is:
    ```
    SSID: yarp-net
    password: <пароль>123
    ```
    To do it, add it to the network manager list of connections by connecting to it once:

    ```
    nmcli d wifi connect yarp-net password gfhjkm123
    ```
    It will also probably disconnect you from the ethernet connection since nmcli allows
    only one active connection. It should connect to the first available interface after
    boot, which means that if the previous wi-fi connection is enabled and ethernet is 
    unplugged, it will connect to the wi-fi. If the ethernet is plugged in and the
    known wi-fi networks are down, it will connect to the ethernet. In any other case
    priorities may do stuff and this guide will not help you there.

# ROS configuration
OS needs to know where all the machines are on the network. This can be done either by specifying
IPs or hostnames. The latter is preferred since IPs tend to be dynamic. A mixed setup can also be
used if for some reason you can not use hostnames for everything.

NOTE: all IPs here are just for reference, use YOUR IPs.

- Set up remote roscore connection using either of the ROS network alternatives (hostname or ip setup). 

- Add the robot's name to `.bashrc` as an environment variable. It is needed for ROS prefixes in
    a multiagent setting:
    ```
    # ROS name for the robot
    export ROBOT=yarp5r1
    ```
    NOTE: yarp5r1 is for the robot number 1.

## ROS network alternative 1 Hostname setup

### Slave

Use this one whenever possible. 

Add to `.bashrc`:
```
# Remote master
source `rospack find canonical_utilities`/scripts/connect_as_ros_slave_name.bash my_master_name.local `hostname`.local
```
The first hostname (`my_master_name.local`) is the master's. The second is the slave's hostname (this machine's). The hostname
must be resolvable. The 'real' hostname returned by `hostname` on Ubuntu is usually NOT resolvable
unless you have a specific network setup. Avahi makes `hostname.local` resolvable in the local
network. For example, if the hostname is `myhost`, the resolvable name will be `myhost.local`. So use
the latter.

### Master

Use this one whenever possible. 

Add to `.bashrc`:
```
# Remote master
source `rospack find canonical_utilities`/scripts/connect_as_ros_master_name.bash `hostname`.local
```
The hostname is the master's. It must be resolvable. The 'real' hostname returned by `hostname` on
Ubuntu is usually NOT resolvable unless you have a specific network setup. Avahi makes 
`hostname.local` resolvable in the local network. For example, if the hostname is `myhost`, the 
resolvable name will be `myhost.local`. So use the latter.

## ROS network alternative 2 IP setup

### Slave

Add to `.bashrc`:
```
# Remote master
source `rospack find canonical_utilities`/scripts/connect_as_ros_slave.bash 192.168.1.4 192.168.1.10
```
The first IP address is the master's IP. It can probably be changed to a host name with 
some tweaking and a proper network configuration. This IP will be hardcoded in this file for
all robots in the group, so obviously the master should always be given this IP. The router's
static DHCP table can help with that (only one MAC can be bound to it). The second IP is the
slave's IP (this machine's). It probably is also dynamic, same fix can be later done as with
the master.

### Master

Add to `.bashrc`:
```
# Remote master
source `rospack find canonical_utilities`/scripts/connect_as_ros_master.bash 192.168.0.101
```
IP address is the master's IP (this machine's). It can probably be changed to a host name 
with some tweaking and a proper network configuration. This IP is hardcoded for all robots
in the group, so obviously the master should always be given this IP. The router's static
DHCP table can help with that (only one MAC can be bound to it).

# Time synchronization

## Server settings
 - Check udpades
```
sudo apt update
```
 - Install server
```
sudo apt install ntp
```

# Troubleshooting

## Network

### Pinging a .local address fails

Something like `ping wheelchair.local` fails either on the master machine, or on a slave machine.
Installing everything avahi-related (the daemon that makes machines discoverable via .local
addresses) may help (and a reboot after that):

```
sudo apt-get install avahi-daemon avahi-discover avahi-utils libnss-mdns mdns-scan
```

Also pay attention that no other machine on the network has the same name, and that both 
`/etc/hosts` and `hostname` (`/etc/hostname`) have the same name configured.
