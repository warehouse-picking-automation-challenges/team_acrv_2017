# Reset DHCP
sudo dhclient e <tab> (probably either eth0, enp0 etc.)

# To find the arm
sudo arp-scan --interface=eth0 <or whatever your interface is> --localnet
ssh to it by IP and run the above reset DHCP command.
An easy way to see if you're sshing to the right IP is  that it works because the keys are all configured.

Check the time sync.
