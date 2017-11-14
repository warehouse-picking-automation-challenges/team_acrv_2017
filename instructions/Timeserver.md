Set router to use time.qut.edu.au as timeserver in System
NTP uses port 23 which QUT blocks. Use their timeserver as it's not blocked.

`sudo service ntp restart`

Set server in /etc/ntp.conf to router hostname
eg if router hostname is apc-router.lan
```server apc-router.lan prefer```

Update time on all client computers with command
```sudo ntpd -gqx```
-g allow first adjustment to be big
-q set time and quit
-x allow up to 600 seconds slew

Check time is correct
```ntpq -p```
Delay, offset, and jitter should be small
Refid should be an IP address, not .INIT.

As a backup, you can grab the time from another computer and set it on the current one
(As a backup, this is probably going to work first time)
sudo date -s "`ssh apc-cameras@apc-cameras date`"
