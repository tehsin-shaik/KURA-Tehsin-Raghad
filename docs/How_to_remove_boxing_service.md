## How to Uninstall boxing service


#### 1. Stop the service

```

sudo systemctl stop boxing.service

```

#### 2. Prevent It From Starting on Boot

```

sudo systemctl disable boxing.service

```


#### 3. Reload systemd

```
sudo systemctl daemon-reload

```


#### 4. Completely Remove the Service

```

sudo rm /etc/systemd/system/boxing.service

```

#### Make sure boxing service is completely removed

1. check the current status:

```
systemctl status boxing.service

```

2. Check if enabled:

```
systemctl is-enabled boxing.service

```

3. Check if running:

```
systemctl is-active boxing.service

```



