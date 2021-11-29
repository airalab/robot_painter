# Systemd services

Save all files in *systemd* folder:
```shell
sudo mv * /etc/systemd/system/
```
Make scripts executable:
```
cd /etc/systemd/system/
sudo chmod a+x gaka_web.sh
sudo chmod a+x image_gen.sh
```
Then reload **systemd daemon**:
```shell
sudo systemctl daemon-reload
```

Run all services:
```shell
sudo systemctl start kuka_eki_hw_interface.service
sudo systemctl start kuka_moveit_config.service
sudo systemctl start kuka_picture_process.service
sudo systemctl start kuka_image_gen.service
sudo systemctl start gaka_web.service
sudo systemctl start mozilla_kiosk.service
```

You can make these services start on boot with `enable` option:
```shell
sudo systemctl enable kuka_eki_hw_interface.service
sudo systemctl enable kuka_moveit_config.service
sudo systemctl enable kuka_picture_process.service
sudo systemctl enable kuka_image_gen.service
sudo systemctl enable gaka_web.service
sudo systemctl enable mozilla_kiosk.service
```
