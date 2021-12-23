# Systemd services

Save all files in *systemd* folder:
```shell
sudo mv * /etc/systemd/system/
```
Make scripts executable:
```shell
cd /etc/systemd/system/
sudo chmod a+x gaka_web.sh
sudo chmod a+x image_gen.sh
sudo chmod a+x ipfs_sendler.sh
sudo chmod a+x video_saver.sh
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
sudo systemctl start video_saver.service
sudo systemctl start ipfs_sendler.service
sudo systemctl start ipfs.service
```

You can make these services start on boot with `enable` option:
```shell
sudo systemctl enable kuka_eki_hw_interface.service
sudo systemctl enable kuka_moveit_config.service
sudo systemctl enable kuka_picture_process.service
sudo systemctl enable kuka_image_gen.service
sudo systemctl enable gaka_web.service
sudo systemctl enable mozilla_kiosk.service
sudo systemctl enable video_saver.service
sudo systemctl enable ipfs_sendler.service
sudo systemctl enable ipfs.service
```
