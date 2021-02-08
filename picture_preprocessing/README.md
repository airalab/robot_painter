# Picture Preprocessing Program
Kuka KR6 Drawer project (AIRA). Picture to draw preprocessor.

## How to use?

You need to have python (version 3.6.3+) and PIL on your computer. Instructions for PIL instalation could be found [here](https://pillow.readthedocs.io/en/4.3.x/installation.html). 

1. Put python script in a folder with image file
2. Create configuration file (script will do this automaticaly if **makeSampleConf** is equal to **True**)
3. Run script
4. Now you have **painting_setup.py** file in your folder! 

## Configuration file

The configuration file has .json format. This file includes parameters of a collor pallet and canvas dimentions. 

- "description" - 
- "file" - name of file with a picture to be drawed 
- "colors" - array of a data about robot's collor pallete
  - "color" - name of the color
  - "ID" - ID of the collor on a pallete
  - "R" - Red component of a color (in RGB)
  - "G" - Green component of a color (in RGB)
  - "B" - Blue component of a color (in RGB)
- "height" - height of the canvas (in m)
- "width" - width of the canvas (in m)
- "brush" - brush diameter (in m)

## Output data file

The programm creates file with list a colored points on a canvas. This data is need to make Kuka draw a picture after pixelating.The output data file includes information as:

- "test" - variable used while debug
- "paintingPoints" - list of colored points
  - "x" - X component of coordinate of the point
  - "y" - Y component of coordinate of the point
  - "R" - Red component of a color (in RGB)
  - "G" - Green component of a color (in RGB)
  - "B" - Blue component of a color (in RGB)
  
  **Remember** that coordinate frame of the picture is plased as showed:
  
  ![alt text](https://github.com/vachernov/Picture-PreProcessing/blob/master/coord_frame.png)

