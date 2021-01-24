# ETRIDriving

[![Video Label](./images/screen_shot_etridriving.jpg)](https://youtu.be/cYyecNvo1Pg)
[**Click the image above for video play!!**]

## Introduction
**ETRIDriving** is the autonomous driving dataset labeled with discrete high-level driving actions of the ego-vehicle. To assemble **ETRIDriving**, we first collected raw data from sensors mounted on the vehicle (two front-facing cameras, a Lidar scanner (OUSTER, OS1-64), GPS/IMU (Spatial FOG Dual), and CAN) at 10Hz while driving. Next, the raw data was processed to be in data formats readable at common PC operating systems. After synchronizing the processed data, we assigned one of the nine actions (**Go, Turn Left/Right, U-Turn, Left/Right Lane Change, Avoidance, Left/Right Way**) to every frame in the data manually. There are 129 sequences of length 10 minutes in **ETRIDriving** and the sequences were collected by three drivers in Daejeon City during 9AM~9PM, on sunny and cloudy day, from August 2019 to December 2019.


![supple_fig1](./images/supple_fig1.png)


## News
+ [21-01-24] We open some sequences to the public. (Links for the download can be found at the end of this webpage.) The rest of the sequences will be open to the public as soon as possible.


## Characteristics
We show in the figure below the distributions of the dataset with respect to time, driving actions, and localization accuracy. Note that (c) shows the normalized histogram of the standard deviation of the localization error (centimeter).

![supple_fig2](./images/supple_fig2.png)


## Annotation (To-be-updated)
Three annotators, who had driven the vehicle, assigned one of the nine actions to each frame of the dataset. The annotators used the tool, which we developed based on PyQt, for the labeling process. The figure below shows an screen shot of the tool.

![annot1](./images/annot1.png)

The tool visualizes all the information about the current state of the vehicle as seen in the figure. 

    (a) frame index, turn signal, steering angle, vehicle speed, GPS accuracy
    (b) visualization of steering angle 
    (c) visualization of rate of change in speed (at current, previous, and previous previous frame index)
    (d) guidelines corresponding to the left side, center line, and right side of the vehicle
    (e) future trajectory up to 2 sec

The annotators were instructed to find the start and end frame indices of the actions based upon the information and their memories of driving. We also made the labeling guidelines on how to find the start and end frame indices of an action. The following is the brief description of our guideline on 'Turn'. (If you want to know details, contact d1024.choi@etri.re.kr.)

![annot2](./images/annot2.png)

Rule1) start frame index of an turn action is found by observing turn signal, steering angle, rate of change in speed, future trajectory, and blue guidelines. Around the start frame index, it is usually found that (see the upper-row of the figure)

    (1) turn signal turns on
    (2) rate of change in speed shows an increasing or decreasing pattern in time
    (3) center line of the vehicle begins to deviate from the center of the lane

Rule2) end frame index of an turn action corresponds to the index where the three blue guidelines are all inside the target lane (see the bottom-row of the figure)


## Dataset Folder Structure
The sequences are named with 4-digits numbers (from 0000 to 0131). The raw data corresponding to a sequence is stored in a folder named after the sequence number. The figure below shows the folder structure for **0019**.

![supple_fig4](./images/supple_fig4.png)

In **00** and **01**, there are front-facing left and right camera image files, respectively. The images were preprocessed to remove lens distortions before they were saved in PNG format of size 1280x360x3. The files are named with 8-digits numbers.

In **bin**, there are bin files storing point clouds from LIDAR scanner. The files are named with 8-digits numbers.

In **label**, there are two csv files, one for extrinsic calibration parameters (0019_cam0_extcalib.csv) and the other for synchronization, GPS/IMU info, CAN info, and labels (0019_drvsts.csv). The figure below shows the elements of 0019_drvsts.csv.

![supple_fig5](./images/supple_fig5.png)


## Development Kit
We share some codes written in python for loading and visualizing the data in **ETRIDriving**. You can run an example code by
```sh
$ python example.py
```

We show some visualization examples in the figure below.
![supple_fig3](./images/supple_fig3.png)

## Download (To-be-updated)
For sharing the sequences in the dataset, we have been applying some image processings (bluring face and license plate) to the front-view images in the sequences. 


