# Converting SDF files to URDF for EZ-RASSOR  

- Ensure that pysdf (in the packages folder) gets installed with `catkin_make`. You will know it installed correctly if you see it on your workspace.  
- Once that is done, run the following set of commands in the bash terminal:  
  ```bash
  roscd ez_rassor_description/models;
  rosrun pysdf sdf2urdf.py model.sdf model.urdf;
  ```
  - If it worked correctly, the terminal will display the outputted urdf file. 
  - If it did not work correctly, you will have to manually copy the pysdf folder to your `.workspace/src` folder, and then run catkin_make. Then repeat the terminal commands above and the conversion should succeed. 