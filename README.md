# Team 3341's Working Competition Code:
The code currently contains the newest versions of the:
- Tele-op Swerve Code
- ~Auto Swerve Code~ (Still needs to be debugged in swerve.zak.morgan.shawn.zhu)
- Intake and Shooter Code
- ~Climber Code~ (Not sure about which version)
- PhotonVision Code

---

# Merged Branch Descriptions:
#### Code from `intakeShooter.adam.levin`:
- +Main Code from Saturday 2/17
- +Code from Jessica's Swerve branch (there are multiple?) Sunday 2/18 at 7pm
- +Modifications on Monday 2/21 at ~9pm
  + Modified Variable Names
  + Displayed RPMs as whole numbers
  + Detection for upper and lower shooter rollers hitting their PID setpoint
- +Modififcations on Wednesday 2/22 at ~1pm
  + New Joystick Mapping
  + Graphing the PID adjustment of the upper and lower shooter rollers
  + Tuned Amp RPM (shooter)
      + 700 RPM for the upper roller and 50 for the lower roller
  + Tuned Speaker RPM (shooter)
      + 3500 for the upper and lower roller
  + Tuned Intake RPM
      + ~4150 RPM or 0.8 speed for the intake rollers
#### Code from `swerve.photonvision`:
- +Photonvision tracking and data collection from 3D objects
- +TargetAprilTag Command to adjust with Swerve using the x offset and the z angle
- +Smartdashboard outputs for if the robot is adjusting, adjusted, and if the object is detected

# How to use this branch:
### If this branch contains the newest code for your subsystems
Work off of this branch and make sure you are always pulling new changes. Before you commit make sure to pull any new changes and then commit and push your changes to github via github desktop or the command line.
### If this branch DOES NOT contain the newest code for your subsystems
Check with a mentor before you merge your code and ensure that it is working properly. If they say it is okay, first make sure your newest code is commited and pushed to your personal branch that you are merging, and through the command line, use `cd` to access the folder where your repository is. 
Now, run these commands (ensure you are not connected to the robot):
1. `git status` To check the status of your PERSONAL branch and to make sure you are on it and all commits are commited and pushed. You can commit with `git commit -m "[YOUR MESSAGE HERE]"` and `git push`
2. `git pull` To check for any new changes that need to be merged into your PERSONAL repo. If there are changed, repeat step 1 again.
3. Type `git switch Working-Competition-Code` to switch branches
4. If there is an error saying the branch doesn't exist, run `git fetch` to fetch the new changes that made the branch
5. Then, once you are in the WORKING COMPETITION CODE branch, run `git fetch` and `git pull` again to ensure the branch is up to date, and run `git status` to check that you are on the right branch.
6. Running `git merge [YOUR PERSONAL BRANCH NAME]` will merge your PERSONAL branch into the WORKING COMPETITION CODE branch.
7. If any merge conflicts occur, open a file editor and look through the files with the conflicts to chose which version of code is correct. You will see HEAD which has the chnages from the competition code branch beneath it and the name of your personal branch with your changes beneath it. All of these are sepparated by =====, >>>>>, and <<<<< symbols. You should delete the changes you don't want and keep the ones you do.
8. Once that is complete, you should commit your changes and push them using the last two commands from step 1.
