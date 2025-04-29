# Nordic_Spring_2025_Senior_Design

## Instructions (Brief)

1. Dowload GitHub Desktop, log in
2. Clone the reposity **AT OR VERY NEAR YOUR C:// DRIVE IF ON WINDOWS, AS BUILD PROBLEMS WILL OCCURE IF ANY FILE PATH NAMES ARE TOO LONG**
3. Probably is smart to make a branch before working so that we don't overwrite main. Do this in git hub desktop
4. make any chnages, re-build, program board, etc.
5. Notify other team members if you are going to push to main, but push to your branch whenever

### To Push
1. Open github desktop
2. Find the project, give a descroption and do "commit to main"
3. Then, click the push button that appears at the top. 

## Building
* Make sure to add the base config file, the extra config file, and the base devicetree overlay file too when making a new build. For example, while building the the "hello_dect" example in this repo (which, as mentioned below, has the working i2c for the bme temp sensor), selecte the
  * prj.conf for the "Base Configuration File"
  * overlay-us.conf for the "extra kconfig fragments"
  * CHANGE_TO_YOUR_BUILD_TARGET_NAME.overlay for the "base device tree overlays"

## Other Notes

* **The current hello_dect is the one that has working i2c**
* The repo is automatically set to disable build files. You will have to re-build whenever you want to test. Local build files will remain
* In rangetest, load transmitter to one board and then reciever to another. Connect reciever to your laptop and run the python script.

