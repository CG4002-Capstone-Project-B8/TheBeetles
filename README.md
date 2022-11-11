# TheBeetles

## Library Installation
1. Install the latest version of [Arduino IDE](https://www.arduino.cc/en/software) on your laptop.
2. Clone this repository into an empty folder.
3. Open one of the `.ino` files in Arduino IDE.
4. In the IDE, under <button>Sketch</button>, hover over _Include Library_.
5. Select _Add .ZIP Library_ and a file browser will pop up for you to select a .ZIP file.
6. Navigate to the empty folder from Step 2 and choose `beetle_lib.zip`
7. Restart Arduino IDE and the Beetle library will be included in the code.

## Beetle Setup
1. First, determine which Beetle is to be connected to which hardware sensor (i.e. IMU, IR transmitter and receiver)
2. Open each `.ino` file individually such that 6 Arduino IDEs are open at a time.
3. Connect one of the Beetles to the laptop's USB port.
4. In the IDE, click on <button>Tools</button> and select the USB port connected to the Beetle (Arduino Uno).
5. Upload the code to the Beetle based on the Beetle's function. (e.g. `latest-p1-transmitter` is uploaded to the Player 1's Beetle connected to the IR transmitter)

## Making Changes to `beetle_lib`
Any changes made to code in `beetle_lib` should be followed by a change to the folder in which Arduino's includes self-written libraries. The steps are as follows:

1. Navigate to your `C:/` drive and open the folder named `Documents`.
2. Navigate to the folder named `Arduino` and open the folder named `libraries`.
3. Copy the most updated version of `beetle_lib` to that folder and overwrite the existing `beetle_lib` version.
