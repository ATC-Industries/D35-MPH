# Step by Step instructions to setup Visual Studio Code and PlatformIO

Here are the step-by-step instructions to help your boss set up their system with Visual Studio Code and PlatformIO to build and upload the code from your GitHub repository to the target hardware.

## Step 1: Install Visual Studio Code (VSCode)

- Visit the official Visual Studio Code website at <https://code.visualstudio.com/>.
- Download the appropriate installer for your boss's operating system (Windows, macOS, or Linux).
- Run the installer and follow the installation prompts to complete the installation.

## Step 2: Install PlatformIO extension in VSCode

- Open Visual Studio Code.
- Click on the Extensions icon on the left sidebar (it looks like a square with smaller squares in the corner).
- Search for "PlatformIO" in the search bar.
- Click on "PlatformIO IDE" by PlatformIO in the search results.
- Click the Install button to install the PlatformIO extension. Wait for the installation to complete.

## Step 3: Clone the GitHub repository

- Install Git on the computer if it's not already installed (download from <https://git-scm.com/downloads>).
- Open a terminal/command prompt on the computer.
- Navigate to the folder where your boss wants to store the project (e.g., cd /path/to/projects/folder).
- Clone the GitHub repository using the following command:

```bash
git clone https://github.com/ATC-Industries/D35-MPH.git
```

- Wait for the cloning process to complete.

## Step 4: Open the project in VSCode

- Open Visual Studio Code.
- Click "File" in the menu, then "Open Folder..." (or "Open..." on macOS).
- Navigate to the folder where the project was cloned and select the "D35-MPH" folder.
- Click "Open" to open the project in VSCode.

## Step 5: Connect the target hardware

Connect the target hardware to the computer using the appropriate USB cable or interface.
Make sure the device drivers are installed if necessary (refer to the hardware documentation for any specific instructions).

## Step 6: Configure PlatformIO settings (if needed)

- Open the platformio.ini file in the project folder (located in the "D35-MPH" folder).
- Verify that the configuration settings match the target hardware (e.g., board type, upload port, etc.). Update the settings if needed.
- Save the changes to the platformio.ini file, if any.

## Step 7: Build and upload the code

- In Visual Studio Code, click on the PlatformIO icon on the left sidebar (it looks like an alien face).
- Under the "Project Tasks" section, click on the arrow icon next to the board name (it should match the target hardware).
- Click on "Build" to compile the code. Wait for the compilation process to complete, and make sure there are no errors.
- Click on "Upload" to upload the compiled code to the target hardware. Wait for the upload process to complete.

By following these instructions, your boss should be able to set up their system with Visual Studio Code and PlatformIO, clone your GitHub repository, and build and upload the code to the target hardware. If there are any issues, make sure to check the PlatformIO documentation (<https://docs.platformio.org/en/latest/>) and the hardware documentation for additional guidance.
