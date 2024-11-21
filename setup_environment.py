import os
import platform
import pyautogui
import subprocess
import time


def main():
    # Detect the operating system
    system = platform.system()
    try:
        if system == "Linux":
            print("Detected Linux system. Running the .sh script.")
            time.sleep(2)
            pyautogui.typewrite('./setup_environment.sh\n', interval=0.1)
        elif system == "Windows":
            print("Detected Windows system. Running the .bat script.")
            time.sleep(2)
            pyautogui.typewrite('setup_environment_win.bat\n', interval=0.1)
        else:
            print(f"Unsupported operating system: {system}")
    except subprocess.CalledProcessError as e:
        print(f"Error occurred while executing the script: {e}")


if __name__ == "__main__":
    main()
