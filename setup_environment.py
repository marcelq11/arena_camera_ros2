import os
import platform
import subprocess


def main():
    # Detect the operating system
    system = platform.system()

    # Define the script names
    linux_script = "setup_environment.sh"
    windows_script = "setup_environment_win.bat"

    try:
        if system == "Linux":
            print("Detected Linux system. Running the .sh script.")
            subprocess.run(["bash", linux_script], check=True)
        elif system == "Windows":
            print("Detected Windows system. Running the .bat script.")
            subprocess.run([windows_script], shell=True, check=True)
        else:
            print(f"Unsupported operating system: {system}")
    except subprocess.CalledProcessError as e:
        print(f"Error occurred while executing the script: {e}")


if __name__ == "__main__":
    main()
