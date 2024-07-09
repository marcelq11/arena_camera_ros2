# camera_gui.py
import tkinter as tk
import yaml
import os
from tkinter import messagebox


class CameraGUI:
    def __init__(self, update_callback, error_func, parameters_path=''):
        #Some predefined parameters
        self.pixel_format = 'bayer_rggb8'
        self.gain = 5.0
        self.exposure = 10000.0
        self.gamma = 1.0
        self.width = 2448
        self.height = 2048
        self.recording = False

        self.parameters_path = parameters_path
        self.error_callback = error_func
        self.update_callback = update_callback
        self.load_parameters()

        self.gamma_step = 0.1
        self.gain_step = 1.0
        self.exposure_step = 1000.0
        self.pixel_format_list = ["rgb8", "rgba8", "rgb16", "rgba16", "bgr8", "bgra8", "bgr16", "bgra16",
                                  "mono8", "mono16", "bayer_rggb8", "bayer_bggr8", "bayer_gbrg8", "bayer_grbg8",
                                  "bayer_rggb16", "bayer_bggr16", "bayer_gbrg16", "bayer_grbg16", "yuv422"]

        self.root = tk.Tk()
        self.root.title("Camera Settings")
        self.root.geometry('550x150+500+500')

        tk.Label(self.root, text="Gain").grid(row=0, column=1)
        self.gain_entry = tk.Entry(self.root)
        self.gain_entry.grid(row=1, column=1)
        self.gain_entry.config(width=7)
        self.gain_entry.insert(0, str(self.gain))

        self.increase_Gain_button = tk.Button(self.root, text="+",
                                              command=lambda: self.change_param("gain", self.gain_step))
        self.increase_Gain_button.grid(row=1, column=2)
        self.decrease_Gain_button = tk.Button(self.root, text="-",
                                              command=lambda: self.change_param("gain", -self.gain_step))
        self.decrease_Gain_button.grid(row=1, column=0)

        tk.Label(self.root, text="Exposure").grid(row=0, column=4)
        self.exposure_entry = tk.Entry(self.root)
        self.exposure_entry.grid(row=1, column=4)
        self.exposure_entry.config(width=7)
        self.exposure_entry.insert(0, str(self.exposure))

        self.increase_Exposure_button = tk.Button(self.root, text="+",
                                                  command=lambda: self.change_param("exposure", self.exposure_step))
        self.increase_Exposure_button.grid(row=1, column=5)
        self.decrease_Exposure_button = tk.Button(self.root, text="-",
                                                  command=lambda: self.change_param("exposure", -self.exposure_step))
        self.decrease_Exposure_button.grid(row=1, column=3)

        tk.Label(self.root, text="Gamma").grid(row=0, column=7)
        self.gamma_entry = tk.Entry(self.root)
        self.gamma_entry.grid(row=1, column=7)
        self.gamma_entry.config(width=7)
        self.gamma_entry.insert(0, str(self.gamma))

        self.increase_Gamma_button = tk.Button(self.root, text="+",
                                               command=lambda: self.change_param("gamma", self.gamma_step))
        self.increase_Gamma_button.grid(row=1, column=8)
        self.decrease_Gamma_button = tk.Button(self.root, text="-",
                                               command=lambda: self.change_param("gamma", -self.gamma_step))
        self.decrease_Gamma_button.grid(row=1, column=6)

        self.pixel_format_var = tk.StringVar(self.root)
        self.pixel_format_var.set(self.pixel_format)
        tk.Label(self.root, text="Pixel Format").grid(row=2, column=7)
        self.pixel_format_menu = tk.OptionMenu(self.root, self.pixel_format_var, *self.pixel_format_list)
        self.pixel_format_menu.grid(row=3, column=7)

        tk.Label(self.root, text="Width").grid(row=2, column=1)
        self.width_entry = tk.Entry(self.root)
        self.width_entry.grid(row=3, column=1)
        self.width_entry.config(width=7)
        self.width_entry.insert(0, str(self.width))

        tk.Label(self.root, text="Height").grid(row=2, column=4)
        self.height_entry = tk.Entry(self.root)
        self.height_entry.grid(row=3, column=4)
        self.height_entry.config(width=7)
        self.height_entry.insert(0, str(self.height))

        self.recording_button = tk.Button(self.root, text="Start Recording", command=self.toggle_recording)
        self.recording_button.grid(row=4, column=1, columnspan=3)
        self.recording_button.config(bg='green')
        self.recording_button.config(activebackground=self.recording_button.cget('background'))


        tk.Button(self.root, text="Update Settings", command=update_callback).grid(row=4, column=4)
        self.update_entries()

    def toggle_recording(self):
        if self.recording:
            self.recording = False
            self.recording_button.config(text="Start Recording")
            self.recording_button.config(bg='green')
        else:
            self.recording = True
            self.recording_button.config(text="Stop Recording")
            self.recording_button.config(bg='red')
        self.recording_button.config(activebackground=self.recording_button.cget('background'))
        self.update_callback()


    def load_parameters_from_file(self):
        if not os.path.exists(self.parameters_path):
            self.error_callback(f"Plik z parametrami o ścieżce {self.parameters_path} nie istnieje.\n"
                                f"Utwórz plik z parametrami lub podaj poprawną ścieżkę.\n"
                                f"W przypadku braku pliku z parametrami zostaną użyte domyślne wartości.")
            return None

        try:
            with open(self.parameters_path, 'r') as file:
                camera_parameters = yaml.load(file, Loader=yaml.FullLoader)
                return camera_parameters
        except Exception as e:
            self.error_callback(f"Błąd podczas ładowania pliku z parametrami: {e}")
            return None

    def load_parameters(self):
        parameters = self.load_parameters_from_file()
        if parameters is not None:
            camera_parameters = parameters['arena_camera_node']['ros__parameters']
            self.gain = camera_parameters['gain']
            self.exposure = camera_parameters['exposure_time']
            self.gamma = camera_parameters['gamma']
            self.pixel_format = camera_parameters['pixelformat']
            self.width = camera_parameters['width']
            self.height = camera_parameters['height']

    def save_parameters(self):
        parameters = self.load_parameters_from_file()
        if parameters is None:
            return
        camera_parameters = parameters['arena_camera_node']['ros__parameters']
        camera_parameters['gain'] = self.gain
        camera_parameters['exposure_time'] = self.exposure
        camera_parameters['gamma'] = self.gamma
        camera_parameters['pixelformat'] = self.pixel_format
        camera_parameters['width'] = self.width
        camera_parameters['height'] = self.height
        with open(self.parameters_path, 'w') as file:
            yaml.dump(parameters, file)

    def run(self):
        self.root.mainloop()

    def update_entries(self):
        self.gain_entry.delete(0, tk.END)
        self.gain_entry.insert(0, str(self.gain))

        self.exposure_entry.delete(0, tk.END)
        self.exposure_entry.insert(0, str(self.exposure))

        self.gamma_entry.delete(0, tk.END)
        self.gamma_entry.insert(0, str(self.gamma))

        self.pixel_format_var.set(self.pixel_format)

        self.width_entry.delete(0, tk.END)
        self.width_entry.insert(0, str(self.width))

        self.height_entry.delete(0, tk.END)
        self.height_entry.insert(0, str(self.height))

    def change_param(self, param_name, value):
        current_value = getattr(self, param_name)
        new_value = current_value + value
        setattr(self, param_name, new_value)
        self.save_parameters()
        self.update_entries()
        self.update_callback()

    def check_is_restart_needed(self, width, height, pixel_format):
        if (self.pixel_format != pixel_format or
                self.width != width or self.height != height):
            return True
        return False

    def handle_parameters(self):
        exposure, gamma, gain, width, height, pixel_format = self.get_parameters()
        if exposure <= 50:
            exposure = 50.0
        elif exposure >= 47000:
            exposure = 47000.0
        if gamma <= 0.2:
            gamma = 0.2
        elif gamma >= 1.9:
            gamma = 1.9
        if gain < 0:
            gain = 0.0
        elif gain >= 48:
            gain = 48.0
        if width <= 200:
            width = self.width
        elif width >= 2448:
            width = 2448
        if height <= 200:
            height = self.height
        elif height >= 2048:
            height = 2048
        self.exposure = exposure
        self.gamma = gamma
        self.gain = gain
        if self.check_is_restart_needed(width, height, pixel_format):
            answer = messagebox.askquestion("Restart",
                                            "Zmiana width/height/pixel format wymaga restartu. Czy kontynuować?")
            if answer == 'yes':
                self.pixel_format = pixel_format
                self.width = width
                self.height = height
        self.update_entries()

    def get_parameters(self):
        exposure = float(self.exposure_entry.get())
        gamma = float(self.gamma_entry.get())
        gain = float(self.gain_entry.get())
        pixel_format = str(self.pixel_format_var.get())
        width = int(self.width_entry.get())
        height = int(self.height_entry.get())
        return exposure, gamma, gain, width, height, pixel_format

    def return_params(self):
        self.handle_parameters()
        return self.gain, self.exposure, self.gamma, self.pixel_format, self.width, self.height, self.recording
