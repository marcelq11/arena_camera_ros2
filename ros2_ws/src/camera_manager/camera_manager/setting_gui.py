import tkinter as tk
from tkinter import ttk
import threading
from tkinter import messagebox

delay = 0.75
timer = None

class CameraGUI:
    def __init__(self, update_callback, error_func, parameters_path=''):
        self.root = tk.Tk()
        self.root.title("Camera Manager")
        self.root.geometry('620x400+500+500')

        self.notebook = ttk.Notebook(self.root)

        self.camera_settings_tab = ttk.Frame(self.notebook)
        self.system_manager_tab = ttk.Frame(self.notebook)

        self.notebook.add(self.system_manager_tab, text="System Manager")
        self.notebook.add(self.camera_settings_tab, text="Camera Settings")

        self.notebook.pack(expand=1, fill='both')

        self.create_camera_settings_tab()

        self.create_system_manager_tab()

        self.load_parameters()

        self.update_callback = update_callback
        self.error_func = error_func

        self.parameters_path = parameters_path

        self.restart_system = False
        self.system_started = False
        self.recording = False
        self.accepted = False


        ([self.exposure_limit_lower, self.exposure_limit_upper], [self.gain_limit_lower, self.gain_limit_upper],
         self.brightness, [self.roi_width, self.roi_height, self.roi_offset_x, self.roi_offset_y],
         [self.width, self.height]) = self.load_parameters()


    def create_camera_settings_tab(self):
        tk.Label(self.camera_settings_tab, text="       Auto Exposure Time Limits:").grid(row=0, column=1, padx=10, pady=5)
        tk.Label(self.camera_settings_tab, text="Upper:").grid(row=1, column=2, padx=10, pady=5,
                                                                                   sticky="w")
        tk.Label(self.camera_settings_tab, text="Lower:").grid(row=1, column=0, padx=10, pady=5,
                                                                                   sticky="w")

        self.exposure_limit_lower_slider = tk.Scale(self.camera_settings_tab, from_=30, to=3000, orient=tk.HORIZONTAL,
                                                    command=self.update_lower_exposure_limit)
        self.exposure_limit_lower_slider.grid(row=1, column=1, columnspan=1, padx=10, pady=5)

        self.exposure_limit_upper_slider = tk.Scale(self.camera_settings_tab, from_=1000, to=30000,
                                                    orient=tk.HORIZONTAL, command=self.update_upper_exposure_limit)
        self.exposure_limit_upper_slider.grid(row=1, column=3, columnspan=3, padx=10, pady=5)

        tk.Label(self.camera_settings_tab, text="       Auto Gain Limits:").grid(row=2, column=1, padx=10,
                                                                                          pady=5)
        tk.Label(self.camera_settings_tab, text="Upper:").grid(row=3, column=2, padx=10, pady=5,
                                                               sticky="w")
        tk.Label(self.camera_settings_tab, text="Lower:").grid(row=3, column=0, padx=10, pady=5,
                                                               sticky="w")

        self.gain_limit_lower_slider = tk.Scale(self.camera_settings_tab, from_=0, to=30, orient=tk.HORIZONTAL, command=self.update_lower_gain_limit)
        self.gain_limit_lower_slider.grid(row=3, column=1, columnspan=1, padx=10, pady=5)

        self.gain_limit_upper_slider = tk.Scale(self.camera_settings_tab, from_=0, to=30, orient=tk.HORIZONTAL, command=self.update_upper_gain_limit)
        self.gain_limit_upper_slider.grid(row=3, column=3, columnspan=3, padx=10, pady=5)

        tk.Label(self.camera_settings_tab, text="Target Brightness:").grid(row=4, column=0, padx=10, pady=5, sticky="w")
        self.brightness_slider = tk.Scale(self.camera_settings_tab, from_=0, to=120, orient=tk.HORIZONTAL, command=self.update_brightness)
        self.brightness_slider.grid(row=4, column=1, columnspan=3, padx=10, pady=5)

        tk.Label(self.camera_settings_tab, text="Image Width:").grid(row=5, column=0, padx=10, pady=5, sticky="w")
        self.width_entry = tk.Entry(self.camera_settings_tab, width=10)
        self.width_entry.grid(row=5, column=1, padx=10, pady=5)
        self.width_entry.bind("<FocusOut>", lambda event: self.update_image_width(self.width_entry.get()))

        tk.Label(self.camera_settings_tab, text="Image Height:").grid(row=5, column=2, padx=10, pady=5, sticky="w")
        self.height_entry = tk.Entry(self.camera_settings_tab, width=10)
        self.height_entry.grid(row=5, column=3, padx=10, pady=5)
        self.height_entry.bind("<FocusOut>", lambda event: self.update_image_height(self.height_entry.get()))

        tk.Label(self.camera_settings_tab, text="ROI Width:").grid(row=6, column=0, padx=10, pady=5, sticky="w")
        self.roi_width_entry = tk.Entry(self.camera_settings_tab, width=10)
        self.roi_width_entry.grid(row=6, column=1, padx=10, pady=5)
        self.roi_width_entry.bind("<FocusOut>", lambda event: self.update_roi_width(self.roi_width_entry.get()))

        tk.Label(self.camera_settings_tab, text="ROI Height:").grid(row=6, column=2, padx=10, pady=5, sticky="w")
        self.roi_height_entry = tk.Entry(self.camera_settings_tab, width=10)
        self.roi_height_entry.grid(row=6, column=3, padx=10, pady=5)
        self.roi_height_entry.bind("<FocusOut>", lambda event: self.update_roi_height(self.roi_height_entry.get()))

        tk.Label(self.camera_settings_tab, text="ROI Offset X:").grid(row=7, column=0, padx=10, pady=5, sticky="w")
        self.roi_offset_x_entry = tk.Entry(self.camera_settings_tab, width=10)
        self.roi_offset_x_entry.grid(row=7, column=1, padx=10, pady=5)
        self.roi_offset_x_entry.bind("<FocusOut>",
                                     lambda event: self.update_roi_offset_x(self.roi_offset_x_entry.get()))

        tk.Label(self.camera_settings_tab, text="ROI Offset Y:").grid(row=7, column=2, padx=10, pady=5, sticky="w")
        self.roi_offset_y_entry = tk.Entry(self.camera_settings_tab, width=10)
        self.roi_offset_y_entry.grid(row=7, column=3, padx=10, pady=5)
        self.roi_offset_y_entry.bind("<FocusOut>",
                                     lambda event: self.update_roi_offset_y(self.roi_offset_y_entry.get()))

        tk.Button(self.camera_settings_tab, text="Update Settings", command=self.update_settings).grid(row=8, column=0, columnspan=4, pady=10)

    def update_lower_exposure_limit(self, val):
        lower_value = int(val)
        if lower_value >= self.exposure_limit_upper_slider.get():
            self.exposure_limit_upper_slider.set(lower_value)
        self.update_settings()

    def update_upper_exposure_limit(self, val):
        upper_value = int(val)
        if upper_value <= self.exposure_limit_lower_slider.get():
            self.exposure_limit_lower_slider.set(upper_value)
        self.update_settings()

    def update_lower_gain_limit(self, val):
        lower_gain = int(val)
        if lower_gain >= self.gain_limit_upper_slider.get():
            self.gain_limit_upper_slider.set(lower_gain)
        self.update_settings()

    def update_upper_gain_limit(self, val):
        upper_gain = int(val)
        if upper_gain <= self.gain_limit_lower_slider.get():
            self.gain_limit_lower_slider.set(upper_gain)
        self.update_settings()

    def update_brightness(self, val):
        self.update_settings()

    def update_image_width(self, val):
        self.restart_system = True
        width = abs(int(val))
        max_width = 2448
        if width > max_width:
            self.width_entry.delete(0, tk.END)
            self.width_entry.insert(0, str(max_width))
        if int(self.roi_width_entry.get()) + int(self.roi_offset_x_entry.get()) > width:
            if int(self.roi_width_entry.get()) > width:
                self.roi_width_entry.delete(0, tk.END)
                self.roi_width_entry.insert(0, str(width))
                self.roi_offset_x_entry.delete(0, tk.END)
                self.roi_offset_x_entry.insert(0, '0')
            else:
                self.roi_offset_x_entry.delete(0, tk.END)
                self.roi_offset_x_entry.insert(0, str(width - int(self.roi_width_entry.get())))

    def update_image_height(self, val):
        self.restart_system = True
        height = abs(int(val))
        max_height = 2048
        if height > max_height:
            self.height_entry.delete(0, tk.END)
            self.height_entry.insert(0, str(max_height))
        if int(self.roi_height_entry.get()) + int(self.roi_offset_y_entry.get()) > height:
            if int(self.roi_height_entry.get()) > height:
                self.roi_height_entry.delete(0, tk.END)
                self.roi_height_entry.insert(0, str(height))
                self.roi_offset_y_entry.delete(0, tk.END)
                self.roi_offset_y_entry.insert(0, '0')
            else:
                self.roi_offset_y_entry.delete(0, tk.END)
                self.roi_offset_y_entry.insert(0, str(height - int(self.roi_height_entry.get())))

    def update_roi_width(self, val):
        self.restart_system = True
        width = int(val)
        max_width = int(self.width_entry.get())
        if width > max_width:
            self.roi_width_entry.delete(0, tk.END)
            self.roi_width_entry.insert(0, str(max_width))
        elif width > max_width - int(self.roi_offset_x_entry.get()):
            self.roi_offset_x_entry.delete(0, tk.END)
            self.roi_offset_x_entry.insert(0, str(max_width - width))

    def update_roi_height(self, val):
        self.restart_system = True
        height = int(val)
        max_height = int(self.height_entry.get())
        if height > max_height:
            self.roi_height_entry.delete(0, tk.END)
            self.roi_height_entry.insert(0, str(max_height))
        elif height > max_height - int(self.roi_offset_y_entry.get()):
            self.roi_offset_y_entry.delete(0, tk.END)
            self.roi_offset_y_entry.insert(0, str(max_height - height))

    def update_roi_offset_x(self, val):
        self.restart_system = True
        offset_x = int(val)
        max_offset_x = int(self.width_entry.get())
        if offset_x > max_offset_x:
            self.roi_offset_x_entry.delete(0, tk.END)
            self.roi_offset_x_entry.insert(0, str(max_offset_x))
        elif offset_x > max_offset_x - int(self.roi_width_entry.get()):
            self.roi_width_entry.delete(0, tk.END)
            self.roi_width_entry.insert(0, str(max_offset_x - offset_x))

    def update_roi_offset_y(self, val):
        self.restart_system = True
        offset_y = int(val)
        max_offset_y = int(self.height_entry.get())
        if offset_y > max_offset_y:
            self.roi_offset_y_entry.delete(0, tk.END)
            self.roi_offset_y_entry.insert(0, str(max_offset_y))
        elif offset_y > max_offset_y - int(self.roi_height_entry.get()):
            self.roi_height_entry.delete(0, tk.END)
            self.roi_height_entry.insert(0, str(max_offset_y - offset_y))

#-------------------------------------------------------------------------
    def create_system_manager_tab(self):
        tk.Button(self.system_manager_tab, text="Start System", command=self.start_system).grid(row=0, column=0,
                                                                                                padx=10, pady=10)

        tk.Button(self.system_manager_tab, text="Stop System", command=self.stop_system).grid(row=1, column=0, padx=10,
                                                                                              pady=10)

    def update_settings(self):
        self.exposure_limit_lower = self.exposure_limit_lower_slider.get()
        self.exposure_limit_upper = self.exposure_limit_upper_slider.get()
        self.gain_limit_lower = self.gain_limit_lower_slider.get()
        self.gain_limit_upper = self.gain_limit_upper_slider.get()
        self.brightness = self.brightness_slider.get()
        if self.restart_system:
            answer = messagebox.askquestion("Restart",
                                            "Zmiana width/height/roi wymaga restartu. Czy kontynuować?")
            if answer == 'yes':
                self.roi_width = int(self.roi_width_entry.get())
                self.roi_height = int(self.roi_height_entry.get())
                self.roi_offset_x = int(self.roi_offset_x_entry.get())
                self.roi_offset_y = int(self.roi_offset_y_entry.get())
                self.width = int(self.width_entry.get())
                self.height = int(self.height_entry.get())
                self.accepted = False
                self.restart_system = False
                self.update_callback()
                return
            elif answer == 'no':
                self.roi_width_entry.delete(0, tk.END)
                self.roi_width_entry.insert(0, str(self.roi_width))
                self.roi_height_entry.delete(0, tk.END)
                self.roi_height_entry.insert(0, str(self.roi_height))
                self.roi_offset_x_entry.delete(0, tk.END)
                self.roi_offset_x_entry.insert(0, str(self.roi_offset_x))
                self.roi_offset_y_entry.delete(0, tk.END)
                self.roi_offset_y_entry.insert(0, str(self.roi_offset_y))
                self.width_entry.delete(0, tk.END)
                self.width_entry.insert(0, str(self.width))
                self.height_entry.delete(0, tk.END)
                self.height_entry.insert(0, str(self.height))
                self.restart_system = False
                return
        global timer
        if timer is not None:
            timer.cancel()
        timer = threading.Timer(delay, self.update_callback)
        timer.start()

    def load_parameters(self):
        exposure_limits = [50, 2000]
        gain_limits = [0, 30]
        brightness = 50
        roi = [1024, 1024, 1024, 1024]
        resolution = [2048, 2048]
        self.exposure_limit_lower_slider.set(exposure_limits[0])
        self.exposure_limit_upper_slider.set(exposure_limits[1])
        self.gain_limit_lower_slider.set(gain_limits[0])
        self.gain_limit_upper_slider.set(gain_limits[1])
        self.brightness_slider.set(brightness)
        self.roi_width_entry.delete(0, tk.END)
        self.roi_width_entry.insert(0, str(roi[0]))
        self.roi_height_entry.delete(0, tk.END)
        self.roi_height_entry.insert(0, str(roi[1]))
        self.roi_offset_x_entry.delete(0, tk.END)
        self.roi_offset_x_entry.insert(0, str(roi[2]))
        self.roi_offset_y_entry.delete(0, tk.END)
        self.roi_offset_y_entry.insert(0, str(roi[3]))
        self.width_entry.delete(0, tk.END)
        self.width_entry.insert(0, str(resolution[0]))
        self.height_entry.delete(0, tk.END)
        self.height_entry.insert(0, str(resolution[1]))

        return exposure_limits, gain_limits, brightness, roi, resolution


    def return_parameters(self):
        return ([self.exposure_limit_lower_slider.get(), self.exposure_limit_upper_slider.get()],
                [self.gain_limit_lower_slider.get(), self.gain_limit_upper_slider.get()], self.brightness_slider.get(),
                [self.roi_width_entry.get(), self.roi_height_entry.get(), self.roi_offset_x_entry.get(),
                self.roi_offset_y_entry.get()], [self.width_entry.get(), self.height_entry.get()], self.system_started,
                self.recording)

    def start_system(self):
        print("System started!")
        self.system_started = True
        self.update_callback()

    def stop_system(self):
        print("System stopped!")
        self.system_started = False
        self.update_callback()

    def run(self):
        self.root.mainloop()

def funkcja2():
    print("Callback test")

def funkcja3():
    print("Error test")

if __name__ == "__main__":
    gui = CameraGUI(update_callback=funkcja2, error_func=funkcja3)
    gui.run()
