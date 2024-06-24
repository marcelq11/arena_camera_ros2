# camera_gui.py
import tkinter as tk


class CameraGUI:
    def __init__(self, update_callback):
        self.pixel_format = 0.0  #None
        self.gain = 0.0  #None
        self.exposure = 0.0  #None
        self.gamma = 0.0  #None
        self.pixel_format_list = ["bayer_rggb8", "bayer_rggb12", "bayer_rggb16", "bayer_bggr8", "bayer_bggr12",
                                  "bayer_bggr16", "bayer_gbrg8", "bayer_gbrg12", "bayer_gbrg16", "bayer_grbg8",
                                  "bayer_grbg12", "bayer_grbg16", "mono8", "mono16"]

        self.gamma_step = 0.1
        self.gain_step = 1.0
        self.exposure_step = 1000.0

        self.update_callback = update_callback
        self.root = tk.Tk()
        self.root.title("Camera Settings")

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

        self.pixel_format = tk.StringVar(self.root)
        self.pixel_format.set(self.pixel_format_list[0])
        tk.Label(self.root, text="Pixel Format").grid(row=0, column=10)
        self.pixel_format_menu = tk.OptionMenu(self.root, self.pixel_format, *self.pixel_format_list)
        self.pixel_format_menu.grid(row=1, column=10)

        tk.Button(self.root, text="Update Settings", command=update_callback).grid(row=2, columnspan=2)


    def run(self):
        self.root.mainloop()

    def update_entries(self):
        self.gain_entry.delete(0, tk.END)
        self.gain_entry.insert(0, str(self.gain))

        self.exposure_entry.delete(0, tk.END)
        self.exposure_entry.insert(0, str(self.exposure))

        self.gamma_entry.delete(0, tk.END)
        self.gamma_entry.insert(0, str(self.gamma))

    def change_param(self, param_name, value):
        current_value = getattr(self, param_name)
        new_value = current_value + value
        print(f"Changing {param_name} from {current_value} to {new_value}")
        setattr(self, param_name, new_value)
        self.update_entries()
        self.update_callback()

    def get_settings(self):
        self.exposure = float(self.exposure_entry.get())
        self.gamma = float(self.gamma_entry.get())
        self.gain = float(self.gain_entry.get())
        self.pixel_format = 'bayer_rggb8'
        print(f"values: {self.gain}, {self.exposure}, {self.gamma}, {self.pixel_format}")
        return self.gain, self.exposure, self.gamma, self.pixel_format


#create a main loop
def main():
    app = CameraGUI(CameraGUI.get_settings)
    app.run()


if __name__ == '__main__':
    main()
