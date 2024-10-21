from filefifo import Filefifo
import time
from ssd1306 import SSD1306_I2C
import framebuf
from machine import UART, Pin, I2C, Timer, ADC
from led import Led
from fifo import Fifo
from piotimer import Piotimer
import micropython
import ujson
import utime
import urequests as requests
import network
import ubinascii
from umqtt.simple import MQTTClient
micropython.alloc_emergency_exception_buf(200)




class Isr_ADC:
    def __init__(self, adc_pin):
        self.adc = ADC(adc_pin)
        self.samples = Fifo(2000)
        self.adcled = Led(22)
        
    def handler(self, tid):
        self.adcled.toggle()
        self.samples.put(self.adc.read_u16())
        
class Encoder:
    def __init__(self, rot_a, rot_b, rot_push):
        self.a = Pin(rot_a, mode = Pin.IN, pull = Pin.PULL_UP)
        self.b = Pin(rot_b, mode = Pin.IN, pull = Pin.PULL_UP)
        self.p = Pin(rot_push, mode = Pin.IN, pull = Pin.PULL_UP)
        self.fifo1 = Fifo(30, typecode = 'i')
        self.fifo2 = Fifo(30, typecode = 'i')
        self.a.irq(handler = self.handler1, trigger = Pin.IRQ_RISING, hard = True)
        self.p.irq(handler = self.handler2, trigger = Pin.IRQ_FALLING, hard = True)
        self.btn_pressed_old = 0
        self.btn_pressed_new = 0
        self.led = Led(20)
    
    def handler1(self, pin):
        self.led.toggle()
        if self.b.value():
            self.fifo2.put(-1)
        else:
            self.fifo2.put(1)
    
    def handler2(self, pin):
        self.btn_pressed_new = time.ticks_ms()
        if (self.btn_pressed_new - self.btn_pressed_old) > 300:
            self.btn_pressed_old = self.btn_pressed_new
            self.fifo1.put(0)

class Oled:
    def __init__(self, width, height,image1, image2):
        self.i2c = I2C(1, scl=Pin(15), sda=Pin(14), freq=400000)
        self.width = width
        self.height = height
        self.oled = SSD1306_I2C(self.width, self.height, self.i2c)
        self.font_pi = 1
        self.font_cha = 8
        self.fbuf = framebuf.FrameBuffer(bytearray(2000), self.oled.width, self.oled.height, framebuf.MONO_VLSB)
        self.fbuf1 = framebuf.FrameBuffer(bytearray(2000), self.width, self.font_cha + self.font_pi, framebuf.MONO_VLSB)
        self.fbuf2 = framebuf.FrameBuffer(bytearray(2000), self.width, self.height - (self.font_cha + 2*self.font_pi), framebuf.MONO_VLSB)
        self.fbuf3 = framebuf.FrameBuffer(bytearray(2000), self.width, self.font_cha + self.font_pi, framebuf.MONO_VLSB)
        self.img1 = framebuf.FrameBuffer(image1, 16, 16, framebuf.MONO_VLSB)
        self.img2 = framebuf.FrameBuffer(image2, 27, 27, framebuf.MONO_VLSB)
        self.oled.fill(0)
        self.oled.show()
    
        
class Data:
    def __init__(self, adc_pin, sample_rate, oled):
        self.adc = Isr_ADC(adc_pin)
        self.sample_rate = sample_rate
        self.oled = oled
        self.count_sample = 1
        self.sample = 0
        #variable for display
        self.x1 = 0
        self.y1 = 40
        self.x2 = 0
        self.y2 = 0
        self.count_display = 0        
        self.mid_val = 65525 / 2
        #variable for processing
        self.avr = 0
        self.min_hr = 40
        self.max_hr = 110        
        self.pre_peak_index = 0
        self.cur_peak_index= 0
        self.pre_peak = 0
        self.cur_peak = 0
        self.sum_sample = 0
        self.interval_num = 0
        self.ppi = 0
        self.sdnn = 0
        self.rmssd = 0
        self.sum_ppi = 0
        self.mean_ppi = 0
        self.mean_hr = 0
        self.ppi_list = []
        self.hr_dic = {}
        self.ppi_display = []

    def read(self):
        self.tmr = Piotimer(mode = Piotimer.PERIODIC, freq = self.sample_rate, callback = self.adc.handler)
        
    def stop_read(self):
        while self.adc.samples.has_data():
            self.adc.samples.get()
        self.tmr.deinit()
                
    def check_variability(self):
        return abs(int((self.cur_peak_index - self.pre_peak_index + 1) * 1000 / self.sample_rate) - self.ppi) < 120
    
    def get_avr(self):
        self.sum_sample += self.sample
        self.avr = self.sum_sample / self.count_sample
            
    def hr_detect(self):
        if self.sample > (self.avr * 1.1):
            if self.sample > self.cur_peak:
                self.cur_peak = self.sample
                self.cur_peak_index = self.count_sample
        else:
            if self.cur_peak > 0:
                if (self.cur_peak_index - self.pre_peak_index) > (60 * self.sample_rate / self.min_hr):
                        self.pre_peak = 0
                        self.pre_peak_index = self.cur_peak_index
                else:
                    if self.cur_peak >= (self.pre_peak * 0.8):
                        if (self.cur_peak_index - self.pre_peak_index) > (60 * self.sample_rate / self.max_hr):
                            if self.pre_peak > 0:
                                if self.ppi != 0:
                                    if self.check_variability():
                                        self.interval_num = self.cur_peak_index - self.pre_peak_index
                                        self.ppi = round(self.interval_num * 1000 / self.sample_rate)
                                        self.ppi_list.append(self.ppi)
                                        self.ppi_display.append(self.ppi)
                                        print(round(60/self.ppi*1000))
                                else:
                                    self.interval_num = self.cur_peak_index - self.pre_peak_index
                                    self.ppi = round(self.interval_num * 1000 / self.sample_rate)
                            self.pre_peak = self.cur_peak
                            self.pre_peak_index = self.cur_peak_index
            self.cur_peak = 0
            
    def cal_mean_ppi(self):
        self.sum_ppi = 0
        for i in self.ppi_list:
            self.sum_ppi += i
        self.mean_ppi = round(self.sum_ppi / len(self.ppi_list))
        self.hr_dic["mean_ppi"] = self.mean_ppi
        
    def cal_mean_hr(self):
        self.mean_hr = round(60 / self.mean_ppi * 1000)  
        self.hr_dic["mean_hr"] = self.mean_hr
        
    def cal_SDNN(self):
        self.sdnn = 0
        for i in self.ppi_list:
            self.sdnn += (i - self.mean_ppi)**2
        self.sdnn = round((self.sdnn / (len(self.ppi_list) - 1))**(1/2))
        self.hr_dic["sdnn"] = self.sdnn
        
    def cal_RMSSD(self):
        self.rmssd = 0
        difference = 0
        for i in range(len(self.ppi_list) - 1):
            difference = self.ppi_list[i+1] - self.ppi_list[i]
            self.rmssd += difference**2
        self.rmssd = round((self.rmssd / (len(self.ppi_list) - 1)) ** 0.5)
        self.hr_dic["rmssd"] = self.rmssd

    def get_peaks_per_interval(self):
       return self.ppi_list

    def result_dictionary(self):
        return self.hr_dic
    
    # methods for display
    def convert(self):
        self.mid_val = 0.8 * self.mid_val + 0.2 * self.sample
        self.y2 = round((self.mid_val-self.sample) / 350 + 25)
        if self.y2 > self.oled.height - (self.oled.font_cha + 2 * self.oled.font_pi) * 2 - 4 * self.oled.font_pi:
            self.y2 = self.oled.height - (self.oled.font_cha + 2 * self.oled.font_pi) * 2 - 4 * self.oled.font_pi
        elif self.y2 < (self.oled.font_pi * 4 ):
            self.y2 = self.oled.font_pi * 4
            
    def first_display(self):
        self.oled.oled.fill(0)
        self.oled.fbuf1.fill(1)
        self.oled.fbuf3.fill(1)
        self.oled.oled.blit(self.oled.fbuf1, 0, 0)
        self.oled.oled.blit(self.oled.fbuf3, 0, self.oled.height - self.oled.font_cha - 2*self.oled.font_pi)
        self.oled.oled.show()
        
    def update_fbuf1(self):
        if len(self.ppi_display) >= 3:
            self.sum_ppi = 0
            for i in self.ppi_display:
                self.sum_ppi += i
                mean_ppi = round(self.sum_ppi / len(self.ppi_display))
                mean_hr = round(60 / mean_ppi * 1000)
            self.text = "HR:" + str(mean_hr)
            self.oled.fbuf1.fill(1)
            self.oled.fbuf1.text(self.text, int(self.oled.width / 2 - len(self.text)* self.oled.font_cha / 2), self.oled.font_pi, 0)
            self.oled.oled.blit(self.oled.fbuf1, 0, 0)
            print(self.ppi_display)
            self.ppi_display = []
    def update_fbuf2(self):
        self.x2 = self.x1 + 1
        self.convert()
        self.oled.fbuf2.line(self.x1, self.y1, self.x2, self.y2, 1)
        self.oled.oled.blit(self.oled.fbuf2, 0, self.oled.font_cha + 2*self.oled.font_pi)
        
    def update_fbuf3(self):
        self.oled.fbuf3.fill(1)
        self.text = "Timer:" + str(round(self.count_sample / self.sample_rate)) + "s"
        self.oled.fbuf3.text(self.text, int(self.oled.width / 2 - len(self.text)* self.oled.font_cha / 2) , self.oled.font_pi, 0)
        self.oled.oled.blit(self.oled.fbuf3, 0, self.oled.height - self.oled.font_cha - 2*self.oled.font_pi)
    # main method    
    def process_and_display(self):
        while self.count_sample < 200:
            if self.adc.samples.has_data():
                self.sample = self.adc.samples.get()
                self.get_avr()
                self.count_sample += 1
        if self.adc.samples.has_data():
            self.sample = self.adc.samples.get()
            self.count_sample += 1
            self.count_display += 1
            self.get_avr()
            if self.count_display % 10 == 0:
                if (self.count_sample / self.sample_rate) % 5 == 0:
                    self.update_fbuf1()                                
                self.update_fbuf2()
                self.update_fbuf3()
                self.oled.oled.show()
                self.x1 = self.x2
                self.y1 = self.y2
                if self.x1 > 127:
                    self.x1 = 0
                    self.oled.fbuf2.fill(0)
                    self.oled.oled.show()
            self.hr_detect()
            
    def reset(self):
        self.x1 = 0
        self.y1 = 40
        self.x2 = 0
        self.y2 = 0
        self.count_display = 0        
        self.mid_val = 65525 / 2
        self.oled.fbuf1.fill(0)
        self.oled.fbuf2.fill(0)
        self.oled.fbuf3.fill(0)
        self.avr = 0
        self.min_hr = 40
        self.max_hr = 110        
        self.pre_peak_index = 0
        self.cur_peak_index= 0
        self.pre_peak = 0
        self.cur_peak = 0
        self.sum_sample = 0
        self.interval_num = 0
        self.ppi = 0
        self.sdnn = 0
        self.rmssd = 0
        self.sum_ppi = 0
        self.mean_ppi = 0
        self.mean_hr = 0
        self.ppi_list = []
        self.hr_dic = {}
        self.ppi_display = []
        
    def check_bad_signal(self):
        if len(self.ppi_list) <= 3:
            return True
        else:
            return False
            
    def display_result_state0c(self):
        if len(self.ppi_list) > 3:
            self.cal_mean_ppi()
            self.cal_mean_hr()
            self.oled.oled.text(f"Mean PPI = {self.mean_ppi}", 0, 5, 1)
            self.oled.oled.text(f"Mean HR = {self.mean_hr}", 0, 15, 1)
            self.oled.oled.text("Press to go back", 0, self.oled.height - 5 - self.oled.font_cha, 1)
            self.oled.oled.show()
        else:
            self.oled.oled.text("Bad signal", 0, 5, 1)
            self.oled.oled.text("Try again", 0, 15, 1)
            self.oled.oled.text("Press to go back", 0, self.oled.height - 5 - self.oled.font_cha, 1)
            self.oled.oled.show()
            
    def display_result_state1c(self):
        self.cal_mean_ppi()
        self.cal_mean_hr()
        self.cal_SDNN()
        self.cal_RMSSD()
        self.oled.oled.text(f"Mean PPI = {self.mean_ppi}", 0, 5, 1)
        self.oled.oled.text(f"Mean HR = {self.mean_hr}", 0, 15, 1)
        self.oled.oled.text(f"SDNN = {self.sdnn}", 0, 25, 1)
        self.oled.oled.text(f"RMSSD = {self.rmssd}", 0, 35, 1)
        self.oled.oled.text("Press to send", 0, self.oled.height - 5 - self.oled.font_cha, 1)
        self.oled.oled.show()
    
        
            
            
class Kubios:
    def __init__(self, wifi_name, wifi_password,oled):
        self.APIKEY = "pbZRUi49X48I56oL1Lq8y8NDjq6rPfzX3AQeNo3a"
        self.CLIENT_ID = "3pjgjdmamlj759te85icf0lucv"
        self.CLIENT_SECRET = "111fqsli1eo7mejcrlffbklvftcnfl4keoadrdv1o45vt9pndlef"
        self.LOGIN_URL = "https://kubioscloud.auth.eu-west-1.amazoncognito.com/login"
        self.TOKEN_URL = "https://kubioscloud.auth.eu-west-1.amazoncognito.com/oauth2/token"
        self.REDIRECT_URI = "https://analysis.kubioscloud.com/v1/portal/login"
        self.data = 'grant_type=client_credentials&client_id={}'.format(self.CLIENT_ID)
        self.headers = {'Content-Type':'application/x-www-form-urlencoded'}
        self.wifi_name = wifi_name
        self.wifi_password = wifi_password
        self.final_result = {}
        self.oled = oled
        self.current_time = 0
        


    def connect(self):
        try:
            self.oled.oled.text("Connecting to", 0, 5, 1)
            self.oled.oled.text("wifi", 10, 15, 1)
            self.oled.oled.show()
            wlan = network.WLAN(network.STA_IF)
            wlan.active(True)
            wlan.connect(self.wifi_name, self.wifi_password)
            count = 1
            while wlan.isconnected() == False and count < 2:
                wlan.active(True)
                wlan.connect(self.wifi_name, self.wifi_password)
                time.sleep(1)
                count += 1
            
        except:
            self.oled.oled.fill(0)
            self.oled.oled.text("Could not connect", 0, 5, 1)
            self.oled.oled.text(" to wifi", 30, 15, 1)
            self.oled.oled.show()

    def get_result(self):
        return self.final_result

    def show_data(self, record):
        try:
            self.oled.oled.fill(0)
            self.oled.oled.text("Sending data...", 0, 5, 1)
            self.oled.oled.show()
            dataset = {
                "type": "RRI",
                "data": record,
                "analysis": {"type": "readiness"}
            }
            response = requests.post(
                url = self.TOKEN_URL,
                data = 'grant_type=client_credentials&client_id={}'.format(self.CLIENT_ID),
                headers = {'Content-Type':'application/x-www-form-urlencoded'},
                auth = (self.CLIENT_ID, self.CLIENT_SECRET))

            response = response.json()
            access_token = response["access_token"]

            response = requests.post(
                url = "https://analysis.kubioscloud.com/v2/analytics/analyze",
                headers = { "Authorization": "Bearer {}".format(access_token),"X-Api-Key": self.APIKEY},
                json = dataset
            )
            response = response.json()
            print(response)
            
            self.current_time = response['analysis']['create_timestamp']
            print(self.current_time)
            date_part, time_part = (self.current_time).split('T')
            print("1")
            year, month, day = date_part.split('-')
            print("2")
            hour, minute = time_part.split(':')[0:2]
            print("3")
            timestamp_list = [int(day), int(month),int(year), int(hour), int(minute)]
            print("4")
            print(timestamp_list)
            timestamp_list[3] += 3
            print(timestamp_list)
            self.current_time = timestamp_list
            
            self.final_result['Mean_HR']= round(response['analysis']["mean_hr_bpm"])
            self.final_result['Mean_RR']= round(response['analysis']["mean_rr_ms"])
            self.final_result['RMSSD']= round(response['analysis']["rmssd_ms"], 2)
            self.final_result['SDNN']= round(response['analysis']["sdnn_ms"], 2)
            self.final_result['SNS']= round(response['analysis']["sns_index"], 2)
            self.final_result['PNS']= round(response['analysis']["pns_index"], 2)

            self.oled.oled.fill(0)
            self.oled.oled.text("Mean_HR", 0, 0, 1)
            self.oled.oled.text("Mean_RR", 0, 8, 1)
            self.oled.oled.text("RMSSD", 0, 16, 1)
            self.oled.oled.text("SDNN", 0, 24, 1)
            self.oled.oled.text("SNS", 0, 32, 1)
            self.oled.oled.text("PNS", 0, 40, 1)
            self.oled.oled.text("PRESS TO GO BACK", 0, 48, 1)

            self.oled.oled.text(str(self.final_result['Mean_HR']), 60, 0, 1)
            self.oled.oled.text(str(self.final_result['Mean_RR']), 60, 8, 1)
            self.oled.oled.text(str(self.final_result['RMSSD']), 60, 16, 1)
            self.oled.oled.text(str(self.final_result['SDNN']), 60, 24, 1)
            self.oled.oled.text(str(self.final_result['SNS']), 60, 32, 1)
            self.oled.oled.text(str(self.final_result['PNS']), 60, 40, 1)
            self.oled.oled.show()
            return True

        except:
            self.oled.oled.fill(0)
            self.oled.oled.text("ERROR SENDING", 0,0, 1)
            self.oled.oled.text("DATA", 32, 10, 1)
            self.oled.oled.text("PRESS BUTTON TO", 0, 25, 1)
            self.oled.oled.text("RETRY", 32, 35, 1)
            self.oled.oled.text("OR WAIT 3 SECONDS TO", 0, 45, 1)
            self.oled.oled.text("GO TO MAIN MENU", 0, 55, 1)
            self.oled.oled.show()
            return False

class MQTT():
    def __init__(self, wifi_name, wifi_password, BROKER_IP):
        self.wifi_name = wifi_name
        self.wifi_password = wifi_password
        self.BROKER_IP = BROKER_IP
        self.sending_successful = True

# Function to connect to WLAN
    def connect(self):
        try:
            wlan = network.WLAN(network.STA_IF)
            wlan.active(True)
            wlan.connect(self.wifi_name, self.wifi_password)
            count = 1
            while wlan.isconnected() == False and count <= 2:
                count += 1
                time.sleep(0.005)
                print("connecting...")
        except:
            print("Could not connect to MQTT. Pico IP")

    def connect_mqtt(self):
        mqtt_client=MQTTClient("", self.BROKER_IP)
        mqtt_client.connect(clean_session=True)
        return mqtt_client

    def publish_data(self, data):
        self.sending_successful = True
        try:
            self.connect()
            mqtt_client = self.connect_mqtt()
            i = 0
            while i < 2 :
                # Sending a message every 3 seconds.
                topic = "HRV-analysis"
                message = data
                json_message = ujson.dumps(message)
                mqtt_client.publish(topic, json_message)
                print(f"Sending to MQTT: {topic} -> {json_message}")
                time.sleep(0.5)
                i += 1

        except Exception as e:
            print(f"Failed to send MQTT message: {e}")
            self.sending_successful = False

    
class History:
    def __init__(self, rotary, oled):
        self.rot = rotary
        self.oled = oled
        self.path = "/history.json"
        self.measurement = {}#this will store the new measurement gotten from kubios cloud
        self.data = {}#this will store the data loaded from json file
        self.position = 1#the position needed to choose measurement number
        self.change_rotate = 0#the change is 1,0, or -1 based on the encoder class
        self.change_button = None
    
    #this function reads data from json file
    def load_data(self):
        with open(self.path, 'r') as file:
            self.data = ujson.load(file)
    
    #this function re-writes the content of json file
    def dump_data(self):
        with open(self.path, 'w') as file:
            ujson.dump(self.data,file)
    
    #this function show the initial display when going in the history tab
    def initial_display(self):
        self.oled.oled.fill(0)
        self.load_data()
        self.oled.oled.text("HISTORY", 35, 0, 1)
        for a in range(28, 96, 1):
            self.oled.oled.pixel(a, 9, 1)
        if len(self.data) == 0:#this shows when there is not yet any data
            self.oled.oled.text("NO DATA", 35, 20, 1)
            self.oled.oled.text("AVAILABLE", 27, 30, 1)
        for a in range(len(self.data)):
            self.oled.oled.text(f"Measurement {a+1}", 8, 12+8*a, 1)#writes how many measurements there are based on self.data length
        self.oled.oled.text("EXIT", 47, 56, 1)
        self.oled.oled.show()
    
    #this function add the latest measurement to the history.json file with time stamp
    def add_measurement(self, measurement, time):
        self.measurement = measurement
        self.load_data()
        timestamp = f"{time[2]:02d}-{time[1]:02d}-{time[0]} {time[3]:02d}:{time[4]:02d}"
        self.measurement["timestamp"]=timestamp
        #if there are less than 5 measurements, it will change the names of existing measurements and then add the new measurement
        #this will keep the order of data by first in first out rule
        if len(self.data) < 5:
            for a in range(len(self.data),0,-1):
                self.data[f"measure{(a+1)}"]=self.data.pop(f"measure{a}")
            self.data["measure1"]=self.measurement
        else:#if there are already 5 measurements, it will first delete the oldest data then rename and add new data
            del self.data["measure5"]
            for a in range(len(self.data),0,-1):
                self.data[f"measure{(a+1)}"]=self.data.pop(f"measure{a}")
            self.data["measure1"]=self.measurement
        self.dump_data()
    
    #this function update the position/the measurement number
    def update(self):
        while self.rot.fifo2.has_data():
            self.change_rotate = self.rot.fifo2.get()
            self.position += self.change_rotate
            if self.position < 1:
                self.position = 1
            elif self.position > len(self.data)+1:
                self.position = len(self.data)+1
        self.update_brackets()
    
    def update2(self):#this loop will ignore scroll and react only when encoder is pressed to go back
        while self.rot.fifo1.has_data():
            self.change_button = self.rot.fifo1.get()
            if self.change_button == 0:
                break
    def main(self):
        self.position = 1
        while True:
            self.initial_display()
            while True:
                self.change_button = None
                self.update()
                self.update2()
                if self.change_button == 0:
                    break
            if self.position <= len(self.data):
                self.show_hrv()
            elif self.position > len(self.data):
                return
            while True:
                self.change_button = None
                self.update2()
                if self.change_button == 0:
                    break
            
    #this function update the brackets according to the rotary knob scroll
    def update_brackets(self):
        if self.position <= len(self.data):
            self.oled.oled.fill_rect(0, 8, 8, 8*6, 0)
            self.oled.oled.fill_rect(8*14, 8, 8, 8*6, 0)
            self.oled.oled.fill_rect(0, 56, 46, 8, 0)
            self.oled.oled.fill_rect(80, 56, 46, 8, 0)
            self.oled.oled.text("[", 0, 4+self.position*8, 1)
            self.oled.oled.text("]", 8*14, 4+self.position*8, 1)
            self.oled.oled.show()
        elif self.position > len(self.data):
            self.oled.oled.fill_rect(0, 8, 8, 8*6, 0)
            self.oled.oled.fill_rect(8*14, 8, 8, 8*6, 0)
            self.oled.oled.text("[", 39, 56, 1)
            self.oled.oled.text("]", 79, 56, 1)
            self.oled.oled.show()
    
    #this function shows the corresponding measurement number
    def show_hrv(self):
        self.oled.oled.fill(0)
        hrv_data = self.data[f"measure{self.position}"]
        self.oled.oled.text(f"{hrv_data["timestamp"]}", 0, 0, 1)
        self.oled.oled.text(f"MEAN HR: {hrv_data["Mean_HR"]}", 0, 8, 1)
        self.oled.oled.text(f"MEAN PPI: {hrv_data["Mean_RR"]}", 0, 16, 1)
        self.oled.oled.text(f"RMSSD: {hrv_data["RMSSD"]}", 0, 24, 1)
        self.oled.oled.text(f"SDNN: {hrv_data["SDNN"]}", 0, 32, 1)
        self.oled.oled.text(f"SNS: {hrv_data["SNS"]}", 0, 40, 1)
        self.oled.oled.text(f"PNS: {hrv_data["PNS"]}", 0, 48, 1)
        self.oled.oled.text("Press to go back", 0, 56, 1)
        self.oled.oled.show()

class States:
    def __init__(self, delay_time, oled, data, rotary, kubios, mqtt, history):
        self.delay = delay_time
        self.oled = oled
        self.data = data
        self.rot = rotary
        self.history = history
        self.text = ""
        self.num_option = 0
        self.change = None
        self.btn_val = False
        self.state = 0
        self.kubios = kubios
        self.mqtt = mqtt
        
    def check_btn_press(self):
        if self.rot.fifo1.has_data():
            self.change = self.rot.fifo1.get()            
            if self.change == 0:
                self.btn_val ^= 1
            else:
                while self.rot.fifo1.has_data():
                    self.rot.fifo1.get()
        return self.btn_val
    
    def change_menu_state(self):
        if self.rot.fifo2.has_data():
            self.change = self.rot.fifo2.get()
            if self.change != 0:
                self.num_option += self.change
                if self.num_option < 0:
                    self.num_option = 0
                elif self.num_option > 4:
                    self.num_option = 4
        
    def clean_oled(self):
        self.oled.oled.fill(0)
        self.oled.oled.show()
        
    def first_menu_display(self):
        self.oled.oled.fill(0)
        self.oled.oled.text(f"_ Measure HR", 0, 0, 1)
        self.oled.oled.text(f"  Basic HRV", 0, (self.oled.font_cha + 4) * 1, 1)
        self.oled.oled.text(f"  Kubios", 0, (self.oled.font_cha + 4) * 2, 1)
        self.oled.oled.text(f"  History", 0, (self.oled.font_cha + 4) * 3, 1)
        self.oled.oled.text(f"  EXIT", 0, (self.oled.font_cha + 4) * 4, 1)
        self.oled.oled.show()
    
    def update_menu_display(self):
        self.oled.oled.fill_rect(0, 0, self.oled.font_cha * 2, self.oled.height, 0)
        self.oled.oled.text("-", 0, (self.oled.font_cha + 4) * self.num_option)
        self.oled.oled.show()
        
    def change_state_based_on_option(self):
        if self.num_option == 0:
            self.state = 3
        elif self.num_option == 1:
            self.state = 6
        elif self.num_option == 2:
            self.state = 11
        elif self.num_option == 3:
            self.state = 18
        elif self.num_option == 4:
            self.state = 19
        
    def state_off(self):
        self.oled.oled.fill(0)
        self.oled.oled.show()
        self.num_option = 0
            
    def state_begin(self):
        self.text = "Smart Heart"
        self.oled.oled.text(self.text, round(self.oled.width / 2) - round(len(self.text) * self.oled.font_cha / 2), 5, 1)
        self.oled.oled.blit(self.oled.img1, 56, 30)
        self.text = "Press to start"
        self.oled.oled.text(self.text, round(self.oled.width / 2) - round(len(self.text) * self.oled.font_cha / 2), self.oled.height - self.oled.font_cha, 1)
        self.oled.oled.show()
        time.sleep(0.2)
        self.oled.oled.blit(self.oled.img2, 50, 25)
        self.oled.oled.show()
        time.sleep(0.2)
        self.oled.oled.fill_rect(0, 25, self.oled.width, 30, 0)

    def state_menu(self):
        while self.rot.fifo2.has_data():
            self.change_menu_state()
        self.update_menu_display()

    #option 0: measure HR
    def state0a(self):
        self.text = "Touch the sensor"
        self.oled.oled.text(self.text, round(self.oled.width / 2) - round(self.oled.font_cha * len(self.text) / 2), 5, 1)
        self.text = "and"
        self.oled.oled.text(self.text, round(self.oled.width / 2) - round(self.oled.font_cha * len(self.text) / 2), self.oled.font_cha + 5*1 + self.oled.font_cha, 1)
        self.text = "Press to start"
        self.oled.oled.text(self.text, round(self.oled.width / 2) - round(self.oled.font_cha * len(self.text) / 2), self.oled.font_cha * 2 + 5*2 + self.oled.font_cha, 1)
        self.oled.oled.show()

    def state0b(self):
        self.data.read()
        self.data.first_display()
        while self.data.count_display <= self.data.sample_rate * 59 and not self.check_btn_press():
            self.data.process_and_display()
        self.data.stop_read()
        self.data.count_sample = 1
        self.btn_val = False
    def state0c(self):
        self.data.display_result_state0c()

    #option 1: basic HRV
    def state1a(self):
        self.text = "Touch the sensor"
        self.oled.oled.text(self.text, round(self.oled.width / 2) - round(self.oled.font_cha * len(self.text) / 2), 5, 1)
        self.text = "and"
        self.oled.oled.text(self.text, round(self.oled.width / 2) - round(self.oled.font_cha * len(self.text) / 2), self.oled.font_cha + 5*1 + self.oled.font_cha, 1)
        self.text = "Press to start"
        self.oled.oled.text(self.text, round(self.oled.width / 2) - round(self.oled.font_cha * len(self.text) / 2), self.oled.font_cha * 2 + 5*2 + self.oled.font_cha, 1)
        self.oled.oled.show()

    def state1b(self):
        self.data.read()
        self.data.first_display()
        while self.data.count_sample <= self.data.sample_rate * 60 and not self.check_btn_press():
            self.data.process_and_display()
        self.data.stop_read()
        self.data.count_sample = 1
        self.btn_val = False

    def state1c(self):
        self.data.display_result_state1c()

    def state1d(self):
        self.oled.oled.text("Sending...", 0, 5, 1)
        self.oled.oled.show()
        self.mqtt.publish_data(self.data.result_dictionary())
        self.clean_oled()
        if self.mqtt.sending_successful:
            self.oled.oled.text("Successful", 0, 5, 1)
            self.oled.oled.show()
        else:
            self.oled.oled.text("Failed", 0, 5, 1)
            self.oled.oled.show()
            
    def state1e(self):
        self.oled.oled.text("Bad signal", 0, 5, 1)
        self.oled.oled.text("Try again", 0, 15, 1)
        self.oled.oled.show()

    #option 2: Kubios
    def state2a(self):
        self.text = "Touch the sensor"
        self.oled.oled.text(self.text, round(self.oled.width / 2) - round(self.oled.font_cha * len(self.text) / 2), 5, 1)
        self.text = "and"
        self.oled.oled.text(self.text, round(self.oled.width / 2) - round(self.oled.font_cha * len(self.text) / 2), self.oled.font_cha + 5*1 + self.oled.font_cha, 1)
        self.text = "Press to start"
        self.oled.oled.text(self.text, round(self.oled.width / 2) - round(self.oled.font_cha * len(self.text) / 2), self.oled.font_cha * 2 + 5*2 + self.oled.font_cha, 1)
        self.oled.oled.show()
        
    def state2b(self):
        self.data.read()
        self.data.first_display()
        while self.data.count_sample <= self.data.sample_rate * 60 and not self.check_btn_press():
            self.data.process_and_display()
        self.data.stop_read()
        self.data.count_sample = 1
        self.btn_val = False
        
    def state2c(self):
        self.kubios.connect()

    def state2d(self):
        return self.kubios.show_data(self.data.get_peaks_per_interval())
    
    def state2e(self):
        self.oled.oled.text("Bad signal", 0, 5, 1)
        self.oled.oled.text("Try again", 0, 15, 1)
        self.oled.oled.show()
    
  #option 3: History
    def state3a(self):
        self.history.main()
        
    #option 4: Exit   
    def state4(self):
        self.text = "Thank you"
        self.oled.oled.text(self.text, round(self.oled.width / 2) - round(self.oled.font_cha * len(self.text) / 2), 5, 1)
        self.text = "See you again"
        self.oled.oled.text(self.text, round(self.oled.width / 2) - round(self.oled.font_cha * len(self.text) / 2), 15, 1)
        self.oled.oled.show()








