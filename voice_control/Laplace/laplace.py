from .PluginSystem.server import PluginServer
from pyaudio import PyAudio
import speech_recognition
from speech_recognition import UnknownValueError
import os

class Laplace(object):
    def __init__(self, recognition_engine="google", hot_word=None,language='en-US', Authentic_key=None,mic_index=None, speaker_index=None, Plugin_path="./Plugins",api_interface=None):
        self._api_interface=api_interface
        self._api_interface.LogTool.info("Now in file %s"%os.getcwd())
        self._api_interface.LogTool.info("Initializing plugin system")
        self._ps = PluginServer(Plugin_path=Plugin_path, api_interface=api_interface)
        self._audio = PyAudio()
        self._Authentic_key = Authentic_key
        self._mic_index = mic_index if mic_index is not None else self._audio.get_default_input_device_info()['index']
        self._speaker_index = speaker_index if speaker_index is not None else self._audio.get_default_output_device_info()['index']

        self._hot_word = hot_word
        self._language = language
        self._waiting_for_hot_word = True if hot_word is not None else False
        self._recognition_engine = recognition_engine

        self._api_interface.LogTool.info("Initializing speech recognition engine")
        self._recognizer = speech_recognition.Recognizer()
        self._mic = speech_recognition.Microphone(device_index=self._mic_index)
        self._ps.append_event("load")
        self._api_interface.LogTool.info("Laplace initialized successfully!")

    @classmethod
    def get_mic_speaker_device_list(cls) -> 'list[dict]':
        audio = PyAudio()
        device_count = audio.get_device_count()
        device_list = []
        for i in range(0,device_count):
            dev_info = audio.get_device_info_by_index(i)
            device_list.append(dev_info)
            print(f'index:[{dev_info["index"]}] ,name: [{dev_info["name"]}]')
        return device_list

    def _get_recognize(self):
        self._api_interface.LogTool.debug("Start recoding...")
        with self._mic as source:
            self._recognizer.adjust_for_ambient_noise(source)
            _input_audio = self._recognizer.listen(source)
        result = None
        lang = self._language
        self._api_interface.LogTool.debug("Starting recognition...")

        # example
        try:
            if self._recognition_engine == "google":
                    result = self._recognizer.recognize_google(_input_audio,language=self._language,key=self._Authentic_key)
            elif self._recognition_engine == "bing":
                    result = self._recognizer.recognize_bing(_input_audio,language=self._language,key=self._Authentic_key)
            elif self._recognition_engine == "google_cloud":
                result = self._recognizer.recognize_google_cloud(_input_audio,language=self._language,credentials_json=self._credentials_json)
            elif self._recognition_engine == "pocketsphinx":
                result = self._recognizer.recognize_sphinx(_input_audio,language=self._language)
            else:
                self._api_interface.LogTool.warning("Wrong recognition engine specified.")
                raise ValueError("Wrong recognition engine specified.")
        except Exception:
            pass
    
            
        return result

    def spin(self):
        while True:
            self.spin_once()
    
    def spin_once(self):
        speechs = ""
        if self._waiting_for_hot_word:
            speechs = self._get_recognize()
            if self._hot_word == speechs:
                self._waiting_for_hot_word = False
        else:
            #pyttsx3.speak("Say command to me")
            speechs = self._get_recognize()
            self._ps.append_event("command",speechs)
            self._waiting_for_hot_word = True if self._hot_word is not None else False
        self._api_interface.LogTool.debug(f"recognized [{speechs}]")
